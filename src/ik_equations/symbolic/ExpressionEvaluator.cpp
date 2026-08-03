#include "ik_equations/symbolic/ExpressionEvaluator.hpp"

#include <cmath>
#include <type_traits>
#include <utility>
#include <variant>

namespace kinemaforge::ik {

namespace {

std::unexpected<EvaluationError> makeError(EvaluationErrorCode code,
                                           std::string symbolName = {})
{
    return std::unexpected(EvaluationError{code, std::move(symbolName)});
}

} // namespace

ExpressionEvaluator::ExpressionEvaluator(SymbolValues values)
    : values_(std::move(values))
{
}

EvaluationStatistics ExpressionEvaluator::statistics() const noexcept
{
    return statistics_;
}

std::expected<double, EvaluationError>
ExpressionEvaluator::evaluate(const Expression& expression)
{
    return evaluateNode(expression);
}

std::expected<double, EvaluationError>
ExpressionEvaluator::evaluateNode(const Expression& expression)
{
    if (const auto cached = memo_.find(expression); cached != memo_.end())
    {
        ++statistics_.cacheHits;
        return cached->second;
    }
    ++statistics_.cacheMisses;

    const auto result = std::visit(
        [this](const auto& node) -> std::expected<double, EvaluationError> {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, ConstantNode>)
            {
                return node.value;
            }
            else if constexpr (std::is_same_v<Node, SymbolNode>)
            {
                const auto binding = values_.find(node.name);
                if (binding == values_.end())
                    return makeError(EvaluationErrorCode::MissingSymbol, node.name);

                // Bindings are validated on read, not in the constructor: an
                // unused NaN binding cannot affect any result, so it is not
                // an error. See AcceptsNonFiniteBindingForUnusedSymbol.
                if (!std::isfinite(binding->second))
                    return makeError(EvaluationErrorCode::NonFiniteSymbolValue, node.name);

                return binding->second;
            }
            else if constexpr (requires { node.lhs; node.rhs; })
            {
                // Left before right, unconditionally. Both are visited unless
                // the left one already failed; neither is skipped because of
                // the other's value.
                const auto lhs = evaluateNode(node.lhs);
                if (!lhs) return std::unexpected(lhs.error());

                const auto rhs = evaluateNode(node.rhs);
                if (!rhs) return std::unexpected(rhs.error());

                if constexpr (std::is_same_v<Node, AddNode>)
                    return *lhs + *rhs;
                else if constexpr (std::is_same_v<Node, SubtractNode>)
                    return *lhs - *rhs;
                else if constexpr (std::is_same_v<Node, MultiplyNode>)
                    return *lhs * *rhs;
                else
                {
                    static_assert(std::is_same_v<Node, DivideNode>);
                    // Exact comparison, no tolerance -- consistent with the
                    // rest of the symbolic layer. Catches -0.0 for free,
                    // since -0.0 == 0.0.
                    if (*rhs == 0.0)
                        return makeError(EvaluationErrorCode::DivisionByZero);
                    return *lhs / *rhs;
                }
            }
            else
            {
                const auto operand = evaluateNode(node.operand);
                if (!operand) return std::unexpected(operand.error());

                if constexpr (std::is_same_v<Node, NegateNode>)
                    return -*operand;
                else if constexpr (std::is_same_v<Node, SinNode>)
                    return std::sin(*operand);
                else
                {
                    static_assert(std::is_same_v<Node, CosNode>);
                    return std::cos(*operand);
                }
            }
        },
        expression.node().value);

    if (!result)
        return result;

    // Checked for every node, leaves included. ExpressionFactory asserts that
    // constants are finite, but that assert is gone under NDEBUG, and an
    // overflow such as q1 * q2 with both at 1e308 is reachable in any build.
    if (!std::isfinite(*result))
        return makeError(EvaluationErrorCode::NonFiniteResult);

    memo_.emplace(expression, *result);
    return result;
}

} // namespace kinemaforge::ik
