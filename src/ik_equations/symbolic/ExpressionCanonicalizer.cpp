#include "ik_equations/symbolic/ExpressionCanonicalizer.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace kinemaforge::ik {

namespace {

static_assert(std::numeric_limits<double>::is_iec559,
              "the exactness predicates assume IEEE-754");
static_assert(std::numeric_limits<double>::digits == 53,
              "the exactness predicates assume a 53-bit significand");
static_assert(std::numeric_limits<double>::max_exponent == 1024,
              "the exactness predicates assume binary64 exponent bounds");
static_assert(std::numeric_limits<double>::min_exponent == -1021,
              "the exactness predicates assume binary64 exponent bounds");
static_assert(std::numeric_limits<double>::has_denorm == std::denorm_present,
              "2Sum assumes gradual underflow; flush-to-zero breaks it");

int kindRank(ExpressionType type) noexcept
{
    switch (type)
    {
    case ExpressionType::Constant: return 0;
    case ExpressionType::Symbol:   return 1;
    case ExpressionType::Sin:      return 2;
    case ExpressionType::Cos:      return 3;
    case ExpressionType::Negate:   return 4;
    case ExpressionType::Multiply: return 5;
    case ExpressionType::Divide:   return 6;
    case ExpressionType::Add:      return 7;
    case ExpressionType::Subtract: return 8;
    }
    return 9;
}

double additionError(double a, double b) noexcept
{
    const double sum = a + b;
    const double shifted = sum - a;
    return (a - (sum - shifted)) + (b - shifted);
}

bool multipliesExactlyImpl(double a, double b) noexcept
{
    const double product = a * b;

    if (!std::isfinite(product))
        return false;
    if (a == 0.0 || b == 0.0)
        return true;

    int exponentA = 0;
    int exponentB = 0;
    const double fractionA = std::frexp(std::fabs(a), &exponentA);
    const double fractionB = std::frexp(std::fabs(b), &exponentB);

    const auto significandA = static_cast<std::uint64_t>(std::ldexp(fractionA, 53));
    const auto significandB = static_cast<std::uint64_t>(std::ldexp(fractionB, 53));

    const std::uint64_t lowA = significandA & 0xFFFFFFFFu;
    const std::uint64_t highA = significandA >> 32;
    const std::uint64_t lowB = significandB & 0xFFFFFFFFu;
    const std::uint64_t highB = significandB >> 32;

    const std::uint64_t cross = lowA * highB + highA * lowB;
    std::uint64_t high = highA * highB + (cross >> 32);
    const std::uint64_t crossLow = cross << 32;
    std::uint64_t low = lowA * lowB + crossLow;
    if (low < crossLow)
        ++high;

    const int trailingZeros =
        low != 0 ? std::countr_zero(low) : 64 + std::countr_zero(high);
    const int bitWidth =
        high != 0 ? 64 + std::bit_width(high) : std::bit_width(low);

    const int significantBits = bitWidth - trailingZeros;
    const int lowestExponent = exponentA + exponentB - 106 + trailingZeros;

    if (significantBits > 53)
        return false;
    if (lowestExponent < -1074)
        return false;
    if (lowestExponent + significantBits - 1 > 1023)
        return false;

    return true;
}

bool isNegativeConstant(const Expression& expression) noexcept
{
    return isConstant(expression) && constantValue(expression) < 0.0;
}

} // namespace

namespace detail {

bool addsExactly(double lhs, double rhs) noexcept
{
    const double sum = lhs + rhs;
    return std::isfinite(sum) && additionError(lhs, rhs) == 0.0;
}

bool multipliesExactly(double lhs, double rhs) noexcept
{
    return multipliesExactlyImpl(lhs, rhs);
}

} // namespace detail

int compareExpressions(const Expression& lhs, const Expression& rhs)
{
    if (sameNode(lhs, rhs))
        return 0;

    const int rankLhs = kindRank(lhs.type());
    const int rankRhs = kindRank(rhs.type());
    if (rankLhs != rankRhs)
        return rankLhs < rankRhs ? -1 : 1;

    return std::visit(
        [&rhs](const auto& node) -> int {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, ConstantNode>)
            {
                const double other = std::get<ConstantNode>(rhs.node().value).value;
                if (node.value == other) return 0;
                return node.value < other ? -1 : 1;
            }
            else if constexpr (std::is_same_v<Node, SymbolNode>)
            {
                const std::string& other = std::get<SymbolNode>(rhs.node().value).name;
                if (node.name == other) return 0;
                return node.name < other ? -1 : 1;
            }
            else if constexpr (requires { node.lhs; node.rhs; })
            {
                const auto& other = std::get<Node>(rhs.node().value);
                if (const int left = compareExpressions(node.lhs, other.lhs); left != 0)
                    return left;
                return compareExpressions(node.rhs, other.rhs);
            }
            else
            {
                const auto& other = std::get<Node>(rhs.node().value);
                return compareExpressions(node.operand, other.operand);
            }
        },
        lhs.node().value);
}

CanonicalizationSession::CanonicalizationSession(ExpressionFactory factory)
    : factory_(std::move(factory))
{
}

Expression CanonicalizationSession::canonicalize(const Expression& expression)
{
    return canonicalizeNode(expression);
}

Expression CanonicalizationSession::canonicalNegate(const Expression& expression)
{
    switch (expression.type())
    {
    case ExpressionType::Constant:
        return factory_.constant(-constantValue(expression));

    case ExpressionType::Negate:
        return std::get<NegateNode>(expression.node().value).operand;

    case ExpressionType::Add:
    {
        std::vector<Expression> terms;
        collectSum(expression, -1, terms);
        if (std::optional<Expression> negated = buildSum(std::move(terms)))
            return *negated;
        break;
    }

    case ExpressionType::Multiply:
    {
        // Route an outer negation through the same parity/zero handling as
        // an inner negative factor. Keep every non-constant factor: it may
        // carry a divisor even when the product has a literal zero factor.
        std::vector<Expression> factors;
        int parity = 1;
        collectProduct(expression, parity, factors);
        if (auto negated = buildProduct(std::move(factors), parity))
            return *negated;
        break;
    }

    case ExpressionType::Divide:
    {
        // A zero numerator makes the outer sign unobservable while the
        // quotient itself must stay to preserve the denominator guard/domain.
        const auto& node = std::get<DivideNode>(expression.node().value);
        if (isZero(node.lhs))
            return expression;
        break;
    }

    case ExpressionType::Symbol:
    case ExpressionType::Sin:
    case ExpressionType::Cos:
    case ExpressionType::Subtract:
        break;
    }

    return factory_.negate(expression);
}

void CanonicalizationSession::collectSum(const Expression& expression, int sign,
                                         std::vector<Expression>& terms)
{
    switch (expression.type())
    {
    case ExpressionType::Add:
    {
        const auto& node = std::get<AddNode>(expression.node().value);
        collectSum(node.lhs, sign, terms);
        collectSum(node.rhs, sign, terms);
        return;
    }
    case ExpressionType::Subtract:
    {
        const auto& node = std::get<SubtractNode>(expression.node().value);
        collectSum(node.lhs, sign, terms);
        collectSum(node.rhs, -sign, terms);
        return;
    }
    case ExpressionType::Negate:
    {
        const auto& node = std::get<NegateNode>(expression.node().value);
        collectSum(node.operand, -sign, terms);
        return;
    }
    default:
        break;
    }

    const Expression term = canonicalizeNode(expression);
    if (term.type() == ExpressionType::Add || term.type() == ExpressionType::Subtract
        || term.type() == ExpressionType::Negate)
    {
        collectSum(term, sign, terms);
        return;
    }

    terms.push_back(sign < 0 ? canonicalNegate(term) : term);
}

void CanonicalizationSession::collectProduct(const Expression& expression, int& parity,
                                             std::vector<Expression>& factors)
{
    switch (expression.type())
    {
    case ExpressionType::Multiply:
    {
        const auto& node = std::get<MultiplyNode>(expression.node().value);
        collectProduct(node.lhs, parity, factors);
        collectProduct(node.rhs, parity, factors);
        return;
    }
    case ExpressionType::Negate:
    {
        const auto& node = std::get<NegateNode>(expression.node().value);
        parity ^= 1;
        collectProduct(node.operand, parity, factors);
        return;
    }
    case ExpressionType::Constant:
    {
        const double value = constantValue(expression);
        if (value < 0.0)
        {
            parity ^= 1;
            factors.push_back(factory_.constant(-value));
            return;
        }
        factors.push_back(expression);
        return;
    }
    default:
        break;
    }

    const Expression factor = canonicalizeNode(expression);
    if (factor.type() == ExpressionType::Multiply || factor.type() == ExpressionType::Negate
        || factor.type() == ExpressionType::Constant)
    {
        collectProduct(factor, parity, factors);
        return;
    }

    factors.push_back(factor);
}

std::optional<Expression> CanonicalizationSession::buildSum(std::vector<Expression> terms)
{
    std::vector<double> constants;
    std::vector<Expression> others;
    others.reserve(terms.size());

    for (const Expression& term : terms)
    {
        if (isConstant(term))
            constants.push_back(constantValue(term));
        else
            others.push_back(term);
    }

    bool joined = true;
    while (joined && constants.size() > 1)
    {
        joined = false;
        std::sort(constants.begin(), constants.end());

        std::vector<double> reduced;
        reduced.push_back(constants.front());
        for (std::size_t index = 1; index < constants.size(); ++index)
        {
            if (detail::addsExactly(reduced.back(), constants[index]))
            {
                reduced.back() += constants[index];
                joined = true;
                continue;
            }
            reduced.push_back(constants[index]);
        }
        constants = std::move(reduced);
    }

    std::erase_if(constants, [](double value) { return value == 0.0; });
    std::sort(others.begin(), others.end(), [](const Expression& lhs, const Expression& rhs) {
        return compareExpressions(lhs, rhs) < 0;
    });

    if (others.empty())
    {
        if (constants.empty())
            return factory_.constant(0.0);
        if (constants.size() == 1)
            return factory_.constant(constants.front());
        return std::nullopt;
    }

    const auto balanced = [this](auto&& self, std::span<const Expression> range) -> Expression {
        if (range.size() == 1)
            return range.front();
        const std::size_t middle = range.size() / 2;
        return factory_.add(self(self, range.first(middle)), self(self, range.subspan(middle)));
    };

    Expression result = balanced(balanced, std::span<const Expression>(others));
    for (auto constant = constants.rbegin(); constant != constants.rend(); ++constant)
        result = factory_.add(factory_.constant(*constant), result);

    return result;
}

std::optional<Expression>
CanonicalizationSession::buildProduct(std::vector<Expression> factors, int parity)
{
    std::vector<double> constants;
    std::vector<Expression> others;
    others.reserve(factors.size());

    for (const Expression& factor : factors)
    {
        if (isConstant(factor))
            constants.push_back(constantValue(factor));
        else
            others.push_back(factor);
    }

    bool joined = true;
    while (joined && constants.size() > 1)
    {
        joined = false;
        std::sort(constants.begin(), constants.end());

        std::vector<double> reduced;
        reduced.push_back(constants.front());
        for (std::size_t index = 1; index < constants.size(); ++index)
        {
            if (detail::multipliesExactly(reduced.back(), constants[index]))
            {
                reduced.back() *= constants[index];
                joined = true;
                continue;
            }
            reduced.push_back(constants[index]);
        }
        constants = std::move(reduced);
    }

    std::erase_if(constants, [](double value) { return value == 1.0; });

    const bool hasZero = std::any_of(constants.begin(), constants.end(),
                                     [](double value) { return value == 0.0; });
    if (hasZero)
        parity = 0;

    std::sort(others.begin(), others.end(), [](const Expression& lhs, const Expression& rhs) {
        return compareExpressions(lhs, rhs) < 0;
    });

    Expression result = factory_.constant(1.0);

    if (hasZero && others.empty())
    {
        result = factory_.constant(0.0);
        constants.clear();
    }

    if (!others.empty())
    {
        const auto balanced = [this](auto&& self, std::span<const Expression> range) -> Expression {
            if (range.size() == 1)
                return range.front();
            const std::size_t middle = range.size() / 2;
            return factory_.multiply(self(self, range.first(middle)),
                                     self(self, range.subspan(middle)));
        };
        result = balanced(balanced, std::span<const Expression>(others));

        for (auto constant = constants.rbegin(); constant != constants.rend(); ++constant)
            result = factory_.multiply(factory_.constant(*constant), result);
    }
    else if (!constants.empty())
    {
        if (constants.size() > 1)
            return std::nullopt;
        result = factory_.constant(constants.front());
    }

    if (parity == 0)
        return result;

    // A factor may itself be a sum or a quotient. Those two need the sign
    // convention enforced by canonicalNegate (sum: push inward; quotient:
    // discard an unobservable sign for a zero numerator). A genuine product
    // is allowed to carry one outer Negate, and calling canonicalNegate on it
    // would recurse back into buildProduct.
    if (result.type() == ExpressionType::Add || result.type() == ExpressionType::Divide)
        return canonicalNegate(result);

    return factory_.negate(result);
}

Expression CanonicalizationSession::canonicalizeQuotient(const Expression& expression)
{
    const auto& node = std::get<DivideNode>(expression.node().value);

    int parity = 0;
    Expression numerator = canonicalizeNode(node.lhs);
    Expression denominator = canonicalizeNode(node.rhs);

    if (numerator.type() == ExpressionType::Negate)
    {
        parity ^= 1;
        numerator = std::get<NegateNode>(numerator.node().value).operand;
    }
    else if (isNegativeConstant(numerator))
    {
        parity ^= 1;
        numerator = factory_.constant(-constantValue(numerator));
    }

    if (denominator.type() == ExpressionType::Negate)
    {
        parity ^= 1;
        denominator = std::get<NegateNode>(denominator.node().value).operand;
    }
    else if (isNegativeConstant(denominator))
    {
        parity ^= 1;
        denominator = factory_.constant(-constantValue(denominator));
    }

    if (isZero(denominator))
        denominator = node.rhs;

    if (isZero(numerator))
        parity = 0;

    const Expression quotient = factory_.divide(numerator, denominator);
    return parity != 0 ? canonicalNegate(quotient) : quotient;
}

Expression CanonicalizationSession::canonicalizeNode(const Expression& expression)
{
    if (const auto cached = memo_.find(expression); cached != memo_.end())
        return cached->second;

    Expression result = expression;

    switch (expression.type())
    {
    case ExpressionType::Constant:
    case ExpressionType::Symbol:
        break;

    case ExpressionType::Add:
    case ExpressionType::Subtract:
    {
        std::vector<Expression> terms;
        collectSum(expression, 1, terms);
        result = buildSum(std::move(terms)).value_or(expression);
        break;
    }

    case ExpressionType::Multiply:
    {
        std::vector<Expression> factors;
        int parity = 0;
        collectProduct(expression, parity, factors);
        result = buildProduct(std::move(factors), parity).value_or(expression);
        break;
    }

    case ExpressionType::Divide:
        result = canonicalizeQuotient(expression);
        break;

    case ExpressionType::Negate:
        result = canonicalNegate(
            canonicalizeNode(std::get<NegateNode>(expression.node().value).operand));
        break;

    case ExpressionType::Sin:
    {
        const Expression operand =
            canonicalizeNode(std::get<SinNode>(expression.node().value).operand);
        result = factory_.sin(operand);
        break;
    }

    case ExpressionType::Cos:
    {
        const Expression operand =
            canonicalizeNode(std::get<CosNode>(expression.node().value).operand);
        result = factory_.cos(operand);
        break;
    }
    }

    if (!sameNode(result, expression) && structurallyEqual(result, expression))
        result = expression;

    memo_.emplace(expression, result);
    return result;
}

Expression canonicalize(const Expression& expression, const ExpressionFactory& factory)
{
    CanonicalizationSession session{factory};
    return session.canonicalize(expression);
}

} // namespace kinemaforge::ik
