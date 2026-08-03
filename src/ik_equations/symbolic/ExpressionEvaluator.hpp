#pragma once

#include "ik_equations/symbolic/Expression.hpp"

#include <cstddef>
#include <expected>
#include <functional>
#include <string>
#include <unordered_map>

namespace kinemaforge::ik {

using SymbolValues = std::unordered_map<std::string, double>;

enum class EvaluationErrorCode
{
    MissingSymbol,
    NonFiniteSymbolValue,
    DivisionByZero,
    NonFiniteResult
};

struct EvaluationError
{
    EvaluationErrorCode code{};

    // Set only for the two symbol errors. Empty otherwise, which is
    // unambiguous: ExpressionFactory rejects empty symbol names.
    std::string symbolName;
};

struct EvaluationStatistics
{
    // Counts the outcome of the cache lookup, not the outcome of the
    // evaluation: a node whose evaluation then fails still counts as a miss.
    std::size_t cacheMisses{};
    std::size_t cacheHits{};
};

// Evaluates an expression DAG numerically under one fixed set of symbol
// values.
//
// One instance is one substitution. All sixteen cells of a forward-kinematics
// transform are meant to be evaluated with the same instance, so they share
// the cache; a different joint configuration needs a new instance. Binding the
// values to the object's lifetime makes it impossible to reuse a cache that
// was filled for different values.
//
// Memoization is mandatory rather than an optimization. A composed FK
// transform measures 281 unique nodes but 21 882 counted with multiplicity for
// kr640.urdf, and 516 vs 153 703 for kr4_r600.urdf.
//
// The cache is keyed by Expression, not by a raw node pointer, so an entry
// keeps its node alive. evaluate() takes a reference that may bind to a
// temporary; with a raw pointer key the node could die, its address be reused
// by the next allocation, and the cache then answer for the wrong node.
//
// Lifetime contract: this is a short-lived session for one symbol binding and
// one related expression DAG. Successfully evaluated nodes stay cached — and
// alive — until the evaluator is destroyed, so the cache grows monotonically.
// Do not use one instance as a long-lived global interpreter.
//
// Not thread-safe: the cache is mutable. The DAG itself is immutable, so
// several evaluators may read the same tree concurrently.
class ExpressionEvaluator
{
public:
    explicit ExpressionEvaluator(SymbolValues values);

    // A session is not a value. Copying would silently duplicate the cache
    // and the statistics, and it is not clear whether that means "snapshot"
    // or "second session" — so it is not offered.
    ExpressionEvaluator(const ExpressionEvaluator&) = delete;
    ExpressionEvaluator& operator=(const ExpressionEvaluator&) = delete;

    // No explicit noexcept: std::unordered_map's move constructor is only
    // conditionally noexcept, so letting the compiler derive the specifier
    // from the members states the truth instead of asserting a stronger
    // guarantee that would turn a throwing move into std::terminate.
    ExpressionEvaluator(ExpressionEvaluator&&) = default;
    ExpressionEvaluator& operator=(ExpressionEvaluator&&) = default;

    // Evaluates both operands of every binary node; nothing is skipped based
    // on the value of the other operand. In particular a zero factor does not
    // short-circuit a Multiply -- that is what keeps (1/q) * 0 reporting
    // DivisionByZero at q = 0 instead of quietly yielding 0, which is the
    // whole reason ExpressionFactory has no x * 0 -> 0 rule.
    //
    // Evaluation does stop at the first error, and operands are visited left
    // to right, so the reported error is deterministic.
    [[nodiscard]] std::expected<double, EvaluationError>
    evaluate(const Expression& expression);

    EvaluationStatistics statistics() const noexcept;

private:
    // Both types must be COMPLETE here: declaring memo_ instantiates
    // unordered_map, which stores Hash and KeyEqual as subobjects. A forward
    // declaration would make the member declaration ill-formed.
    struct ExpressionIdentityHash
    {
        std::size_t operator()(const Expression& expression) const noexcept
        {
            return std::hash<const ExpressionNode*>{}(&expression.node());
        }
    };

    struct ExpressionIdentityEqual
    {
        bool operator()(const Expression& lhs, const Expression& rhs) const noexcept
        {
            return sameNode(lhs, rhs);
        }
    };

    using Memo = std::unordered_map<Expression, double,
                                    ExpressionIdentityHash, ExpressionIdentityEqual>;

    std::expected<double, EvaluationError> evaluateNode(const Expression& expression);

    SymbolValues values_;
    Memo memo_;
    EvaluationStatistics statistics_{};
};

} // namespace kinemaforge::ik
