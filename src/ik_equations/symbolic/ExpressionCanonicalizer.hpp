#pragma once

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/ExpressionIdentityMap.hpp"

#include <optional>
#include <vector>

namespace kinemaforge::ik {

namespace detail {

[[nodiscard]] bool addsExactly(double lhs, double rhs) noexcept;
[[nodiscard]] bool multipliesExactly(double lhs, double rhs) noexcept;

} // namespace detail

[[nodiscard]] int compareExpressions(const Expression& lhs, const Expression& rhs);

// What the canonical form promises, and what it does not.
//
// Promised:
//   * commutativity, associativity and sign placement do not affect the
//     result: x+y and y+x, x-y and x+(-y), -(x*y) and (-x)*y come out equal;
//   * canonicalisation is idempotent;
//   * the set of divisors is preserved. An expression whose divisor is
//     algebraically zero never becomes an expression without that divisor.
//
// NOT promised:
//   * Recovering constants ExpressionFactory already rounded. The factory folds
//     two adjacent constants immediately, with rounding; the canonicaliser
//     merges constants only when the result is exact. (x + 1.91) + 0.1 and
//     x + (1.91 + 0.1) therefore have different canonical forms: the form is a
//     function of the expression as built.
//   * The same ExpressionEvaluator outcome in double. Terms are reordered, so
//     rounding changes -- including whether a divisor evaluates to exactly
//     zero. DivisionByZero before can be a large finite value after, and the
//     reverse. The guarantee is algebraic, not bitwise; do not detect
//     singularities by evaluating a canonical form.

class CanonicalizationSession
{
public:
    explicit CanonicalizationSession(ExpressionFactory factory);

    CanonicalizationSession(const CanonicalizationSession&) = delete;
    CanonicalizationSession& operator=(const CanonicalizationSession&) = delete;
    CanonicalizationSession(CanonicalizationSession&&) = default;
    CanonicalizationSession& operator=(CanonicalizationSession&&) = default;

    [[nodiscard]] Expression canonicalize(const Expression& expression);

private:
    [[nodiscard]] Expression canonicalNegate(const Expression& expression);
    [[nodiscard]] Expression canonicalizeNode(const Expression& expression);

    void collectSum(const Expression& expression, int sign, std::vector<Expression>& terms);
    void collectProduct(const Expression& expression, int& parity,
                        std::vector<Expression>& factors);

    [[nodiscard]] std::optional<Expression> buildSum(std::vector<Expression> terms);
    [[nodiscard]] std::optional<Expression> buildProduct(std::vector<Expression> factors,
                                                         int parity);
    [[nodiscard]] Expression canonicalizeQuotient(const Expression& expression);

    ExpressionFactory factory_;
    detail::ExpressionMap<Expression> memo_;
};

[[nodiscard]] Expression canonicalize(const Expression& expression,
                                      const ExpressionFactory& factory);

} // namespace kinemaforge::ik
