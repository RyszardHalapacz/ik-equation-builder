#pragma once

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionCanonicalizer.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/ExpressionIdentityMap.hpp"
#include "ik_equations/symbolic/RewriteRule.hpp"

#include <vector>

namespace kinemaforge::ik {

struct TrigonometricRules
{
    [[nodiscard]] static RewriteRule cosineEven();
};

// A short-lived session that adds a deliberately small trigonometric layer on
// top of the F3.2 AC-normal form. It is not a general CAS.
//
// The Pythagorean rule works on sum terms of the shape
//
//     sign * (other factors) * t(u) * t(u)        t in {sin, cos}
//
// Two terms that differ ONLY in t collapse to  sign * (other factors).  So
// a - (sin^2 u + cos^2 u), 2 sin^2 u + 2 cos^2 u and x sin^2 u + x cos^2 u all
// reduce; 2 sin^2 u + 3 cos^2 u does not, because no coefficient arithmetic is
// performed and no new rounded constant is ever introduced.
//
// Contract:
//   * simplify() returns a fixed point: simplify(simplify(e)) is structurally
//     equal to simplify(e).
//   * The set of divisors is preserved. The identity never fires when u itself
//     contains a divisor, and the other factors stay in the result.
//   * The guarantee is ALGEBRAIC, exactly like canonicalisation's: evaluating
//     the result in double may round differently from the input, including
//     whether a divisor comes out as exactly zero.
class TrigonometricSimplificationSession
{
public:
    explicit TrigonometricSimplificationSession(ExpressionFactory factory);

    TrigonometricSimplificationSession(const TrigonometricSimplificationSession&) = delete;
    TrigonometricSimplificationSession& operator=(const TrigonometricSimplificationSession&) = delete;
    TrigonometricSimplificationSession(TrigonometricSimplificationSession&&) = default;
    TrigonometricSimplificationSession& operator=(TrigonometricSimplificationSession&&) = default;

    [[nodiscard]] Expression simplify(const Expression& expression);

private:
    [[nodiscard]] Expression simplifyNode(const Expression& expression);
    [[nodiscard]] Expression simplifySum(const Expression& expression);
    [[nodiscard]] bool reducePythagoreanPairs(std::vector<Expression>& terms);
    [[nodiscard]] Expression rebuildSum(const std::vector<Expression>& terms,
                                        const Expression& fallback);

    void collectAddTerms(const Expression& expression, std::vector<Expression>& terms);

    ExpressionFactory factory_;
    CanonicalizationSession canonicalizer_;
    detail::ExpressionMap<Expression> memo_;
};

[[nodiscard]] Expression simplifyTrigonometry(const Expression& expression,
                                               const ExpressionFactory& factory);

} // namespace kinemaforge::ik
