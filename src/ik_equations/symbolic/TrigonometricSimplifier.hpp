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
