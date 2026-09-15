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
