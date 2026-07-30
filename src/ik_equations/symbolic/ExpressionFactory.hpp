#pragma once

#include "ik_equations/symbolic/Expression.hpp"

#include <string>

namespace kinemaforge::ik {

class ExpressionFactory
{
public:
    ExpressionPtr constant(double value) const;
    ExpressionPtr symbol(const std::string& name) const;

    ExpressionPtr add(const ExpressionPtr& lhs, const ExpressionPtr& rhs) const;
    ExpressionPtr subtract(const ExpressionPtr& lhs, const ExpressionPtr& rhs) const;
    ExpressionPtr multiply(const ExpressionPtr& lhs, const ExpressionPtr& rhs) const;
    ExpressionPtr divide(const ExpressionPtr& lhs, const ExpressionPtr& rhs) const;

    ExpressionPtr sin(const ExpressionPtr& operand) const;
    ExpressionPtr cos(const ExpressionPtr& operand) const;
    ExpressionPtr negate(const ExpressionPtr& operand) const;
};

} // namespace kinemaforge::ik
