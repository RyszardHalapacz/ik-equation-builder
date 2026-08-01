#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <cassert>
#include <cmath>
#include <utility>

namespace kinemaforge::ik {

Expression ExpressionFactory::constant(double value) const
{
    assert(std::isfinite(value) && "constants must be finite");
    if (value == 0.0)
        return Expression{}; // shared zero node
    return Expression{ExpressionNode{ConstantNode{value}}};
}

Expression ExpressionFactory::symbol(std::string name) const
{
    assert(!name.empty() && "symbol names must not be empty");
    return Expression{ExpressionNode{SymbolNode{std::move(name)}}};
}

Expression ExpressionFactory::add(Expression lhs, Expression rhs) const
{
    if (isConstant(lhs) && isConstant(rhs))
        return constant(constantValue(lhs) + constantValue(rhs));
    if (isZero(rhs))
        return lhs;
    if (isZero(lhs))
        return rhs;
    return Expression{ExpressionNode{AddNode{std::move(lhs), std::move(rhs)}}};
}

Expression ExpressionFactory::subtract(Expression lhs, Expression rhs) const
{
    if (isConstant(lhs) && isConstant(rhs))
        return constant(constantValue(lhs) - constantValue(rhs));
    if (isZero(rhs))
        return lhs;
    return Expression{ExpressionNode{SubtractNode{std::move(lhs), std::move(rhs)}}};
}

Expression ExpressionFactory::multiply(Expression lhs, Expression rhs) const
{
    if (isConstant(lhs) && isConstant(rhs))
        return constant(constantValue(lhs) * constantValue(rhs));
    // No x * 0 -> 0 here: see the class comment.
    if (isOne(rhs))
        return lhs;
    if (isOne(lhs))
        return rhs;
    return Expression{ExpressionNode{MultiplyNode{std::move(lhs), std::move(rhs)}}};
}

Expression ExpressionFactory::divide(Expression lhs, Expression rhs) const
{
    assert(!isZero(rhs) && "division by literal zero");
    if (isConstant(lhs) && isConstant(rhs))
        return constant(constantValue(lhs) / constantValue(rhs));
    if (isOne(rhs))
        return lhs;
    return Expression{ExpressionNode{DivideNode{std::move(lhs), std::move(rhs)}}};
}

Expression ExpressionFactory::negate(Expression operand) const
{
    if (isConstant(operand))
        return constant(-constantValue(operand));
    return Expression{ExpressionNode{NegateNode{std::move(operand)}}};
}

Expression ExpressionFactory::sin(Expression operand) const
{
    if (isConstant(operand))
        return constant(std::sin(constantValue(operand)));
    return Expression{ExpressionNode{SinNode{std::move(operand)}}};
}

Expression ExpressionFactory::cos(Expression operand) const
{
    if (isConstant(operand))
        return constant(std::cos(constantValue(operand)));
    return Expression{ExpressionNode{CosNode{std::move(operand)}}};
}

} // namespace kinemaforge::ik
