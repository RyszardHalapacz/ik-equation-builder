#pragma once

#include <memory>
#include <string>
#include <variant>

namespace kinemaforge::ik {

enum class ExpressionType
{
    Constant,
    Symbol,
    Add,
    Subtract,
    Multiply,
    Divide,
    Sin,
    Cos,
    Negate
};

class Expression;
using ExpressionPtr = std::shared_ptr<const Expression>;

struct ConstantExpression
{
    double value{};
};

struct SymbolExpression
{
    std::string name;
};

struct BinaryExpression
{
    ExpressionPtr left;
    ExpressionPtr right;
};

struct UnaryExpression
{
    ExpressionPtr operand;
};

struct AddExpression : BinaryExpression {};
struct SubtractExpression : BinaryExpression {};
struct MultiplyExpression : BinaryExpression {};
struct DivideExpression : BinaryExpression {};

struct SinExpression : UnaryExpression {};
struct CosExpression : UnaryExpression {};
struct NegateExpression : UnaryExpression {};

using ExpressionNode = std::variant<
    ConstantExpression,
    SymbolExpression,
    AddExpression,
    SubtractExpression,
    MultiplyExpression,
    DivideExpression,
    SinExpression,
    CosExpression,
    NegateExpression
>;

class Expression
{
public:
    // Defaults to the constant 0 — lets SymbolicMatrix default-construct
    // its cells without needing a placeholder "empty" expression state.
    Expression() : node_(ConstantExpression{0.0}) {}
    explicit Expression(ExpressionNode node);

    ExpressionType type() const;
    const ExpressionNode& node() const;

private:
    ExpressionNode node_;
};

} // namespace kinemaforge::ik
