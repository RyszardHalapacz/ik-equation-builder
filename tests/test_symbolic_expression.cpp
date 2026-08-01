#include <gtest/gtest.h>

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <variant>

using kinemaforge::ik::AddNode;
using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::ExpressionType;
using kinemaforge::ik::MultiplyNode;
using kinemaforge::ik::SinNode;
using kinemaforge::ik::SymbolNode;
using kinemaforge::ik::constantValue;
using kinemaforge::ik::isConstant;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::sameNode;
using kinemaforge::ik::structurallyEqual;

TEST(SymbolicExpressionTest, DefaultConstructedIsConstantZero)
{
    const Expression expression;
    EXPECT_EQ(expression.type(), ExpressionType::Constant);
    EXPECT_TRUE(isConstant(expression));
    EXPECT_TRUE(isZero(expression));
    EXPECT_DOUBLE_EQ(constantValue(expression), 0.0);
}

TEST(SymbolicExpressionTest, DefaultConstructedInstancesShareOneNode)
{
    EXPECT_TRUE(sameNode(Expression{}, Expression{}));
}

TEST(SymbolicExpressionTest, CreatesConstant)
{
    const ExpressionFactory factory;
    const auto expression = factory.constant(2.5);

    EXPECT_EQ(expression.type(), ExpressionType::Constant);
    EXPECT_DOUBLE_EQ(constantValue(expression), 2.5);
    EXPECT_FALSE(isZero(expression));
    EXPECT_FALSE(isOne(expression));
}

TEST(SymbolicExpressionTest, CreatesSymbol)
{
    const ExpressionFactory factory;
    const auto expression = factory.symbol("q1");

    EXPECT_EQ(expression.type(), ExpressionType::Symbol);
    EXPECT_FALSE(isConstant(expression));
    EXPECT_EQ(std::get<SymbolNode>(expression.node().value).name, "q1");
}

TEST(SymbolicExpressionTest, CopyingSharesNodeInsteadOfCloning)
{
    const ExpressionFactory factory;
    const auto original = factory.symbol("q1");
    const auto copy = original;

    EXPECT_TRUE(sameNode(original, copy));
}

TEST(SymbolicExpressionTest, CopiedExpressionOutlivesSource)
{
    const ExpressionFactory factory;

    const auto outer = [&factory] {
        const auto inner = factory.symbol("q9");
        return factory.sin(inner); // inner goes out of scope here
    }();

    ASSERT_EQ(outer.type(), ExpressionType::Sin);
    const auto& operand = std::get<SinNode>(outer.node().value).operand;
    EXPECT_EQ(std::get<SymbolNode>(operand.node().value).name, "q9");
}

TEST(SymbolicExpressionTest, BinaryNodeExposesBothOperands)
{
    const ExpressionFactory factory;
    const auto lhs = factory.symbol("x");
    const auto rhs = factory.symbol("y");
    const auto sum = factory.add(lhs, rhs);

    ASSERT_EQ(sum.type(), ExpressionType::Add);
    const auto& node = std::get<AddNode>(sum.node().value);
    EXPECT_TRUE(sameNode(node.lhs, lhs));
    EXPECT_TRUE(sameNode(node.rhs, rhs));
}

TEST(SymbolicExpressionTest, UnaryNodeExposesOperand)
{
    const ExpressionFactory factory;
    const auto operand = factory.symbol("q1");
    const auto sine = factory.sin(operand);

    ASSERT_EQ(sine.type(), ExpressionType::Sin);
    EXPECT_TRUE(sameNode(std::get<SinNode>(sine.node().value).operand, operand));
}

TEST(SymbolicExpressionTest, RepeatedOperandIsSharedNotDuplicated)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");
    const auto product = factory.multiply(x, x);

    ASSERT_EQ(product.type(), ExpressionType::Multiply);
    const auto& node = std::get<MultiplyNode>(product.node().value);
    EXPECT_TRUE(sameNode(node.lhs, node.rhs));
}

TEST(SymbolicExpressionTest, StructurallyEqualMatchesIdenticalTrees)
{
    const ExpressionFactory factory;
    const auto left = factory.add(factory.symbol("x"), factory.symbol("y"));
    const auto right = factory.add(factory.symbol("x"), factory.symbol("y"));

    EXPECT_TRUE(structurallyEqual(left, right));
    EXPECT_FALSE(sameNode(left, right)); // independently built
}

TEST(SymbolicExpressionTest, StructurallyEqualRejectsDifferentShape)
{
    const ExpressionFactory factory;
    const auto sum = factory.add(factory.symbol("x"), factory.symbol("y"));
    const auto product = factory.multiply(factory.symbol("x"), factory.symbol("y"));

    EXPECT_FALSE(structurallyEqual(sum, product));
}

TEST(SymbolicExpressionTest, StructurallyEqualIsNotAlgebraic)
{
    const ExpressionFactory factory;
    const auto left = factory.add(factory.symbol("x"), factory.symbol("y"));
    const auto swapped = factory.add(factory.symbol("y"), factory.symbol("x"));

    // Equal as mathematics, different as trees. This is the documented
    // limit of the predicate, not a defect.
    EXPECT_FALSE(structurallyEqual(left, swapped));
}

TEST(SymbolicExpressionTest, StructurallyEqualIsNotNumeric)
{
    const ExpressionFactory factory;
    EXPECT_FALSE(structurallyEqual(factory.constant(0.1 + 0.2), factory.constant(0.3)));
}

TEST(SymbolicExpressionTest, SameNodeIsStrongerThanStructuralEquality)
{
    const ExpressionFactory factory;
    const auto left = factory.constant(1.0);
    const auto right = factory.constant(1.0);

    EXPECT_TRUE(structurallyEqual(left, right));
    EXPECT_FALSE(sameNode(left, right)); // no interning for non-zero constants
}
