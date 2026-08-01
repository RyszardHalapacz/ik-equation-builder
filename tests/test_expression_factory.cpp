#include <gtest/gtest.h>

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <cmath>
#include <numbers>

using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::ExpressionType;
using kinemaforge::ik::constantValue;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::sameNode;
using kinemaforge::ik::structurallyEqual;

TEST(ExpressionFactoryTest, BuildsBinaryOperations)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");
    const auto y = factory.symbol("y");

    EXPECT_EQ(factory.add(x, y).type(), ExpressionType::Add);
    EXPECT_EQ(factory.subtract(x, y).type(), ExpressionType::Subtract);
    EXPECT_EQ(factory.multiply(x, y).type(), ExpressionType::Multiply);
    EXPECT_EQ(factory.divide(x, y).type(), ExpressionType::Divide);
}

TEST(ExpressionFactoryTest, BuildsUnaryOperations)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    EXPECT_EQ(factory.negate(x).type(), ExpressionType::Negate);
    EXPECT_EQ(factory.sin(x).type(), ExpressionType::Sin);
    EXPECT_EQ(factory.cos(x).type(), ExpressionType::Cos);
}

TEST(ExpressionFactoryTest, FoldsConstantArithmetic)
{
    const ExpressionFactory factory;

    EXPECT_DOUBLE_EQ(constantValue(factory.add(factory.constant(2.0), factory.constant(3.0))), 5.0);
    EXPECT_DOUBLE_EQ(constantValue(factory.subtract(factory.constant(5.0), factory.constant(3.0))), 2.0);
    EXPECT_DOUBLE_EQ(constantValue(factory.multiply(factory.constant(3.0), factory.constant(4.0))), 12.0);
    EXPECT_DOUBLE_EQ(constantValue(factory.divide(factory.constant(6.0), factory.constant(3.0))), 2.0);
}

TEST(ExpressionFactoryTest, FoldsNegateOfConstant)
{
    const ExpressionFactory factory;
    EXPECT_DOUBLE_EQ(constantValue(factory.negate(factory.constant(2.0))), -2.0);
}

TEST(ExpressionFactoryTest, FoldsConstantTrig)
{
    const ExpressionFactory factory;

    EXPECT_TRUE(isZero(factory.sin(factory.constant(0.0))));
    EXPECT_TRUE(isOne(factory.cos(factory.constant(0.0))));
}

TEST(ExpressionFactoryTest, DoesNotSnapTinyConstantsToZero)
{
    const ExpressionFactory factory;
    const auto tiny = factory.constant(1e-300);

    // No tolerance anywhere in the factory: a tiny constant stays itself.
    // This is the canonicalization contract stated without involving libm
    // at all.
    EXPECT_FALSE(isZero(tiny));
    EXPECT_DOUBLE_EQ(constantValue(tiny), 1e-300);
}

TEST(ExpressionFactoryTest, TrigFoldingMatchesStdSinExactly)
{
    const ExpressionFactory factory;
    const auto sinePi = factory.sin(factory.constant(std::numbers::pi));

    // Folding must hand back exactly what std::sin produced, with no
    // canonicalization of near-zero results — a tolerance would become
    // part of the symbolic semantics, which is a separate, deferred
    // decision.
    //
    // The reference goes through volatile so it is computed by the same
    // runtime libm call the factory made. Writing std::sin(std::numbers::pi)
    // directly would be constant-folded by the compiler (correctly rounded
    // via MPFR) and disagree with libm in the last few bits: 1.2246468e-16
    // versus 1.2246064e-16 on this toolchain, at -O0 and -O2 alike.
    //
    // No specific residual is pinned, so this stays valid on any libm.
    volatile double argument = std::numbers::pi;
    const double expected = std::sin(argument);

    ASSERT_EQ(sinePi.type(), ExpressionType::Constant);
    EXPECT_DOUBLE_EQ(constantValue(sinePi), expected);
}

TEST(ExpressionFactoryTest, DropsAdditiveIdentity)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");
    const auto zero = factory.constant(0.0);

    EXPECT_TRUE(sameNode(factory.add(x, zero), x));
    EXPECT_TRUE(sameNode(factory.add(zero, x), x));
}

TEST(ExpressionFactoryTest, DropsSubtractiveIdentity)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    EXPECT_TRUE(sameNode(factory.subtract(x, factory.constant(0.0)), x));
}

TEST(ExpressionFactoryTest, DropsMultiplicativeIdentity)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");
    const auto one = factory.constant(1.0);

    EXPECT_TRUE(sameNode(factory.multiply(x, one), x));
    EXPECT_TRUE(sameNode(factory.multiply(one, x), x));
}

TEST(ExpressionFactoryTest, DropsDivisionByOne)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    EXPECT_TRUE(sameNode(factory.divide(x, factory.constant(1.0)), x));
}

TEST(ExpressionFactoryTest, KeepsSymbolicMultiplicationByZero)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    // x * 0 -> 0 is deliberately absent: it would erase the domain of x.
    // (1/q) * 0 is undefined at q = 0, but a folded 0 would claim
    // otherwise.
    const auto product = factory.multiply(x, factory.constant(0.0));
    EXPECT_EQ(product.type(), ExpressionType::Multiply);
}

TEST(ExpressionFactoryTest, FoldsConstantMultipliedByZero)
{
    const ExpressionFactory factory;

    // Both operands known and finite: nothing about a domain is lost.
    const auto product = factory.multiply(factory.constant(3.0), factory.constant(0.0));
    EXPECT_TRUE(isZero(product));
}

TEST(ExpressionFactoryTest, ConstantZeroSharesTheDefaultNode)
{
    const ExpressionFactory factory;
    EXPECT_TRUE(sameNode(factory.constant(0.0), Expression{}));
}

TEST(ExpressionFactoryTest, DoesNotReorderOperands)
{
    const ExpressionFactory factory;
    const auto forward = factory.subtract(factory.symbol("a"), factory.symbol("b"));
    const auto reversed = factory.subtract(factory.symbol("b"), factory.symbol("a"));

    EXPECT_FALSE(structurallyEqual(forward, reversed));
}

TEST(ExpressionFactoryTest, DoesNotSimplifyBeyondScope)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    // x - x -> 0 needs operand equivalence checking, which belongs to a
    // future EquationSimplifier.
    EXPECT_EQ(factory.subtract(x, x).type(), ExpressionType::Subtract);
}
