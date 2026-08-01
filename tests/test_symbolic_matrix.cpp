#include <gtest/gtest.h>

#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicMatrix.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <variant>

using kinemaforge::ik::AddNode;
using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::ExpressionType;
using kinemaforge::ik::MultiplyNode;
using kinemaforge::ik::SymbolicMatrix;
using kinemaforge::ik::SymbolicTransform;
using kinemaforge::ik::constantValue;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::multiply;
using kinemaforge::ik::sameNode;

TEST(SymbolicMatrixTest, DefaultMatrixCellsAreZero)
{
    const SymbolicMatrix<4, 4> matrix;

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
            EXPECT_TRUE(isZero(matrix(row, column)));
}

TEST(SymbolicMatrixTest, DefaultMatrixCellsShareOneNode)
{
    const SymbolicMatrix<4, 4> matrix;
    EXPECT_TRUE(sameNode(matrix(0, 0), matrix(2, 3)));
}

TEST(SymbolicMatrixTest, SubscriptReadsAndWritesCell)
{
    const ExpressionFactory factory;
    SymbolicMatrix<4, 4> matrix;
    const auto value = factory.symbol("s");

    matrix(1, 2) = value;
    EXPECT_TRUE(sameNode(matrix(1, 2), value));
}

TEST(SymbolicMatrixTest, SubscriptIsRowMajor)
{
    const ExpressionFactory factory;
    SymbolicMatrix<4, 4> matrix;

    matrix(0, 1) = factory.symbol("s");
    EXPECT_TRUE(isZero(matrix(1, 0)));
}

TEST(SymbolicMatrixTest, ZerosMatchesDefaultConstruction)
{
    const auto matrix = SymbolicMatrix<3, 3>::zeros();

    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            EXPECT_TRUE(isZero(matrix(row, column)));
}

TEST(SymbolicMatrixTest, IdentityHasOnesOnDiagonal)
{
    const auto matrix = SymbolicMatrix<4, 4>::identity();

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            if (row == column)
                EXPECT_TRUE(isOne(matrix(row, column)));
            else
                EXPECT_TRUE(isZero(matrix(row, column)));
        }
}

TEST(SymbolicMatrixTest, IdentityDiagonalSharesOneNode)
{
    const auto matrix = SymbolicMatrix<4, 4>::identity();
    EXPECT_TRUE(sameNode(matrix(0, 0), matrix(3, 3)));
}

TEST(SymbolicMatrixTest, MultiplySmallMatrices)
{
    const ExpressionFactory factory;
    SymbolicMatrix<2, 2> lhs;
    SymbolicMatrix<2, 2> rhs;

    lhs(0, 0) = factory.symbol("a"); lhs(0, 1) = factory.symbol("b");
    lhs(1, 0) = factory.symbol("c"); lhs(1, 1) = factory.symbol("d");
    rhs(0, 0) = factory.symbol("e"); rhs(0, 1) = factory.symbol("f");
    rhs(1, 0) = factory.symbol("g"); rhs(1, 1) = factory.symbol("h");

    const auto product = multiply(lhs, rhs, factory);

    // Expect (a*e) + (b*g): left-folded, left operand from lhs.
    ASSERT_EQ(product(0, 0).type(), ExpressionType::Add);
    const auto& sum = std::get<AddNode>(product(0, 0).node().value);

    ASSERT_EQ(sum.lhs.type(), ExpressionType::Multiply);
    const auto& first = std::get<MultiplyNode>(sum.lhs.node().value);
    EXPECT_TRUE(sameNode(first.lhs, lhs(0, 0)));
    EXPECT_TRUE(sameNode(first.rhs, rhs(0, 0)));

    ASSERT_EQ(sum.rhs.type(), ExpressionType::Multiply);
    const auto& second = std::get<MultiplyNode>(sum.rhs.node().value);
    EXPECT_TRUE(sameNode(second.lhs, lhs(0, 1)));
    EXPECT_TRUE(sameNode(second.rhs, rhs(1, 0)));
}

TEST(SymbolicMatrixTest, MultiplyProducesLeftFoldedSum)
{
    const ExpressionFactory factory;
    SymbolicMatrix<1, 3> lhs;
    SymbolicMatrix<3, 1> rhs;

    lhs(0, 0) = factory.symbol("a"); lhs(0, 1) = factory.symbol("b"); lhs(0, 2) = factory.symbol("c");
    rhs(0, 0) = factory.symbol("d"); rhs(1, 0) = factory.symbol("e"); rhs(2, 0) = factory.symbol("f");

    const auto product = multiply(lhs, rhs, factory);

    // Expect ((a*d + b*e) + c*f): the outer Add holds the third term.
    ASSERT_EQ(product(0, 0).type(), ExpressionType::Add);
    const auto& outer = std::get<AddNode>(product(0, 0).node().value);
    EXPECT_EQ(outer.lhs.type(), ExpressionType::Add);
    EXPECT_EQ(outer.rhs.type(), ExpressionType::Multiply);
}

TEST(SymbolicMatrixTest, MultiplyConstantMatrixByIdentityReturnsSameValues)
{
    const ExpressionFactory factory;
    SymbolicMatrix<2, 2> constants;
    constants(0, 0) = factory.constant(1.0); constants(0, 1) = factory.constant(2.0);
    constants(1, 0) = factory.constant(3.0); constants(1, 1) = factory.constant(4.0);

    const auto product = multiply(constants, SymbolicMatrix<2, 2>::identity(), factory);

    // Constant folding collapses each c*0 to 0, after which x+0 -> x
    // fires, so the original values come back.
    EXPECT_DOUBLE_EQ(constantValue(product(0, 0)), 1.0);
    EXPECT_DOUBLE_EQ(constantValue(product(0, 1)), 2.0);
    EXPECT_DOUBLE_EQ(constantValue(product(1, 0)), 3.0);
    EXPECT_DOUBLE_EQ(constantValue(product(1, 1)), 4.0);
}

TEST(SymbolicMatrixTest, MultiplySymbolicMatrixByIdentityKeepsZeroProducts)
{
    const ExpressionFactory factory;
    SymbolicMatrix<2, 2> symbols;
    symbols(0, 0) = factory.symbol("a00"); symbols(0, 1) = factory.symbol("a01");
    symbols(1, 0) = factory.symbol("a10"); symbols(1, 1) = factory.symbol("a11");

    const auto product = multiply(symbols, SymbolicMatrix<2, 2>::identity(), factory);

    // With x * 0 -> 0 deliberately absent, A * I is (a00 + (a01 * 0)),
    // not a00. Recovering A needs a simplifier that tracks domains.
    ASSERT_EQ(product(0, 0).type(), ExpressionType::Add);
    EXPECT_FALSE(sameNode(product(0, 0), symbols(0, 0)));

    const auto& sum = std::get<AddNode>(product(0, 0).node().value);
    EXPECT_TRUE(sameNode(sum.lhs, symbols(0, 0)));
    ASSERT_EQ(sum.rhs.type(), ExpressionType::Multiply);
    const auto& zeroProduct = std::get<MultiplyNode>(sum.rhs.node().value);
    EXPECT_TRUE(sameNode(zeroProduct.lhs, symbols(0, 1)));
    EXPECT_TRUE(isZero(zeroProduct.rhs));
}

TEST(SymbolicMatrixTest, MultiplyOfConstantMatricesFoldsToConstants)
{
    const ExpressionFactory factory;
    SymbolicMatrix<2, 2> lhs;
    SymbolicMatrix<2, 2> rhs;

    lhs(0, 0) = factory.constant(1.0); lhs(0, 1) = factory.constant(2.0);
    lhs(1, 0) = factory.constant(3.0); lhs(1, 1) = factory.constant(4.0);
    rhs(0, 0) = factory.constant(5.0); rhs(0, 1) = factory.constant(6.0);
    rhs(1, 0) = factory.constant(7.0); rhs(1, 1) = factory.constant(8.0);

    const auto product = multiply(lhs, rhs, factory);

    EXPECT_DOUBLE_EQ(constantValue(product(0, 0)), 1.0 * 5.0 + 2.0 * 7.0);
    EXPECT_DOUBLE_EQ(constantValue(product(1, 1)), 3.0 * 6.0 + 4.0 * 8.0);
}

TEST(SymbolicMatrixTest, MultiplyFourByFourTransforms)
{
    const ExpressionFactory factory;
    const auto left = SymbolicTransform::identity();
    auto right = SymbolicTransform::identity();
    const auto tx = factory.symbol("tx");
    right(0, 3) = tx;

    const auto product = multiply(left, right, factory);

    EXPECT_EQ(SymbolicTransform::rows, 4u);
    EXPECT_EQ(SymbolicTransform::columns, 4u);

    // Cells whose terms are all constant fold away completely.
    EXPECT_TRUE(isOne(product(0, 0)));
    EXPECT_TRUE(isZero(product(1, 0)));

    // The symbol reaches (0,3) unchanged: 1*tx folds by the identity rule
    // and the remaining 0*0 terms fold to constant 0, which x+0 -> x drops.
    EXPECT_TRUE(sameNode(product(0, 3), tx));

    // (3,3) is where the missing annihilator shows: the 0*tx term cannot
    // fold (tx is symbolic), so the cell is Add(Multiply(0, tx), 1) rather
    // than the constant 1 a full simplifier would produce.
    ASSERT_EQ(product(3, 3).type(), ExpressionType::Add);
    const auto& sum = std::get<AddNode>(product(3, 3).node().value);

    ASSERT_EQ(sum.lhs.type(), ExpressionType::Multiply);
    const auto& zeroProduct = std::get<MultiplyNode>(sum.lhs.node().value);
    EXPECT_TRUE(isZero(zeroProduct.lhs));
    EXPECT_TRUE(sameNode(zeroProduct.rhs, tx));

    EXPECT_TRUE(isOne(sum.rhs));
}
