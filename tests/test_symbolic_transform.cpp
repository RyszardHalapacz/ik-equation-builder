#include <gtest/gtest.h>

#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <cstddef>

using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::SymbolicRotation;
using kinemaforge::ik::SymbolicTransform;
using kinemaforge::ik::SymbolicVector3;
using kinemaforge::ik::hasCanonicalHomogeneousLastRow;
using kinemaforge::ik::isIdentityTransform;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::multiplyTransforms;
using kinemaforge::ik::structurallyEqual;

namespace {

SymbolicRotation rotationAboutZ(const Expression& angle, const ExpressionFactory& factory)
{
    auto rotation = SymbolicRotation::identity();
    rotation(0, 0) = factory.cos(angle);
    rotation(0, 1) = factory.negate(factory.sin(angle));
    rotation(1, 0) = factory.sin(angle);
    rotation(1, 1) = factory.cos(angle);
    return rotation;
}

SymbolicRotation rotationAboutX(const Expression& angle, const ExpressionFactory& factory)
{
    auto rotation = SymbolicRotation::identity();
    rotation(1, 1) = factory.cos(angle);
    rotation(1, 2) = factory.negate(factory.sin(angle));
    rotation(2, 1) = factory.sin(angle);
    rotation(2, 2) = factory.cos(angle);
    return rotation;
}

SymbolicVector3 vector3(double x, double y, double z, const ExpressionFactory& factory)
{
    SymbolicVector3 vector;
    vector(0, 0) = factory.constant(x);
    vector(1, 0) = factory.constant(y);
    vector(2, 0) = factory.constant(z);
    return vector;
}

SymbolicTransform makeTransform(const SymbolicRotation& rotation,
                                const SymbolicVector3& translation)
{
    auto transform = SymbolicTransform::identity();
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
            transform(row, column) = rotation(row, column);
        transform(row, 3) = translation(row, 0);
    }
    return transform;
}

void expectCellsEqual(const SymbolicTransform& actual, const SymbolicTransform& expected)
{
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            EXPECT_TRUE(structurallyEqual(actual(row, column), expected(row, column)));
        }
}

} // namespace

// --- predicates -----------------------------------------------------

TEST(SymbolicTransformTest, RecognizesCanonicalHomogeneousLastRow)
{
    const ExpressionFactory factory;
    EXPECT_TRUE(hasCanonicalHomogeneousLastRow(SymbolicTransform::identity()));

    const auto general = makeTransform(rotationAboutZ(factory.symbol("q1"), factory),
                                       vector3(1.0, 2.0, 3.0, factory));
    EXPECT_TRUE(hasCanonicalHomogeneousLastRow(general));
}

TEST(SymbolicTransformTest, RejectsMalformedHomogeneousLastRow)
{
    // Each of the four cells perturbed separately: a predicate that always
    // returned true would pass every other test in this file.
    const ExpressionFactory factory;

    for (std::size_t column = 0; column < 3; ++column)
    {
        SCOPED_TRACE(testing::Message() << "perturbed (3, " << column << ")");
        auto transform = SymbolicTransform::identity();
        transform(3, column) = factory.constant(7.0);
        EXPECT_FALSE(hasCanonicalHomogeneousLastRow(transform));
    }
    {
        SCOPED_TRACE("perturbed (3, 3)");
        auto transform = SymbolicTransform::identity();
        transform(3, 3) = factory.constant(0.0);
        EXPECT_FALSE(hasCanonicalHomogeneousLastRow(transform));
    }
}

TEST(SymbolicTransformTest, RecognizesIdentityTransform)
{
    EXPECT_TRUE(isIdentityTransform(SymbolicTransform::identity()));
}

TEST(SymbolicTransformTest, RejectsNonIdentityTransform)
{
    // All sixteen cells, including the last row: isIdentityTransform promises
    // a statement about the whole matrix.
    const ExpressionFactory factory;

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message() << "perturbed (" << row << ", " << column << ")");
            auto transform = SymbolicTransform::identity();
            transform(row, column) = factory.constant(7.0);
            EXPECT_FALSE(isIdentityTransform(transform));
        }
}

// --- neutral elements -----------------------------------------------

TEST(SymbolicTransformTest, LeftIdentityReturnsRightStructurally)
{
    const ExpressionFactory factory;
    const auto transform = makeTransform(rotationAboutZ(factory.symbol("q1"), factory),
                                         vector3(0.35, 0.0, 0.0, factory));

    expectCellsEqual(multiplyTransforms(SymbolicTransform::identity(), transform, factory),
                     transform);
}

TEST(SymbolicTransformTest, RightIdentityReturnsLeftStructurally)
{
    // Without the fast path this fails in 8 of 16 cells: R * I leaves
    // Multiply(cell, Constant(0)) terms behind, since the factory has no
    // x * 0 -> 0 rule.
    const ExpressionFactory factory;
    const auto transform = makeTransform(rotationAboutZ(factory.symbol("q1"), factory),
                                         vector3(0.35, 0.0, 0.0, factory));

    expectCellsEqual(multiplyTransforms(transform, SymbolicTransform::identity(), factory),
                     transform);
}

TEST(SymbolicTransformTest, PreservesLeftRotationWhenRightRotationIsIdentity)
{
    // A pure-translation fixed joint on the right — the common case for a
    // tool offset. Without the fast path, 6 of the 9 rotation cells change.
    const ExpressionFactory factory;
    const auto rotation = rotationAboutZ(factory.symbol("q1"), factory);
    const auto lhs = makeTransform(rotation, vector3(0.35, 0.0, 0.0, factory));
    const auto rhs = makeTransform(SymbolicRotation::identity(),
                                   vector3(0.0, 0.29, 0.0, factory));

    const auto product = multiplyTransforms(lhs, rhs, factory);

    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            EXPECT_TRUE(structurallyEqual(product(row, column), rotation(row, column)));
        }
}

TEST(SymbolicTransformTest, PreservesLeftTranslationWhenRightTranslationIsZero)
{
    // A pure-rotation joint on the right — the basic case for an industrial
    // robot. p = p_lhs + R_lhs * 0 must stay exactly p_lhs.
    const ExpressionFactory factory;
    const auto translation = vector3(0.35, 0.1, 0.0, factory);
    const auto lhs = makeTransform(rotationAboutZ(factory.symbol("q1"), factory), translation);
    const auto rhs = makeTransform(rotationAboutX(factory.symbol("q2"), factory),
                                   vector3(0.0, 0.0, 0.0, factory));

    const auto product = multiplyTransforms(lhs, rhs, factory);

    for (std::size_t row = 0; row < 3; ++row)
    {
        SCOPED_TRACE(testing::Message() << "row " << row);
        EXPECT_TRUE(structurallyEqual(product(row, 3), translation(row, 0)));
    }
}

// --- the general case -----------------------------------------------

TEST(SymbolicTransformTest, MultipliesHomogeneousTransformsStructurally)
{
    // lhs = Rz(q1) with p = [a, 0, 0] ; rhs = Rx(q2) with p = [0, b, 0].
    //
    // Expected cells are written out from the formula, not produced by calling
    // the same helpers the implementation calls in the same order.
    //
    // Mathematically:  p = [a - b*sin(q1), b*cos(q1), 0]
    //
    // Structurally the cos(q1)*0 and sin(q1)*0 terms survive, because the
    // factory has no x * 0 -> 0 rule. They are exactly what a future
    // simplifier will remove; asserting the clean formula here would be
    // asserting something this layer does not yet produce.
    const ExpressionFactory factory;
    const double a = 0.35;
    const double b = 0.29;

    const Expression q1 = factory.symbol("q1");
    const Expression q2 = factory.symbol("q2");

    const auto lhs = makeTransform(rotationAboutZ(q1, factory), vector3(a, 0.0, 0.0, factory));
    const auto rhs = makeTransform(rotationAboutX(q2, factory), vector3(0.0, b, 0.0, factory));

    const auto product = multiplyTransforms(lhs, rhs, factory);

    const Expression c1 = factory.cos(q1);
    const Expression s1 = factory.sin(q1);
    const Expression c2 = factory.cos(q2);
    const Expression s2 = factory.sin(q2);
    const Expression zero = factory.constant(0.0);
    const Expression bb = factory.constant(b);

    // p_x = a + (cos(q1)*0 + (-sin(q1))*b)
    EXPECT_TRUE(structurallyEqual(
        product(0, 3),
        factory.add(factory.constant(a),
                    factory.add(factory.multiply(c1, zero),
                                factory.multiply(factory.negate(s1), bb)))));

    // p_y = sin(q1)*0 + cos(q1)*b   -- the leading 0 + ... folds away
    EXPECT_TRUE(structurallyEqual(
        product(1, 3),
        factory.add(factory.multiply(s1, zero), factory.multiply(c1, bb))));

    EXPECT_TRUE(isZero(product(2, 3)));

    // One rotation cell of Rz(q1)*Rx(q2), left-folded over ascending k as
    // SymbolicMatrix::multiply documents.
    EXPECT_TRUE(structurallyEqual(
        product(0, 1),
        factory.add(factory.add(factory.multiply(c1, zero),
                                factory.multiply(factory.negate(s1), c2)),
                    factory.multiply(zero, s2))));
}

TEST(SymbolicTransformTest, PreservesCanonicalHomogeneousLastRow)
{
    // Both operands non-trivial, so no fast path fires and the general branch
    // runs — the branch a full 4x4 product would corrupt.
    const ExpressionFactory factory;
    const auto lhs = makeTransform(rotationAboutZ(factory.symbol("q1"), factory),
                                   vector3(0.35, 0.1, 0.2, factory));
    const auto rhs = makeTransform(rotationAboutX(factory.symbol("q2"), factory),
                                   vector3(0.0, 0.29, 0.3, factory));

    const auto product = multiplyTransforms(lhs, rhs, factory);

    EXPECT_TRUE(hasCanonicalHomogeneousLastRow(product));
    EXPECT_TRUE(isZero(product(3, 0)));
    EXPECT_TRUE(isZero(product(3, 1)));
    EXPECT_TRUE(isZero(product(3, 2)));
    EXPECT_TRUE(isOne(product(3, 3)));
}
