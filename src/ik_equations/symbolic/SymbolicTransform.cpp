#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <cassert>
#include <cstddef>

namespace kinemaforge::ik {

namespace {

// Asks about constants, not about equivalence of expressions — the same
// question JointTransformBuilder asks about an origin rotation.
bool isIdentityRotation(const SymbolicRotation& rotation) noexcept
{
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
        {
            const bool ok = (row == column) ? isOne(rotation(row, column))
                                            : isZero(rotation(row, column));
            if (!ok) return false;
        }
    return true;
}

bool isZeroVector(const SymbolicVector3& vector) noexcept
{
    for (std::size_t row = 0; row < 3; ++row)
        if (!isZero(vector(row, 0))) return false;
    return true;
}

SymbolicRotation rotationBlock(const SymbolicTransform& transform)
{
    SymbolicRotation rotation;
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            rotation(row, column) = transform(row, column);
    return rotation;
}

SymbolicVector3 translationBlock(const SymbolicTransform& transform)
{
    SymbolicVector3 translation;
    for (std::size_t row = 0; row < 3; ++row)
        translation(row, 0) = transform(row, 3);
    return translation;
}

// Starting from identity leaves the homogeneous last row correct by
// construction; nothing writes to row 3.
SymbolicTransform assembleTransform(const SymbolicRotation& rotation,
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

} // namespace

bool hasCanonicalHomogeneousLastRow(const SymbolicTransform& transform) noexcept
{
    return isZero(transform(3, 0)) && isZero(transform(3, 1)) &&
           isZero(transform(3, 2)) && isOne(transform(3, 3));
}

bool isIdentityTransform(const SymbolicTransform& transform) noexcept
{
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            const bool ok = (row == column) ? isOne(transform(row, column))
                                            : isZero(transform(row, column));
            if (!ok) return false;
        }
    return true;
}

// Five explicit fast paths, reproducing six algebraic identities:
//
//     I·T = T     T·I = T     I·R = R     R·I = R     R·0 = 0     I·p = p
//
// These are not optimizations that happen to pay off on the robots in
// data/urdf — they restore identities that the symbolic layer otherwise
// loses. ExpressionFactory deliberately has no x·0 -> 0 rule, so a plain
// product with an identity block does NOT give the other operand back: it
// gives it back buried under Multiply(cell, Constant(0)) terms. Measured:
// T·I differs from T in 8 of 16 cells, and R_b = I corrupts 6 of the 9
// rotation cells.
//
// Each branch selects which expression to build, based on constants known
// before anything is built. Nothing already built is ever rewritten — that
// distinction is what keeps this out of simplifier territory.
//
// There is deliberately no fast path for a zero p_lhs: the final addition
// goes through the factory, which folds 0 + x -> x as well as x + 0 -> x.
SymbolicTransform multiplyTransforms(const SymbolicTransform& lhs,
                                     const SymbolicTransform& rhs,
                                     const ExpressionFactory& factory)
{
    assert(hasCanonicalHomogeneousLastRow(lhs) &&
           "lhs must be a canonical homogeneous transform");
    assert(hasCanonicalHomogeneousLastRow(rhs) &&
           "rhs must be a canonical homogeneous transform");

    if (isIdentityTransform(lhs)) return rhs;
    if (isIdentityTransform(rhs)) return lhs;

    const SymbolicRotation lhsRotation = rotationBlock(lhs);
    const SymbolicRotation rhsRotation = rotationBlock(rhs);
    const SymbolicVector3 lhsTranslation = translationBlock(lhs);
    const SymbolicVector3 rhsTranslation = translationBlock(rhs);

    // Computed once: the translation branch below asks the same question.
    const bool lhsRotationIsIdentity = isIdentityRotation(lhsRotation);

    SymbolicRotation resultRotation;
    if (lhsRotationIsIdentity)
        resultRotation = rhsRotation;
    else if (isIdentityRotation(rhsRotation))
        resultRotation = lhsRotation;
    else
        resultRotation = multiply(lhsRotation, rhsRotation, factory);

    SymbolicVector3 rotatedTranslation;
    if (isZeroVector(rhsTranslation))
        rotatedTranslation = SymbolicVector3::zeros();
    else if (lhsRotationIsIdentity)
        rotatedTranslation = rhsTranslation;
    else
        rotatedTranslation = multiply(lhsRotation, rhsTranslation, factory);

    SymbolicVector3 resultTranslation;
    for (std::size_t row = 0; row < 3; ++row)
        resultTranslation(row, 0) =
            factory.add(lhsTranslation(row, 0), rotatedTranslation(row, 0));

    return assembleTransform(resultRotation, resultTranslation);
}

} // namespace kinemaforge::ik
