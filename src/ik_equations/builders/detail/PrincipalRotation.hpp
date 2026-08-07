#pragma once

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

namespace kinemaforge::ik::detail {

enum class PrincipalAxis
{
    X,
    Y,
    Z
};

// Rotation about one principal axis.
//
// `negated` folds the sign into the sine rather than wrapping the result,
// since R(-a, q) = R(a, -q) and cos is even: without it the cells would carry
// Negate(Negate(Sin)).
//
// An implementation detail shared by JointTransformBuilder and
// RigidTransformConstruction, deliberately outside the module's surface: a
// consumer has no reason to know about `negated`.
SymbolicRotation makePrincipalRotation(PrincipalAxis axis,
                                       bool negated,
                                       const Expression& angle,
                                       const ExpressionFactory& factory);

} // namespace kinemaforge::ik::detail
