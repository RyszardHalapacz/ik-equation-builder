#pragma once

#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicMatrix.hpp"

namespace kinemaforge::ik {

using SymbolicRotation = SymbolicMatrix<3, 3>;
using SymbolicVector3 = SymbolicMatrix<3, 1>;
using SymbolicTransform = SymbolicMatrix<4, 4>;

// True when the last row is exactly [0 0 0 1] — the cells are recognised by
// isZero/isOne, not merely equal to zero and one mathematically.
//
// SymbolicTransform is only an alias for a 4x4 matrix, so the type carries no
// such guarantee on its own. This predicate is what multiplyTransforms asserts
// about its operands.
bool hasCanonicalHomogeneousLastRow(const SymbolicTransform& transform) noexcept;

// True when all sixteen cells are exactly the constants of an identity matrix.
// Checks the full matrix rather than relying on the last row being canonical:
// the name promises a statement about the matrix, so it has to make one.
bool isIdentityTransform(const SymbolicTransform& transform) noexcept;

// Product of two homogeneous transforms, composed blockwise:
//
//     R = R_lhs * R_rhs
//     p = p_lhs + R_lhs * p_rhs
//     last row assembled as [0 0 0 1], never multiplied
//
// Preconditions, asserted rather than validated: both operands are canonical
// homogeneous transforms. Every producer in this project guarantees it —
// JointTransformBuilder, SymbolicTransform::identity(), and this function
// itself — so the contract is closed under composition.
//
// Multiplying two full 4x4 matrices instead would destroy that: the last row
// would become 0*R00 + 0*R10 + 0*R20, which is zero mathematically but no
// longer a Constant(0) that isZero() recognises. Measured on kr640.urdf, that
// cell grows to 31 738 nodes.
SymbolicTransform multiplyTransforms(const SymbolicTransform& lhs,
                                     const SymbolicTransform& rhs,
                                     const ExpressionFactory& factory);

} // namespace kinemaforge::ik
