#pragma once

#include "ik_equations/model/FixedRigidTransform.hpp"
#include "ik_equations/model/Vector3.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

namespace kinemaforge::ik {

// R_rpy = Rz(yaw) * Ry(pitch) * Rx(roll) -- URDF's fixed-axis convention.
//
// The single production implementation of that convention. JointTransformBuilder
// uses it for joint origins, buildFixedRigidTransform for constant frame
// offsets. A second copy would be a second place for the convention to drift,
// and STATUS.md records that nothing external verifies it -- so a drift there
// is precisely the kind this project cannot currently detect.
SymbolicRotation makeRpyRotation(const Vector3& rpy, const ExpressionFactory& factory);

// A plain Vector3 as a column of symbolic constants.
SymbolicVector3 toSymbolicVector(const Vector3& value, const ExpressionFactory& factory);

// A constant homogeneous transform from translation + rpy.
//
// Every cell folds to a constant during construction, so this introduces no
// symbolic variables. The result satisfies hasCanonicalHomogeneousLastRow by
// construction.
SymbolicTransform buildFixedRigidTransform(const FixedRigidTransform& transform,
                                           const ExpressionFactory& factory);

} // namespace kinemaforge::ik
