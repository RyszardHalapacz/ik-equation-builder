#pragma once

#include "ik_equations/model/Vector3.hpp"

namespace kinemaforge::ik {

// A constant rigid transform T_parent_child between two frames.
//
//   translation : position of the child frame's origin, expressed in parent
//                 unit: metres
//   rpy         : orientation of child relative to parent
//                 unit: radians
//                 R = Rz(yaw) * Ry(pitch) * Rx(roll)
//                 -- URDF's fixed-axis convention, the same one joint origins
//                    use, so a value read off a URDF and a value typed by hand
//                    mean the same thing
//
// The direction is part of the contract and the type cannot enforce it:
// nothing stops a caller from filling this with T_child_parent, and no
// compiler will notice. For a TCP, `parent` is the tip of the currently
// selected kinematic chain and `child` is the tool centre point.
//
// Validity is exactly "all six values are finite". There is no other
// condition -- any six finite numbers describe a rigid transform, which is
// why this representation needs no tolerance to validate.
struct FixedRigidTransform
{
    Vector3 translation;
    Vector3 rpy;
};

} // namespace kinemaforge::ik
