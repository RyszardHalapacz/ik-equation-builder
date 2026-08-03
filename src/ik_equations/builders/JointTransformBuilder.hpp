#pragma once

#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

namespace kinemaforge::ik {

// Turns one joint into T_parent_child(q), the homogeneous transform from
// the parent link frame to the child link frame:
//
//     T_parent_child(q) = T_origin * T_motion(q)
//     T_origin          = Translation(origin.translation) * R_rpy(origin.rpy)
//
// The joint axis is expressed in the joint frame — that is, after origin
// has been applied — which is why T_motion uses it unrotated.
//
// Preconditions, established by UrdfModelLoader and KinematicChainBuilder
// and asserted here rather than re-validated:
//   * a fixed joint has no variable
//   * an actuated joint has one, and its axis is a unit vector
class JointTransformBuilder
{
public:
    explicit JointTransformBuilder(ExpressionFactory factory = {});

    SymbolicTransform build(const KinematicJoint& joint) const;

private:
    ExpressionFactory factory_;
};

} // namespace kinemaforge::ik
