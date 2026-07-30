#pragma once

#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

namespace kinemaforge::ik {

class JointTransformBuilder
{
public:
    SymbolicTransform build(const KinematicJoint& joint) const;
};

} // namespace kinemaforge::ik
