#pragma once

#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

namespace kinemaforge::ik {

class ForwardKinematicsBuilder
{
public:
    SymbolicTransform build(
        const KinematicChain& chain,
        const JointTransformBuilder& transformBuilder
    ) const;
};

} // namespace kinemaforge::ik
