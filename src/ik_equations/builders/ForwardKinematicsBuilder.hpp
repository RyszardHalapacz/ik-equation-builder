#pragma once

#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

namespace kinemaforge::ik {

// Composes a whole kinematic chain into one symbolic transform:
//
//     T_base_tool(q) = T_1(q1) * T_2(q2) * ... * T_n(qn)
//
// The chain is ordered base -> tool, and the child link of joint i is the
// parent link of joint i+1, so the product telescopes into the transform
// from the base frame to the tool frame.
//
// An empty chain (baseLink == toolLink, which KinematicChainBuilder reports
// as success rather than an error) yields the identity — the empty product.
//
// No error model: no KinematicChain can make this computation fail.
// Topology was checked by KinematicChainBuilder, geometry by the loader, and
// per-joint invariants are asserted by JointTransformBuilder.
class ForwardKinematicsBuilder
{
public:
    explicit ForwardKinematicsBuilder(ExpressionFactory factory = {});

    SymbolicTransform build(const KinematicChain& chain,
                            const JointTransformBuilder& transformBuilder) const;

private:
    ExpressionFactory factory_;
};

} // namespace kinemaforge::ik
