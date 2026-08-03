#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"

#include <utility>

namespace kinemaforge::ik {

ForwardKinematicsBuilder::ForwardKinematicsBuilder(ExpressionFactory factory)
    : factory_(std::move(factory))
{
}

SymbolicTransform ForwardKinematicsBuilder::build(
    const KinematicChain& chain,
    const JointTransformBuilder& transformBuilder) const
{
    // Left fold, accumulator on the left. Starting from identity makes the
    // empty chain fall out without a special case, and multiplyTransforms
    // returns the first joint's transform unchanged rather than composing
    // with the identity accumulator.
    auto result = SymbolicTransform::identity();
    for (const auto& joint : chain.joints)
        result = multiplyTransforms(result, transformBuilder.build(joint), factory_);
    return result;
}

} // namespace kinemaforge::ik
