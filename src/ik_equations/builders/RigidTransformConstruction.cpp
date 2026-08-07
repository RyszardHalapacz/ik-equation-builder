#include "ik_equations/builders/RigidTransformConstruction.hpp"

#include "ik_equations/builders/detail/PrincipalRotation.hpp"

namespace kinemaforge::ik {

SymbolicRotation makeRpyRotation(const Vector3& rpy, const ExpressionFactory& factory)
{
    using detail::PrincipalAxis;

    // Composed from three principal rotations rather than the nine closed-form
    // entries: every input here is a constant, so folding collapses the
    // products immediately and the code stays readable as the formula it is.
    const auto roll = detail::makePrincipalRotation(
        PrincipalAxis::X, false, factory.constant(rpy.x), factory);
    const auto pitch = detail::makePrincipalRotation(
        PrincipalAxis::Y, false, factory.constant(rpy.y), factory);
    const auto yaw = detail::makePrincipalRotation(
        PrincipalAxis::Z, false, factory.constant(rpy.z), factory);

    return multiply(multiply(yaw, pitch, factory), roll, factory);
}

SymbolicVector3 toSymbolicVector(const Vector3& value, const ExpressionFactory& factory)
{
    SymbolicVector3 vector;
    vector(0, 0) = factory.constant(value.x);
    vector(1, 0) = factory.constant(value.y);
    vector(2, 0) = factory.constant(value.z);
    return vector;
}

SymbolicTransform buildFixedRigidTransform(const FixedRigidTransform& transform,
                                           const ExpressionFactory& factory)
{
    return assembleTransform(makeRpyRotation(transform.rpy, factory),
                             toSymbolicVector(transform.translation, factory));
}

} // namespace kinemaforge::ik
