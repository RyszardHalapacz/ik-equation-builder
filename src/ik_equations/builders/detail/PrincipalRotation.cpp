#include "ik_equations/builders/detail/PrincipalRotation.hpp"

namespace kinemaforge::ik::detail {

SymbolicRotation makePrincipalRotation(PrincipalAxis axis,
                                       bool negated,
                                       const Expression& angle,
                                       const ExpressionFactory& factory)
{
    const Expression cosine = factory.cos(angle);
    const Expression rawSine = factory.sin(angle);

    const Expression sine      = negated ? factory.negate(rawSine) : rawSine;
    const Expression minusSine = negated ? rawSine : factory.negate(rawSine);

    auto rotation = SymbolicRotation::identity();
    switch (axis)
    {
    case PrincipalAxis::X:
        rotation(1, 1) = cosine; rotation(1, 2) = minusSine;
        rotation(2, 1) = sine;   rotation(2, 2) = cosine;
        break;
    case PrincipalAxis::Y:
        rotation(0, 0) = cosine; rotation(0, 2) = sine;
        rotation(2, 0) = minusSine; rotation(2, 2) = cosine;
        break;
    case PrincipalAxis::Z:
        rotation(0, 0) = cosine; rotation(0, 1) = minusSine;
        rotation(1, 0) = sine;   rotation(1, 1) = cosine;
        break;
    }
    return rotation;
}

} // namespace kinemaforge::ik::detail
