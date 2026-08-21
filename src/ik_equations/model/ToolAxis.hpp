#pragma once

#include <cstddef>
#include <optional>

namespace kinemaforge::ik {

// Which axis of the tool the task talks about.
//
// WHOSE axis: the final frame of the SymbolicTransform handed to
// ConstraintBuilder. If that transform came from tcpForwardKinematics(), this is
// an axis of the TCP; if it ended at the chain tip, it is an axis of the last
// link. The builder cannot tell them apart and does not try -- the same class of
// gap as "a target does not know its frame".
//
// Only the principal axes. A tool tilted off its frame is described by the TCP
// transform, not by a free axis here: two places encoding tool geometry would
// eventually disagree.
enum class ToolAxis
{
    X,
    Y,
    Z
};

// The one and only conversion from an axis to a column of the rotation block.
//
// Returns nullopt for a value forged with static_cast<ToolAxis>(99): such an
// object is a valid ToolAxis with no enumerator behind it, and indexing with it
// would read out of bounds. The same defence the model already has for
// CartesianComponent.
//
// std::optional rather than std::expected on purpose: this header must not know
// any error vocabulary. The axis is validated in two layers with two different
// error types -- TargetValidationErrorCode::InvalidToolAxis for a target,
// IkEquationSystemErrorCode::InvalidToolAxis for an equation source -- and an
// expected would force this file to pick one of them.
[[nodiscard]] constexpr std::optional<std::size_t> toolAxisColumn(ToolAxis axis) noexcept
{
    switch (axis)
    {
    case ToolAxis::X: return 0;
    case ToolAxis::Y: return 1;
    case ToolAxis::Z: return 2;
    }
    return std::nullopt;
}

} // namespace kinemaforge::ik
