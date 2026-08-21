#include "ik_equations/model/IkEquationSystem.hpp"

#include <compare>
#include <cstddef>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <variant>

namespace kinemaforge::ik {

namespace {

std::unexpected<IkEquationSystemError> makeError(IkEquationSystemErrorCode code,
                                                 std::string message)
{
    return std::unexpected(IkEquationSystemError{code, std::move(message)});
}

const char* describe(CartesianComponent component) noexcept
{
    switch (component)
    {
    case CartesianComponent::X: return "x";
    case CartesianComponent::Y: return "y";
    case CartesianComponent::Z: return "z";
    }
    return "?";
}

// Overload set for std::visit. Deliberately not a generic lambda anywhere in
// this file: a new EquationSource alternative must fail to compile.
template <class... Ts>
struct Overloaded : Ts...
{
    using Ts::operator()...;
};

// Never `present[static_cast<std::size_t>(component)]`. A CartesianComponent
// forged with static_cast<CartesianComponent>(99) is a valid object of the
// enum type but has no enumerator, and indexing with it writes out of bounds.
// The switch converts only the three values that exist and rejects the rest.
std::expected<std::size_t, IkEquationSystemError> componentIndex(CartesianComponent component)
{
    switch (component)
    {
    case CartesianComponent::X: return 0;
    case CartesianComponent::Y: return 1;
    case CartesianComponent::Z: return 2;
    }

    return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                     "equation source carries an invalid Cartesian component");
}

// Each rule gets its own pass over the whole vector. That is deliberate: the
// priority in create() is per rule, not per element, so an empty name in the
// last unknown must beat a duplicate index in the first.
std::expected<void, IkEquationSystemError>
checkUnknowns(const std::vector<JointVariable>& unknowns)
{
    if (unknowns.empty())
        return makeError(IkEquationSystemErrorCode::NoUnknowns,
                         "an IK task without unknowns is not an IK task");

    for (const auto& unknown : unknowns)
        if (unknown.name.empty())
            return makeError(IkEquationSystemErrorCode::EmptyUnknownName,
                             "unknown at index " + std::to_string(unknown.index) +
                                 " has an empty name");

    std::unordered_set<std::string> names;
    for (const auto& unknown : unknowns)
        if (!names.insert(unknown.name).second)
            return makeError(IkEquationSystemErrorCode::DuplicateUnknownName,
                             "duplicate unknown name '" + unknown.name +
                                 "'; the symbolic layer would treat both as one symbol");

    std::unordered_set<std::size_t> indices;
    for (const auto& unknown : unknowns)
        if (!indices.insert(unknown.index).second)
            return makeError(IkEquationSystemErrorCode::DuplicateUnknownIndex,
                             "duplicate unknown index " + std::to_string(unknown.index));

    for (std::size_t position = 1; position < unknowns.size(); ++position)
        if (unknowns[position].index <= unknowns[position - 1].index)
            return makeError(IkEquationSystemErrorCode::UnorderedUnknowns,
                             "unknowns must be ordered by ascending index");

    return {};
}

// Content first, duplicates second, order third -- three different faults
// with three different fixes.
// The tool axis carried by a source, if it carries one. Two alternatives do.
//
// An explicit overload set rather than get_if on one type: the question "does
// this source name a tool axis" concerns EVERY alternative, and a new one must
// answer it deliberately. Reaching for get_if<DirectionEquationSource> here was
// exactly the trap F2.6b would have fallen into -- the angle source carries an
// axis too, and would have slipped past both checks below.
std::optional<ToolAxis> toolAxisOf(const EquationSource& source)
{
    return std::visit(
        Overloaded{
            [](const PositionEquationSource&) { return std::optional<ToolAxis>{}; },
            [](const OrientationEquationSource&) { return std::optional<ToolAxis>{}; },
            [](const DirectionEquationSource& direction) {
                return std::optional<ToolAxis>{direction.axis};
            },
            [](const DirectionAngleEquationSource& angle) {
                return std::optional<ToolAxis>{angle.axis};
            }},
        source);
}

// What a task kind is allowed to contain. The zeros matter as much as the
// completes: without them a Position task could carry a direction equation.
struct EquationProfile
{
    bool position{};             // exactly X, Y, Z
    std::size_t orientation{};   // exact count: 0 or 9
    bool direction{};            // exactly X, Y, Z of ONE tool axis
    std::size_t angle{};         // exact count: 0 or 1
};

std::expected<EquationProfile, IkEquationSystemError> profileOf(IkTaskKind taskKind)
{
    // No default: a new IkTaskKind trips -Wswitch, which -Werror turns into a
    // build failure. Each valid case returns, so a value forged with
    // static_cast<IkTaskKind>(99) falls through to the rejection below instead
    // of quietly succeeding.
    switch (taskKind)
    {
    case IkTaskKind::Position:                 return EquationProfile{true, 0, false, 0};
    case IkTaskKind::Pose:                     return EquationProfile{true, 9, false, 0};
    case IkTaskKind::ToolDirection:            return EquationProfile{false, 0, true, 0};
    case IkTaskKind::PositionAndToolDirection: return EquationProfile{true, 0, true, 0};
    case IkTaskKind::DirectionAngle:           return EquationProfile{false, 0, false, 1};
    case IkTaskKind::PositionAndDirectionAngle: return EquationProfile{true, 0, false, 1};
    }

    return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                     "unsupported IK task kind");
}

// Three passes, not one. The priority promised by create() is per rule, not per
// element: with a single pass the reported code would depend on which broken
// equation came first in the vector, so
//
//     [0] direction with a forged CartesianComponent
//     [1] direction with a forged ToolAxis
//
// would report TaskEquationMismatch, and the same two equations swapped would
// report InvalidToolAxis. Same rule as checkUnknowns, for the same reason.
std::expected<void, IkEquationSystemError>
checkEquationContent(IkTaskKind taskKind, const std::vector<Equation>& equations)
{
    // PASS 1 -- every tool axis is a real enumerator. First, because a forged
    // axis makes every later statement about "the" axis meaningless.
    for (const auto& equation : equations)
        if (const auto axis = toolAxisOf(equation.source); axis && !toolAxisColumn(*axis))
            return makeError(IkEquationSystemErrorCode::InvalidToolAxis,
                             "an equation carries an invalid tool axis");

    // PASS 2 -- all tool-axis equations agree about which axis. Completeness
    // alone would accept three equations about three DIFFERENT tool axes: X, Y
    // and Z all present, and the system internally contradictory.
    std::optional<ToolAxis> toolAxis;
    for (const auto& equation : equations)
        if (const auto axis = toolAxisOf(equation.source))
        {
            if (toolAxis && *toolAxis != *axis)
                return makeError(IkEquationSystemErrorCode::MixedToolAxes,
                                 "equations disagree about the tool axis");
            toolAxis = *axis;
        }

    // Only now. profileOf() reports TaskEquationMismatch for a forged
    // IkTaskKind, so running it earlier would let a broken task kind outrank a
    // broken tool axis -- and the promised order would depend on which of the
    // two the caller got wrong. The sequence is also the honest one: integrity
    // of a single source, then relations between sources, then agreement with
    // the task kind.
    const auto profile = profileOf(taskKind);
    if (!profile)
        return std::unexpected(profile.error());

    // PASS 3 -- components and the task profile. Everything here reports
    // TaskEquationMismatch, so the order WITHIN this pass is not observable.
    bool position[3] = {false, false, false};
    bool direction[3] = {false, false, false};
    std::size_t orientationCount = 0;
    std::size_t angleCount = 0;

    for (const auto& equation : equations)
    {
        // Explicit visit, no `if position else orientation`: with the latter,
        // every future alternative would be silently counted as orientation.
        const auto checked = std::visit(
            Overloaded{
                [&position](const PositionEquationSource& source)
                    -> std::expected<void, IkEquationSystemError> {
                    const auto slot = componentIndex(source.component);
                    if (!slot)
                        return std::unexpected(slot.error());
                    position[*slot] = true;
                    return {};
                },
                [&orientationCount](const OrientationEquationSource&)
                    -> std::expected<void, IkEquationSystemError> {
                    ++orientationCount;
                    return {};
                },
                [&direction](const DirectionEquationSource& source)
                    -> std::expected<void, IkEquationSystemError> {
                    // The axis is already known good: pass 1.
                    const auto slot = componentIndex(source.component);
                    if (!slot)
                        return std::unexpected(slot.error());
                    direction[*slot] = true;
                    return {};
                },
                [&angleCount](const DirectionAngleEquationSource&)
                    -> std::expected<void, IkEquationSystemError> {
                    ++angleCount;
                    return {};
                }},
            equation.source);

        if (!checked)
            return std::unexpected(checked.error());
    }

    // Position is complete or absent, per the profile. A task constraining two
    // of three coordinates would be a different IkTaskKind, not a truncated one.
    for (std::size_t slot = 0; slot < 3; ++slot)
        if (position[slot] != profile->position)
            return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                             profile->position
                                 ? std::string("missing position equation for ")
                                       + describe(static_cast<CartesianComponent>(slot))
                                 : std::string("this task cannot carry position equations"));

    // A count, deliberately, not a completeness check. Nine equations plus a
    // valid OrientationEquationSource plus no duplicate source already means
    // nine distinct cells -- and diagnosing "missing cell" here would stop a
    // duplicate from ever reaching DuplicateEquationSource, which exists to
    // report exactly that.
    if (orientationCount != profile->orientation)
        return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                         "this task needs exactly " + std::to_string(profile->orientation)
                             + " orientation equations, got "
                             + std::to_string(orientationCount));

    for (std::size_t slot = 0; slot < 3; ++slot)
        if (direction[slot] != profile->direction)
            return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                             profile->direction
                                 ? std::string("missing direction equation for ")
                                       + describe(static_cast<CartesianComponent>(slot))
                                 : std::string("this task cannot carry direction equations"));

    if (angleCount != profile->angle)
        return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                         "this task needs exactly " + std::to_string(profile->angle)
                             + " direction-angle equations, got " + std::to_string(angleCount));

    return {};
}

// Two equations occupying the same canonical constraint slot are redundant at
// best and contradictory at worst. Rejected explicitly rather than left to fall out of
// the strict ordering rule -- the architecture and the code must say the same
// thing about duplicates.
std::expected<void, IkEquationSystemError>
checkDuplicateSources(const std::vector<Equation>& equations)
{
    for (std::size_t outer = 0; outer < equations.size(); ++outer)
        for (std::size_t inner = outer + 1; inner < equations.size(); ++inner)
            if (sameSource(equations[outer].source, equations[inner].source))
                return makeError(IkEquationSystemErrorCode::DuplicateEquationSource,
                                 "equations " + std::to_string(outer) + " and " +
                                     std::to_string(inner) + " constrain the same quantity");
    return {};
}

// (rank of the source kind, index within the kind). Strictly increasing over the
// whole vector -- one rule instead of per-kind slot arithmetic, and it covers
// every task kind without branching.
struct SourceOrderKey
{
    std::size_t rank{};
    std::size_t index{};

    friend auto operator<=>(const SourceOrderKey&, const SourceOrderKey&) noexcept = default;
};

// Defensive on purpose: this must NOT assume checkEquationContent already ran.
// create() calls them in the right order today, but swapping two lines must not
// turn a validation error into an out-of-range read -- the same reason
// componentIndex exists at all.
std::expected<SourceOrderKey, IkEquationSystemError>
sourceOrderKey(const EquationSource& source)
{
    return std::visit(
        Overloaded{
            [](const PositionEquationSource& position)
                -> std::expected<SourceOrderKey, IkEquationSystemError> {
                const auto index = componentIndex(position.component);
                if (!index)
                    return std::unexpected(index.error());
                return SourceOrderKey{0, *index};
            },
            [](const OrientationEquationSource& orientation)
                -> std::expected<SourceOrderKey, IkEquationSystemError> {
                // row() and column() are 0..2 by construction: the type has no
                // public constructor that could produce anything else.
                return SourceOrderKey{1, orientation.row() * 3 + orientation.column()};
            },
            [](const DirectionEquationSource& direction)
                -> std::expected<SourceOrderKey, IkEquationSystemError> {
                const auto index = componentIndex(direction.component);
                if (!index)
                    return std::unexpected(index.error());

                if (!toolAxisColumn(direction.axis))
                    return makeError(IkEquationSystemErrorCode::InvalidToolAxis,
                                     "a direction equation carries an invalid tool axis");

                // The axis is not part of the key: MixedToolAxes already
                // guarantees one axis per system.
                return SourceOrderKey{2, *index};
            },
            [](const DirectionAngleEquationSource& angle)
                -> std::expected<SourceOrderKey, IkEquationSystemError> {
                if (!toolAxisColumn(angle.axis))
                    return makeError(IkEquationSystemErrorCode::InvalidToolAxis,
                                     "a direction-angle equation carries an invalid tool axis");

                // One such equation per system, so the index is constant.
                return SourceOrderKey{3, 0};
            }},
        source);
}

std::expected<void, IkEquationSystemError>
checkEquationOrder(const std::vector<Equation>& equations)
{
    std::optional<SourceOrderKey> previous;

    for (std::size_t position = 0; position < equations.size(); ++position)
    {
        const auto key = sourceOrderKey(equations[position].source);
        if (!key)
            return std::unexpected(key.error());

        if (previous && !(*previous < *key))
            return makeError(IkEquationSystemErrorCode::UnorderedEquations,
                             "equation " + std::to_string(position) +
                                 " does not follow the canonical order: position (X, Y, Z), "
                                 "then orientation row-major, then direction (X, Y, Z), "
                                 "then the direction angle");

        previous = *key;
    }

    return {};
}

} // namespace

IkEquationSystem::IkEquationSystem(IkTaskKind taskKind,
                                   std::vector<JointVariable> unknowns,
                                   std::vector<Equation> equations)
    : taskKind_(taskKind), unknowns_(std::move(unknowns)), equations_(std::move(equations))
{
}

std::expected<IkEquationSystem, IkEquationSystemError>
IkEquationSystem::create(IkTaskKind taskKind,
                         std::vector<JointVariable> unknowns,
                         std::vector<Equation> equations)
{
    if (equations.empty())
        return makeError(IkEquationSystemErrorCode::NoEquations,
                         "a system without equations constrains nothing");

    if (auto checked = checkUnknowns(unknowns); !checked)
        return std::unexpected(checked.error());

    if (auto checked = checkEquationContent(taskKind, equations); !checked)
        return std::unexpected(checked.error());

    if (auto checked = checkDuplicateSources(equations); !checked)
        return std::unexpected(checked.error());

    if (auto checked = checkEquationOrder(equations); !checked)
        return std::unexpected(checked.error());

    return IkEquationSystem{taskKind, std::move(unknowns), std::move(equations)};
}

IkTaskKind IkEquationSystem::taskKind() const noexcept
{
    return taskKind_;
}

std::span<const JointVariable> IkEquationSystem::unknowns() const noexcept
{
    return unknowns_;
}

std::span<const Equation> IkEquationSystem::equations() const noexcept
{
    return equations_;
}

} // namespace kinemaforge::ik
