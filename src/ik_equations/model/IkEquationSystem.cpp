#include "ik_equations/model/IkEquationSystem.hpp"

#include <cstddef>
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
                     "position equation carries an invalid Cartesian component");
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
std::expected<void, IkEquationSystemError>
checkEquationContent(IkTaskKind taskKind, const std::vector<Equation>& equations)
{
    bool present[3] = {false, false, false};
    std::size_t orientationCount = 0;

    for (const auto& equation : equations)
    {
        // Explicit visit, no `if position else orientation`: with the latter,
        // every future alternative would be silently counted as orientation.
        const auto checked = std::visit(
            Overloaded{
                [&present](const PositionEquationSource& position)
                    -> std::expected<void, IkEquationSystemError> {
                    const auto slot = componentIndex(position.component);
                    if (!slot)
                        return std::unexpected(slot.error());
                    present[*slot] = true;
                    return {};
                },
                [&orientationCount](const OrientationEquationSource&)
                    -> std::expected<void, IkEquationSystemError> {
                    ++orientationCount;
                    return {};
                }},
            equation.source);

        if (!checked)
            return std::unexpected(checked.error());
    }

    // Position is complete in both tasks. A task constraining two of three
    // coordinates would be a different IkTaskKind, not a truncated Pose.
    for (std::size_t slot = 0; slot < 3; ++slot)
        if (!present[slot])
            return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                             std::string("missing position equation for ") +
                                 describe(static_cast<CartesianComponent>(slot)));

    // No default: a new IkTaskKind trips -Wswitch, which -Werror turns into a
    // build failure. Each valid case returns, so a value forged with
    // static_cast<IkTaskKind>(99) falls through to the rejection below instead
    // of quietly succeeding -- the two protections cover different things and
    // neither replaces the other.
    switch (taskKind)
    {
    case IkTaskKind::Position:
        if (orientationCount != 0)
            return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                             "a position task cannot carry orientation equations");
        return {};

    case IkTaskKind::Pose:
        // How MANY orientation equations is F2.5's decision; presence is not.
        if (orientationCount == 0)
            return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                             "a pose task needs at least one orientation equation");
        return {};
    }

    return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                     "unsupported IK task kind");
}

// Two equations addressing one matrix cell are redundant at best and
// contradictory at worst. Rejected explicitly rather than left to fall out of
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

std::expected<void, IkEquationSystemError>
checkEquationOrder(const std::vector<Equation>& equations)
{
    // Defensive, not decorative: without it this function is only safe because
    // checkEquationContent ran first, and swapping two calls in create() would
    // turn a validation error into an out-of-range access.
    if (equations.size() < 3)
        return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                         "the system does not contain complete X, Y, Z constraints");

    for (std::size_t slot = 0; slot < 3; ++slot)
    {
        const auto* position = std::get_if<PositionEquationSource>(&equations[slot].source);
        if (position == nullptr)
            return makeError(IkEquationSystemErrorCode::UnorderedEquations,
                             "position equations must come first, in X, Y, Z order");

        // Through componentIndex rather than a raw cast, so this helper does
        // not depend on checkEquationContent having run first.
        const auto component = componentIndex(position->component);
        if (!component)
            return std::unexpected(component.error());

        if (*component != slot)
            return makeError(IkEquationSystemErrorCode::UnorderedEquations,
                             "position equations must come first, in X, Y, Z order");
    }

    const OrientationEquationSource* previous = nullptr;
    for (std::size_t slot = 3; slot < equations.size(); ++slot)
    {
        const auto* orientation =
            std::get_if<OrientationEquationSource>(&equations[slot].source);
        if (orientation == nullptr)
            return makeError(IkEquationSystemErrorCode::UnorderedEquations,
                             "orientation equations must follow the position ones");

        if (previous != nullptr)
        {
            const bool increasing =
                previous->row() < orientation->row() ||
                (previous->row() == orientation->row() &&
                 previous->column() < orientation->column());
            if (!increasing)
                return makeError(IkEquationSystemErrorCode::UnorderedEquations,
                                 "orientation equations must be in row-major order");
        }
        previous = orientation;
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
