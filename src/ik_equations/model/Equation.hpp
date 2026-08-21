#pragma once

#include "ik_equations/model/ToolAxis.hpp"
#include "ik_equations/symbolic/Expression.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <variant>

namespace kinemaforge::ik {

// What the caller asked for.
enum class IkTaskKind
{
    Position,
    Pose,
    ToolDirection,
    PositionAndToolDirection,
    DirectionAngle,
    PositionAndDirectionAngle
};

// What a single equation constrains. Derived from EquationSource, never
// stored -- see kindOf below.
enum class EquationKind
{
    Position,
    Orientation,
    Direction,
    DirectionAngle
};

enum class CartesianComponent
{
    X,
    Y,
    Z
};

struct PositionEquationSource
{
    CartesianComponent component{};

    friend bool operator==(const PositionEquationSource&,
                           const PositionEquationSource&) noexcept = default;
};

// Closed, because it has an invariant an aggregate cannot hold: both indices
// must be in 0..2. Public fields plus a factory would leave
// `OrientationEquationSource{7, 9}` legal, which would make the factory a
// suggestion rather than a gate.
class OrientationEquationSource
{
public:
    [[nodiscard]] static std::optional<OrientationEquationSource>
    create(std::size_t row, std::size_t column) noexcept
    {
        if (row > 2 || column > 2)
            return std::nullopt;
        return OrientationEquationSource{static_cast<std::uint8_t>(row),
                                         static_cast<std::uint8_t>(column)};
    }

    [[nodiscard]] std::size_t row() const noexcept { return row_; }
    [[nodiscard]] std::size_t column() const noexcept { return column_; }

    friend bool operator==(const OrientationEquationSource&,
                           const OrientationEquationSource&) noexcept = default;

private:
    OrientationEquationSource(std::uint8_t row, std::uint8_t column) noexcept
        : row_(row), column_(column)
    {
    }

    std::uint8_t row_;
    std::uint8_t column_;
};

// One component of the tool axis direction, expressed in the base frame.
//
// NOT an OrientationEquationSource, even though for ToolAxis::Z these are
// literally cells (0,2), (1,2), (2,2). A source says WHAT is constrained, not
// which cell happened to be read: three orientation cells inside a Pose task
// are a fragment of a nine-cell constraint, while the same three cells in a
// ToolDirection task are the WHOLE constraint, with a different
// degree-of-freedom count and a different way of being solved. A solver that
// cannot tell them apart counts the degrees of freedom wrong.
struct DirectionEquationSource
{
    CartesianComponent component{};
    ToolAxis axis{ToolAxis::Z};

    friend bool operator==(const DirectionEquationSource&,
                           const DirectionEquationSource&) noexcept = default;
};

// A variant rather than {kind, optional<cell>}: a position equation has no way
// to name a rotation cell, and a rotation cell cannot exist without a kind
// that gives it meaning. Adding a constraint of a new kind forces every
// consumer using an explicit overload set to handle it, at compile time. That
// is not hypothetical any more: DirectionEquationSource was added later, and
// every explicit overload set in the model had to grow before the build would
// pass again.
// The angle between the tool axis and a reference direction:
//
//     dot(R(q) e_t, n) = cos(angle)
//
// One scalar equation, so there is no index to carry -- only which tool axis it
// talks about. The name deliberately avoids "axis-angle": that is a
// representation of a rotation, and confusing the two would be expensive.
//
// It carries neither the reference direction nor the angle, and that is not an
// omission: a source names the SLOT an equation occupies, not what the equation
// says. Two targets with different references and different angles legitimately
// share this source.
struct DirectionAngleEquationSource
{
    ToolAxis axis{ToolAxis::Z};

    friend bool operator==(const DirectionAngleEquationSource&,
                           const DirectionAngleEquationSource&) noexcept = default;
};

using EquationSource = std::variant<PositionEquationSource,
                                    OrientationEquationSource,
                                    DirectionEquationSource,
                                    DirectionAngleEquationSource>;

namespace detail {

// Explicit overloads, no generic operator(): adding a new alternative to
// EquationSource must stop the build here rather than silently classify it.
// A holds_alternative check would have called it Orientation, which destroys
// the whole reason for using a variant.
struct EquationKindVisitor
{
    EquationKind operator()(const PositionEquationSource&) const noexcept
    {
        return EquationKind::Position;
    }

    EquationKind operator()(const OrientationEquationSource&) const noexcept
    {
        return EquationKind::Orientation;
    }

    EquationKind operator()(const DirectionEquationSource&) const noexcept
    {
        return EquationKind::Direction;
    }

    EquationKind operator()(const DirectionAngleEquationSource&) const noexcept
    {
        return EquationKind::DirectionAngle;
    }
};

} // namespace detail

[[nodiscard]] inline EquationKind kindOf(const EquationSource& source) noexcept
{
    return std::visit(detail::EquationKindVisitor{}, source);
}

// Identity of the canonical constraint slot a given equation occupies within a
// task:
//
//   Position        one Cartesian component of the translation
//   Orientation     one cell of the rotation block
//   Direction       one Cartesian component of one tool axis direction
//   DirectionAngle  the single scalar angle slot for one tool axis
//
// Equal sources mean two equations occupy the SAME slot -- redundant at best,
// contradictory at worst. They do NOT imply equal expression trees or equal
// right-hand sides: a direction-angle source carries neither the reference
// direction nor the angle, because those describe what the constraint says, not
// which slot it fills. Nor does a slot have to be a cell of the matrix any more:
// the angle equation's left-hand side is a computed sum of products.
//
// Delegates to std::variant's own operator==, which compares alternatives only
// when the indices match and reports false otherwise. Hand-written dispatch
// with std::get would throw bad_variant_access the moment a new alternative
// appeared; this cannot, and a new alternative simply has to be
// equality-comparable or the variant stops compiling.
[[nodiscard]] inline bool sameSource(const EquationSource& lhs,
                                     const EquationSource& rhs) noexcept
{
    return lhs == rhs;
}

// lhs = rhs, stored as given. The model never performs lhs - rhs implicitly:
// subtracting would destroy which side came from the robot and which from the
// task, and that boundary cannot be recovered without algebraic decomposition
// -- which this project does not have. Normalisation belongs to the future
// simplifier.
//
// Both sides own their Expression by value. That is cheap (a shared_ptr
// handle, never a tree) and it is what makes a system a stable snapshot rather
// than a view into whatever the builder had on its stack.
struct Equation
{
    Expression lhs;
    Expression rhs;
    EquationSource source;
};

} // namespace kinemaforge::ik
