#pragma once

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
    Pose
};

// What a single equation constrains. Derived from EquationSource, never
// stored -- see kindOf below.
enum class EquationKind
{
    Position,
    Orientation
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

// A variant rather than {kind, optional<cell>}: a position equation has no way
// to name a rotation cell, and a rotation cell cannot exist without a kind
// that gives it meaning. Adding a directional constraint later forces every
// consumer using an explicit overload set to handle it, at compile time.
using EquationSource = std::variant<PositionEquationSource, OrientationEquationSource>;

namespace detail {

// Explicit overloads, no generic operator(): adding a third alternative to
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
};

} // namespace detail

[[nodiscard]] inline EquationKind kindOf(const EquationSource& source) noexcept
{
    return std::visit(detail::EquationKindVisitor{}, source);
}

// Identity of the constrained quantity. Two equations with equal sources
// address the same matrix cell -- either redundant or contradictory.
//
// Delegates to std::variant's own operator==, which compares alternatives only
// when the indices match and reports false otherwise. Hand-written dispatch
// with std::get would throw bad_variant_access the moment a third alternative
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
