# Proposal: model równań i targetów IK — implementacja

## Prompt

> Werdykt: APPROVE. Następny krok: `proposal-ik-equation-model-implementation.md` z pełnym kodem. Gate: clean build z `-Wall -Wextra -Wpedantic -Werror`, pomiar rotacji zaokrąglonych do 9 miejsc, próg `1e-8` bez cichej zmiany.

*(Gate mówił o 272/272; po dołożeniu dziewiątego kodu błędu i jego testu — §15.2 — obowiązuje **273/273**.)*

Realizacja architektury zatwierdzonej w `proposal-ik-equation-model-architecture.md` (v3, `APPROVE` po trzech rundach).

## Status weryfikacji

**Kod poniżej nie został skompilowany.** Proposal jest dokumentem; kod trafia na dysk po zatwierdzeniu.

Zweryfikowane przez lekturę repo: `JointVariable` to `{std::string name; std::size_t index;}`; `Expression` jest uchwytem nad `shared_ptr<const ExpressionNode>` z publicznym `sameNode`; `model/` zawiera dziś wyłącznie nagłówki; `<expected>` i `<span>` są już używane w `IkEquationBuilder`.

**§9 wymienia ryzyka**, a **§15 — poprawki z trzech rund review**, w tym jedno odstępstwo od nazwy przyjętej w architekturze.

---

## 1. Stan obecny

242/242 (dorobek TCP w drzewie roboczym, niezacommitowany). Po tej zmianie oczekiwane **273/273**.

`model/` nie ma jeszcze żadnego `.cpp`; ta zmiana dokłada dwa.

---

## 2. `src/ik_equations/model/Equation.hpp` (nowy plik)

```cpp
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
```

### 2.1 Dlaczego bez `Equation.cpp`

Cała logika w tym nagłówku — `create`, `kindOf`, `sameSource`, `EquationKindVisitor` — jest `inline`: metody zdefiniowane w ciele klasy niejawnie, funkcje wolne jawnie. Żadna nie robi nic poza wyborem alternatywy albo porównaniem, więc osobna jednostka translacji nie dałaby ani izolacji, ani skrócenia czasu kompilacji.

To **nie** koliduje z zasadą przyjętą przy `detail/PrincipalRotation` („nagłówki nie niosą definicji"): tamten problem dotyczył definicji **nie-`inline`** w nagłówku włączanym przez dwie jednostki, czyli naruszenia ODR. `inline` jest dokładnie mechanizmem, który to rozwiązuje.

---

## 3. `src/ik_equations/model/IkEquationSystem.hpp` (nowy plik)

```cpp
#pragma once

#include "ik_equations/model/Equation.hpp"
#include "ik_equations/model/JointVariable.hpp"

#include <expected>
#include <span>
#include <string>
#include <vector>

namespace kinemaforge::ik {

enum class IkEquationSystemErrorCode
{
    NoEquations,
    NoUnknowns,
    EmptyUnknownName,
    DuplicateUnknownName,
    DuplicateUnknownIndex,
    UnorderedUnknowns,
    TaskEquationMismatch,
    DuplicateEquationSource,
    UnorderedEquations
};

struct IkEquationSystemError
{
    IkEquationSystemErrorCode code{};
    std::string message;
};

// The input of the future EquationSimplifier and EquationSolver, and the
// output of the future ConstraintBuilder.
//
// Self-contained on purpose: it carries its own unknowns, so the pipeline
// IkEquationSystem -> EquationSimplifier -> EquationSolver is one object
// rather than three that have to be carried together. Recovering the unknowns
// from the equation trees is not an option -- there is no public symbol
// collector, DAG traversal order is not joint order, sorting by name gives
// q1, q10, q2, and a symbolic target parameter would be indistinguishable
// from a joint variable.
//
// Closed, because create() enforces nine invariants an aggregate cannot.
// Cheap to copy anyway: copying duplicates Expression handles, never trees.
class IkEquationSystem
{
public:
    // Validates in a fixed order, so each code has a deterministic meaning
    // when several invariants are broken at once:
    //
    //   NoEquations, NoUnknowns, EmptyUnknownName, DuplicateUnknownName,
    //   DuplicateUnknownIndex, UnorderedUnknowns, TaskEquationMismatch,
    //   DuplicateEquationSource, UnorderedEquations
    //
    // Two orderings are forced rather than arbitrary:
    //
    //   DuplicateUnknownIndex before UnorderedUnknowns -- two equal indices
    //   already break a strictly increasing sequence, so the reverse order
    //   would report a duplicate as bad ordering: true, but useless.
    //
    //   DuplicateEquationSource before UnorderedEquations -- for the same
    //   reason, and because two equations addressing one matrix cell are
    //   either redundant or contradictory, which is worth its own diagnosis.
    [[nodiscard]] static std::expected<IkEquationSystem, IkEquationSystemError>
    create(IkTaskKind taskKind,
           std::vector<JointVariable> unknowns,
           std::vector<Equation> equations);

    [[nodiscard]] IkTaskKind taskKind() const noexcept;

    // Ordered by ascending JointVariable::index -- part of the contract.
    [[nodiscard]] std::span<const JointVariable> unknowns() const noexcept;

    // Position X, Y, Z first; then orientation cells in row-major order.
    [[nodiscard]] std::span<const Equation> equations() const noexcept;

private:
    IkEquationSystem(IkTaskKind taskKind,
                     std::vector<JointVariable> unknowns,
                     std::vector<Equation> equations);

    IkTaskKind taskKind_;
    std::vector<JointVariable> unknowns_;
    std::vector<Equation> equations_;
};

} // namespace kinemaforge::ik
```

---

## 4. `src/ik_equations/model/IkEquationSystem.cpp` (nowy plik)

```cpp
#include "ik_equations/model/IkEquationSystem.hpp"

#include <cstddef>
#include <string>
#include <unordered_set>
#include <utility>

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
```

### 4.1 Trzy warstwy obrony przed nieprawidłowymi wartościami

Warto je odróżnić, bo żadna nie zastępuje pozostałych:

| Zagrożenie | Obrona | Kiedy działa |
|---|---|---|
| nowa alternatywa `EquationSource` | jawne zestawy przeciążeń w `std::visit` | kompilacja |
| nowy enumerator `IkTaskKind` | `switch` bez `default` → `-Wswitch` + `-Werror` | kompilacja |
| sfabrykowana wartość enuma (`static_cast<…>(99)`) | `componentIndex` i zwrot spoza `switch` | wykonanie |
| za krótki wektor równań | `size() < 3` w `checkEquationOrder` | wykonanie |

Ostatnia pozycja jest defensywna, nie dekoracyjna: bez niej funkcja byłaby bezpieczna wyłącznie dzięki temu, że `checkEquationContent` wykonało się wcześniej, a przestawienie dwóch linii w `create` zamieniłoby błąd walidacji w dostęp poza zakres.

---

## 5. `src/ik_equations/model/IkTarget.hpp` (nowy plik)

```cpp
#pragma once

#include "ik_equations/model/Vector3.hpp"

#include <array>
#include <variant>

namespace kinemaforge::ik {

// values[row][column], row-major, both indices 0..2 -- the same indexing
// OrientationEquationSource::row()/column() use.
//
// A plain aggregate: no operator(), no methods. SymbolicMatrix has an
// operator() because it bounds-checks; here there is nothing to check beyond
// what std::array already does.
//
// Validity is not a construction condition -- see TargetValidation.
struct RotationMatrix3
{
    std::array<std::array<double, 3>, 3> values{};
};

// A point expressed in the base frame of whatever SymbolicTransform the
// constraint builder is given. The model deliberately carries no frame name:
// it does not know, and must not know, whether that transform ends at the TCP
// or at the chain tip. Pairing the right target with the right transform is
// the caller's responsibility, and no type here can check it.
struct PositionTarget
{
    Vector3 position;   // metres
};

// T_base_target: the position and orientation of the target frame, expressed
// in the base frame. Not "a pose" -- the direction is part of the contract.
struct PoseTarget
{
    Vector3 position;             // metres
    RotationMatrix3 orientation;
};

// A closed set. A struct of optionals would admit a target that constrains
// nothing, and combinations nobody defined.
using IkTarget = std::variant<PositionTarget, PoseTarget>;

} // namespace kinemaforge::ik
```

---

## 6. `src/ik_equations/model/TargetValidation.hpp` (nowy plik)

```cpp
#pragma once

#include "ik_equations/model/IkTarget.hpp"

#include <expected>
#include <string>

namespace kinemaforge::ik {

enum class TargetValidationErrorCode
{
    NonFinitePosition,
    NonFiniteOrientation,
    NonOrthogonalOrientation,
    InvalidOrientationDeterminant
};

struct TargetValidationError
{
    TargetValidationErrorCode code{};
    std::string message;
};

// Absolute tolerance for accepting an input rotation matrix.
//
// NOT the 1e-12 used when comparing FK results: that bound was measured on
// error accumulated inside our own computation, this one accepts data that
// arrived from elsewhere. Measured on 500 000 random rotations rounded to nine
// decimal places, the worst deviations were |R^T R - I| = 1.68e-9 and
// |det - 1| = 1.94e-9, so 1e-8 leaves rather more than five times headroom.
//
// The supported input contract is double and text of reasonable precision.
// Data that has passed through float deviates by ~1e-6 and is rejected on
// purpose: the right fix is an explicit orthonormalisation on the caller's
// side, not a threshold loose enough to let a genuinely non-orthogonal target
// through and make the equation system quietly unsatisfiable.
inline constexpr double kOrientationTolerance = 1e-8;

// Checked in this order, and the order is observable:
//   1. all values finite      -> NonFiniteOrientation
//   2. |(R^T R - I)ij| <= tol -> NonOrthogonalOrientation
//   3. |det(R) - 1|    <= tol -> InvalidOrientationDeterminant
//
// Step 3 is NOT "therefore a reflection". That would hold for exactly
// orthogonal matrices, where det is +1 or -1 -- but step 2 accepts a
// tolerance, so a uniform scaling can slip through it and fail here with a
// positive determinant. diag(1+4e-9, 1+4e-9, 1+4e-9) deviates from
// orthogonality by 8e-9 (accepted) and from unit determinant by 1.2e-8
// (rejected), and is no reflection at all. Hence the neutral name.
[[nodiscard]] std::expected<void, TargetValidationError> validate(const PositionTarget& target);
[[nodiscard]] std::expected<void, TargetValidationError> validate(const RotationMatrix3& rotation);
[[nodiscard]] std::expected<void, TargetValidationError> validate(const PoseTarget& target);
[[nodiscard]] std::expected<void, TargetValidationError> validate(const IkTarget& target);

} // namespace kinemaforge::ik
```

---

## 7. `src/ik_equations/model/TargetValidation.cpp` (nowy plik)

```cpp
#include "ik_equations/model/TargetValidation.hpp"

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <variant>

namespace kinemaforge::ik {

namespace {

std::unexpected<TargetValidationError> makeError(TargetValidationErrorCode code,
                                                 std::string message)
{
    return std::unexpected(TargetValidationError{code, std::move(message)});
}

// std::to_string prints six decimals, so a determinant of 1 + 1.2e-8 -- the
// very value that caused the rejection -- would render as "1.000000". At these
// magnitudes the message has to carry full precision or it is worse than no
// message.
std::string format(double value)
{
    std::ostringstream stream;
    stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    return stream.str();
}

const char* describeAxis(std::size_t index) noexcept
{
    switch (index)
    {
    case 0: return "x";
    case 1: return "y";
    default: return "z";
    }
}

// Rejected, never repaired. Re-orthonormalising would change the target the
// caller asked for and hide their mistake -- the same reason the quaternion
// reference checks its norm instead of fixing it.
std::expected<void, TargetValidationError> checkFinite(const RotationMatrix3& rotation)
{
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            if (!std::isfinite(rotation.values[row][column]))
                return makeError(TargetValidationErrorCode::NonFiniteOrientation,
                                 "orientation (" + std::to_string(row) + ", " +
                                     std::to_string(column) + ") is not finite");
    return {};
}

std::expected<void, TargetValidationError> checkOrthogonal(const RotationMatrix3& rotation)
{
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
        {
            double product = 0.0;
            for (std::size_t k = 0; k < 3; ++k)
                product += rotation.values[k][i] * rotation.values[k][j];

            const double expected = (i == j) ? 1.0 : 0.0;
            if (std::abs(product - expected) > kOrientationTolerance)
                return makeError(TargetValidationErrorCode::NonOrthogonalOrientation,
                                 "(R^T R)(" + std::to_string(i) + ", " + std::to_string(j) +
                                     ") = " + format(product));
        }
    return {};
}

double determinant(const RotationMatrix3& rotation) noexcept
{
    const auto& m = rotation.values;
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

// Explicit overload set, not a generic lambda: adding a third alternative to
// IkTarget must fail to compile until somebody decides how to validate it.
struct TargetVisitor
{
    std::expected<void, TargetValidationError> operator()(const PositionTarget& target) const
    {
        return validate(target);
    }

    std::expected<void, TargetValidationError> operator()(const PoseTarget& target) const
    {
        return validate(target);
    }
};

} // namespace

std::expected<void, TargetValidationError> validate(const PositionTarget& target)
{
    const double components[3] = {target.position.x, target.position.y, target.position.z};
    for (std::size_t index = 0; index < 3; ++index)
        if (!std::isfinite(components[index]))
            return makeError(TargetValidationErrorCode::NonFinitePosition,
                             std::string("position ") + describeAxis(index) + " is not finite");
    return {};
}

std::expected<void, TargetValidationError> validate(const RotationMatrix3& rotation)
{
    if (auto checked = checkFinite(rotation); !checked)
        return checked;

    if (auto checked = checkOrthogonal(rotation); !checked)
        return checked;

    // Computed once, and NOT called a reflection: a matrix that passed the
    // tolerant orthogonality check can still be a slight uniform scaling with
    // a positive determinant.
    const double determinantValue = determinant(rotation);
    if (std::abs(determinantValue - 1.0) > kOrientationTolerance)
        return makeError(TargetValidationErrorCode::InvalidOrientationDeterminant,
                         "orientation determinant differs from +1: det = " +
                             format(determinantValue));

    return {};
}

std::expected<void, TargetValidationError> validate(const PoseTarget& target)
{
    if (auto checked = validate(PositionTarget{target.position}); !checked)
        return checked;

    return validate(target.orientation);
}

std::expected<void, TargetValidationError> validate(const IkTarget& target)
{
    return std::visit(TargetVisitor{}, target);
}

} // namespace kinemaforge::ik
```

---

## 8. Testy

### 8.1 `tests/test_ik_equation_model.cpp` (nowy plik) — 15 testów

```cpp
#include <gtest/gtest.h>

#include "ik_equations/model/Equation.hpp"
#include "ik_equations/model/IkEquationSystem.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using kinemaforge::ik::CartesianComponent;
using kinemaforge::ik::Equation;
using kinemaforge::ik::EquationKind;
using kinemaforge::ik::EquationSource;
using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::IkEquationSystem;
using kinemaforge::ik::IkEquationSystemErrorCode;
using kinemaforge::ik::IkTaskKind;
using kinemaforge::ik::JointVariable;
using kinemaforge::ik::OrientationEquationSource;
using kinemaforge::ik::PositionEquationSource;
using kinemaforge::ik::kindOf;
using kinemaforge::ik::sameNode;

namespace {

Equation positionEquation(const ExpressionFactory& factory, CartesianComponent component,
                          const char* symbol = "q1")
{
    return Equation{factory.symbol(symbol), factory.constant(1.0),
                    PositionEquationSource{component}};
}

Equation orientationEquation(const ExpressionFactory& factory, std::size_t row,
                             std::size_t column)
{
    const auto source = OrientationEquationSource::create(row, column);
    // Throws rather than EXPECT + dereference: a broken factory would
    // otherwise turn every test using this helper into UB instead of a
    // failure.
    if (!source)
        throw std::logic_error("valid orientation cell rejected by create()");
    return Equation{factory.symbol("q1"), factory.constant(0.0), *source};
}

std::vector<Equation> positionEquations(const ExpressionFactory& factory)
{
    return {positionEquation(factory, CartesianComponent::X),
            positionEquation(factory, CartesianComponent::Y),
            positionEquation(factory, CartesianComponent::Z)};
}

std::vector<JointVariable> twoUnknowns()
{
    return {JointVariable{"q1", 1}, JointVariable{"q2", 2}};
}

} // namespace

// --- Expression ownership -------------------------------------------

TEST(IkEquationModelTest, StoresEquationSidesByValue)
{
    // The equation must outlive the handles it was built from -- a system is a
    // snapshot, not a view into the builder's stack.
    const ExpressionFactory factory;

    Equation equation = [&factory] {
        const Expression lhs = factory.symbol("q1");
        const Expression rhs = factory.constant(2.5);
        return Equation{lhs, rhs, PositionEquationSource{CartesianComponent::X}};
    }();

    EXPECT_EQ(equation.lhs.type(), kinemaforge::ik::ExpressionType::Symbol);
    EXPECT_TRUE(kinemaforge::ik::isConstant(equation.rhs));
    EXPECT_DOUBLE_EQ(kinemaforge::ik::constantValue(equation.rhs), 2.5);
}

TEST(IkEquationModelTest, CopiedEquationSharesImmutableExpressionNodes)
{
    // By value is cheap: Expression is a handle over shared_ptr<const Node>,
    // so copying an equation copies two pointers, never two trees.
    const ExpressionFactory factory;
    const Equation original = positionEquation(factory, CartesianComponent::X);

    const Equation copy = original;

    EXPECT_TRUE(sameNode(original.lhs, copy.lhs));
    EXPECT_TRUE(sameNode(original.rhs, copy.rhs));
}

TEST(IkEquationModelTest, CopiesIkEquationSystemWithoutDeepCopyingExpressions)
{
    const ExpressionFactory factory;
    const auto original =
        IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(), positionEquations(factory));
    ASSERT_TRUE(original.has_value());

    const IkEquationSystem copy = *original;

    ASSERT_EQ(copy.equations().size(), original->equations().size());
    for (std::size_t index = 0; index < copy.equations().size(); ++index)
    {
        SCOPED_TRACE(testing::Message() << "equation " << index);
        EXPECT_TRUE(sameNode(original->equations()[index].lhs, copy.equations()[index].lhs));
        EXPECT_TRUE(sameNode(original->equations()[index].rhs, copy.equations()[index].rhs));
    }
}

// --- IkEquationSystem invariants ------------------------------------

TEST(IkEquationModelTest, CreatesValidSystem)
{
    const ExpressionFactory factory;

    const auto system =
        IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(), positionEquations(factory));

    ASSERT_TRUE(system.has_value());
    EXPECT_EQ(system->taskKind(), IkTaskKind::Position);
    EXPECT_EQ(system->unknowns().size(), 2u);
    EXPECT_EQ(system->equations().size(), 3u);
    EXPECT_EQ(kindOf(system->equations()[0].source), EquationKind::Position);
}

TEST(IkEquationModelTest, RejectsSystemWithoutEquations)
{
    {
        SCOPED_TRACE("equations only");
        const auto system = IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(), {});
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::NoEquations);
    }
    {
        // Both broken at once: NoEquations wins, per the documented order.
        // Without this the priority is a comment, not a contract.
        SCOPED_TRACE("equations and unknowns both empty");
        const auto system = IkEquationSystem::create(IkTaskKind::Position, {}, {});
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::NoEquations);
    }
}

TEST(IkEquationModelTest, RejectsSystemWithoutUnknowns)
{
    const ExpressionFactory factory;

    const auto system = IkEquationSystem::create(IkTaskKind::Position, {}, positionEquations(factory));

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::NoUnknowns);
}

TEST(IkEquationModelTest, RejectsUnknownWithEmptyName)
{
    const ExpressionFactory factory;
    std::vector<JointVariable> unknowns{JointVariable{"q1", 1}, JointVariable{"", 2}};

    const auto system = IkEquationSystem::create(IkTaskKind::Position, std::move(unknowns),
                                                 positionEquations(factory));

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::EmptyUnknownName);
}

TEST(IkEquationModelTest, RejectsDuplicateUnknownNames)
{
    const ExpressionFactory factory;

    {
        // Distinct indices, same name. For the symbolic layer "q1" is ONE
        // node, so the solver would get two supposedly different unknowns
        // pointing at it.
        SCOPED_TRACE("same name, distinct indices");
        std::vector<JointVariable> unknowns{JointVariable{"q1", 1}, JointVariable{"q1", 2}};
        const auto system = IkEquationSystem::create(IkTaskKind::Position, std::move(unknowns),
                                                     positionEquations(factory));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::DuplicateUnknownName);
    }
    {
        // Breaks three rules at once: duplicate name, duplicate index, and
        // non-increasing order. The name must win.
        SCOPED_TRACE("same name and same index");
        std::vector<JointVariable> unknowns{JointVariable{"q1", 1}, JointVariable{"q1", 1}};
        const auto system = IkEquationSystem::create(IkTaskKind::Position, std::move(unknowns),
                                                     positionEquations(factory));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::DuplicateUnknownName);
    }
}

TEST(IkEquationModelTest, RejectsDuplicateUnknownIndices)
{
    // Distinct names, same index. Must be reported as a duplicate rather than
    // as bad ordering -- equal indices also break the strict ordering rule, so
    // the check order is what makes this diagnosis useful.
    const ExpressionFactory factory;
    std::vector<JointVariable> unknowns{JointVariable{"q1", 1}, JointVariable{"q2", 1}};

    const auto system = IkEquationSystem::create(IkTaskKind::Position, std::move(unknowns),
                                                 positionEquations(factory));

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::DuplicateUnknownIndex);
}

TEST(IkEquationModelTest, RejectsUnorderedUnknowns)
{
    const ExpressionFactory factory;
    std::vector<JointVariable> unknowns{JointVariable{"q2", 2}, JointVariable{"q1", 1}};

    const auto system = IkEquationSystem::create(IkTaskKind::Position, std::move(unknowns),
                                                 positionEquations(factory));

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::UnorderedUnknowns);
}

TEST(IkEquationModelTest, RejectsTaskEquationKindMismatch)
{
    const ExpressionFactory factory;

    {
        SCOPED_TRACE("orientation equation in a position task");
        auto equations = positionEquations(factory);
        equations.push_back(orientationEquation(factory, 0, 0));

        const auto system = IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::TaskEquationMismatch);
    }
    {
        SCOPED_TRACE("pose task without orientation equations");
        const auto system = IkEquationSystem::create(IkTaskKind::Pose, twoUnknowns(),
                                                     positionEquations(factory));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::TaskEquationMismatch);
    }
    {
        SCOPED_TRACE("pose task missing a position component");
        std::vector<Equation> equations{positionEquation(factory, CartesianComponent::X),
                                        positionEquation(factory, CartesianComponent::Y),
                                        orientationEquation(factory, 0, 0)};

        const auto system = IkEquationSystem::create(IkTaskKind::Pose, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::TaskEquationMismatch);
    }
    {
        // A forged enumerator: a valid object of the enum type with no
        // enumerator behind it. Without componentIndex this would index
        // present[99].
        SCOPED_TRACE("invalid Cartesian component");
        auto equations = positionEquations(factory);
        equations[0].source = PositionEquationSource{static_cast<CartesianComponent>(99)};

        const auto system = IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::TaskEquationMismatch);
    }
    {
        // The same for the task kind: -Wswitch guards new enumerators at
        // compile time, this guards forged ones at run time.
        SCOPED_TRACE("invalid IK task kind");
        const auto system = IkEquationSystem::create(static_cast<IkTaskKind>(99), twoUnknowns(),
                                                     positionEquations(factory));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::TaskEquationMismatch);
    }
}

TEST(IkEquationModelTest, RejectsDuplicateEquationSource)
{
    // Two equations addressing one cell: redundant at best, contradictory at
    // worst. Reported as its own fault rather than falling out of the strict
    // ordering rule -- the document and the code must agree on duplicates.
    const ExpressionFactory factory;

    {
        SCOPED_TRACE("duplicate position component");
        std::vector<Equation> equations{positionEquation(factory, CartesianComponent::X),
                                        positionEquation(factory, CartesianComponent::Y),
                                        positionEquation(factory, CartesianComponent::Z),
                                        positionEquation(factory, CartesianComponent::Y)};

        const auto system = IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::DuplicateEquationSource);
    }
    {
        SCOPED_TRACE("duplicate orientation cell");
        auto equations = positionEquations(factory);
        equations.push_back(orientationEquation(factory, 1, 1));
        equations.push_back(orientationEquation(factory, 1, 1));

        const auto system = IkEquationSystem::create(IkTaskKind::Pose, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::DuplicateEquationSource);
    }
}

TEST(IkEquationModelTest, RejectsUnorderedEquations)
{
    const ExpressionFactory factory;

    {
        SCOPED_TRACE("position components out of order");
        std::vector<Equation> equations{positionEquation(factory, CartesianComponent::Z),
                                        positionEquation(factory, CartesianComponent::X),
                                        positionEquation(factory, CartesianComponent::Y)};

        const auto system = IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::UnorderedEquations);
    }
    {
        SCOPED_TRACE("orientation cells not row-major");
        auto equations = positionEquations(factory);
        equations.push_back(orientationEquation(factory, 1, 0));
        equations.push_back(orientationEquation(factory, 0, 1));

        const auto system = IkEquationSystem::create(IkTaskKind::Pose, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::UnorderedEquations);
    }
}

// --- EquationSource -------------------------------------------------

TEST(IkEquationModelTest, CreatesOrientationSourceForValidCell)
{
    const auto corner = OrientationEquationSource::create(0, 0);
    ASSERT_TRUE(corner.has_value());
    EXPECT_EQ(corner->row(), 0u);
    EXPECT_EQ(corner->column(), 0u);

    const auto opposite = OrientationEquationSource::create(2, 2);
    ASSERT_TRUE(opposite.has_value());
    EXPECT_EQ(opposite->row(), 2u);
    EXPECT_EQ(opposite->column(), 2u);

    // The other half of kindOf: only the position side was covered above.
    EXPECT_EQ(kindOf(EquationSource{*corner}), EquationKind::Orientation);
}

TEST(IkEquationModelTest, RejectsOutOfRangeOrientationSource)
{
    // create is the ONLY way to build one: the constructor is private, so
    // there is no OrientationEquationSource{7, 9} to reject at all.
    EXPECT_FALSE(OrientationEquationSource::create(3, 0).has_value());
    EXPECT_FALSE(OrientationEquationSource::create(0, 3).has_value());
    EXPECT_FALSE(OrientationEquationSource::create(7, 9).has_value());
}
```

### 8.2 `tests/test_target_validation.cpp` (nowy plik) — 16 testów

```cpp
#include <gtest/gtest.h>

#include "ik_equations/model/TargetValidation.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

using kinemaforge::ik::IkTarget;
using kinemaforge::ik::PoseTarget;
using kinemaforge::ik::PositionTarget;
using kinemaforge::ik::RotationMatrix3;
using kinemaforge::ik::TargetValidationErrorCode;
using kinemaforge::ik::Vector3;
using kinemaforge::ik::kOrientationTolerance;
using kinemaforge::ik::validate;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

RotationMatrix3 identity()
{
    RotationMatrix3 rotation{};
    rotation.values[0][0] = 1.0;
    rotation.values[1][1] = 1.0;
    rotation.values[2][2] = 1.0;
    return rotation;
}

RotationMatrix3 rotationAboutZ(double angle)
{
    RotationMatrix3 rotation{};
    rotation.values[0][0] = std::cos(angle);
    rotation.values[0][1] = -std::sin(angle);
    rotation.values[1][0] = std::sin(angle);
    rotation.values[1][1] = std::cos(angle);
    rotation.values[2][2] = 1.0;
    return rotation;
}

// Scaling the first axis by (1 + epsilon) makes (R^T R)(0,0) = (1 + epsilon)^2,
// so the orthogonality deviation is 2*epsilon to first order. That gives exact
// control over which side of kOrientationTolerance the matrix lands on, while
// keeping |det - 1| = epsilon, i.e. half the deviation -- so orthogonality is
// always the check that decides, as the documented order requires.
RotationMatrix3 withOrthogonalityDeviation(double deviation)
{
    RotationMatrix3 rotation = identity();
    rotation.values[0][0] = 1.0 + 0.5 * deviation;
    return rotation;
}

} // namespace

// --- position -------------------------------------------------------

TEST(TargetValidationTest, AcceptsFinitePositionTarget)
{
    EXPECT_TRUE(validate(PositionTarget{Vector3{1.6, 0.0, 2.335}}).has_value());
}

TEST(TargetValidationTest, RejectsNonFinitePositionTargetX)
{
    for (const double value : {kNaN, kInfinity, -kInfinity})
    {
        SCOPED_TRACE(testing::Message() << "x = " << value);
        const auto result = validate(PositionTarget{Vector3{value, 0.0, 0.0}});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFinitePosition);
        EXPECT_FALSE(result.error().message.empty());
    }
}

TEST(TargetValidationTest, RejectsNonFinitePositionTargetY)
{
    for (const double value : {kNaN, kInfinity, -kInfinity})
    {
        SCOPED_TRACE(testing::Message() << "y = " << value);
        const auto result = validate(PositionTarget{Vector3{0.0, value, 0.0}});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFinitePosition);
    }
}

TEST(TargetValidationTest, RejectsNonFinitePositionTargetZ)
{
    for (const double value : {kNaN, kInfinity, -kInfinity})
    {
        SCOPED_TRACE(testing::Message() << "z = " << value);
        const auto result = validate(PositionTarget{Vector3{0.0, 0.0, value}});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFinitePosition);
    }
}

// --- orientation ----------------------------------------------------

TEST(TargetValidationTest, AcceptsIdentityRotation)
{
    EXPECT_TRUE(validate(identity()).has_value());
}

TEST(TargetValidationTest, AcceptsValidRotationMatrix)
{
    EXPECT_TRUE(validate(rotationAboutZ(0.7)).has_value());
}

TEST(TargetValidationTest, RejectsNonFiniteRotationMatrix)
{
    // All nine cells, not one: a loop that skipped the first or last row would
    // otherwise pass while leaving a hole in the check.
    for (const double value : {kNaN, kInfinity, -kInfinity})
        for (std::size_t row = 0; row < 3; ++row)
            for (std::size_t column = 0; column < 3; ++column)
            {
                SCOPED_TRACE(testing::Message() << "value = " << value << " at (" << row
                                                << ", " << column << ")");
                RotationMatrix3 rotation = identity();
                rotation.values[row][column] = value;

                const auto result = validate(rotation);
                ASSERT_FALSE(result.has_value());
                EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFiniteOrientation);
            }
}

TEST(TargetValidationTest, RejectsNonOrthogonalRotationMatrix)
{
    // diag(1, 1, 0.5): scaling. It also has det != 1, but orthogonality is
    // checked first, so THIS is the code it must produce.
    RotationMatrix3 rotation = identity();
    rotation.values[2][2] = 0.5;

    const auto result = validate(rotation);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonOrthogonalOrientation);
}

TEST(TargetValidationTest, RejectsInvalidOrientationDeterminant)
{
    {
        // diag(1, 1, -1): a reflection. Exactly orthogonal, so it reaches the
        // determinant check.
        SCOPED_TRACE("reflection, det = -1");
        RotationMatrix3 rotation = identity();
        rotation.values[2][2] = -1.0;

        const auto result = validate(rotation);
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code,
                  TargetValidationErrorCode::InvalidOrientationDeterminant);
    }
    {
        // The case that makes "reflection" the wrong name: a uniform scaling
        // by (1 + 4e-9) deviates from orthogonality by ~8e-9 (inside the
        // tolerance) and from unit determinant by ~1.2e-8 (outside it), with a
        // POSITIVE determinant.
        SCOPED_TRACE("uniform scaling, det > 1 but not a reflection");
        RotationMatrix3 rotation{};
        const double scale = 1.0 + 4e-9;
        rotation.values[0][0] = scale;
        rotation.values[1][1] = scale;
        rotation.values[2][2] = scale;

        const auto result = validate(rotation);
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code,
                  TargetValidationErrorCode::InvalidOrientationDeterminant);
    }
}

TEST(TargetValidationTest, AcceptsRotationJustWithinTolerance)
{
    // Deviations far from the threshold would not pin it: any implementation
    // using anything between them would pass. These two sit on either side.
    EXPECT_TRUE(validate(withOrthogonalityDeviation(0.9 * kOrientationTolerance)).has_value());
}

TEST(TargetValidationTest, RejectsRotationJustBeyondTolerance)
{
    const auto result = validate(withOrthogonalityDeviation(1.1 * kOrientationTolerance));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonOrthogonalOrientation);
}

// --- pose -----------------------------------------------------------

TEST(TargetValidationTest, AcceptsValidPoseTarget)
{
    EXPECT_TRUE(validate(PoseTarget{Vector3{1.6, 0.0, 2.335}, rotationAboutZ(0.7)}).has_value());
}

TEST(TargetValidationTest, RejectsPoseTargetWithNonFinitePosition)
{
    const auto result = validate(PoseTarget{Vector3{0.0, kNaN, 0.0}, identity()});

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFinitePosition);
}

TEST(TargetValidationTest, RejectsPoseTargetWithInvalidOrientation)
{
    RotationMatrix3 reflection = identity();
    reflection.values[2][2] = -1.0;

    const auto result = validate(PoseTarget{Vector3{1.0, 0.0, 0.0}, reflection});

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::InvalidOrientationDeterminant);
}

// --- IkTarget dispatcher --------------------------------------------

TEST(TargetValidationTest, ValidatesPositionTargetThroughIkTarget)
{
    EXPECT_TRUE(validate(IkTarget{PositionTarget{Vector3{1.0, 2.0, 3.0}}}).has_value());

    const auto result = validate(IkTarget{PositionTarget{Vector3{kNaN, 0.0, 0.0}}});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFinitePosition);
}

TEST(TargetValidationTest, ValidatesPoseTargetThroughIkTarget)
{
    EXPECT_TRUE(
        validate(IkTarget{PoseTarget{Vector3{1.0, 0.0, 0.0}, rotationAboutZ(0.3)}}).has_value());

    RotationMatrix3 scaling = identity();
    scaling.values[0][0] = 0.5;

    const auto result = validate(IkTarget{PoseTarget{Vector3{}, scaling}});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonOrthogonalOrientation);
}
```

---

## 9. Ryzyka

### 9.1 Konstrukcja testów progowych — ryzyko **średnie**

`withOrthogonalityDeviation` opiera się na tym, że skalowanie pierwszej osi o `(1 + ε)` daje `(RᵀR)(0,0) = (1 + ε)²`, czyli odchylenie `2ε + ε²`. Dla `ε ≈ 4.5e-9` człon kwadratowy to `~2e-17` — poniżej rozdzielczości `double` przy tej wartości, więc odchylenie jest praktycznie `2ε`.

**Wyprowadzenie jest ręczne i nieskompilowane.** Jeżeli `AcceptsRotationJustWithinTolerance` albo `RejectsRotationJustBeyondTolerance` failuje, przyczyną jest najprawdopodobniej ta arytmetyka, a nie próg — ale **nie poprawię progu**, tylko konstrukcję macierzy, i zaraportuję rzeczywiste odchylenie.

### 9.2 `std::expected<void, E>` i `return checked;` — ryzyko niskie

W `validate(const RotationMatrix3&)` zwracam `checked` bezpośrednio z gałęzi błędu. Typ zgadza się (`std::expected<void, TargetValidationError>` w obie strony), ale to pierwsze miejsce w projekcie, gdzie propaguję `expected<void, …>` przez zwrot zmiennej zamiast `std::unexpected(...)`.

---

## 10. Pomiar progu — obowiązkowy przy wdrożeniu

Gate z review wymaga powtórzenia pomiaru. Wykonam:

1. losowanie `N = 500 000` macierzy rotacji (kwaternion z rozkładu jednostajnego na `S³`, konwersja do macierzy);
2. zaokrąglenie każdej komórki do **dziewięciu miejsc po przecinku**;
3. maksimum z `|(RᵀR − I)ᵢⱼ|` i `|det − 1|`.

Oczekiwane: `~1.68e-9` i `~1.94e-9` (pomiar z review). **Zaraportuję rzeczywiste liczby.** Rozjazd o rząd wielkości byłby ustaleniem do review, nie powodem do zmiany `kOrientationTolerance`.

Program pomiarowy jest jednorazowy i **nie wchodzi do repo** — jak `fk_probe` przy walidacji FK.

---

## 11. CMake

`CMakeLists.txt` — dwie linie:

```cmake
    src/ik_equations/model/IkEquationSystem.cpp
    src/ik_equations/model/TargetValidation.cpp
```

`tests/CMakeLists.txt` — dwie linie:

```cmake
    test_ik_equation_model.cpp
    test_target_validation.cpp
```

---

## 12. `STATUS.md` — zmiany

1. Nagłówek `242/242` → `273/273`.
2. Diagram — dopisać `IK equation model` za `TCP transform`, jako drugi element Fazy 2.
3. „Done" — nowy wpis: `Equation` bez niejawnej normalizacji, `EquationSource` jako wariant, `IkEquationSystem` z dziewięcioma inwariantami i własną listą niewiadomych, targety, próg `1e-8` z uzasadnieniem różnicy wobec `1e-12`.
4. Known gaps — **nowy wiersz**: target nie zna frame'u, więc sparowanie targetu TCP z transformacją do końca łańcucha jest niewykrywalne przez typ (§5 architektury, ta sama klasa luki co kierunek `FixedRigidTransform`).
5. Known gaps — **nowy wiersz**: dane orientacji po `float` są odrzucane świadomie; właściwą drogą jest jawne `orthonormalize` po stronie wołającego.
6. Tabela dokumentów — architektura `approved (v3)`, ten proposal `implemented`.
7. Next step — F2.4, `ConstraintBuilder` dla `PositionOnly`.

---

## 13. Zgodność z zatwierdzoną architekturą

| Decyzja (v3) | Gdzie |
|---|---|
| `Equation{lhs, rhs, source}` jako agregat, bez normalizacji | §2 |
| `IkTaskKind` oddzielone od `EquationKind` | §2 |
| `EquationSource` jako wariant | §2 |
| `OrientationEquationSource` domknięty, prywatny konstruktor | §2 |
| `kindOf` jawnie w `Equation.hpp` | §2 |
| `IkEquationSystem` domknięty, `create` z **dziewięcioma** inwariantami (ósmy i dziewiąty: §15) | §3, §4 |
| ustalona kolejność sprawdzeń, `DuplicateUnknownIndex` przed `UnorderedUnknowns` | §4 |
| dokładnie trzy równania pozycyjne w obu zadaniach | §4 `checkEquationContent` |
| liczba równań orientacji otwarta | j.w. — sprawdzana obecność, nie liczność |
| rozdział „wadliwa zawartość" / „wadliwa kolejność" | §4 — dwie osobne funkcje, dwa kody |
| uporządkowana lista niewiadomych | §3, §4 |
| `RotationMatrix3` jako agregat, `values[row][column]`, row-major | §5 |
| brak adapterów kwaternion/RPY | §5, §6 — nie ma ich w ogóle |
| `IkTarget` = `variant<PositionTarget, PoseTarget>` | §5 |
| `PoseTarget` = `T_base_target` | §5, komentarz |
| kolejność walidacji: skończoność → ortogonalność → wyznacznik | §7 |
| **odstępstwo:** `ImproperRotation` → `InvalidOrientationDeterminant` | §15 — nazwa z architektury opierała się na fałszywej przesłance |
| próg `1e-8` z uzasadnieniem | §6, `kOrientationTolerance` |
| odrzucanie zamiast normalizacji | §7, komentarz |
| cztery przeciążenia `validate` | §6, §7 |
| dispatcher z jawnym zestawem przeciążeń | §7 `TargetVisitor` |
| testy progu tuż przy granicy | §8.2 `withOrthogonalityDeviation` |

---

## 14. Jak to zweryfikuję po zatwierdzeniu

```powershell
cmake -B build -G Ninja
cmake --build build --clean-first
ctest --test-dir build --output-on-failure
.\build\tests\kinemaforge_tests.exe --gtest_filter=IkEquationModelTest.*
.\build\tests\kinemaforge_tests.exe --gtest_filter=TargetValidationTest.*
```

Oczekiwane: **242 + 31 = 273 zielone** (15 + 16). Plus pomiar z §10, zaraportowany niezależnie od wyniku.

## 15. Poprawki z review — wprowadzone

Review wyodrębniło pokazane pliki do rzeczywistego drzewa i skompilowało je w C++23 z `-Wall -Wextra -Wpedantic -Werror`, plus harness potwierdzający konstrukcję testów progowych (`0.9×` i `1.0×` przechodzą, `1.1×` odpada).

| # | Zarzut | Werdykt | Gdzie |
|---|---|---|---|
| 1 | zły wyznacznik nie zawsze oznacza odbicie | **przyjęty — blocker** | §6, §7, §8.2 |
| 2 | `kindOf` i `checkEquationContent` nie wymuszają obsługi nowej alternatywy | **przyjęty — blocker** | §2, §4 |
| 3 | semantyka duplikatów źródeł przeczy architekturze | **przyjęty — blocker** | §4, §8.1 |
| 4 | bezpieczeństwo zależne od kolejności helperów | **przyjęty** | §4 `checkEquationOrder` |
| 5 | testy nie pinują priorytetu błędów | **przyjęty** | §8.1 |
| 6 | walidacja nieskończoności testowana na jednej komórce | **przyjęty** | §8.2 |
| 7 | `kindOf` dla orientacji, wyznacznik liczony dwa razy, `EXPECT` + dereferencja w helperze | **przyjęty** | §7, §8.1 |

### 15.1 Odstępstwo od architektury — nazwa `ImproperRotation`

Architektura (v3, §9.3) ustaliła nazwę `ImproperRotation` wraz z uzasadnieniem: *„po przejściu kroku 2 macierz jest ortogonalna, więc `det ∈ {+1, −1}`"*. **To uzasadnienie jest fałszywe**, bo krok 2 akceptuje ortogonalność **z tolerancją**.

Kontrprzykład z review: `diag(1+4e-9, 1+4e-9, 1+4e-9)` odchyla się od ortogonalności o `~8e-9` (przechodzi) i od jedynkowego wyznacznika o `~1.2e-8` (odpada), przy wyznaczniku **dodatnim**. Nie jest odbiciem, a poprzedni kod nazwałby go tak i wypisał `det = 1.000000`, bo `std::to_string` zaokrągla do sześciu miejsc — czyli komunikat ukrywałby wartość, która spowodowała odrzucenie.

Wracam więc do neutralnego `InvalidOrientationDeterminant` i wprowadzam `format()` z `max_digits10`. Zgłaszam to jako **odstępstwo od zatwierdzonej architektury**, nie jako szczegół implementacyjny: zmienia nazwę publicznego kodu błędu.

Przy okazji ten sam problem dotyczył komunikatu ortogonalności — `std::to_string(product)` dla `product = 1 + 8e-9` też drukowało `1.000000`. Poprawione.

### 15.2 Dwa razy ten sam mechanizm, dwa razy przeoczony (druga runda)

Zarzuty 2 i 3 mają wspólne źródło: **argumentem za wariantem była wymuszona obsługa nowych alternatyw, a napisałem trzy miejsca, które ją omijają** — `kindOf` przez `holds_alternative`, `checkEquationContent` przez `if position else orientation`, oraz `switch` bez wyczerpania na `IkTaskKind`. Wariant, którego konsumenci używają `else`, jest zwykłym enumem z dodatkowym kosztem.

Zarzut 3 jest natomiast rozjazdem między dokumentami: architektura mówiła „duplikat nie jest błędem strukturalnym", a implementacja odrzucała duplikaty **dwiema różnymi drogami z dwoma różnymi kodami**. Wybieram jawny zakaz z własnym kodem `DuplicateEquationSource` — dwa równania na tę samą komórkę są w najlepszym razie nadmiarowe, w najgorszym sprzeczne. To dziewiąty inwariant i dziewiąty kod.

### 15.3 Trzecia runda — poprawki z gotowym patchem

| Zarzut | Werdykt |
|---|---|
| `sameSource` z ręcznym `std::get` rzuci `bad_variant_access` po rozszerzeniu wariantu | **przyjęty** — `operator==` domyślne na obu alternatywach, `sameSource` sprowadzone do `lhs == rhs` |
| `present[static_cast<std::size_t>(component)]` zapisuje poza tablicą dla sfabrykowanego enumeratora | **przyjęty** — `componentIndex` ze `switch` zwracającym `expected` |
| `switch` na `IkTaskKind` przepuszcza wartość spoza zbioru enumeratorów | **przyjęty** — każdy poprawny przypadek zwraca, wartość sfabrykowana wypada poniżej |
| brak `using EquationSource` w teście | przyjęty |

Rozwiązanie z `operator== = default` jest **lepsze niż kolejny visitor**, którego bym napisał: różne alternatywy są różne z definicji `std::variant`, nie ma `std::get` na niewłaściwej alternatywie, a nowy typ źródła **musi** być porównywalny albo wariant przestaje się kompilować. Jedna deklaracja zamiast trzech gałęzi.

Warto nazwać wzorzec, który łączy dwa pierwsze zarzuty: **`enum class` nie jest typem zbioru wartości.** `static_cast<CartesianComponent>(99)` daje poprawny obiekt typu wyliczeniowego bez odpowiadającego enumeratora, więc indeksowanie nim tablicy i poleganie na wyczerpaniu `switch` to dwie różne obrony — pierwsza chroni przed wartością sfabrykowaną w czasie wykonania, druga przed nowym enumeratorem w czasie kompilacji. Napisałem drugą i uznałem, że pokrywa pierwszą.

## Do zatwierdzenia

1. Kod (§2–§7) i testy (§8) po siedmiu poprawkach.
2. **§15.1 — odstępstwo od architektury**: `ImproperRotation` → `InvalidOrientationDeterminant`, bo nazwa z architektury opierała się na przesłance fałszywej przy tolerancyjnej ortogonalności.
3. **§15.2 — dziewiąty inwariant** `DuplicateEquationSource` i jawny zakaz duplikatów, zamiast zapisu z architektury, że duplikat nie jest błędem strukturalnym.

**31 testów (15 + 16), oczekiwany stan 273.** Bez commita.
