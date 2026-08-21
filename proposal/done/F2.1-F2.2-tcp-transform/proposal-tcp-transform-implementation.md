# Proposal: stała transformacja TCP — implementacja

## Prompt

> Werdykt: APPROVE. Trzy decyzje z §15 rozstrzygnięte: ekstrakcja RPY wariant B, publiczne `assembleTransform`, `PrincipalRotation` z osobnym `.cpp`. Następny dokument powinien już zawierać pełny kod implementacji i dokładnie 242 oczekiwane testy.

Realizacja architektury zatwierdzonej w `proposal-tcp-transform-architecture.md` (v3, werdykt `APPROVE` po dwóch rundach `REQUEST CHANGES`).

## Status weryfikacji

**Kod poniżej nie został skompilowany.** Proposal jest dokumentem; kod trafia na dysk po zatwierdzeniu.

Zweryfikowane przez lekturę repo: `Vector3` w `UrdfJoint.hpp` (linie 15–20), `UrdfJoint.hpp` włączany przez dwa pliki, `symbolic/` bez zależności od `model/`, dwie kopie `assembleTransform`, helpery RPY prywatne w `JointTransformBuilder.cpp`, `sameNode` publiczne w `Expression.hpp`, główny `CMakeLists.txt` wymieniający źródła jawnie.

**§5 opisuje decyzję, której architektura nie przewidziała.** Nie wprowadzam jej po cichu.

---

## 1. Stan obecny

221/221. Po tej zmianie oczekiwane **242/242**.

`IkEquationBuilder` ma trzy pola `optional`, cztery kody błędu, pięć metod. `Vector3` mieszka w `UrdfJoint.hpp`. `assembleTransform` istnieje w dwóch anonimowych kopiach (`SymbolicTransform.cpp`, `JointTransformBuilder.cpp`).

---

## 2. Nowe pliki modelu

### 2.1 `src/ik_equations/model/Vector3.hpp`

```cpp
#pragma once

namespace kinemaforge::ik {

struct Vector3
{
    double x{};
    double y{};
    double z{};
};

} // namespace kinemaforge::ik
```

### 2.2 `src/ik_equations/model/UrdfJoint.hpp` — zmiana

Usunąć definicję `struct Vector3` (linie 15–20), dodać na górze:

```cpp
#include "ik_equations/model/Vector3.hpp"
```

Reszta pliku bez zmian. `KinematicChain.hpp` i `RobotDescription.hpp` dostają `Vector3` tranzytywnie tak jak dotąd — **nie wymagają zmian**.

### 2.3 `src/ik_equations/model/FixedRigidTransform.hpp`

```cpp
#pragma once

#include "ik_equations/model/Vector3.hpp"

namespace kinemaforge::ik {

// A constant rigid transform T_parent_child between two frames.
//
//   translation : position of the child frame's origin, expressed in parent
//                 unit: metres
//   rpy         : orientation of child relative to parent
//                 unit: radians
//                 R = Rz(yaw) * Ry(pitch) * Rx(roll)
//                 -- URDF's fixed-axis convention, the same one joint origins
//                    use, so a value read off a URDF and a value typed by hand
//                    mean the same thing
//
// The direction is part of the contract and the type cannot enforce it:
// nothing stops a caller from filling this with T_child_parent, and no
// compiler will notice. For a TCP, `parent` is the tip of the currently
// selected kinematic chain and `child` is the tool centre point.
//
// Validity is exactly "all six values are finite". There is no other
// condition -- any six finite numbers describe a rigid transform, which is
// why this representation needs no tolerance to validate.
struct FixedRigidTransform
{
    Vector3 translation;
    Vector3 rpy;
};

} // namespace kinemaforge::ik
```

---

## 3. Warstwa symboliczna — upublicznienie `assembleTransform`

### 3.1 `SymbolicTransform.hpp` — dodane deklaracje

Po `isIdentityTransform`, przed `multiplyTransforms`:

```cpp
// Builds [R p; 0 1] from its blocks. The last row is exactly [0 0 0 1] by
// construction -- the result starts as identity and nothing writes to row 3.
//
// Public because three call sites need it and two of them carried their own
// copy. Producing the same thing by multiplying two full 4x4 matrices would
// turn the last row into 0*R00 + 0*R10 + 0*R20: zero mathematically, but no
// longer a Constant(0) that isZero() recognises.
SymbolicTransform assembleTransform(const SymbolicRotation& rotation,
                                    const SymbolicVector3& translation);
```

### 3.2 `SymbolicTransform.cpp` — zmiana

Przenieść istniejące `assembleTransform` **z anonimowej przestrzeni nazw** do przestrzeni `kinemaforge::ik`. Ciało bez zmian. Reszta pliku bez zmian.

---

## 4. Nowe pliki w `builders/`

### 4.1 `src/ik_equations/builders/detail/PrincipalRotation.hpp`

```cpp
#pragma once

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

namespace kinemaforge::ik::detail {

enum class PrincipalAxis
{
    X,
    Y,
    Z
};

// Rotation about one principal axis.
//
// `negated` folds the sign into the sine rather than wrapping the result,
// since R(-a, q) = R(a, -q) and cos is even: without it the cells would carry
// Negate(Negate(Sin)).
//
// An implementation detail shared by JointTransformBuilder and
// RigidTransformConstruction, deliberately outside the module's surface: a
// consumer has no reason to know about `negated`.
SymbolicRotation makePrincipalRotation(PrincipalAxis axis,
                                       bool negated,
                                       const Expression& angle,
                                       const ExpressionFactory& factory);

} // namespace kinemaforge::ik::detail
```

### 4.2 `src/ik_equations/builders/detail/PrincipalRotation.cpp`

Ciało przeniesione **bez zmian** z `JointTransformBuilder.cpp` (linie 31–59), z podmienioną nazwą i przestrzenią:

```cpp
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
```

### 4.3 `src/ik_equations/builders/RigidTransformConstruction.hpp`

```cpp
#pragma once

#include "ik_equations/model/FixedRigidTransform.hpp"
#include "ik_equations/model/Vector3.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

namespace kinemaforge::ik {

// R_rpy = Rz(yaw) * Ry(pitch) * Rx(roll) -- URDF's fixed-axis convention.
//
// The single production implementation of that convention. JointTransformBuilder
// uses it for joint origins, buildFixedRigidTransform for constant frame
// offsets. A second copy would be a second place for the convention to drift,
// and STATUS.md records that nothing external verifies it -- so a drift there
// is precisely the kind this project cannot currently detect.
SymbolicRotation makeRpyRotation(const Vector3& rpy, const ExpressionFactory& factory);

// A plain Vector3 as a column of symbolic constants.
SymbolicVector3 toSymbolicVector(const Vector3& value, const ExpressionFactory& factory);

// A constant homogeneous transform from translation + rpy.
//
// Every cell folds to a constant during construction, so this introduces no
// symbolic variables. The result satisfies hasCanonicalHomogeneousLastRow by
// construction.
SymbolicTransform buildFixedRigidTransform(const FixedRigidTransform& transform,
                                           const ExpressionFactory& factory);

} // namespace kinemaforge::ik
```

### 4.4 `src/ik_equations/builders/RigidTransformConstruction.cpp`

```cpp
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
```

### 4.5 `src/ik_equations/builders/JointTransformBuilder.cpp` — zmiana

Plik ma 303 linie; zmienia się jego początek i trzy wywołania. Wypisuję dokładnie, zamiast powielać 240 niezmienionych linii.

**Dodane include'y** (po istniejącym `#include "ik_equations/builders/JointTransformBuilder.hpp"`):

```cpp
#include "ik_equations/builders/RigidTransformConstruction.hpp"
#include "ik_equations/builders/detail/PrincipalRotation.hpp"
```

**Usunięte z anonimowej przestrzeni nazw:**

| Element | Linie | Zastąpione przez |
|---|---|---|
| `enum class PrincipalAxis` | 29 | `detail::PrincipalAxis` |
| `buildPrincipalRotation` | 31–59 | `detail::makePrincipalRotation` |
| `buildRpyRotation` | 67–74 | `makeRpyRotation` |
| `toSymbolicVector` | 192–199 | `toSymbolicVector` (teraz z nagłówka) |
| `assembleTransform` | 215–226 | `assembleTransform` (teraz z `SymbolicTransform.hpp`) |

**Pozostają bez zmian** w anonimowej przestrzeni: `kUnitAxisTolerance`, `isUnitLength`, `isIdentityRotation`, `PrincipalAxisMatch`, `matchPrincipalAxis`, `buildAxisAngleRotation`, `scaledAxisComponent`, `buildPrismaticDisplacement`, `addVectors`, `composeRotations`, `rotateVector`, `isActuated`.

**Podmiany w wywołaniach:**

| Było | Jest |
|---|---|
| `struct PrincipalAxisMatch { PrincipalAxis axis; bool negated; };` | `struct PrincipalAxisMatch { detail::PrincipalAxis axis; bool negated; };` |
| `PrincipalAxisMatch{PrincipalAxis::X, false}` *(i pięć analogicznych)* | `PrincipalAxisMatch{detail::PrincipalAxis::X, false}` |
| `buildPrincipalRotation(principal->axis, principal->negated, variable, factory)` | `detail::makePrincipalRotation(principal->axis, principal->negated, variable, factory)` |
| `buildRpyRotation(joint.origin.rpy, factory_)` | `makeRpyRotation(joint.origin.rpy, factory_)` |

`build()` poza jedną z tych podmian **jest niezmieniony**, łącznie z obiema asercjami i wszystkimi fast pathami.

---

## 5. Decyzja, której architektura nie przewidziała

Przy pisaniu testów wyszło, że `test_tcp_transform.cpp` potrzebuje dokładnie tych samych narzędzi, co `test_numeric_fk_validation.cpp`: typu `Matrix4`, ewaluacji `SymbolicTransform` **jednym** evaluatorem, konwersji `RigidTransform → Matrix4`, porównania z tolerancją `1e-12` i wydruku `[ MEASURE ]`. To około **80 linii**.

| Wariant | Ocena |
|---|---|
| skopiować helpery do drugiego pliku testowego | **odrzucony** — dwie kopie progu `1e-12` i dwie kopie pętli ewaluacyjnej; zmiana tolerancji w jednym miejscu cicho rozjechałaby oba zestawy |
| **wydzielić do `tests/support/TransformComparison.hpp/.cpp`** | **przyjęty** |
| przenieść porównanie do produkcji | odrzucony — to rusztowanie testowe, nie funkcja biblioteki |

Konsekwencja: `test_numeric_fk_validation.cpp` traci swoje lokalne helpery i włącza nowy nagłówek. To **modyfikacja kodu testowego zatwierdzonego dwa etapy wcześniej**, więc zgłaszam ją jawnie zamiast wprowadzać przy okazji. Zasięg jest ograniczony i w pełni pokryty: 17 testów walidacji numerycznej wykryje każdy błąd tej ekstrakcji.

**Do rozstrzygnięcia w review.** Jeżeli odpadnie, wchodzi duplikacja i dokument dostaje wiersz w known gaps.

### 5.1 `tests/support/TransformComparison.hpp`

```cpp
#pragma once

#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <array>
#include <optional>
#include <string_view>

namespace kinemaforge::testsupport {

using Matrix4 = std::array<std::array<double, 4>, 4>;

// Candidate tolerance, measured once and approved: the worst observed error
// across 18 matrix comparisons was 5.55e-16. Exceeding it is a finding for
// review, NOT a licence to raise the number.
inline constexpr double kAbsoluteTolerance = 1e-12;
inline constexpr double kRelativeTolerance = 1e-12;

bool withinTolerance(double actual, double expected);

Matrix4 toMatrix4(const RigidTransform& transform);

// One evaluator for all sixteen cells -- that is the whole reason
// ExpressionEvaluator is a session. Returns nullopt and records a gtest
// failure if any cell fails to evaluate.
std::optional<Matrix4> evaluateSymbolic(const ik::SymbolicTransform& transform,
                                        const ik::SymbolValues& values);

// Per-cell comparison; prints the worst rotation and translation error plus
// the orientation angle unconditionally, so reports can quote the run.
void expectMatrixMatches(const Matrix4& actual, const Matrix4& expected,
                         std::string_view label);

} // namespace kinemaforge::testsupport
```

### 5.2 `tests/support/TransformComparison.cpp`

Nie da się przenieść ciał dosłownie: w `test_numeric_fk_validation.cpp` wołane są przez alias `support::`, którego w środku `namespace kinemaforge::testsupport` nie ma, a `ADD_FAILURE`, `SCOPED_TRACE` i `testing::Message` wymagają `<gtest/gtest.h>`. Pełna treść:

```cpp
#include "support/TransformComparison.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>

namespace kinemaforge::testsupport {

namespace {

// R_error = R_expected^T * R_actual ; angle = acos((trace - 1) / 2).
//
// Diagnostic only -- the pass/fail condition stays per-cell. This answers the
// question a single cell difference cannot: is the discrepancy a real
// orientation error, or noise in one entry?
double orientationAngleError(const Matrix4& actual, const Matrix4& expected)
{
    // trace(R_expected^T * R_actual) = sum_i sum_k R_expected(k,i) * R_actual(k,i)
    double trace = 0.0;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t k = 0; k < 3; ++k)
            trace += expected[k][i] * actual[k][i];

    // clamp: with errors near 1e-16 the argument can leave [-1, 1].
    return std::acos(std::clamp(0.5 * (trace - 1.0), -1.0, 1.0));
}

} // namespace

bool withinTolerance(double actual, double expected)
{
    return std::abs(actual - expected)
           <= kAbsoluteTolerance + kRelativeTolerance * std::abs(expected);
}

Matrix4 toMatrix4(const RigidTransform& transform)
{
    // Unqualified: this translation unit is already inside
    // kinemaforge::testsupport, where toRotationMatrix lives.
    const Matrix3 rotation = toRotationMatrix(transform.rotation);

    Matrix4 result{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            result[row][column] = rotation[row][column];

    result[0][3] = transform.translation.x;
    result[1][3] = transform.translation.y;
    result[2][3] = transform.translation.z;
    result[3][3] = 1.0;
    return result;
}

std::optional<Matrix4> evaluateSymbolic(const ik::SymbolicTransform& transform,
                                        const ik::SymbolValues& values)
{
    // One evaluator for all sixteen cells -- that is the whole reason
    // ExpressionEvaluator is a session. A per-cell evaluator would drop the
    // cache between roots.
    ik::ExpressionEvaluator evaluator{values};

    Matrix4 numeric{};
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            const auto value = evaluator.evaluate(transform(row, column));
            if (!value)
            {
                ADD_FAILURE() << "evaluation failed at cell (" << row << ", " << column
                              << "), code " << static_cast<int>(value.error().code)
                              << " symbol '" << value.error().symbolName << "'";
                return std::nullopt;
            }
            numeric[row][column] = *value;
        }
    return numeric;
}

void expectMatrixMatches(const Matrix4& actual, const Matrix4& expected,
                         std::string_view label)
{
    double maxRotationError = 0.0;
    double maxTranslationError = 0.0;

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message()
                         << label << " cell (" << row << ", " << column << ")");

            const double error = std::abs(actual[row][column] - expected[row][column]);
            if (row < 3 && column < 3)
                maxRotationError = std::max(maxRotationError, error);
            else if (row < 3)
                maxTranslationError = std::max(maxTranslationError, error);

            EXPECT_TRUE(withinTolerance(actual[row][column], expected[row][column]))
                << "actual=" << actual[row][column]
                << " expected=" << expected[row][column]
                << " error=" << error;
        }

    // Printed unconditionally: the implementation report quotes these.
    std::cout << "[ MEASURE  ] " << label
              << "  rotation=" << maxRotationError
              << "  translation=" << maxTranslationError
              << "  angle=" << orientationAngleError(actual, expected) << "\n";
}

} // namespace kinemaforge::testsupport
```

### 5.3 `tests/test_numeric_fk_validation.cpp` — zmiana

Usunąć z anonimowej przestrzeni: `Matrix4`, `kAbsoluteTolerance`, `kRelativeTolerance`, `withinTolerance`, `toMatrix4`, `orientationAngleError`, `evaluateSymbolic`, `expectMatrixMatches`, `nearLimitConfiguration`. Dodać:

```cpp
#include "support/TransformComparison.hpp"

using support::Matrix4;
using support::evaluateSymbolic;
using support::expectMatrixMatches;
using support::nearLimitConfiguration;
using support::toMatrix4;
```

Bez „itd." — pełna lista, bo brak choćby jednej pozycji daje błąd kompilacji w połowie pliku. **Wszystkie 17 testów bez zmian w treści.**

---

## 6. Rozszerzenie referencji kwaternionowej

### 6.1 `tests/support/NumericForwardKinematics.hpp` — dodana deklaracja

```cpp
// Forward kinematics to the TCP, composed the same way the symbolic side does
// it but in a different representation:
//
//     T_base_tcp = T_base_tip * T_tip_tcp
//
// compose() implements p = p_a + rotate(q_a, p_b), which is exactly the
// tool-frame semantics -- so this validates the composition order
// independently, with quaternions instead of matrices.
RigidTransform numericTcpForwardKinematics(const ik::KinematicChain& chain,
                                           const JointConfiguration& configuration,
                                           const ik::FixedRigidTransform& tcp);
```

Plus `#include "ik_equations/model/FixedRigidTransform.hpp"`.

### 6.3 `nearLimitConfiguration` przenoszony do wspólnego wsparcia

Żyje dziś w anonimowej przestrzeni `test_numeric_fk_validation.cpp`, a §9 potrzebuje go w drugim pliku. Ta sama logika co w §5: druga kopia oznaczałaby dwa miejsca definiujące „5% zakresu". Przenosimy do `NumericForwardKinematics.hpp`, obok `JointConfiguration`, które ten nagłówek już deklaruje:

```cpp
// Alternating lower + 5% of span, upper - 5% of span, derived from the loaded
// model rather than hard-coded. Deliberately not the exact limits: this
// validates kinematics, not boundary handling.
JointConfiguration nearLimitConfiguration(const ik::KinematicChain& chain);
```

Ciało przeniesione z pliku testowego, bez zmian:

```cpp
JointConfiguration nearLimitConfiguration(const ik::KinematicChain& chain)
{
    JointConfiguration configuration;
    bool useLower = true;

    for (const auto& joint : chain.joints)
    {
        if (!joint.variable) continue;

        const double span = joint.limits.upper - joint.limits.lower;
        configuration.push_back(useLower ? joint.limits.lower + 0.05 * span
                                         : joint.limits.upper - 0.05 * span);
        useLower = !useLower;
    }
    return configuration;
}
```

`test_numeric_fk_validation.cpp` traci lokalną kopię i dostaje `using support::nearLimitConfiguration;` obok pozostałych — §5.3.

### 6.2 `.cpp` — dodana definicja

```cpp
RigidTransform numericTcpForwardKinematics(const ik::KinematicChain& chain,
                                           const JointConfiguration& configuration,
                                           const ik::FixedRigidTransform& tcp)
{
    return compose(numericForwardKinematics(chain, configuration),
                   RigidTransform{fromRollPitchYaw(tcp.rpy),
                                  toVector3d(tcp.translation)});
}
```

---

## 7. `IkEquationBuilder.hpp` — zmiany

**Dodane include'y:** `"ik_equations/model/FixedRigidTransform.hpp"`.

**Enum — trzy nowe wartości:**

```cpp
enum class IkEquationBuilderErrorCode
{
    RobotModelNotLoaded,
    KinematicChainNotSelected,
    UrdfLoadFailed,
    ChainBuildFailed,
    ForwardKinematicsNotBuilt,
    TcpNotSet,
    InvalidTcpTransform
};
```

**Komentarz klasy — akapit o wskaźnikach zastąpiony:**

```cpp
// Accessors return nullptr until the corresponding step has succeeded.
//
// A pointer obtained earlier must not be treated as access to the previous
// result after an operation that changes its node -- see the table below.
// This is a semantic contract, not an address one: assigning into an already
// engaged std::optional constructs in place, so the address can survive while
// the value behind it becomes a different result. That is worse than a
// dangling pointer, because nothing crashes; the caller simply reads the new
// value believing it holds the old one.
//
//     kinematicChain()       stale after loadRobotModel, selectChain
//     forwardKinematics()    ... plus buildForwardKinematics
//     tcp()                  after loadRobotModel, selectChain, setTcp, clearTcp
//     tcpForwardKinematics() after any of the six operations
//
// The converse is a promise, and it is tested: setTcp does NOT affect
// forwardKinematics(), and buildForwardKinematics does NOT affect tcp().
```

**Dodane metody** (po `buildForwardKinematics`):

```cpp
    // A TCP offset is defined relative to the tip of the currently selected
    // chain, so a chain must be selected first. Requires only that; it may be
    // called before or after buildForwardKinematics, because the TCP does not
    // depend on the transform.
    [[nodiscard]] std::expected<void, IkEquationBuilderError>
    setTcp(const FixedRigidTransform& tcp);

    // Idempotent, and valid in any state: before a chain is selected, with no
    // TCP set, and repeatedly. Afterwards tcp() and tcpForwardKinematics() are
    // both null.
    void clearTcp() noexcept;

    // T_base_tcp = T_base_chain_tip * T_chain_tip_tcp.
    //
    // Reports the FIRST missing prerequisite -- chain, then transform, then
    // TCP -- so the caller is told the step to take next rather than the last
    // one in the sequence.
    [[nodiscard]] std::expected<void, IkEquationBuilderError>
    buildTcpForwardKinematics();
```

**Dodane akcesory:**

```cpp
    [[nodiscard]] const FixedRigidTransform* tcp() const noexcept;
    [[nodiscard]] const SymbolicTransform* tcpForwardKinematics() const noexcept;
```

**Dodane pola:**

```cpp
    std::optional<FixedRigidTransform> tcp_;
    std::optional<SymbolicTransform> tcpForwardKinematics_;
```

---

## 8. `IkEquationBuilder.cpp` — zmiany

**Dodane include'y:**

```cpp
#include "ik_equations/builders/RigidTransformConstruction.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <cmath>
```

**Dodane do anonimowej przestrzeni nazw:**

```cpp
// Names the first offending component so the message is actionable; the
// structured code alone cannot say which of the six numbers was wrong.
const char* firstNonFiniteComponent(const FixedRigidTransform& transform) noexcept
{
    if (!std::isfinite(transform.translation.x)) return "translation x";
    if (!std::isfinite(transform.translation.y)) return "translation y";
    if (!std::isfinite(transform.translation.z)) return "translation z";
    if (!std::isfinite(transform.rpy.x))         return "rpy roll";
    if (!std::isfinite(transform.rpy.y))         return "rpy pitch";
    if (!std::isfinite(transform.rpy.z))         return "rpy yaw";
    return nullptr;
}
```

**`loadRobotModel` — commit rozszerzony o dwie linie:**

```cpp
    robotDescription_ = std::move(loaded);
    kinematicChain_.reset();
    forwardKinematics_.reset();
    tcp_.reset();
    tcpForwardKinematics_.reset();
    return {};
```

**`selectChain` — commit rozszerzony o dwie linie:**

```cpp
    kinematicChain_ = std::move(*selected);
    forwardKinematics_.reset();
    tcp_.reset();                 // a TCP offset names a frame of one specific tip
    tcpForwardKinematics_.reset();
    return {};
```

**`buildForwardKinematics` — jedna linia dodana, `tcp_` NIETKNIĘTY:**

```cpp
    forwardKinematics_ = fkBuilder_.build(*kinematicChain_, jointTransformBuilder_);
    // Rebuilding creates fresh nodes, so a retained T_base_tcp would point at
    // the previous tree. The TCP itself is not a descendant of the transform
    // and survives.
    tcpForwardKinematics_.reset();
    return {};
```

**Nowe definicje:**

```cpp
std::expected<void, IkEquationBuilderError>
IkEquationBuilder::setTcp(const FixedRigidTransform& tcp)
{
    if (!kinematicChain_)
        return makeError(IkEquationBuilderErrorCode::KinematicChainNotSelected,
                         "no kinematic chain selected; a TCP offset is defined "
                         "relative to the chain tip");

    if (const char* offending = firstNonFiniteComponent(tcp))
        return makeError(IkEquationBuilderErrorCode::InvalidTcpTransform,
                         std::string("tcp ") + offending + " is not finite");

    // Validated before the commit: a rejected update leaves the previous TCP
    // and the previous T_base_tcp untouched.
    tcp_ = tcp;
    tcpForwardKinematics_.reset();
    return {};
}

void IkEquationBuilder::clearTcp() noexcept
{
    tcp_.reset();
    tcpForwardKinematics_.reset();
}

std::expected<void, IkEquationBuilderError>
IkEquationBuilder::buildTcpForwardKinematics()
{
    if (!kinematicChain_)
        return makeError(IkEquationBuilderErrorCode::KinematicChainNotSelected,
                         "no kinematic chain selected; call selectChain first");

    if (!forwardKinematics_)
        return makeError(IkEquationBuilderErrorCode::ForwardKinematicsNotBuilt,
                         "forward kinematics not built; call buildForwardKinematics first");

    if (!tcp_)
        return makeError(IkEquationBuilderErrorCode::TcpNotSet,
                         "no TCP set; call setTcp first");

    // Local and stateless. ExpressionFactory has no members, so a facade field
    // would share nothing while looking as though it did -- see the
    // value-semantics entry in STATUS.md.
    const ExpressionFactory factory;

    const SymbolicTransform fixed = buildFixedRigidTransform(*tcp_, factory);
    tcpForwardKinematics_ = multiplyTransforms(*forwardKinematics_, fixed, factory);
    return {};
}

const FixedRigidTransform* IkEquationBuilder::tcp() const noexcept
{
    return tcp_ ? &*tcp_ : nullptr;
}

const SymbolicTransform* IkEquationBuilder::tcpForwardKinematics() const noexcept
{
    return tcpForwardKinematics_ ? &*tcpForwardKinematics_ : nullptr;
}
```

---

## 9. `tests/test_tcp_transform.cpp` — 21 testów

Szkielet i wybrane testy w całości; pozostałe różnią się wyłącznie danymi i asercją.

```cpp
#include <gtest/gtest.h>

#include "ik_equations/IkEquationBuilder.hpp"
#include "ik_equations/model/FixedRigidTransform.hpp"
#include "support/NumericForwardKinematics.hpp"
#include "support/TransformComparison.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace ik = kinemaforge::ik;
namespace support = kinemaforge::testsupport;

using ik::FixedRigidTransform;
using ik::IkEquationBuilder;
using ik::IkEquationBuilderErrorCode;
using ik::sameNode;
using support::JointConfiguration;
using support::Matrix4;

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

// Drives the facade up to a built T_base_tip.
void prepare(IkEquationBuilder& builder, const char* fileName)
{
    ASSERT_TRUE(builder.loadRobotModel(urdfPath(fileName)).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());
}

FixedRigidTransform translationTcp(double x, double y, double z)
{
    return FixedRigidTransform{{x, y, z}, {}};
}

// The same nine configurations Phase 1 validated FK against: zero, six
// single-joint poses, mixed, and near-limits. Checking only zero and mixed
// would leave the document's claim ("the same configurations as Phase 1")
// wider than what the tests actually cover.
std::vector<JointConfiguration> tcpValidationConfigurations(const ik::KinematicChain& chain)
{
    std::vector<JointConfiguration> configurations;

    configurations.emplace_back(6, 0.0);

    for (std::size_t index = 0; index < 6; ++index)
    {
        JointConfiguration single(6, 0.0);
        single[index] = (index % 2 == 0) ? 0.25 : -0.25;
        configurations.push_back(std::move(single));
    }

    configurations.push_back({0.35, -0.45, 0.55, -0.65, 0.40, -0.30});
    configurations.push_back(support::nearLimitConfiguration(chain));

    return configurations;
}

// Nine configurations against the quaternion reference, for one robot.
void expectTcpMatchesReference(IkEquationBuilder& builder, const char* fileName,
                               const FixedRigidTransform& tcp)
{
    prepare(builder, fileName);
    ASSERT_TRUE(builder.setTcp(tcp).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto* chain = builder.kinematicChain();
    ASSERT_NE(chain, nullptr);
    const auto* symbolic = builder.tcpForwardKinematics();
    ASSERT_NE(symbolic, nullptr);

    std::size_t index = 0;
    for (const auto& configuration : tcpValidationConfigurations(*chain))
    {
        SCOPED_TRACE(testing::Message() << fileName << " configuration " << index);

        const auto numeric =
            support::evaluateSymbolic(*symbolic, support::makeSymbolValues(*chain, configuration));
        ASSERT_TRUE(numeric.has_value());

        const Matrix4 reference = support::toMatrix4(
            support::numericTcpForwardKinematics(*chain, configuration, tcp));

        support::expectMatrixMatches(
            *numeric, reference,
            std::string(fileName) + " tcp cfg " + std::to_string(index));
        ++index;
    }
}

} // namespace

// --- composition ----------------------------------------------------

TEST(TcpTransformTest, IdentityTcpLeavesForwardKinematicsUnchanged)
{
    // "adds no nodes" is a claim about node identity, not about shape: a tree
    // rebuilt from scratch in the same shape would satisfy structurallyEqual.
    // multiplyTransforms literally returns lhs for an identity rhs, so
    // sameNode is the assertion that actually pins it.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    ASSERT_TRUE(builder.setTcp(FixedRigidTransform{}).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto* tip = builder.forwardKinematics();
    const auto* tcp = builder.tcpForwardKinematics();
    ASSERT_NE(tip, nullptr);
    ASSERT_NE(tcp, nullptr);

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            EXPECT_TRUE(sameNode((*tip)(row, column), (*tcp)(row, column)));
        }
}

TEST(TcpTransformTest, AppliesTcpTranslationInToolFrame)
{
    // The single most important test of this stage.
    //
    // kr640 at q1 = pi/2 has R_base_tip = Rz(pi/2) and p_base_tip =
    // (0, 1.600, 2.335), both hand-computed and already pinned by the Phase 1
    // validation. A TCP of (0.1, 0, 0) is expressed in the TOOL frame, so:
    //
    //   p_base_tcp = (0, 1.600, 2.335) + Rz(pi/2) * (0.1, 0, 0)
    //              = (0, 1.700, 2.335)
    //
    // Adding the offset in the base frame instead would give
    // (0.1, 1.600, 2.335) -- a difference in a different axis, so this cannot
    // pass by accident. The expectation depends on neither FK implementation.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    ASSERT_TRUE(builder.setTcp(translationTcp(0.1, 0.0, 0.0)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    JointConfiguration configuration(6, 0.0);
    configuration[0] = kPi / 2.0;

    const auto* chain = builder.kinematicChain();
    ASSERT_NE(chain, nullptr);

    const auto numeric = support::evaluateSymbolic(
        *builder.tcpForwardKinematics(), support::makeSymbolValues(*chain, configuration));
    ASSERT_TRUE(numeric.has_value());

    Matrix4 expected{};
    expected[0][0] = 0.0; expected[0][1] = -1.0; expected[0][2] = 0.0; expected[0][3] = 0.000;
    expected[1][0] = 1.0; expected[1][1] =  0.0; expected[1][2] = 0.0; expected[1][3] = 1.700;
    expected[2][0] = 0.0; expected[2][1] =  0.0; expected[2][2] = 1.0; expected[2][3] = 2.335;
    expected[3][3] = 1.0;

    support::expectMatrixMatches(*numeric, expected, "kr640 q1=pi/2 + tcp x0.1 (hand computed)");
}

TEST(TcpTransformTest, AppliesTranslationOnlyTcp)
{
    // At q = 0 every kr640 transform is a pure translation, so a tool-frame
    // offset adds directly: (1.600, 0, 2.335) + (0, 0, 0.05).
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto numeric = support::evaluateSymbolic(
        *builder.tcpForwardKinematics(),
        support::makeSymbolValues(*builder.kinematicChain(), JointConfiguration(6, 0.0)));
    ASSERT_TRUE(numeric.has_value());

    Matrix4 expected{};
    expected[0][0] = expected[1][1] = expected[2][2] = expected[3][3] = 1.0;
    expected[0][3] = 1.600;
    expected[2][3] = 2.385;

    support::expectMatrixMatches(*numeric, expected, "kr640 zero + tcp z0.05");
}

TEST(TcpTransformTest, AppliesRotationOnlyTcp)
{
    // Rotation about Z by pi/2 at q = 0, where R_base_tip is identity, so the
    // result is exactly the TCP rotation and the position is unchanged.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    ASSERT_TRUE(builder.setTcp(FixedRigidTransform{{}, {0.0, 0.0, kPi / 2.0}}).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto numeric = support::evaluateSymbolic(
        *builder.tcpForwardKinematics(),
        support::makeSymbolValues(*builder.kinematicChain(), JointConfiguration(6, 0.0)));
    ASSERT_TRUE(numeric.has_value());

    Matrix4 expected{};
    expected[0][1] = -1.0;
    expected[1][0] =  1.0;
    expected[2][2] =  1.0;
    expected[0][3] = 1.600;
    expected[2][3] = 2.335;
    expected[3][3] = 1.0;

    support::expectMatrixMatches(*numeric, expected, "kr640 zero + tcp Rz(pi/2)");
}

TEST(TcpTransformTest, AppliesCombinedTcp)
{
    // Both at once, checked against the quaternion reference rather than by
    // hand -- the point here is that translation and rotation compose, which
    // the two tests above verify separately.
    const FixedRigidTransform tcp{{0.05, -0.02, 0.13}, {0.2, -0.3, 0.4}};
    const JointConfiguration configuration{0.35, -0.45, 0.55, -0.65, 0.40, -0.30};

    IkEquationBuilder builder;
    prepare(builder, "kr4_r600.urdf");
    ASSERT_TRUE(builder.setTcp(tcp).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto numeric = support::evaluateSymbolic(
        *builder.tcpForwardKinematics(),
        support::makeSymbolValues(*builder.kinematicChain(), configuration));
    ASSERT_TRUE(numeric.has_value());

    const Matrix4 reference = support::toMatrix4(
        support::numericTcpForwardKinematics(*builder.kinematicChain(), configuration, tcp));

    support::expectMatrixMatches(*numeric, reference, "kr4 mixed + combined tcp");
}

TEST(TcpTransformTest, PreservesCanonicalHomogeneousLastRow)
{
    IkEquationBuilder builder;
    prepare(builder, "kr4_r600.urdf");
    ASSERT_TRUE(builder.setTcp(FixedRigidTransform{{0.05, -0.02, 0.13},
                                                   {0.2, -0.3, 0.4}}).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto* result = builder.tcpForwardKinematics();
    ASSERT_NE(result, nullptr);
    EXPECT_TRUE(ik::hasCanonicalHomogeneousLastRow(*result));
}

// --- state graph ----------------------------------------------------

TEST(TcpTransformTest, ChangingTcpInvalidatesTcpForwardKinematics)
{
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());
    ASSERT_NE(builder.tcpForwardKinematics(), nullptr);

    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.09)).has_value());

    EXPECT_EQ(builder.tcpForwardKinematics(), nullptr);
}

TEST(TcpTransformTest, ClearingTcpInvalidatesTcpForwardKinematics)
{
    // Valid in any state, including before a chain is selected -- the contract
    // says clearTcp must not depend on the chain, and void/noexcept alone does
    // not guarantee that.
    {
        IkEquationBuilder fresh;
        fresh.clearTcp();
        EXPECT_EQ(fresh.tcp(), nullptr);
        EXPECT_EQ(fresh.tcpForwardKinematics(), nullptr);
    }

    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    builder.clearTcp();

    EXPECT_EQ(builder.tcp(), nullptr);
    EXPECT_EQ(builder.tcpForwardKinematics(), nullptr);

    // Idempotent: a second call, and a call with nothing set, both succeed.
    builder.clearTcp();
    EXPECT_EQ(builder.tcp(), nullptr);
}

TEST(TcpTransformTest, RebuildingForwardKinematicsInvalidatesTcpForwardKinematicsButPreservesTcp)
{
    // The TCP is not a descendant of the transform in the dependency graph.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    EXPECT_NE(builder.tcp(), nullptr);
    EXPECT_EQ(builder.tcpForwardKinematics(), nullptr);
}

TEST(TcpTransformTest, SettingTcpPreservesForwardKinematics)
{
    // The other side of the graph: T_base_tip does not depend on the TCP, so
    // its pointer stays valid AND keeps pointing at the same result.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    const auto* fkBefore = builder.forwardKinematics();
    ASSERT_NE(fkBefore, nullptr);

    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());

    EXPECT_EQ(builder.forwardKinematics(), fkBefore);
}

TEST(TcpTransformTest, ChangingChainInvalidatesTcpAndTcpForwardKinematics)
{
    // The same three numbers would mean an offset from a DIFFERENT physical
    // frame after the tip changes, so keeping them would be a plausible wrong
    // answer.
    IkEquationBuilder builder;
    prepare(builder, "kr4_r600.urdf");
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    ASSERT_TRUE(builder.selectChain("base_link", "base").has_value());

    EXPECT_EQ(builder.tcp(), nullptr);
    EXPECT_EQ(builder.tcpForwardKinematics(), nullptr);
}

TEST(TcpTransformTest, LoadingNewRobotInvalidatesTcpAndTcpForwardKinematics)
{
    IkEquationBuilder builder;
    prepare(builder, "kr4_r600.urdf");
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());

    EXPECT_EQ(builder.tcp(), nullptr);
    EXPECT_EQ(builder.tcpForwardKinematics(), nullptr);
}

// --- errors ---------------------------------------------------------

TEST(TcpTransformTest, RejectsTcpBeforeChainSelection)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());

    const auto result = builder.setTcp(translationTcp(0.0, 0.0, 0.05));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::KinematicChainNotSelected);
    EXPECT_EQ(builder.tcp(), nullptr);
}

TEST(TcpTransformTest, RejectsTcpForwardKinematicsBeforeChainSelection)
{
    // Pins the prerequisite ORDER, not just the failure: an implementation
    // checking the transform first would return ForwardKinematicsNotBuilt here
    // and still pass every other test in this file.
    IkEquationBuilder builder;

    const auto result = builder.buildTcpForwardKinematics();

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::KinematicChainNotSelected);
}

TEST(TcpTransformTest, RejectsTcpForwardKinematicsBeforeForwardKinematics)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());

    const auto result = builder.buildTcpForwardKinematics();

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::ForwardKinematicsNotBuilt);
}

TEST(TcpTransformTest, RejectsTcpForwardKinematicsWithoutTcp)
{
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    const auto result = builder.buildTcpForwardKinematics();

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::TcpNotSet);
}

TEST(TcpTransformTest, RejectsNonFiniteTcpTranslation)
{
    // All three components, all three non-finite values. The implementation
    // has one branch per component, so checking only z would let a dropped
    // check on x through unnoticed.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    for (const double value : {kNaN, kInfinity, -kInfinity})
        for (const auto& tcp : {FixedRigidTransform{{value, 0.0, 0.0}, {}},
                                FixedRigidTransform{{0.0, value, 0.0}, {}},
                                FixedRigidTransform{{0.0, 0.0, value}, {}}})
        {
            SCOPED_TRACE(testing::Message()
                         << "translation (" << tcp.translation.x << ", "
                         << tcp.translation.y << ", " << tcp.translation.z << ")");

            const auto result = builder.setTcp(tcp);

            ASSERT_FALSE(result.has_value());
            EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::InvalidTcpTransform);
            EXPECT_FALSE(result.error().message.empty());
        }
}

TEST(TcpTransformTest, RejectsNonFiniteTcpRotation)
{
    // Roll, pitch and yaw, all three non-finite values -- same reasoning.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    for (const double value : {kNaN, kInfinity, -kInfinity})
        for (const auto& tcp : {FixedRigidTransform{{}, {value, 0.0, 0.0}},
                                FixedRigidTransform{{}, {0.0, value, 0.0}},
                                FixedRigidTransform{{}, {0.0, 0.0, value}}})
        {
            SCOPED_TRACE(testing::Message() << "rpy (" << tcp.rpy.x << ", " << tcp.rpy.y
                                            << ", " << tcp.rpy.z << ")");

            const auto result = builder.setTcp(tcp);

            ASSERT_FALSE(result.has_value());
            EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::InvalidTcpTransform);
            EXPECT_FALSE(result.error().message.empty());
        }
}

TEST(TcpTransformTest, FailedTcpUpdatePreservesPreviousState)
{
    // Validation and the transactional guarantee are different contracts, so
    // this is a separate test: pointer identity proves the object is
    // untouched, not merely still populated.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto* tcpBefore = builder.tcp();
    const auto* resultBefore = builder.tcpForwardKinematics();
    ASSERT_NE(tcpBefore, nullptr);
    ASSERT_NE(resultBefore, nullptr);

    const auto result = builder.setTcp(translationTcp(0.0, 0.0, kNaN));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(builder.tcp(), tcpBefore);
    EXPECT_EQ(builder.tcpForwardKinematics(), resultBefore);
    EXPECT_DOUBLE_EQ(builder.tcp()->translation.z, 0.05);
}

// --- real robots against the quaternion reference -------------------

TEST(TcpTransformTest, BuildsKr4TcpForwardKinematics)
{
    IkEquationBuilder builder;
    expectTcpMatchesReference(builder, "kr4_r600.urdf",
                              FixedRigidTransform{{0.05, -0.02, 0.13}, {0.2, -0.3, 0.4}});
}

TEST(TcpTransformTest, BuildsKr640TcpForwardKinematics)
{
    IkEquationBuilder builder;
    expectTcpMatchesReference(builder, "kr640.urdf",
                              FixedRigidTransform{{0.05, -0.02, 0.13}, {0.2, -0.3, 0.4}});
}
```

Dwa testy GTest, **dziewięć konfiguracji każdy — 18 porównań macierzy**, tyle samo co w Fazie 1.

---

## 10. CMake

`CMakeLists.txt` — **dwie nowe linie** (plik wymienia źródła jawnie):

```cmake
    src/ik_equations/builders/JointTransformBuilder.cpp
    src/ik_equations/builders/ForwardKinematicsBuilder.cpp
    src/ik_equations/builders/RigidTransformConstruction.cpp
    src/ik_equations/builders/detail/PrincipalRotation.cpp
```

`tests/CMakeLists.txt` — dwie linie:

```cmake
    test_tcp_transform.cpp
    support/TransformComparison.cpp
```

---

## 11. `STATUS.md` i `README.md`

1. Nagłówek `221/221` → `242/242`.
2. Diagram pipeline'u — dopisać `TCP transform` za `IkEquationBuilder`, jako pierwszy element Fazy 2.
3. „Done" — nowy wpis: `FixedRigidTransform` z kontraktem `T_parent_child`, graf zależności, kolejność prerekwizytów, tożsamościowy TCP za darmo dzięki fast pathowi.
4. Known gaps — **nowy wiersz** o kierunku transformacji: typ nie umie wymusić `T_parent_child`, wykryje to dopiero test.
5. Tabela dokumentów — architektura `approved (v3)`, ten proposal `implemented`.
6. Next step — F2.3, model równań i targetów.
7. `README.md` — wzmianka o TCP w sekcji komponentów; przykład użycia **bez zmian**, bo TCP jest opcjonalne.

---

## 12. Ryzyka

| Ryzyko | Ocena | Reakcja |
|---|---|---|
| ekstrakcja RPY psuje `JointTransformBuilder` | **średnie** | 22 + 17 testów, w tym `NumericReferenceUsesCorrectRpyOrder` i dwa ręczne oracle |
| ekstrakcja helperów testowych (§5) psuje walidację numeryczną | **średnie** | 17 testów; przeniesienie jest mechaniczne, ciała bez zmian |
| `AppliesRotationOnlyTcp` — oczekiwana macierz | niskie | przy `q = 0` `R_base_tip` jest jednostkowe, więc wynik to sam TCP; gdyby jednak `R_base_tip` nie było jednostkowe, test failuje i **zgłoszę to jako ustalenie**, nie poprawię oczekiwania |
| `detail/PrincipalRotation.hpp` włączany przez dwie jednostki | niskie | osobny `.cpp`, zero definicji w nagłówku |
| kolejność `tcp_.reset()` w `selectChain` | niskie | reset po commicie łańcucha, w tej samej sekcji co pozostałe |

---

## 13. Zgodność z zatwierdzoną architekturą

| Decyzja (v3) | Gdzie |
|---|---|
| `FixedRigidTransform{translation, rpy}` z kontraktem `T_parent_child`, metry, radiany | §2.3 |
| `Vector3` wydzielone | §2.1, §2.2 |
| `symbolic` bez wiedzy o `model/` | §3 — `assembleTransform` bierze wyłącznie typy symboliczne |
| konwersja w `builders/`, wolna funkcja | §4.3, §4.4 |
| jedna implementacja konwencji RPY | `makeRpyRotation`, §4.5 usuwa drugą |
| `PrincipalAxis` w `detail/`, osobny `.cpp` | §4.1, §4.2 |
| `assembleTransform` upublicznione, dwie kopie usunięte | §3.1, §3.2, §4.5 |
| lokalna bezstanowa fabryka | §8 — `const ExpressionFactory factory;` |
| `setTcp` wymaga łańcucha | §8 |
| kolejność prerekwizytów chain → FK → TCP | §8 |
| `clearTcp` idempotentne, `noexcept` | §8, test w §9 |
| graf zależności: `setTcp` zachowuje FK, `buildForwardKinematics` zachowuje TCP | §8, dwa testy w §9 |
| `sameNode` dla tożsamościowego TCP | §9 |
| gwarancja transakcyjna `setTcp` | §8, test w §9 |
| rozszerzenie referencji kwaternionowej | §6 |
| tolerancja `1e-12` bez zmian | §5.1 |

---

## 14. Jak to zweryfikuję po zatwierdzeniu

```powershell
cmake -B build -G Ninja
cmake --build build --clean-first
ctest --test-dir build --output-on-failure
.\build\tests\kinemaforge_tests.exe --gtest_filter=TcpTransformTest.*
```

Oczekiwane: **221 + 21 = 242 zielone**, a wewnątrz nich **18 porównań macierzy TCP** (9 konfiguracji × 2 roboty) — liczba widoczna w wydruku `[ MEASURE ]`, więc raport poda ją z przebiegu, nie z deklaracji.

Zaraportuję maksymalne błędy z linii `[ MEASURE ]` dla testów TCP i porównam je ze zmierzonymi w Fazie 1 (`5.55e-16`). Wzrost powyżej rzędu wielkości oznaczałby, że dodatkowe złożenie kosztuje więcej, niż zakładałem — zgłoszę to jako ustalenie, nie podniosę tolerancji.

## 15. Pięć poprawek z review — wprowadzone

Review odtworzyło zmiany produkcyjne w osobnej kopii repo, skompilowało je w C++23 z `-Wall -Wextra -Wpedantic -Werror` i potwierdziło harnessem zarówno `(0, 1.7, 2.335)`, jak i `sameNode` dla tożsamościowego TCP. **Kod produkcyjny zatwierdzony bez zmian.** Poprawki dotyczyły wyłącznie części testowej:

| # | Korekta | Gdzie |
|---|---|---|
| 1 | **blocker** — `BuildsKr640TcpForwardKinematics` był zaślepką z komentarzem: policzyłby się do 242, zawsze przeszedł i **nie zweryfikował KR640** | §9 |
| 2 | **blocker zakresu** — deklarowałem „te same konfiguracje co w Fazie 1", a sprawdzałem dwie z dziewięciu | §9 (`tcpValidationConfigurations`), §6.3, §11 architektury |
| 3 | `TransformComparison.cpp` opisany zamiast podany; ciał **nie da się** przenieść dosłownie (alias `support::`, brak `<gtest/gtest.h>`) | §5.2 |
| 4 | walidacja skończoności testowana na dwóch z sześciu składowych | §9 |
| 5 | `clearTcp` na świeżej fasadzie | §9 |

Blocker 1 jest wart nazwania, bo to najgorszy rodzaj błędu w tym projekcie: **zaślepka, która podnosi licznik testów i zawsze przechodzi**. Raport pokazałby 242/242 i wyglądałby na kompletny, nie mówiąc nic o KR640. Blocker 2 jest tym samym wzorcem w innej skali — opis szedł dalej niż to, co testy faktycznie sprawdzały.

Oba to ten sam wzorzec, który wracał w tym projekcie już kilka razy: **deklaracja szersza niż asercja**. Przy `x·0`, przy oracle'u `q = 0`, przy kontrakcie wskaźników, teraz tutaj.

## Do zatwierdzenia

1. Kod produkcyjny (§2–§4, §7, §8) — zatwierdzony w review, bez zmian.
2. Część testowa po czterech poprawkach (§5.2, §6.3, §9).
3. **§5 — ekstrakcja helperów porównawczych do `tests/support/TransformComparison`** plus przeniesienie `nearLimitConfiguration` (§6.3), czyli modyfikacja kodu testowego zatwierdzonego wcześniej. Rekomendacja: tak; alternatywą są dwie kopie progu `1e-12` i dwie definicje „5% zakresu".

**21 testów GTest, 18 porównań macierzy TCP, 242/242.** Bez commita.
