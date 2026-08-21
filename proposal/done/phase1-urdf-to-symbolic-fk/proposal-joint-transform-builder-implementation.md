# Proposal: `JointTransformBuilder` — implementacja

## Prompt

> Tworzymy `proposal/proposal-joint-transform-builder-implementation.md` [...] Publiczne API powinno przejść ze stuba do zatwierdzonej wersji [...] Nie należy teraz od razu pisać kodu produkcyjnego. Zgodnie z przyjętym procesem najpierw powstaje pełny proposal implementacyjny, potem review, a dopiero potem kod.

Realizacja architektury zatwierdzonej w `proposal-joint-transform-builder-architecture.md` (werdykt `approve` po dwóch rundach review, warunkowo na wdrożenie walidacji loadera — co jest już zrobione, commit `eb2a8ea`).

## Status weryfikacji

**Kod poniżej nie został skompilowany.** Zgodnie z ustaleniem: proposal jest dokumentem, kod trafia na dysk dopiero po zatwierdzeniu.

Deklaruję to wprost, bo w tym projekcie brak weryfikacji dwa razy zbiegł się z realnym błędem, którego przegląd nie wychwycił.

## Jedna nowa decyzja do zatwierdzenia

Przy pisaniu kodu wyszła konsekwencja, której architektura nie przewidziała, a która **zmienia widoczny wynik** i kształt testów. Zgłaszam ją osobno, zamiast wstawiać po cichu do implementacji.

### `R_origin · R_motion` produkuje śmieci, gdy `R_origin` jest jednostkowe

Architektura (§13) mówi dla revolute: `R = R_origin · R_motion`, mnożenie bloków 3×3. Ale dla `origin.rpy = (0,0,0)` macierz `R_origin` jest **dokładnie jednostkowa**, a mnożenie przez nią — przy braku anihilatora `x·0 → 0` — nie zwraca `R_motion`:

```
(0,0) = 1·c + 0·s + 0·0
      = Add( Cos(q), Multiply(Constant(0), Sin(q)) )     zamiast  Cos(q)
```

Pierwszy człon zwija się przez `x·1 → x`, trzeci przez zwijanie stałych, ale **drugi zostaje** — `0 · sin(q)` to stała razy wyrażenie symboliczne, czyli dokładnie przypadek, którego anihilator nie obsługuje.

To nie jest przypadek brzegowy: **wszystkie 7 jointów `kr640.urdf` ma `rpy="0 0 0"`**, więc dotyczyłoby każdej transformacji tego robota. Test `BuildsRevoluteJointAroundZAxis` sprawdzający `M(0,0).type() == Cos` **failowałby**.

**Proponowane rozwiązanie: pominąć mnożenie, gdy `R_origin` jest dokładnie jednostkowa.**

```cpp
if (isIdentityRotation(originRotation))
    return motionRotation;                       // R_origin · R_motion = R_motion
return multiply(originRotation, motionRotation, factory);
```

Uzasadnienie — ta sama zasada, co przy fast path dla osi osiowych (§8.1 architektury): **wybór konstrukcji na podstawie stałych znanych w chwili budowania**, nie przekształcenie symboliczne. `isIdentityRotation` sprawdza dziewięć komórek predykatami `isOne`/`isZero`, czyli pyta o stałe, nie o równoważność wyrażeń. Obie ścieżki dają tę samą funkcję matematyczną.

Gdyby to odrzucić, alternatywą jest osłabienie testów do „komórka jest `Add` zawierającym `Cos`" — czyli rezygnacja ze sprawdzania, że macierz *wygląda* jak `Rz(q)`. Uważam to za gorsze, ale decyzja jest Twoja.

## Stan obecny

### `src/ik_equations/builders/JointTransformBuilder.hpp` (cały plik)

```cpp
#pragma once

#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

namespace kinemaforge::ik {

class JointTransformBuilder
{
public:
    SymbolicTransform build(const KinematicJoint& joint) const;
};

} // namespace kinemaforge::ik
```

Brak `.cpp`, brak wpisu w `CMakeLists.txt`, brak testów.

### Co jest dostępne

`ExpressionFactory`: `constant`, `symbol`, `add`, `subtract`, `multiply`, `divide`, `negate`, `sin`, `cos`. Zwijanie stałych i elementy neutralne, **bez** `x·0 → 0`.

`SymbolicMatrix<R,C>`: `operator()`, `zeros()`, `identity()` (kwadratowe), wolna `multiply(lhs, rhs, factory)`. Aliasy: `SymbolicRotation` = 3×3, `SymbolicVector3` = 3×1, `SymbolicTransform` = 4×4.

`KinematicJoint`: `type`, `origin{translation, rpy}`, `axis`, `variable` (`optional<JointVariable>` z polem `name`).

**Kontrakt z loadera** (commit `eb2a8ea`): dla jointu aktywnego `axis` jest skończona, niezerowa i numerycznie znormalizowana; `origin` ma wyłącznie skończone wartości.

### `CMakeLists.txt` / `tests/CMakeLists.txt` — fragmenty

```cmake
add_library(kinemaforge_ik STATIC
    ...
    src/ik_equations/builders/KinematicChainBuilder.cpp
    src/ik_equations/symbolic/Expression.cpp
    src/ik_equations/symbolic/ExpressionFactory.cpp
)
```

```cmake
add_executable(kinemaforge_tests
    test_kinematics.cpp
    test_urdf_model_loader.cpp
    test_kinematic_chain_builder.cpp
    test_symbolic_expression.cpp
    test_expression_factory.cpp
    test_symbolic_matrix.cpp
)
```

## Co się zmienia

### 1. `src/ik_equations/builders/JointTransformBuilder.hpp` (nowa pełna treść)

```cpp
#pragma once

#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

namespace kinemaforge::ik {

// Turns one joint into T_parent_child(q), the homogeneous transform from
// the parent link frame to the child link frame:
//
//     T_parent_child(q) = T_origin * T_motion(q)
//     T_origin          = Translation(origin.translation) * R_rpy(origin.rpy)
//
// The joint axis is expressed in the joint frame — that is, after origin
// has been applied — which is why T_motion uses it unrotated.
//
// Preconditions, established by UrdfModelLoader and KinematicChainBuilder
// and asserted here rather than re-validated:
//   * a fixed joint has no variable
//   * an actuated joint has one, and its axis is a unit vector
class JointTransformBuilder
{
public:
    explicit JointTransformBuilder(ExpressionFactory factory = {});

    SymbolicTransform build(const KinematicJoint& joint) const;

private:
    ExpressionFactory factory_;
};

} // namespace kinemaforge::ik
```

### 2. `src/ik_equations/builders/JointTransformBuilder.cpp` (nowy plik)

```cpp
#include "ik_equations/builders/JointTransformBuilder.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>

namespace kinemaforge::ik {

namespace {

// An axis that is off unit length by more than this is a broken contract,
// not rounding: the loader normalizes, so real error is a few ULP. The
// bound is loose on purpose — it exists to catch a hand-built joint or a
// regression upstream, not to police the last bits.
constexpr double kUnitAxisTolerance = 1e-9;

bool isUnitLength(const Vector3& axis) noexcept
{
    return std::abs(std::hypot(axis.x, axis.y, axis.z) - 1.0) <= kUnitAxisTolerance;
}

// --- primitive blocks -----------------------------------------------

// Rotation about one principal axis. `negated` folds the sign into the
// sine rather than wrapping the result, since R(-a, q) = R(a, -q) and
// cos is even: without it the cells would carry Negate(Negate(Sin)).
enum class PrincipalAxis { X, Y, Z };

SymbolicRotation buildPrincipalRotation(PrincipalAxis axis,
                                        bool negated,
                                        const Expression& variable,
                                        const ExpressionFactory& factory)
{
    const Expression cosine = factory.cos(variable);
    const Expression rawSine = factory.sin(variable);

    const Expression sine     = negated ? factory.negate(rawSine) : rawSine;
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

// R_rpy = Rz(yaw) * Ry(pitch) * Rx(roll) — URDF's fixed-axis convention.
//
// Built by composing the three principal rotations rather than writing out
// the nine closed-form entries: every cell here is a constant, so folding
// collapses the products immediately and the code stays readable as the
// formula it implements.
SymbolicRotation buildRpyRotation(const Vector3& rpy, const ExpressionFactory& factory)
{
    const auto roll  = buildPrincipalRotation(PrincipalAxis::X, false, factory.constant(rpy.x), factory);
    const auto pitch = buildPrincipalRotation(PrincipalAxis::Y, false, factory.constant(rpy.y), factory);
    const auto yaw   = buildPrincipalRotation(PrincipalAxis::Z, false, factory.constant(rpy.z), factory);

    return multiply(multiply(yaw, pitch, factory), roll, factory);
}

// True when every cell is exactly the constant it would be in an identity
// matrix. Asks about constants, not about equivalence of expressions.
bool isIdentityRotation(const SymbolicRotation& rotation) noexcept
{
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
        {
            const bool ok = (row == column) ? isOne(rotation(row, column))
                                            : isZero(rotation(row, column));
            if (!ok) return false;
        }
    return true;
}

// Recognises an axis that is exactly a signed principal direction, so the
// caller can emit a canonical Rx/Ry/Rz instead of the general formula.
// The components are constants from the URDF, so exact comparison is the
// right test — a slightly tilted axis is a different axis, not this one.
struct PrincipalAxisMatch { PrincipalAxis axis; bool negated; };

std::optional<PrincipalAxisMatch> matchPrincipalAxis(const Vector3& unitAxis) noexcept
{
    if (unitAxis.y == 0.0 && unitAxis.z == 0.0)
    {
        if (unitAxis.x ==  1.0) return PrincipalAxisMatch{PrincipalAxis::X, false};
        if (unitAxis.x == -1.0) return PrincipalAxisMatch{PrincipalAxis::X, true};
    }
    if (unitAxis.x == 0.0 && unitAxis.z == 0.0)
    {
        if (unitAxis.y ==  1.0) return PrincipalAxisMatch{PrincipalAxis::Y, false};
        if (unitAxis.y == -1.0) return PrincipalAxisMatch{PrincipalAxis::Y, true};
    }
    if (unitAxis.x == 0.0 && unitAxis.y == 0.0)
    {
        if (unitAxis.z ==  1.0) return PrincipalAxisMatch{PrincipalAxis::Z, false};
        if (unitAxis.z == -1.0) return PrincipalAxisMatch{PrincipalAxis::Z, true};
    }
    return std::nullopt;
}

// Rodrigues' rotation formula for a unit axis a = [x, y, z]:
//
//     R = [ t*x^2 + c    t*x*y - s*z   t*x*z + s*y ]
//         [ t*x*y + s*z  t*y^2 + c     t*y*z - s*x ]
//         [ t*x*z - s*y  t*y*z + s*x   t*z^2 + c   ]
//
// with c = cos(q), s = sin(q), t = 1 - c.
//
// A signed principal axis takes a shortcut to the canonical Rx/Ry/Rz. That
// is not an algebraic simplification: the axis is a constant known while
// building, and the general formula would leave cells such as
// ((1-cos(q))*0)*0 + cos(q) that nothing in this project can reduce.
SymbolicRotation buildAxisAngleRotation(const Vector3& unitAxis,
                                        const Expression& variable,
                                        const ExpressionFactory& factory)
{
    if (const auto principal = matchPrincipalAxis(unitAxis))
        return buildPrincipalRotation(principal->axis, principal->negated, variable, factory);

    const Expression cosine = factory.cos(variable);
    const Expression sine   = factory.sin(variable);
    const Expression versine = factory.subtract(factory.constant(1.0), cosine);

    const Expression x = factory.constant(unitAxis.x);
    const Expression y = factory.constant(unitAxis.y);
    const Expression z = factory.constant(unitAxis.z);

    const Expression tx = factory.multiply(versine, x);
    const Expression ty = factory.multiply(versine, y);
    const Expression tz = factory.multiply(versine, z);

    const Expression sx = factory.multiply(sine, x);
    const Expression sy = factory.multiply(sine, y);
    const Expression sz = factory.multiply(sine, z);

    SymbolicRotation rotation;
    rotation(0, 0) = factory.add(factory.multiply(tx, x), cosine);
    rotation(0, 1) = factory.subtract(factory.multiply(tx, y), sz);
    rotation(0, 2) = factory.add(factory.multiply(tx, z), sy);

    rotation(1, 0) = factory.add(factory.multiply(tx, y), sz);
    rotation(1, 1) = factory.add(factory.multiply(ty, y), cosine);
    rotation(1, 2) = factory.subtract(factory.multiply(ty, z), sx);

    rotation(2, 0) = factory.subtract(factory.multiply(tx, z), sy);
    rotation(2, 1) = factory.add(factory.multiply(ty, z), sx);
    rotation(2, 2) = factory.add(factory.multiply(tz, z), cosine);
    return rotation;
}

// component * q, without building a multiplication for a zero component.
//
// Safe even though x * 0 -> 0 is deliberately absent from the factory:
// that rule was rejected because an arbitrary x may have a restricted
// domain, whereas here the other operand is a bare joint variable — total
// on the reals — and the component is a constant known while building.
Expression scaledAxisComponent(double component,
                               const Expression& variable,
                               const ExpressionFactory& factory)
{
    if (component == 0.0)
        return factory.constant(0.0);
    return factory.multiply(factory.constant(component), variable);
}

SymbolicVector3 buildPrismaticDisplacement(const Vector3& unitAxis,
                                           const Expression& variable,
                                           const ExpressionFactory& factory)
{
    SymbolicVector3 displacement;
    displacement(0, 0) = scaledAxisComponent(unitAxis.x, variable, factory);
    displacement(1, 0) = scaledAxisComponent(unitAxis.y, variable, factory);
    displacement(2, 0) = scaledAxisComponent(unitAxis.z, variable, factory);
    return displacement;
}

SymbolicVector3 toSymbolicVector(const Vector3& value, const ExpressionFactory& factory)
{
    SymbolicVector3 vector;
    vector(0, 0) = factory.constant(value.x);
    vector(1, 0) = factory.constant(value.y);
    vector(2, 0) = factory.constant(value.z);
    return vector;
}

SymbolicVector3 addVectors(const SymbolicVector3& lhs,
                           const SymbolicVector3& rhs,
                           const ExpressionFactory& factory)
{
    SymbolicVector3 sum;
    for (std::size_t row = 0; row < 3; ++row)
        sum(row, 0) = factory.add(lhs(row, 0), rhs(row, 0));
    return sum;
}

// Starting from identity leaves the homogeneous last row [0 0 0 1] correct
// by construction. Multiplying two full 4x4 matrices would not: the row
// would become 0*R00 + 0*R10 + 0*R20, which is zero mathematically but not
// a Constant(0) node, so isZero() would stop recognising it.
SymbolicTransform assembleTransform(const SymbolicRotation& rotation,
                                    const SymbolicVector3& translation)
{
    auto transform = SymbolicTransform::identity();
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
            transform(row, column) = rotation(row, column);
        transform(row, 3) = translation(row, 0);
    }
    return transform;
}

// R_origin * R_motion, skipping the product when origin contributes no
// rotation. Without this, every joint of a robot whose origins carry no
// rpy — every joint of kr640.urdf — would drag along terms like
// 0 * sin(q), which the factory deliberately does not fold away.
SymbolicRotation composeRotations(const SymbolicRotation& originRotation,
                                  const SymbolicRotation& motionRotation,
                                  const ExpressionFactory& factory)
{
    if (isIdentityRotation(originRotation))
        return motionRotation;
    return multiply(originRotation, motionRotation, factory);
}

SymbolicVector3 rotateVector(const SymbolicRotation& rotation,
                             const SymbolicVector3& vector,
                             const ExpressionFactory& factory)
{
    if (isIdentityRotation(rotation))
        return vector;
    return multiply(rotation, vector, factory);
}

bool isActuated(JointType type) noexcept
{
    switch (type)
    {
    case JointType::Revolute:
    case JointType::Prismatic:
    case JointType::Continuous:
        return true;
    case JointType::Fixed:
        return false;
    }
    return false;
}

} // namespace

JointTransformBuilder::JointTransformBuilder(ExpressionFactory factory)
    : factory_(std::move(factory))
{
}

SymbolicTransform JointTransformBuilder::build(const KinematicJoint& joint) const
{
    assert(isActuated(joint.type) == joint.variable.has_value() &&
           "exactly the actuated joints carry a symbolic variable");

    const SymbolicRotation originRotation = buildRpyRotation(joint.origin.rpy, factory_);
    const SymbolicVector3 originTranslation = toSymbolicVector(joint.origin.translation, factory_);

    if (joint.type == JointType::Fixed)
        return assembleTransform(originRotation, originTranslation);

    assert(isUnitLength(joint.axis) && "actuated joint axis must be a unit vector");

    // The name comes from the chain builder; inventing one here would let
    // forward kinematics and the solver drift onto different symbols.
    const Expression variable = factory_.symbol(joint.variable->name);

    if (joint.type == JointType::Prismatic)
    {
        const auto displacement = buildPrismaticDisplacement(joint.axis, variable, factory_);
        const auto offset = rotateVector(originRotation, displacement, factory_);
        return assembleTransform(originRotation,
                                 addVectors(originTranslation, offset, factory_));
    }

    // Revolute and Continuous are geometrically identical; the difference
    // is a limit, which is not this component's concern.
    const auto motionRotation = buildAxisAngleRotation(joint.axis, variable, factory_);
    return assembleTransform(composeRotations(originRotation, motionRotation, factory_),
                             originTranslation);
}

} // namespace kinemaforge::ik
```


### 3. `tests/test_joint_transform_builder.cpp` (nowy plik)

```cpp
#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <numbers>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

using kinemaforge::ik::CosNode;
using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::ExpressionType;
using kinemaforge::ik::JointType;
using kinemaforge::ik::JointTransformBuilder;
using kinemaforge::ik::JointVariable;
using kinemaforge::ik::KinematicJoint;
using kinemaforge::ik::NegateNode;
using kinemaforge::ik::SinNode;
using kinemaforge::ik::SymbolNode;
using kinemaforge::ik::SymbolicTransform;
using kinemaforge::ik::Vector3;
using kinemaforge::ik::constantValue;
using kinemaforge::ik::isConstant;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::structurallyEqual;

namespace {

constexpr double kPi = std::numbers::pi;

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

// Recursive walk looking for a named symbol. Kept in the test file: the
// only consumer today is this suite, and a public API would want to
// return every symbol rather than answer about one.
bool containsSymbol(const Expression& expression, std::string_view name)
{
    return std::visit(
        [name](const auto& node) -> bool {
            using Node = std::decay_t<decltype(node)>;
            if constexpr (std::is_same_v<Node, SymbolNode>)
                return node.name == name;
            else if constexpr (requires { node.lhs; node.rhs; })
                return containsSymbol(node.lhs, name) || containsSymbol(node.rhs, name);
            else if constexpr (requires { node.operand; })
                return containsSymbol(node.operand, name);
            else
                return false;
        },
        expression.node().value);
}

// Same walk as containsSymbol, without caring which symbol. Checking only
// whether a cell is a composite node would be wrong: Add(Constant,
// Constant) is composite and carries no symbol at all.
bool containsAnySymbol(const Expression& expression)
{
    return std::visit(
        [](const auto& node) -> bool {
            using Node = std::decay_t<decltype(node)>;
            if constexpr (std::is_same_v<Node, SymbolNode>)
                return true;
            else if constexpr (requires { node.lhs; node.rhs; })
                return containsAnySymbol(node.lhs) || containsAnySymbol(node.rhs);
            else if constexpr (requires { node.operand; })
                return containsAnySymbol(node.operand);
            else
                return false;
        },
        expression.node().value);
}

bool transformContainsAnySymbol(const SymbolicTransform& transform)
{
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
            if (containsAnySymbol(transform(row, column))) return true;
    return false;
}

KinematicJoint makeJoint(JointType type, Vector3 axis, Vector3 translation, Vector3 rpy,
                         std::string variableName = "q1")
{
    KinematicJoint joint;
    joint.name = "j";
    joint.type = type;
    joint.axis = axis;
    joint.origin.translation = translation;
    joint.origin.rpy = rpy;
    if (type != JointType::Fixed)
        joint.variable = JointVariable{std::move(variableName), 1};
    return joint;
}

KinematicJoint fixedJoint(Vector3 translation = {}, Vector3 rpy = {})
{
    return makeJoint(JointType::Fixed, Vector3{1.0, 0.0, 0.0}, translation, rpy);
}

KinematicJoint revoluteJoint(Vector3 axis, Vector3 translation = {}, Vector3 rpy = {},
                             std::string variableName = "q1")
{
    return makeJoint(JointType::Revolute, axis, translation, rpy, std::move(variableName));
}

void expectHomogeneousLastRow(const SymbolicTransform& transform)
{
    EXPECT_TRUE(isZero(transform(3, 0)));
    EXPECT_TRUE(isZero(transform(3, 1)));
    EXPECT_TRUE(isZero(transform(3, 2)));
    EXPECT_TRUE(isOne(transform(3, 3)));
}

// Rotation blocks come from folded constants, so they compare numerically.
void expectRotationBlock(const SymbolicTransform& transform, const double (&expected)[3][3])
{
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            // constantValue asserts on a non-constant, which would abort the
            // process instead of reporting a failure. Check first.
            ASSERT_TRUE(isConstant(transform(row, column)))
                << "rotation cell did not fold to a constant";
            EXPECT_NEAR(constantValue(transform(row, column)), expected[row][column], 1e-12);
        }
}

} // namespace

// --- fixed joints and origin ----------------------------------------

TEST(JointTransformBuilderTest, BuildsIdentityForFixedJointWithoutOrigin)
{
    const JointTransformBuilder builder;
    const auto transform = builder.build(fixedJoint());

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            if (row == column)
                EXPECT_TRUE(isOne(transform(row, column)));
            else
                EXPECT_TRUE(isZero(transform(row, column)));
        }
}

TEST(JointTransformBuilderTest, BuildsTranslationFromFixedJointOrigin)
{
    const JointTransformBuilder builder;
    const auto transform = builder.build(fixedJoint({1.0, 2.0, 3.0}));

    EXPECT_DOUBLE_EQ(constantValue(transform(0, 3)), 1.0);
    EXPECT_DOUBLE_EQ(constantValue(transform(1, 3)), 2.0);
    EXPECT_DOUBLE_EQ(constantValue(transform(2, 3)), 3.0);

    const double identity[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    expectRotationBlock(transform, identity);
    expectHomogeneousLastRow(transform);
}

TEST(JointTransformBuilderTest, BuildsRotationFromFixedJointRpy)
{
    const JointTransformBuilder builder;
    const auto transform = builder.build(fixedJoint({}, {0.0, 0.0, kPi / 2.0}));

    const double expected[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};   // Rz(pi/2)
    expectRotationBlock(transform, expected);
}

TEST(JointTransformBuilderTest, MapsRollPitchYawToCorrectAxes)
{
    const JointTransformBuilder builder;
    const double half = kPi / 2.0;

    const double aroundX[3][3] = {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}};
    const double aroundY[3][3] = {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
    const double aroundZ[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};

    {
        SCOPED_TRACE("roll");
        expectRotationBlock(builder.build(fixedJoint({}, {half, 0.0, 0.0})), aroundX);
    }
    {
        SCOPED_TRACE("pitch");
        expectRotationBlock(builder.build(fixedJoint({}, {0.0, half, 0.0})), aroundY);
    }
    {
        SCOPED_TRACE("yaw");
        expectRotationBlock(builder.build(fixedJoint({}, {0.0, 0.0, half})), aroundZ);
    }
}

TEST(JointTransformBuilderTest, ComposesRpyInFixedAxisOrder)
{
    // rpy = (pi/2, 0, -pi/2), taken from joint_4 of kr4_r600.urdf. Chosen
    // because the two conventions disagree here: a single non-zero
    // component, or two rotations by pi, would give the same matrix either
    // way and prove nothing.
    const JointTransformBuilder builder;
    const auto transform = builder.build(fixedJoint({}, {kPi / 2.0, 0.0, -kPi / 2.0}));

    // Rz(-pi/2) * Ry(0) * Rx(pi/2). The reversed order would give
    // {{0,1,0},{0,0,-1},{-1,0,0}}.
    const double expected[3][3] = {{0, 0, -1}, {-1, 0, 0}, {0, 1, 0}};
    expectRotationBlock(transform, expected);
}

TEST(JointTransformBuilderTest, CombinesTranslationAndRotationInCorrectOrder)
{
    // Translation * Rotation keeps the translation column verbatim.
    // Rotation * Translation would rotate it to [0, 1, 0].
    const JointTransformBuilder builder;
    const auto transform = builder.build(fixedJoint({1.0, 0.0, 0.0}, {0.0, 0.0, kPi / 2.0}));

    EXPECT_NEAR(constantValue(transform(0, 3)), 1.0, 1e-12);
    EXPECT_NEAR(constantValue(transform(1, 3)), 0.0, 1e-12);
    EXPECT_NEAR(constantValue(transform(2, 3)), 0.0, 1e-12);
}

TEST(JointTransformBuilderTest, CombinesOriginAndMotionInCorrectOrder)
{
    // T_origin * T_motion leaves the translation column constant: the joint
    // turns about a point 350 mm away. T_motion * T_origin would make that
    // column depend on q, which is geometrically wrong.
    const JointTransformBuilder builder;
    const auto transform = builder.build(revoluteJoint({0.0, 1.0, 0.0}, {0.35, 0.0, 0.0}));

    EXPECT_DOUBLE_EQ(constantValue(transform(0, 3)), 0.35);
    EXPECT_TRUE(isZero(transform(1, 3)));
    EXPECT_TRUE(isZero(transform(2, 3)));
}

TEST(JointTransformBuilderTest, ComposesOriginAndMotionRotationsInCorrectOrder)
{
    // The translation-column test above uses rpy = 0, so it cannot tell
    // R_origin * R_motion from R_motion * R_origin — with identity origin
    // both give the same rotation block. Here origin turns about Z and the
    // joint about X, which do not commute:
    //
    //   Rz(pi/2) * Rx(q) = [[0, -c,  s], [1, 0, 0], [0, s, c]]
    //   Rx(q) * Rz(pi/2) = [[0, -1,  0], [c, 0, -s], [s, 0, c]]
    //
    // Cell (1,0) is the discriminator: constant 1 in the correct order, a
    // cosine in the reversed one. Cell (0,1) is the mirror of that.
    const JointTransformBuilder builder;
    const auto transform =
        builder.build(revoluteJoint({1.0, 0.0, 0.0}, {}, {0.0, 0.0, kPi / 2.0}));

    EXPECT_TRUE(isOne(transform(1, 0)));
    EXPECT_FALSE(isConstant(transform(0, 1)));
    EXPECT_TRUE(containsSymbol(transform(0, 1), "q1"));
    expectHomogeneousLastRow(transform);
}

// --- revolute: principal and arbitrary axes -------------------------

TEST(JointTransformBuilderTest, BuildsRevoluteJointAroundXAxis)
{
    const JointTransformBuilder builder;
    const auto transform = builder.build(revoluteJoint({1.0, 0.0, 0.0}));

    EXPECT_TRUE(isOne(transform(0, 0)));
    EXPECT_EQ(transform(1, 1).type(), ExpressionType::Cos);
    EXPECT_EQ(transform(2, 2).type(), ExpressionType::Cos);
    EXPECT_EQ(transform(2, 1).type(), ExpressionType::Sin);
    EXPECT_EQ(transform(1, 2).type(), ExpressionType::Negate);
    expectHomogeneousLastRow(transform);
}

TEST(JointTransformBuilderTest, BuildsRevoluteJointAroundYAxis)
{
    // Ry carries the opposite sign pattern to Rx and Rz: sin sits above the
    // diagonal, minus-sin below. Copying either neighbour breaks here.
    const JointTransformBuilder builder;
    const auto transform = builder.build(revoluteJoint({0.0, 1.0, 0.0}));

    EXPECT_TRUE(isOne(transform(1, 1)));
    EXPECT_EQ(transform(0, 2).type(), ExpressionType::Sin);
    EXPECT_EQ(transform(2, 0).type(), ExpressionType::Negate);
}

TEST(JointTransformBuilderTest, BuildsRevoluteJointAroundZAxis)
{
    const JointTransformBuilder builder;
    const auto transform = builder.build(revoluteJoint({0.0, 0.0, 1.0}));

    EXPECT_EQ(transform(0, 0).type(), ExpressionType::Cos);
    EXPECT_EQ(transform(1, 1).type(), ExpressionType::Cos);
    EXPECT_EQ(transform(1, 0).type(), ExpressionType::Sin);
    EXPECT_EQ(transform(0, 1).type(), ExpressionType::Negate);
    EXPECT_TRUE(isOne(transform(2, 2)));
}

TEST(JointTransformBuilderTest, BuildsRevoluteJointAroundNegativeZAxis)
{
    // R(-Z, q) = R(Z, -q): the sines swap places relative to +Z.
    const JointTransformBuilder builder;
    const auto transform = builder.build(revoluteJoint({0.0, 0.0, -1.0}));

    EXPECT_EQ(transform(0, 1).type(), ExpressionType::Sin);
    EXPECT_EQ(transform(1, 0).type(), ExpressionType::Negate);
    EXPECT_EQ(transform(0, 0).type(), ExpressionType::Cos);
}

TEST(JointTransformBuilderTest, AxisAlignedFastPathBuildsCanonicalZRotation)
{
    const ExpressionFactory factory;
    const JointTransformBuilder builder{factory};
    const auto transform = builder.build(revoluteJoint({0.0, 0.0, 1.0}));

    const Expression variable = factory.symbol("q1");
    EXPECT_TRUE(structurallyEqual(transform(0, 0), factory.cos(variable)));
    EXPECT_TRUE(structurallyEqual(transform(1, 0), factory.sin(variable)));
    EXPECT_TRUE(structurallyEqual(transform(0, 1), factory.negate(factory.sin(variable))));
}

TEST(JointTransformBuilderTest, BuildsRevoluteJointAroundArbitraryAxis)
{
    // Every component non-zero: with [1,1,0] the s*z terms vanish and the
    // test would pass even for an implementation that dropped them.
    const double scale = 1.0 / std::sqrt(14.0);
    const Vector3 axis{1.0 * scale, 2.0 * scale, 3.0 * scale};

    const ExpressionFactory factory;
    const JointTransformBuilder builder{factory};
    const auto transform = builder.build(revoluteJoint(axis));

    // Rebuild the expected cells from the formula. Checking only the node
    // types and that the two differ would also pass for an implementation
    // using the wrong component — say s*x instead of s*z.
    const Expression variable = factory.symbol("q1");
    const Expression cosine = factory.cos(variable);
    const Expression sine = factory.sin(variable);
    const Expression versine = factory.subtract(factory.constant(1.0), cosine);

    const Expression x = factory.constant(axis.x);
    const Expression y = factory.constant(axis.y);
    const Expression z = factory.constant(axis.z);
    const Expression tx = factory.multiply(versine, x);
    const Expression sz = factory.multiply(sine, z);

    // t*x*y - s*z   and   t*x*y + s*z
    EXPECT_TRUE(structurallyEqual(
        transform(0, 1), factory.subtract(factory.multiply(tx, y), sz)));
    EXPECT_TRUE(structurallyEqual(
        transform(1, 0), factory.add(factory.multiply(tx, y), sz)));

    // t*x^2 + c
    EXPECT_TRUE(structurallyEqual(
        transform(0, 0), factory.add(factory.multiply(tx, x), cosine)));

    expectHomogeneousLastRow(transform);
}

// --- continuous and prismatic ---------------------------------------

TEST(JointTransformBuilderTest, BuildsContinuousJointLikeRevolute)
{
    const JointTransformBuilder builder;
    const Vector3 axis{0.0, 0.0, 1.0};

    const auto revolute = builder.build(makeJoint(JointType::Revolute, axis, {}, {}));
    const auto continuous = builder.build(makeJoint(JointType::Continuous, axis, {}, {}));

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            EXPECT_TRUE(structurallyEqual(revolute(row, column), continuous(row, column)));
        }
}

TEST(JointTransformBuilderTest, BuildsPrismaticJointTranslationAlongAxis)
{
    // Two non-zero components: with [0,0,1] the test would pass even for an
    // implementation that hard-coded q into one cell.
    const double scale = 1.0 / std::sqrt(5.0);
    const Vector3 axis{1.0 * scale, 2.0 * scale, 0.0};

    const ExpressionFactory factory;
    const JointTransformBuilder builder{factory};
    const auto transform = builder.build(makeJoint(JointType::Prismatic, axis, {}, {}));

    // Compare against the exact expected products: asserting only "is a
    // Multiply containing q1" would also pass if both cells got the same
    // coefficient, or if the two were swapped.
    const Expression variable = factory.symbol("q1");
    EXPECT_TRUE(structurallyEqual(
        transform(0, 3), factory.multiply(factory.constant(axis.x), variable)));
    EXPECT_TRUE(structurallyEqual(
        transform(1, 3), factory.multiply(factory.constant(axis.y), variable)));

    // The zero component builds no multiplication at all.
    EXPECT_TRUE(isZero(transform(2, 3)));

    expectHomogeneousLastRow(transform);
}

TEST(JointTransformBuilderTest, PrismaticDoesNotRotate)
{
    const double scale = 1.0 / std::sqrt(5.0);
    const Vector3 axis{1.0 * scale, 2.0 * scale, 0.0};

    const JointTransformBuilder builder;
    const auto transform = builder.build(makeJoint(JointType::Prismatic, axis, {}, {}));

    const double identity[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    expectRotationBlock(transform, identity);
}

TEST(JointTransformBuilderTest, RotatesPrismaticDisplacementByOriginRotation)
{
    // p = p_origin + R_origin * (axis * q). With origin turning about Z by
    // pi/2 and the slide along X, the displacement must land in Y. Dropping
    // the rotation entirely would leave it in X, and every other prismatic
    // test here uses an identity origin, so none of them would notice.
    const ExpressionFactory factory;
    const JointTransformBuilder builder{factory};
    const auto transform = builder.build(
        makeJoint(JointType::Prismatic, {1.0, 0.0, 0.0}, {}, {0.0, 0.0, kPi / 2.0}));

    const Expression variable = factory.symbol("q1");
    EXPECT_TRUE(structurallyEqual(transform(1, 3), variable));
    EXPECT_FALSE(structurallyEqual(transform(0, 3), variable));
    expectHomogeneousLastRow(transform);
}

// --- variable, invariants, real robots ------------------------------

TEST(JointTransformBuilderTest, UsesJointVariableName)
{
    const JointTransformBuilder builder;
    const auto transform =
        builder.build(revoluteJoint({0.0, 0.0, 1.0}, {}, {}, "theta_custom"));

    EXPECT_TRUE(containsSymbol(transform(0, 0), "theta_custom"));
    EXPECT_FALSE(containsSymbol(transform(0, 0), "q1"));
}

TEST(JointTransformBuilderTest, PreservesHomogeneousLastRow)
{
    // Every joint type, each with a non-zero origin, since the last row is
    // exactly what a full 4x4 product would quietly destroy.
    const JointTransformBuilder builder;
    const Vector3 translation{0.1, 0.2, 0.3};
    const Vector3 rpy{kPi / 2.0, 0.0, -kPi / 2.0};
    const double scale = 1.0 / std::sqrt(14.0);
    const Vector3 tilted{1.0 * scale, 2.0 * scale, 3.0 * scale};

    {
        SCOPED_TRACE("fixed");
        expectHomogeneousLastRow(builder.build(fixedJoint(translation, rpy)));
    }
    {
        SCOPED_TRACE("revolute");
        expectHomogeneousLastRow(
            builder.build(makeJoint(JointType::Revolute, tilted, translation, rpy)));
    }
    {
        SCOPED_TRACE("continuous");
        expectHomogeneousLastRow(
            builder.build(makeJoint(JointType::Continuous, tilted, translation, rpy)));
    }
    {
        SCOPED_TRACE("prismatic");
        expectHomogeneousLastRow(
            builder.build(makeJoint(JointType::Prismatic, tilted, translation, rpy)));
    }
}

TEST(JointTransformBuilderTest, IgnoresAxisForFixedJoint)
{
    // A fixed joint's axis takes no part in the transform, so a degenerate
    // one must not be rejected — the loader lets it through for exactly
    // this reason.
    KinematicJoint joint = fixedJoint({0.0, 0.0, 1.0});
    joint.axis = Vector3{0.0, 0.0, 0.0};

    const JointTransformBuilder builder;
    const auto transform = builder.build(joint);

    EXPECT_DOUBLE_EQ(constantValue(transform(2, 3)), 1.0);
    expectHomogeneousLastRow(transform);
}

TEST(JointTransformBuilderTest, BuildsAllKr640ChainJoints)
{
    // One pass over real data, to catch drift between the hand-built joints
    // above and what the pipeline actually produces.
    kinemaforge::ik::UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    kinemaforge::ik::KinematicChainBuilder chainBuilder;
    const auto chain = chainBuilder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(chain.has_value());
    ASSERT_EQ(chain->joints.size(), 7u);

    const JointTransformBuilder builder;
    for (const auto& joint : chain->joints)
    {
        SCOPED_TRACE(joint.name);
        const auto transform = builder.build(joint);
        expectHomogeneousLastRow(transform);

        if (joint.variable)
        {
            bool found = false;
            for (std::size_t row = 0; row < 3 && !found; ++row)
                for (std::size_t column = 0; column < 3 && !found; ++column)
                    found = containsSymbol(transform(row, column), joint.variable->name);
            EXPECT_TRUE(found) << "variable missing from the rotation block";
        }
        else
        {
            EXPECT_FALSE(transformContainsAnySymbol(transform))
                << "a fixed joint must not introduce a symbol";
        }
    }
}
```

### 4. `CMakeLists.txt` — jedna dodana linia

```cmake
add_library(kinemaforge_ik STATIC
    src/Kinematics.cpp
    src/kinematics/robot_model.cpp
    src/kinematics/robot_model_loader.cpp
    src/ik_equations/IkEquationBuilder.cpp
    src/ik_equations/UrdfModelLoader.cpp
    src/ik_equations/builders/KinematicChainBuilder.cpp
    src/ik_equations/builders/JointTransformBuilder.cpp
    src/ik_equations/symbolic/Expression.cpp
    src/ik_equations/symbolic/ExpressionFactory.cpp
)
```

### 5. `tests/CMakeLists.txt` — jedna dodana linia

```cmake
add_executable(kinemaforge_tests
    test_kinematics.cpp
    test_urdf_model_loader.cpp
    test_kinematic_chain_builder.cpp
    test_joint_transform_builder.cpp
    test_symbolic_expression.cpp
    test_expression_factory.cpp
    test_symbolic_matrix.cpp
)
```

## Zgodność z zatwierdzoną architekturą

| Decyzja | Gdzie w kodzie |
|---|---|
| `T_parent_child = T_origin · T_motion` | `build()` — origin po lewej we wszystkich gałęziach |
| `T_origin = Translation · RotationRPY` | `assembleTransform` — blok obrotu z `R_rpy`, kolumna z `translation` bez obracania |
| `R_rpy = Rz(yaw)·Ry(pitch)·Rx(roll)` | `buildRpyRotation` |
| Rodrigues dla dowolnej osi | `buildAxisAngleRotation`, gałąź po `matchPrincipalAxis` |
| Fast path dla `±X`, `±Y`, `±Z` | `matchPrincipalAxis` + `buildPrincipalRotation` |
| `Continuous` jak `Revolute` | wspólna gałąź na końcu `build()` |
| Prismatic: translacja, nie obrót | `buildPrismaticDisplacement`, blok obrotu zostaje `R_origin` |
| `scaledAxisComponent` dla zerowej składowej | j.w., z uzasadnieniem różnicy wobec anihilatora |
| Konstrukcja blokowa, nie mnożenie 4×4 | `assembleTransform` startuje od `identity()` |
| `SymbolicRotation` / `SymbolicVector3` jako typy pośrednie | wszystkie helpery |
| Fabryka jako pole z domyślnym argumentem | `JointTransformBuilder(ExpressionFactory factory = {})` |
| `assert` dla inwariantów, bez `std::expected` | `build()` — dwie asercje |
| Nazwa symbolu z `joint.variable->name` | `build()` |
| Brak `normalizeAxis` | oś przychodzi jednostkowa z loadera; `isUnitLength` tylko asertuje |
| Helpery w anonimowej przestrzeni nazw | całe `.cpp` |
| `containsSymbol` w pliku testowym | `tests/test_joint_transform_builder.cpp` |

## Jak to zweryfikuję po zatwierdzeniu

```powershell
cmake -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

Oczekiwany wynik: **117 dotychczasowych + 22 nowe = 139 zielonych**. Liczba policzona z treści tego dokumentu (`grep -c "^TEST(JointTransformBuilderTest,"`), nie oszacowana.

Dwadzieścia dwa, nie dziewiętnaście jak w prompcie. Lista z promptu obejmowała §16.1–16.4; doszły trzy:

- `CombinesOriginAndMotionInCorrectOrder` — obowiązkowy test kolejności `T_origin · T_motion` wymagany przez architekturę (§4.3),
- `ComposesOriginAndMotionRotationsInCorrectOrder` — ten sam test dla **bloku obrotu**, z nieprzemiennymi obrotami (uwaga z review: wersja z `rpy = 0` nie rozróżnia kolejności),
- `RotatesPrismaticDisplacementByOriginRotation` — pokrywa `rotateVector`, które inaczej dałoby się usunąć bez failowania żadnego testu.

Plus integracyjny `BuildsAllKr640ChainJoints`.

Rzeczy, które mogą wyjść dopiero przy budowaniu:

1. **`std::visit` z `requires` w `containsSymbol`** — ten sam wzorzec działa w `Expression.cpp` (`StructuralComparator`), ale tam jest na typach węzłów, nie na lambdzie generycznej. Ryzyko niskie.
2. **`SymbolicRotation::identity()`** — `requires (Rows == Columns)` spełnione dla 3×3, ale nie użyłem dotąd tego aliasu nigdzie w kodzie produkcyjnym.
3. **Czy `buildRpyRotation` zawsze zwija się do stałych** — powinno, bo wszystkie wejścia są stałe, ale jeśli gdzieś zostanie węzeł złożony, `expectRotationBlock` zgłosi to teraz jako **zwykły błąd testu** (`ASSERT_TRUE(isConstant(...))` przed `constantValue`), a nie przerwie procesu asercją.

## Do zatwierdzenia

Dwie rzeczy:

1. **Nowa decyzja z §„Jedna nowa decyzja"** — pominięcie mnożenia przez jednostkową `R_origin`. Bez niej `BuildsRevoluteJointAroundZAxis` i pokrewne trzeba osłabić.
2. Reszta implementacji.

Czekam na ok przed naniesieniem na pliki źródłowe. Bez commita.
