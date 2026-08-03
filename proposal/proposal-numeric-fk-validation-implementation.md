# Proposal: numeryczna walidacja FK — implementacja

## Prompt

> APPROVE z trzema obowiązkowymi korektami. Po tych korektach można od razu przygotowywać pełny proposal implementacyjny. [...] usunąć twierdzenie, że oracle q=0 testuje kolejność; dodać ręczny oracle KR640 dla q1=π/2; wprowadzić `JointConfiguration` po kolejności aktywnych jointów, niezależne od nazw symboli; wymagać bezpośredniego quaternion-vector rotation i testu właściwego obrotu; najlepiej dodać syntetyczny test ujemnej osi głównej; zaktualizować liczbę oczekiwanych testów do 202 albo 203.

Realizacja architektury zatwierdzonej w `proposal-numeric-fk-validation-architecture.md` (werdykt `APPROVE`, wersja v2 po naniesieniu korekt).

## Status weryfikacji

**Kod poniżej nie został skompilowany.** Zgodnie z procesem: proposal jest dokumentem, kod trafia na dysk dopiero po zatwierdzeniu.

Zweryfikowane **przez lekturę plików repozytorium**, nie przez uruchomienie:

| Ustalenie | Źródło |
|---|---|
| `JointLimits` ma pola `lower`, `upper`, `hasPositionLimits` | `src/ik_equations/model/UrdfJoint.hpp` |
| `Vector3` ma `x`, `y`, `z`; `JointOrigin` ma `translation`, `rpy` | j.w. |
| geometria KR640 — wszystkie `rpy` zerowe, sumy translacji | `data/urdf/kr640.urdf` |
| **konfiguracja mieszana mieści się w limitach obu robotów** | oba pliki URDF — §5.3 |
| **`±0.25` mieści się w limitach obu robotów** | j.w. |

## Trzy korekty z drugiej rundy review — wprowadzone

Review wyciągnęło kod z dokumentu, skompilowało go z aktualnymi API i uruchomiło pod ASan/UBSan. Zgłoszone poprawki:

| # | Korekta | Gdzie |
|---|---|---|
| 1 | **blocker** — `orientationAngleError` w bloku kodu zawierał `if (index == index)`, co pod `-Werror=tautological-compare` **nie kompiluje się**; dopisek „przy wdrożeniu użyć innej wersji" nie wystarcza | §4, §8.1 |
| 2 | `makeSymbolValues` nie wykrywał zduplikowanych nazw — `emplace` przy duplikacie **nie nadpisuje**, tylko nie wstawia; teraz `throw std::logic_error` | §3, §3.2 |
| 3 | opis `emplace` w dokumencie był po prostu nieprawdziwy | §3.2 |

Plus rekomendacja architektoniczna dotycząca umiejscowienia `makeSymbolValues` — odniesienie w §3.3.

Review potwierdziło również poprawność matematyczną referencji (kolejność RPY, składanie kwaternionów, wzór na obrót wektora, znaki w konwersji do macierzy), poprawność obu oracle ręcznych, testu ujemnej osi i prismatic, oraz liczbę 16 nowych testów. **Zmierzone błędy — §8.3.**

§8 wymienia pozostałe ryzyka.

---

## 1. Stan obecny

Brak katalogu `tests/support/`. `tests/CMakeLists.txt` nie ma `target_include_directories`, więc trzeba je dodać, żeby `#include "support/..."` się rozwiązywało.

Dostępne i wykorzystywane: `UrdfModelLoader`, `KinematicChainBuilder`, `JointTransformBuilder`, `ForwardKinematicsBuilder`, `ExpressionEvaluator`, `SymbolValues`. **Żaden plik w `src/` nie jest zmieniany.**

Stan testów: 187/187. Po tej zmianie oczekiwane **204/204**.

---

## 2. `tests/support/NumericForwardKinematics.hpp` (nowy plik)

```cpp
#pragma once

#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/symbolic/ExpressionEvaluator.hpp"

#include <array>
#include <vector>

// Independent numeric forward kinematics, used only to cross-check the
// symbolic pipeline. Deliberately NOT part of kinemaforge_ik: the only value
// this code has is being a second implementation, and that value disappears
// the moment anything in the product starts calling it.
//
// Independence is by representation, not by rewording: rotations are carried
// as quaternions and composed by quaternion multiplication, whereas production
// builds Rodrigues matrices and multiplies 3x3 blocks. A 3x3 matrix appears
// here exactly once, when the final result is handed to a test.
//
// What this does NOT protect against: a shared misreading of the URDF spec.
// Both sides get their axes, origins and joint order from the same
// KinematicChain, and this reference was written from the same understanding
// of "rpy" as the production code. Only the hand-computed oracles in
// test_numeric_fk_validation.cpp guard that.
namespace kinemaforge::testsupport {

struct Vector3d
{
    double x{};
    double y{};
    double z{};
};

// (w, x, y, z); the default is the identity rotation.
struct Quaternion
{
    double w{1.0};
    double x{};
    double y{};
    double z{};
};

struct RigidTransform
{
    Quaternion rotation;
    Vector3d translation;
};

using Matrix3 = std::array<std::array<double, 3>, 3>;

// One value per actuated joint, in chain order. Deliberately positional: the
// reference must not address joints by the same symbol names the symbolic side
// uses, or a duplicated name would be confirmed rather than caught.
using JointConfiguration = std::vector<double>;

Quaternion multiply(const Quaternion& lhs, const Quaternion& rhs);
Quaternion fromAxisAngle(const ik::Vector3& axis, double angle);
Quaternion fromRollPitchYaw(const ik::Vector3& rpy);
double norm(const Quaternion& quaternion);

// Rotates a vector without ever forming a rotation matrix.
Vector3d rotate(const Quaternion& quaternion, const Vector3d& vector);

RigidTransform compose(const RigidTransform& lhs, const RigidTransform& rhs);

// The one and only place a matrix is built.
Matrix3 toRotationMatrix(const Quaternion& quaternion);

RigidTransform numericForwardKinematics(const ik::KinematicChain& chain,
                                        const JointConfiguration& configuration);

// The symbolic side's view of the same configuration, addressed by name.
// Kept next to numericForwardKinematics on purpose: both walk the chain the
// same way, and sharing one isActuated plus one consumption order is what
// keeps the positional and the by-name mapping from drifting apart.
ik::SymbolValues makeSymbolValues(const ik::KinematicChain& chain,
                                  const JointConfiguration& configuration);

} // namespace kinemaforge::testsupport
```

---

## 3. `tests/support/NumericForwardKinematics.cpp` (nowy plik)

```cpp
#include "support/NumericForwardKinematics.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace kinemaforge::testsupport {

namespace {

Vector3d cross(const Vector3d& lhs, const Vector3d& rhs)
{
    return {lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x};
}

Vector3d add(const Vector3d& lhs, const Vector3d& rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vector3d scale(const Vector3d& vector, double factor)
{
    return {vector.x * factor, vector.y * factor, vector.z * factor};
}

Vector3d toVector3d(const ik::Vector3& vector)
{
    return {vector.x, vector.y, vector.z};
}

bool isActuated(ik::JointType type)
{
    return type == ik::JointType::Revolute
        || type == ik::JointType::Continuous
        || type == ik::JointType::Prismatic;
}

} // namespace

Quaternion multiply(const Quaternion& lhs, const Quaternion& rhs)
{
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w
    };
}

Quaternion fromAxisAngle(const ik::Vector3& axis, double angle)
{
    const double half = 0.5 * angle;
    const double sine = std::sin(half);
    return {std::cos(half), axis.x * sine, axis.y * sine, axis.z * sine};
}

Quaternion fromRollPitchYaw(const ik::Vector3& rpy)
{
    // URDF fixed-axis convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    const Quaternion roll = fromAxisAngle(ik::Vector3{1.0, 0.0, 0.0}, rpy.x);
    const Quaternion pitch = fromAxisAngle(ik::Vector3{0.0, 1.0, 0.0}, rpy.y);
    const Quaternion yaw = fromAxisAngle(ik::Vector3{0.0, 0.0, 1.0}, rpy.z);
    return multiply(multiply(yaw, pitch), roll);
}

double norm(const Quaternion& quaternion)
{
    return std::sqrt(quaternion.w * quaternion.w + quaternion.x * quaternion.x +
                     quaternion.y * quaternion.y + quaternion.z * quaternion.z);
}

Vector3d rotate(const Quaternion& quaternion, const Vector3d& vector)
{
    // v + 2w(u x v) + 2u x (u x v), with u the vector part.
    //
    // Deliberately not "convert to a matrix and multiply": that would be the
    // production representation, and a shared mistake could then survive in
    // both places.
    const Vector3d vectorPart{quaternion.x, quaternion.y, quaternion.z};
    const Vector3d twiceCross = scale(cross(vectorPart, vector), 2.0);
    return add(add(vector, scale(twiceCross, quaternion.w)),
               cross(vectorPart, twiceCross));
}

RigidTransform compose(const RigidTransform& lhs, const RigidTransform& rhs)
{
    return {multiply(lhs.rotation, rhs.rotation),
            add(lhs.translation, rotate(lhs.rotation, rhs.translation))};
}

Matrix3 toRotationMatrix(const Quaternion& quaternion)
{
    const double w = quaternion.w;
    const double x = quaternion.x;
    const double y = quaternion.y;
    const double z = quaternion.z;

    return Matrix3{{
        {{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y)}},
        {{2.0 * (x * y + w * z),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)}},
        {{2.0 * (x * z - w * y),       2.0 * (y * z + w * x),       1.0 - 2.0 * (x * x + y * y)}}
    }};
}

RigidTransform numericForwardKinematics(const ik::KinematicChain& chain,
                                        const JointConfiguration& configuration)
{
    RigidTransform result;   // identity
    std::size_t next = 0;

    for (const auto& joint : chain.joints)
    {
        const RigidTransform origin{fromRollPitchYaw(joint.origin.rpy),
                                    toVector3d(joint.origin.translation)};

        if (!isActuated(joint.type))
        {
            result = compose(result, origin);
            continue;
        }

        assert(next < configuration.size() &&
               "configuration has fewer values than the chain has actuated joints");
        const double value = configuration[next++];

        RigidTransform motion;   // identity
        if (joint.type == ik::JointType::Prismatic)
            motion.translation = scale(toVector3d(joint.axis), value);
        else
            motion.rotation = fromAxisAngle(joint.axis, value);

        result = compose(result, compose(origin, motion));
    }

    assert(next == configuration.size() && "configuration has unused trailing values");
    return result;
}

ik::SymbolValues makeSymbolValues(const ik::KinematicChain& chain,
                                  const JointConfiguration& configuration)
{
    ik::SymbolValues values;
    std::size_t next = 0;

    for (const auto& joint : chain.joints)
    {
        if (!isActuated(joint.type))
            continue;

        assert(joint.variable.has_value() && "an actuated joint must carry a variable");
        assert(next < configuration.size() &&
               "configuration has fewer values than the chain has actuated joints");

        const auto [iterator, inserted] =
            values.emplace(joint.variable->name, configuration[next++]);
        (void) iterator;

        // A duplicate name means the KinematicChain contract is broken, not
        // that this configuration is malformed -- so it throws rather than
        // asserting, and stays live under NDEBUG.
        //
        // Detecting it here matters: emplace does NOT overwrite on a
        // duplicate, it simply does not insert. The symbolic side would then
        // silently evaluate two joints from one binding, and if the two
        // configuration values happened to be equal the matrix comparison
        // would not notice at all.
        if (!inserted)
            throw std::logic_error("duplicate actuated-joint variable name: " +
                                   joint.variable->name);
    }

    assert(next == configuration.size() && "configuration has unused trailing values");
    return values;
}

} // namespace kinemaforge::testsupport
```

### 3.1 Dwie drogi od jednej liczby do jednego jointu

`numericForwardKinematics` konsumuje `configuration[next++]` **po pozycji**, idąc po łańcuchu. `makeSymbolValues` mapuje tę samą pozycję na `joint.variable->name`, a strona symboliczna odnajduje ją **po nazwie**. Wspólna jest tylko liczba.

### 3.2 Duplikat nazwy — poprawka po review, i mój błąd rzeczowy

W v1 napisałem, że przy zduplikowanej nazwie `makeSymbolValues` „nadpisałoby wpis w mapie", więc rozjazd wykryłoby porównanie macierzy. **To nieprawda.** `unordered_map::emplace` przy duplikacie **nie wstawia i nie nadpisuje** — zwraca `inserted == false` i zostawia starą wartość.

Konsekwencja była gorsza niż sam błąd w opisie: mechanizm wykrywania duplikatu, dla którego rozdzieliłem adresowanie pozycyjne od nazwowego, **w ogóle nie istniał**. Strona symboliczna po cichu policzyłaby dwa jointy z jednego wiązania, a gdyby obie wartości konfiguracji były równe, porównanie macierzy nie zauważyłoby niczego.

Dlatego duplikat jest teraz odrzucany **jawnie**, przez `throw std::logic_error` — nie przez `assert`. Uzasadnienie: zduplikowana nazwa oznacza **złamany kontrakt `KinematicChain`**, a nie błędną konfigurację testową, więc sprawdzenie musi działać także pod `NDEBUG`. Preconditions samej konfiguracji (liczba wartości, obecność `variable`) zostają asercjami, bo tam błąd leży po stronie testu.

### 3.3 Umiejscowienie `makeSymbolValues` — odstępstwo od rekomendacji

Review sugeruje przeniesienie `makeSymbolValues` do `test_numeric_fk_validation.cpp`, żeby moduł referencji nie zależał od warstwy symbolicznej. Zgadzam się z celem, ale **proponuję zostawić funkcję tam, gdzie jest**, i chcę to uzasadnić, a nie przemilczeć.

Powód jest korektnościowy, nie estetyczny. Obie funkcje wykonują **ten sam przebieg po łańcuchu**: pomijają jointy nieaktywne i konsumują `configuration` sekwencyjnie. Dopóki leżą obok siebie, dzielą jeden `isActuated` i jedną definicję kolejności — więc **nie mogą się rozjechać**. Rozdzielone, każda dostałaby własne pojęcie „joint aktywny"; wystarczyłoby, że ktoś doda nowy `JointType` i zaktualizuje jedną kopię, żeby mapowanie pozycja → nazwa cicho przestało odpowiadać mapowaniu pozycja → transformacja. Wtedy referencja i strona symboliczna liczyłyby **różne konfiguracje**, a test porównywałby dwie poprawne odpowiedzi na dwa różne pytania.

Zależność, o którą chodzi review, jest przy tym bardzo cienka: `ExpressionEvaluator.hpp` jest włączony wyłącznie dla aliasu `SymbolValues`, czyli `std::unordered_map<std::string, double>`. Żaden algorytm FK ani nic z ewaluacji nie przechodzi tą drogą, więc niezależność oracle'a pozostaje nienaruszona.

**Do rozstrzygnięcia w review** — jeżeli mimo tego wolisz rozdzielenie, przeniosę funkcję i wyeksportuję `isActuated` z modułu referencji, żeby wspólny przebieg nadal miał jedno źródło.

---

## 4. `tests/test_numeric_fk_validation.cpp` (nowy plik) — 17 testów

```cpp
#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ik = kinemaforge::ik;
namespace support = kinemaforge::testsupport;

using support::JointConfiguration;
using support::Matrix3;
using support::Quaternion;
using support::RigidTransform;

namespace {

constexpr double kPi = std::numbers::pi;

// Candidate tolerance, approved as a candidate only. The implementation report
// must state the measured worst-case error; exceeding this bound is a finding
// for review, NOT a licence to raise the number.
constexpr double kAbsoluteTolerance = 1e-12;
constexpr double kRelativeTolerance = 1e-12;

using Matrix4 = std::array<std::array<double, 4>, 4>;

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

bool withinTolerance(double actual, double expected)
{
    return std::abs(actual - expected)
           <= kAbsoluteTolerance + kRelativeTolerance * std::abs(expected);
}

Matrix4 toMatrix4(const RigidTransform& transform)
{
    const Matrix3 rotation = support::toRotationMatrix(transform.rotation);

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

// One evaluator for all sixteen cells -- that is the whole reason
// ExpressionEvaluator is a session. A per-cell evaluator would drop the cache
// between roots.
std::optional<Matrix4> evaluateSymbolic(const ik::SymbolicTransform& fk,
                                        const ik::SymbolValues& values)
{
    ik::ExpressionEvaluator evaluator{values};

    Matrix4 numeric{};
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            const auto value = evaluator.evaluate(fk(row, column));
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

struct LoadedRobot
{
    ik::KinematicChain chain;
    ik::SymbolicTransform fk;
};

std::optional<LoadedRobot> loadRobot(const char* fileName)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath(fileName));

    ik::KinematicChainBuilder chainBuilder;
    auto chain = chainBuilder.build(robot, "base_link", "tool0");
    if (!chain)
    {
        ADD_FAILURE() << "chain build failed for " << fileName;
        return std::nullopt;
    }

    const ik::JointTransformBuilder transformBuilder;
    const ik::ForwardKinematicsBuilder fkBuilder;
    auto fk = fkBuilder.build(*chain, transformBuilder);

    return LoadedRobot{std::move(*chain), std::move(fk)};
}

std::size_t actuatedJointCount(const ik::KinematicChain& chain)
{
    std::size_t count = 0;
    for (const auto& joint : chain.joints)
        if (joint.variable) ++count;
    return count;
}

// Alternating lower + 5% of span, upper - 5% of span. Derived from the loaded
// model rather than hard-coded, so it stays correct if the URDF changes.
// Deliberately not the exact limits: this validates FK, not boundary handling.
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

void expectConfigurationMatchesReference(const LoadedRobot& robot,
                                         const JointConfiguration& configuration,
                                         std::string_view label)
{
    const auto symbolic =
        evaluateSymbolic(robot.fk, support::makeSymbolValues(robot.chain, configuration));
    if (!symbolic) return;

    const Matrix4 reference =
        toMatrix4(support::numericForwardKinematics(robot.chain, configuration));

    expectMatrixMatches(*symbolic, reference, label);
}

// --- synthetic chains -----------------------------------------------

ik::KinematicJoint makeJoint(ik::JointType type, ik::Vector3 axis,
                             ik::Vector3 translation, ik::Vector3 rpy,
                             std::string variableName = "q1")
{
    ik::KinematicJoint joint;
    joint.name = "j";
    joint.type = type;
    joint.axis = axis;
    joint.origin.translation = translation;
    joint.origin.rpy = rpy;
    if (type != ik::JointType::Fixed)
        joint.variable = ik::JointVariable{std::move(variableName), 1};
    return joint;
}

ik::KinematicChain makeChain(std::vector<ik::KinematicJoint> joints)
{
    ik::KinematicChain chain;
    chain.baseLink = "base";
    chain.toolLink = "tool";
    chain.joints = std::move(joints);
    return chain;
}

LoadedRobot buildSynthetic(std::vector<ik::KinematicJoint> joints)
{
    auto chain = makeChain(std::move(joints));
    const ik::JointTransformBuilder transformBuilder;
    const ik::ForwardKinematicsBuilder fkBuilder;
    auto fk = fkBuilder.build(chain, transformBuilder);
    return LoadedRobot{std::move(chain), std::move(fk)};
}

} // namespace

// --- KR4 against the quaternion reference ---------------------------

TEST(NumericFkValidationTest, EvaluatesKr4ZeroConfigurationAgainstNumericReference)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());
    ASSERT_EQ(actuatedJointCount(robot->chain), 6u);

    expectConfigurationMatchesReference(*robot, JointConfiguration(6, 0.0), "kr4 zero");
}

TEST(NumericFkValidationTest, EvaluatesKr4SingleJointConfigurationsAgainstNumericReference)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    // One joint at a time, so a fault in a single joint cannot be masked by
    // the others sitting at zero. +-0.25 is inside every limit of both robots.
    for (std::size_t index = 0; index < 6; ++index)
    {
        JointConfiguration configuration(6, 0.0);
        configuration[index] = (index % 2 == 0) ? 0.25 : -0.25;
        expectConfigurationMatchesReference(
            *robot, configuration, "kr4 single joint " + std::to_string(index + 1));
    }
}

TEST(NumericFkValidationTest, EvaluatesKr4MixedConfigurationAgainstNumericReference)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    const JointConfiguration configuration{0.35, -0.45, 0.55, -0.65, 0.40, -0.30};
    expectConfigurationMatchesReference(*robot, configuration, "kr4 mixed");
}

TEST(NumericFkValidationTest, EvaluatesKr4NearLimitsAgainstNumericReference)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    expectConfigurationMatchesReference(*robot, nearLimitConfiguration(robot->chain),
                                        "kr4 near limits");
}

// --- KR640 against the quaternion reference -------------------------

TEST(NumericFkValidationTest, EvaluatesKr640ZeroConfigurationAgainstNumericReference)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());
    ASSERT_EQ(actuatedJointCount(robot->chain), 6u);

    expectConfigurationMatchesReference(*robot, JointConfiguration(6, 0.0), "kr640 zero");
}

TEST(NumericFkValidationTest, EvaluatesKr640SingleJointConfigurationsAgainstNumericReference)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    for (std::size_t index = 0; index < 6; ++index)
    {
        JointConfiguration configuration(6, 0.0);
        configuration[index] = (index % 2 == 0) ? 0.25 : -0.25;
        expectConfigurationMatchesReference(
            *robot, configuration, "kr640 single joint " + std::to_string(index + 1));
    }
}

TEST(NumericFkValidationTest, EvaluatesKr640MixedConfigurationAgainstNumericReference)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    const JointConfiguration configuration{0.35, -0.45, 0.55, -0.65, 0.40, -0.30};
    expectConfigurationMatchesReference(*robot, configuration, "kr640 mixed");
}

TEST(NumericFkValidationTest, EvaluatesKr640NearLimitsAgainstNumericReference)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    expectConfigurationMatchesReference(*robot, nearLimitConfiguration(robot->chain),
                                        "kr640 near limits");
}

// --- hand-computed oracles ------------------------------------------

TEST(NumericFkValidationTest, Kr640ZeroConfigurationMatchesHandComputedPose)
{
    // Every kr640 joint has rpy="0 0 0", so at q = 0 every transform is a pure
    // translation and the tool position is a plain sum read off the URDF:
    //
    //   x = 0.350 + 1.250                  = 1.600
    //   z = 0.750 + 1.150 + 0.145 + 0.290  = 2.335
    //
    // Independent of BOTH implementations -- this is the only kind of check
    // that can catch a shared misunderstanding rather than a coding slip.
    //
    // It does NOT test composition order: pure translations commute, so
    // reversing the whole chain would give the same answer. See the quarter
    // turn test below for that.
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    const auto symbolic = evaluateSymbolic(
        robot->fk, support::makeSymbolValues(robot->chain, JointConfiguration(6, 0.0)));
    ASSERT_TRUE(symbolic.has_value());

    Matrix4 expected{};
    expected[0][0] = expected[1][1] = expected[2][2] = expected[3][3] = 1.0;
    expected[0][3] = 1.600;
    expected[1][3] = 0.000;
    expected[2][3] = 2.335;

    expectMatrixMatches(*symbolic, expected, "kr640 zero (hand computed)");
}

TEST(NumericFkValidationTest, Kr640Joint1QuarterTurnMatchesHandComputedPose)
{
    // q1 = pi/2 turns everything downstream about +Z. Translations past a1 sum
    // to x = 0.350 + 1.250 = 1.600 and z = 1.150 + 0.145 + 0.290 = 1.585, so
    //
    //   p = (0, 0, 0.750) + Rz(pi/2) * (1.600, 0, 1.585) = (0, 1.600, 2.335)
    //
    // Unlike the zero pose this DOES pin composition order, the sign of the a1
    // axis, and translation propagation through an earlier rotation.
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    JointConfiguration configuration(6, 0.0);
    configuration[0] = kPi / 2.0;

    const auto symbolic =
        evaluateSymbolic(robot->fk, support::makeSymbolValues(robot->chain, configuration));
    ASSERT_TRUE(symbolic.has_value());

    Matrix4 expected{};
    expected[0][0] = 0.0;  expected[0][1] = -1.0; expected[0][2] = 0.0; expected[0][3] = 0.000;
    expected[1][0] = 1.0;  expected[1][1] = 0.0;  expected[1][2] = 0.0; expected[1][3] = 1.600;
    expected[2][0] = 0.0;  expected[2][1] = 0.0;  expected[2][2] = 1.0; expected[2][3] = 2.335;
    expected[3][3] = 1.0;

    expectMatrixMatches(*symbolic, expected, "kr640 q1=pi/2 (hand computed)");
}

TEST(NumericFkValidationTest, EvaluatesNegativePrincipalAxisAgainstHandComputedPose)
{
    // Neither kr4 nor kr640 has a negative principal axis, so the negated
    // branch of JointTransformBuilder's fast path is never exercised by real
    // data. R(-Z, pi/2) = Rz(-pi/2).
    const auto robot = buildSynthetic(
        {makeJoint(ik::JointType::Revolute, {0.0, 0.0, -1.0}, {}, {}, "q1")});

    const auto symbolic =
        evaluateSymbolic(robot.fk, support::makeSymbolValues(robot.chain, {kPi / 2.0}));
    ASSERT_TRUE(symbolic.has_value());

    Matrix4 expected{};
    expected[0][0] =  0.0; expected[0][1] = 1.0; expected[0][2] = 0.0;
    expected[1][0] = -1.0; expected[1][1] = 0.0; expected[1][2] = 0.0;
    expected[2][2] =  1.0;
    expected[3][3] =  1.0;

    expectMatrixMatches(*symbolic, expected, "negative Z axis (hand computed)");
}

// --- synthetic, against the quaternion reference --------------------

TEST(NumericFkValidationTest, EvaluatesArbitraryAxisRevoluteAgainstQuaternionReference)
{
    // Both robots use principal axes only, so the general Rodrigues branch is
    // never reached by real data -- every real joint takes the fast path.
    const double scale = 1.0 / std::sqrt(14.0);
    const ik::Vector3 axis{1.0 * scale, 2.0 * scale, 3.0 * scale};

    const auto robot = buildSynthetic(
        {makeJoint(ik::JointType::Revolute, axis, {0.1, 0.2, 0.3}, {0.3, -0.2, 0.5}, "q1")});

    expectConfigurationMatchesReference(robot, {0.7}, "arbitrary axis revolute");
}

TEST(NumericFkValidationTest, EvaluatesRotatedPrismaticAgainstQuaternionReference)
{
    // No real robot in data/urdf has a prismatic joint, so without this the
    // branch stays numerically unvalidated. Origin turns about Z by pi/2 and
    // the slide is along X, so the displacement must land on +Y:
    //
    //   Rz(pi/2) * (0.3, 0, 0) = (0, 0.3, 0)
    //
    // The expectation is hand computed, not taken from the reference.
    const auto robot = buildSynthetic({makeJoint(
        ik::JointType::Prismatic, {1.0, 0.0, 0.0}, {}, {0.0, 0.0, kPi / 2.0}, "q1")});

    const auto symbolic =
        evaluateSymbolic(robot.fk, support::makeSymbolValues(robot.chain, {0.3}));
    ASSERT_TRUE(symbolic.has_value());

    Matrix4 expected{};
    expected[0][0] = 0.0; expected[0][1] = -1.0; expected[0][2] = 0.0; expected[0][3] = 0.0;
    expected[1][0] = 1.0; expected[1][1] =  0.0; expected[1][2] = 0.0; expected[1][3] = 0.3;
    expected[2][2] = 1.0; expected[2][3] = 0.0;
    expected[3][3] = 1.0;

    expectMatrixMatches(*symbolic, expected, "rotated prismatic (hand computed)");

    // Also cross-check against the quaternion reference, so the test covers
    // both oracles for this branch.
    const Matrix4 reference =
        toMatrix4(support::numericForwardKinematics(robot.chain, {0.3}));
    expectMatrixMatches(*symbolic, reference, "rotated prismatic (reference)");
}

// --- the reference itself -------------------------------------------

TEST(NumericFkValidationTest, RejectsDuplicateJointVariableNames)
{
    // The guarantee exists in makeSymbolValues; without this test nothing
    // pins it. A duplicate name would otherwise let the symbolic side
    // evaluate two joints from one binding -- invisible whenever the two
    // configuration values happen to be equal.
    const auto chain = makeChain({
        makeJoint(ik::JointType::Revolute, {0.0, 0.0, 1.0}, {}, {}, "q1"),
        makeJoint(ik::JointType::Revolute, {0.0, 1.0, 0.0}, {}, {}, "q1"),
    });

    EXPECT_THROW(support::makeSymbolValues(chain, {0.1, 0.2}), std::logic_error);
}

TEST(NumericFkValidationTest, NumericReferenceUsesCorrectRpyOrder)
{
    // rpy = (pi/2, 0, -pi/2), taken from joint_4 of kr4_r600.urdf: the two
    // conventions disagree here, whereas a single non-zero component would
    // give the same matrix either way and prove nothing.
    const Quaternion quaternion =
        support::fromRollPitchYaw(ik::Vector3{kPi / 2.0, 0.0, -kPi / 2.0});
    const Matrix3 rotation = support::toRotationMatrix(quaternion);

    // Rz(-pi/2) * Ry(0) * Rx(pi/2). The reversed order gives
    // {{0,1,0},{0,0,-1},{-1,0,0}}.
    const double expected[3][3] = {{0, 0, -1}, {-1, 0, 0}, {0, 1, 0}};

    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            EXPECT_NEAR(rotation[row][column], expected[row][column], 1e-12);
        }
}

TEST(NumericFkValidationTest, NumericReferenceUsesCorrectCompositionOrder)
{
    // compose(a, b) must mean a * b: p = p_a + R_a * p_b.
    //
    // a = Rz(pi/2) with no translation, b = pure translation along X. The
    // correct order puts the result on +Y; the reversed one leaves it on +X.
    const RigidTransform a{support::fromAxisAngle(ik::Vector3{0.0, 0.0, 1.0}, kPi / 2.0),
                           {0.0, 0.0, 0.0}};
    const RigidTransform b{Quaternion{}, {1.0, 0.0, 0.0}};

    const RigidTransform composed = support::compose(a, b);

    EXPECT_NEAR(composed.translation.x, 0.0, 1e-12);
    EXPECT_NEAR(composed.translation.y, 1.0, 1e-12);
    EXPECT_NEAR(composed.translation.z, 0.0, 1e-12);
}

TEST(NumericFkValidationTest, NumericReferenceProducesProperRigidTransform)
{
    // A unit quaternion encodes a proper rotation mathematically, but in double
    // arithmetic the norm drifts and a wrong conversion can still produce a
    // non-orthogonal matrix. So the invariants are asserted, not assumed.
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    const JointConfiguration configuration{0.35, -0.45, 0.55, -0.65, 0.40, -0.30};
    const RigidTransform result =
        support::numericForwardKinematics(robot->chain, configuration);

    EXPECT_NEAR(support::norm(result.rotation), 1.0, 1e-12);

    const Matrix3 rotation = support::toRotationMatrix(result.rotation);

    // R^T * R == I
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
        {
            SCOPED_TRACE(testing::Message() << "R^T R (" << row << ", " << column << ")");
            double value = 0.0;
            for (std::size_t k = 0; k < 3; ++k)
                value += rotation[k][row] * rotation[k][column];
            EXPECT_NEAR(value, row == column ? 1.0 : 0.0, 1e-12);
        }

    // det(R) == +1, not -1: a reflection would satisfy R^T R = I too.
    const double determinant =
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1]) -
        rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0]) +
        rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0]);
    EXPECT_NEAR(determinant, 1.0, 1e-12);

    // The homogeneous last row, once embedded in 4x4.
    const Matrix4 embedded = toMatrix4(result);
    EXPECT_DOUBLE_EQ(embedded[3][0], 0.0);
    EXPECT_DOUBLE_EQ(embedded[3][1], 0.0);
    EXPECT_DOUBLE_EQ(embedded[3][2], 0.0);
    EXPECT_DOUBLE_EQ(embedded[3][3], 1.0);
}
```

---

## 5. Uwagi do konstrukcji testów

### 5.1 Jeden evaluator na macierz

`evaluateSymbolic` tworzy **jeden** `ExpressionEvaluator` i przepuszcza przez niego wszystkie 16 komórek. To pierwsze realne użycie sesyjności evaluatora poza mikrotestami — i jedyny sposób, żeby cache działał między korzeniami.

### 5.2 Błąd ewaluacji przerywa, nie porównuje

`evaluate` zwraca `std::expected`. Jeżeli którakolwiek komórka zawiedzie, `evaluateSymbolic` zgłasza `ADD_FAILURE` z kodem i nazwą symbolu, i zwraca `std::nullopt`. Żadne porównanie nie zobaczy niezainicjowanej wartości.

### 5.3 Konfiguracje zweryfikowane wobec limitów obu robotów

Odczytane z plików URDF:

| Joint | KR4 | KR640 |
|---|---|---|
| 1 | ±2.9671 | ±3.2288 |
| 2 | [−3.4034, **0.6981**] | [−2.0944, **0.6109**] |
| 3 | [−2.0071, 2.6180] | [−2.0944, 2.7576] |
| 4 | ±3.2289 | ±6.1087 |
| 5 | ±2.0944 | ±2.1817 |
| 6 | ±6.1087 | ±6.1087 |

Najciaśniejsza granica to górny limit jointu 2: **0.6109** dla KR640. Zarówno `±0.25`, jak i wektor mieszany `{0.35, −0.45, 0.55, −0.65, 0.40, −0.30}` mieszczą się w limitach **obu** robotów — sprawdzone pozycja po pozycji. Konfiguracja mieszana z promptu nie wymagała korekty.

Konfiguracja „blisko limitów" liczona jest **z załadowanego modelu**, nie zaszyta.

### 5.4 Pomiar drukowany bezwarunkowo

`expectMatrixMatches` wypisuje `[ MEASURE ]` z maksymalnym błędem obrotu, translacji i kątem orientacji dla **każdego** porównania, także zaliczonego. Raport z wdrożenia cytuje te linie — nie ma potrzeby dopisywania osobnego trybu pomiarowego.

---

## 6. `tests/CMakeLists.txt` — trzy zmiany

```cmake
add_executable(kinemaforge_tests
    test_kinematics.cpp
    test_urdf_model_loader.cpp
    test_kinematic_chain_builder.cpp
    test_joint_transform_builder.cpp
    test_forward_kinematics_builder.cpp
    test_numeric_fk_validation.cpp
    test_symbolic_expression.cpp
    test_expression_factory.cpp
    test_expression_evaluator.cpp
    test_symbolic_matrix.cpp
    test_symbolic_transform.cpp
    support/NumericForwardKinematics.cpp
)
target_link_libraries(kinemaforge_tests PRIVATE kinemaforge_ik GTest::gtest_main)
target_include_directories(kinemaforge_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(kinemaforge_tests PRIVATE
    KINEMAFORGE_URDF_DATA_DIR="${CMAKE_SOURCE_DIR}/data/urdf"
)
```

`target_include_directories` na `tests/` jest konieczne, żeby `#include "support/NumericForwardKinematics.hpp"` rozwiązywało się także z `support/NumericForwardKinematics.cpp`.

**`src/` i główny `CMakeLists.txt` pozostają nietknięte.**

---

## 7. `STATUS.md` — zmiany

1. **Nagłówek** — `187/187` → `204/204`.
2. **Tabela postępu** — Phase 1 `~90%` → `~95%`.
3. **Diagram** — `numeric FK validation` ⬜ → ✅; `IkEquationBuilder` jako jedyna pozycja Fazy 1.
4. **„Done"** — nowy wpis: referencja kwaternionowa w testach, 18 porównań macierzy, dwa oracle ręczne, zmierzona tolerancja.
5. **Known gaps — przepisać wiersz „FK is not numerically validated"** na węższy: walidacja obejmuje potok **od `KinematicChain` w górę**; loader i chain builder nie są przez nią sprawdzane, bo obie strony czytają ten sam łańcuch.
6. **Known gaps — nowy wiersz:** konwencja RPY dla nietrywialnych kątów potwierdzona zgodnością dwóch implementacji **tego samego autora z tego samego przekonania**; pełne domknięcie wymagałoby zewnętrznego narzędzia (KDL / `tf2` / Pinocchio) albo opublikowanych poz KR4. Świadomie odłożone.
7. **„What Phase 1 still needs"** — zostaje wyłącznie fasada.
8. **Tabela dokumentów** — architektura `approved (v2)`, ten proposal `implemented`.
9. **Next step** — `IkEquationBuilder`.

---

## 8. Ryzyka — co może wyjść przy budowaniu

### 8.1 `orientationAngleError` — **naprawione w kodzie**

W v1 zostawiłem w bloku kodu wersję z martwym warunkiem `if (index == index)` i dopiskiem „przy wdrożeniu użyć innej". Review słusznie to odrzuciło z dwóch powodów: pod `-Werror=tautological-compare` to **nie kompiluje się w ogóle**, a proposal implementacyjny ma zawierać wyłącznie kod przeznaczony do skopiowania — nie kod plus errata.

Blok w §4 zawiera teraz poprawną wersję i nie ma już drugiej. Ryzyko zamknięte.

### 8.2 Wynik testu — ryzyko średnie i pożądane

Ten zestaw powstaje **po to**, żeby mógł wykryć błąd. Jeżeli któraś konfiguracja nie przejdzie, raport poda robota, konfigurację, komórkę, `actual`, `expected` i kąt orientacji — a poprawka pójdzie **osobnym proposalem**, zgodnie z §9 architektury. Nie będę naprawiał produkcji w tej samej zmianie, która dodaje test.

### 8.3 Tolerancja — **zmierzona w review**, ryzyko zamknięte

Review uruchomiło obie ścieżki na łańcuchach odtworzonych z aktualnych URDF-ów:

| Przypadek | rotacja | translacja |
|---|---:|---:|
| KR4 | 5.55e-16 | 2.22e-16 |
| KR640 | 2.22e-16 | 2.22e-16 |
| dowolna oś | 2.22e-16 | — |
| prismatic | 1.61e-16 | — |
| oracle KR640 `q1 = π/2` | \~9.80e-17 | \~9.80e-17 |

Najgorszy przypadek `5.55e-16` to około **trzy rzędy wielkości zapasu** do kandydata `1e-12`. Moje oczekiwanie z v1 (`1e-15`–`1e-14`) było o rząd zbyt pesymistyczne; realny błąd siedzi na poziomie kilku ULP `double`.

Kandydat `1e-12 / 1e-12` zostaje bez zmian — konserwatywny, ale nie na tyle szeroki, żeby przepuścić realny błąd geometryczny, który objawiałby się rzędem `1e-3` i większym. Reguła obowiązuje niezmieniona: przekroczenie przy wdrożeniu to raport i osobne review, nie podniesienie liczby.

---

## 9. Zgodność z zatwierdzoną architekturą

| Decyzja (architektura v2) | Gdzie |
|---|---|
| referencja kwaternionowa poza `kinemaforge_ik` | `tests/support/`, dodana do `kinemaforge_tests` |
| obrót wektora bez konwersji do macierzy | `rotate` — wzór `v + 2w(u×v) + 2u×(u×v)` |
| macierz 3×3 tylko na końcu | `toRotationMatrix`, wołane wyłącznie z `toMatrix4` i testów referencji |
| brak bezwarunkowej normalizacji kwaternionu | `norm` tylko sprawdzana w teście, nigdzie nie normalizujemy |
| `JointConfiguration` po pozycji | `numericForwardKinematics`, `makeSymbolValues` |
| asercje spójności konfiguracji | asercje kompletności konfiguracji oraz kontraktu zmiennych jointów; duplikat nazwy odrzucany przez `throw` |
| jeden evaluator na macierz | `evaluateSymbolic` |
| limity z modelu, nie zaszyte | `nearLimitConfiguration` |
| cztery grupy konfiguracji × 2 roboty | osiem testów |
| dwa oracle ręczne | `Kr640ZeroConfiguration…`, `Kr640Joint1QuarterTurn…` |
| test ujemnej osi głównej | `EvaluatesNegativePrincipalAxis…` |
| `NumericReferenceProducesProperRigidTransform` | `RᵀR`, `det`, `‖q‖`, ostatni wiersz |
| tolerancja abs + rel, `1e-12` jako kandydat | `withinTolerance` |
| kąt orientacji jako diagnostyka, nie warunek | `orientationAngleError`, tylko w wydruku |
| brak zmian w `src/` | plan zmian §6 |

---

## 10. Jak to zweryfikuję po zatwierdzeniu

```powershell
cmake -B build -G Ninja
cmake --build build --clean-first
ctest --test-dir build --output-on-failure
.\build\tests\kinemaforge_tests.exe --gtest_filter=NumericFkValidationTest.*
```

Oczekiwane: **187 + 17 = 204 zielone.**

Raport poda, zgodnie z zatwierdzoną zasadą: **maksymalny błąd elementu rotacji i translacji**, robota, konfigurację, komórkę, `actual`, `expected` oraz kąt orientacji — dla najgorszego przypadku ze wszystkich 18 porównań macierzy. Przekroczenie `1e-12` zaraportuję jako ustalenie do review, nie skoryguję tolerancji.

## Do zatwierdzenia

1. Kod referencji (§2, §3) i testów (§4) — **bloki kodu są teraz kompletne i przeznaczone do skopiowania bez erraty**, plus zmiany w `tests/CMakeLists.txt` (§6) i `STATUS.md` (§7).
2. **§3.3 — umiejscowienie `makeSymbolValues`.** Proponuję zostawić w module referencji (argument: wspólny przebieg po łańcuchu nie może się rozjechać). Jeżeli wolisz rozdzielenie, przeniosę i wyeksportuję `isActuated`.

Czekam na ok przed naniesieniem na pliki źródłowe. Bez commita.
