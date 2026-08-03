# Proposal: `IkEquationBuilder` — implementacja

## Prompt

> APPROVE z obowiązkowymi korektami przed proposalem implementacyjnym. Do przeniesienia: `catch (const std::runtime_error&)`, nie `std::exception`; wskaźnikowe akcesory z jawnym kontraktem unieważniania; `[[nodiscard]]` na operacjach i akcesorach; aktualizacja `README.md`; pozostawienie `main.cpp` bez zmian; poprawne oczekiwania testów akcesorów — `nullptr`, nie kod błędu; inwariant pola `chainError`.

Realizacja architektury zatwierdzonej w `proposal-ik-equation-builder-architecture.md`. Ostatni komponent Fazy 1.

## Status weryfikacji

**Kod poniżej nie został skompilowany.** Proposal jest dokumentem; kod trafia na dysk po zatwierdzeniu.

Zweryfikowane przez lekturę repo: `UrdfModelLoader::load` rzuca `std::runtime_error` (`UrdfModelLoader.cpp:86`); `KinematicChainError` ma cztery wartości; `IkEquationBuilder.cpp` zawiera dziś wyłącznie domyślny konstruktor; przykład w `README.md` (linie 35–42) używa starych sygnatur.

## Siedem korekt z review — wprowadzone

| # | Korekta | Gdzie |
|---|---|---|
| 1 | `catch (const std::runtime_error&)`, nie `std::exception` — moja rekomendacja **przeczyła sama sobie** | §3, §5.1 |
| 2 | wskaźnikowe akcesory + **jawny kontrakt unieważniania** | §2, §5.2 |
| 3 | `[[nodiscard]]` na operacjach i akcesorach | §2 |
| 4 | `README.md` do planu zmian | §6 |
| 5 | `main.cpp` bez zmian | §7 |
| 6 | testy akcesorów sprawdzają `nullptr`, nie kod błędu — moja §10.2 była **wewnętrznie sprzeczna** | §4 |
| 7 | inwariant `chainError` zapisany i przetestowany | §2, §4 |

---

## 1. Stan obecny

`IkEquationBuilder.cpp`:

```cpp
#include "ik_equations/IkEquationBuilder.hpp"

namespace kinemaforge::ik {

IkEquationBuilder::IkEquationBuilder() = default;

} // namespace kinemaforge::ik
```

Nagłówek deklaruje pięć niezdefiniowanych metod i trzy pola bez `optional`. Stan testów: 204/204. Po tej zmianie oczekiwane **221/221**.

---

## 2. `src/ik_equations/IkEquationBuilder.hpp` (nowa pełna treść)

```cpp
#pragma once

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/model/KinematicChainError.hpp"
#include "ik_equations/model/RobotDescription.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <expected>
#include <filesystem>
#include <optional>
#include <string>

namespace kinemaforge::ik {

enum class IkEquationBuilderErrorCode
{
    RobotModelNotLoaded,
    KinematicChainNotSelected,
    UrdfLoadFailed,
    ChainBuildFailed
};

// Errors returned by IkEquationBuilder satisfy this invariant: chainError has
// a value if and only if code == ChainBuildFailed. The type is a public
// aggregate, so a caller can hand-build an inconsistent value; that is not
// worth a variant or a factory, but the guarantee is about what this class
// produces, not about what the struct can hold.
//
// UrdfLoadFailed carries only `message`, and that is not an oversight:
// UrdfModelLoader consumes its structured LoadError internally and throws a
// std::runtime_error carrying an already-assembled string, so no typed code
// survives to be preserved here. Giving the facade a typed load error means
// changing UrdfModelLoader -- a separate change, recorded in STATUS.md.
struct IkEquationBuilderError
{
    IkEquationBuilderErrorCode code{};
    std::string message;
    std::optional<KinematicChainError> chainError;
};

// The public entry point of this module: URDF file -> symbolic forward
// kinematics, with everything underneath kept private.
//
// State machine, with each successful step invalidating what it obsoletes:
//
//     loadRobotModel         -> new model, clears chain and transform
//     selectChain            -> new chain, clears transform
//     buildForwardKinematics -> new transform
//
// A KinematicChain names links of one specific robot and a transform carries
// symbols of one specific chain, so anything surviving a change upstream would
// be an answer to a question no longer being asked.
//
// Failure leaves the object untouched: results are built into locals and only
// committed once they exist. That promise covers domain errors -- a bad path,
// a missing link -- not catastrophic ones such as std::bad_alloc.
//
// Not safe for concurrent modification, like any object with state.
class IkEquationBuilder
{
public:
    IkEquationBuilder();

    [[nodiscard]] std::expected<void, IkEquationBuilderError>
    loadRobotModel(const std::filesystem::path& urdfPath);

    [[nodiscard]] std::expected<void, IkEquationBuilderError>
    selectChain(const std::string& baseLink, const std::string& toolLink);

    [[nodiscard]] std::expected<void, IkEquationBuilderError>
    buildForwardKinematics();

    // Returns nullptr until the corresponding step has succeeded.
    //
    // Returned pointers are non-owning and may be invalidated by any
    // successful state-changing operation on this object, as well as by its
    // destruction. Do not hold one across a call to loadRobotModel,
    // selectChain or buildForwardKinematics.
    [[nodiscard]] const KinematicChain* kinematicChain() const noexcept;
    [[nodiscard]] const SymbolicTransform* forwardKinematics() const noexcept;

private:
    UrdfModelLoader urdfLoader_;
    KinematicChainBuilder chainBuilder_;
    JointTransformBuilder jointTransformBuilder_;
    ForwardKinematicsBuilder fkBuilder_;

    std::optional<RobotDescription> robotDescription_;
    std::optional<KinematicChain> kinematicChain_;
    std::optional<SymbolicTransform> forwardKinematics_;
};

} // namespace kinemaforge::ik
```

### 2.1 Dlaczego `optional`, a nie zwykłe pola

Nie tylko „pusty obiekt wygląda jak wynik". Konkretnie: domyślnie skonstruowany `SymbolicTransform` to macierz **samych zer** — nawet nie jednostkowa. Zwrócony po nieudanej sekwencji byłby cichym, geometrycznie bezsensownym wynikiem, a nie oczywistą pustką.

### 2.2 Dlaczego wskaźnik, a nie `expected` w akcesorach

Akcesor może zawieść z **dokładnie jednego** powodu — odpowiedni krok nie został wykonany. `std::expected` przenosi informację *dlaczego*; gdy „dlaczego" jest jedno, degeneruje się do droższego `bool` z obowiązkowym `->value().get()` u wołającego.

Kontrakt unieważniania jest w komentarzu, bo jest realny: `const auto* fk = builder.forwardKinematics();` po którym następuje udany `selectChain` zostawia wiszący wskaźnik. To nie wada surowego wskaźnika — referencja czy `optional&` miałyby ten sam problem — ale musi być powiedziane wprost.

---

## 3. `src/ik_equations/IkEquationBuilder.cpp` (nowa pełna treść)

```cpp
#include "ik_equations/IkEquationBuilder.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace kinemaforge::ik {

namespace {

std::unexpected<IkEquationBuilderError> makeError(IkEquationBuilderErrorCode code,
                                                  std::string message)
{
    return std::unexpected(
        IkEquationBuilderError{code, std::move(message), std::nullopt});
}

const char* describe(KinematicChainError error) noexcept
{
    switch (error)
    {
    case KinematicChainError::BaseLinkNotFound:       return "base link not found";
    case KinematicChainError::ToolLinkNotFound:       return "tool link not found";
    case KinematicChainError::NoPathFound:            return "no path from base to tool";
    case KinematicChainError::InvalidRobotDescription: return "invalid robot description";
    }
    return "unknown kinematic chain error";
}

} // namespace

IkEquationBuilder::IkEquationBuilder() = default;

std::expected<void, IkEquationBuilderError>
IkEquationBuilder::loadRobotModel(const std::filesystem::path& urdfPath)
{
    RobotDescription loaded;
    try
    {
        loaded = urdfLoader_.load(urdfPath);
    }
    catch (const std::runtime_error& error)
    {
        // Exactly std::runtime_error, which is what UrdfModelLoader throws.
        // Catching std::exception would also swallow std::bad_alloc and report
        // it as a URDF problem -- a lie the caller cannot see through. Anything
        // else propagates.
        return makeError(IkEquationBuilderErrorCode::UrdfLoadFailed, error.what());
    }

    // Committed only past the throw: on failure the previous model, chain and
    // transform all survive untouched. Clearing first would leave a failed load
    // worse off than no call at all.
    robotDescription_ = std::move(loaded);
    kinematicChain_.reset();
    forwardKinematics_.reset();
    return {};
}

std::expected<void, IkEquationBuilderError>
IkEquationBuilder::selectChain(const std::string& baseLink, const std::string& toolLink)
{
    if (!robotDescription_)
        return makeError(IkEquationBuilderErrorCode::RobotModelNotLoaded,
                         "no robot model loaded; call loadRobotModel first");

    auto selected = chainBuilder_.build(*robotDescription_, baseLink, toolLink);
    if (!selected)
        return std::unexpected(IkEquationBuilderError{
            IkEquationBuilderErrorCode::ChainBuildFailed,
            "cannot select chain '" + baseLink + "' -> '" + toolLink + "': " +
                describe(selected.error()),
            selected.error()});

    kinematicChain_ = std::move(*selected);
    forwardKinematics_.reset();
    return {};
}

std::expected<void, IkEquationBuilderError>
IkEquationBuilder::buildForwardKinematics()
{
    if (!kinematicChain_)
        return makeError(IkEquationBuilderErrorCode::KinematicChainNotSelected,
                         "no kinematic chain selected; call selectChain first");

    // Cannot fail: an empty chain is a valid input yielding the identity.
    forwardKinematics_ = fkBuilder_.build(*kinematicChain_, jointTransformBuilder_);
    return {};
}

const KinematicChain* IkEquationBuilder::kinematicChain() const noexcept
{
    return kinematicChain_ ? &*kinematicChain_ : nullptr;
}

const SymbolicTransform* IkEquationBuilder::forwardKinematics() const noexcept
{
    return forwardKinematics_ ? &*forwardKinematics_ : nullptr;
}

} // namespace kinemaforge::ik
```

### 3.1 Inwariant `chainError` w praktyce

`makeError` **zawsze** ustawia `chainError` na `std::nullopt`, a jedyne miejsce, które je wypełnia, to gałąź `ChainBuildFailed` w `selectChain`. Inwariant z §2 wynika więc z konstrukcji, a nie z dyscypliny wołających — nie ma ścieżki, którą dałoby się zbudować `UrdfLoadFailed` z ustawionym `chainError`.

`describe()` istnieje po to, żeby `message` niósł informację także dla kogoś, kto ogląda tylko string — ale typowany kod zostaje obok, więc nikt nie musi parsować tekstu.

---

## 4. `tests/test_ik_equation_builder.cpp` (nowy plik) — 17 testów

```cpp
#include <gtest/gtest.h>

#include "ik_equations/IkEquationBuilder.hpp"

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>

using kinemaforge::ik::Expression;
using kinemaforge::ik::IkEquationBuilder;
using kinemaforge::ik::IkEquationBuilderErrorCode;
using kinemaforge::ik::KinematicChainError;
using kinemaforge::ik::SymbolicTransform;
using kinemaforge::ik::SymbolNode;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::structurallyEqual;

namespace {

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

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

bool transformContainsSymbol(const SymbolicTransform& transform, std::string_view name)
{
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
            if (containsSymbol(transform(row, column), name)) return true;
    return false;
}

void expectHomogeneousLastRow(const SymbolicTransform& transform)
{
    EXPECT_TRUE(isZero(transform(3, 0)));
    EXPECT_TRUE(isZero(transform(3, 1)));
    EXPECT_TRUE(isZero(transform(3, 2)));
    EXPECT_TRUE(isOne(transform(3, 3)));
}

// Drives the whole pipeline the way README shows it.
void expectBuildsThroughFacade(IkEquationBuilder& builder, const char* fileName)
{
    ASSERT_TRUE(builder.loadRobotModel(urdfPath(fileName)).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    const auto* chain = builder.kinematicChain();
    ASSERT_NE(chain, nullptr);
    EXPECT_EQ(chain->joints.size(), 7u);

    const auto* fk = builder.forwardKinematics();
    ASSERT_NE(fk, nullptr);

    for (const auto& joint : chain->joints)
        if (joint.variable)
        {
            SCOPED_TRACE(joint.variable->name);
            EXPECT_TRUE(transformContainsSymbol(*fk, joint.variable->name));
        }

    expectHomogeneousLastRow(*fk);
}

} // namespace

// --- happy path -----------------------------------------------------

TEST(IkEquationBuilderTest, LoadsRobotModel)
{
    IkEquationBuilder builder;

    EXPECT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());

    // Loading a model alone produces nothing downstream.
    EXPECT_EQ(builder.kinematicChain(), nullptr);
    EXPECT_EQ(builder.forwardKinematics(), nullptr);
}

TEST(IkEquationBuilderTest, SelectsChainAfterLoadingModel)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());

    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());

    const auto* chain = builder.kinematicChain();
    ASSERT_NE(chain, nullptr);
    EXPECT_EQ(chain->joints.size(), 7u);
    EXPECT_EQ(builder.forwardKinematics(), nullptr);
}

TEST(IkEquationBuilderTest, BuildsForwardKinematicsEndToEnd)
{
    // Exactly the sequence README documents as the public entry point.
    IkEquationBuilder builder;

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    ASSERT_NE(builder.forwardKinematics(), nullptr);
}

TEST(IkEquationBuilderTest, BuildsKr4ThroughFacade)
{
    IkEquationBuilder builder;
    expectBuildsThroughFacade(builder, "kr4_r600.urdf");
}

TEST(IkEquationBuilderTest, BuildsKr640ThroughFacade)
{
    IkEquationBuilder builder;
    expectBuildsThroughFacade(builder, "kr640.urdf");
}

// --- calls out of order ---------------------------------------------

TEST(IkEquationBuilderTest, RejectsChainSelectionBeforeLoadingModel)
{
    IkEquationBuilder builder;

    const auto result = builder.selectChain("base_link", "tool0");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::RobotModelNotLoaded);
    EXPECT_FALSE(result.error().chainError.has_value());
}

TEST(IkEquationBuilderTest, RejectsForwardKinematicsBeforeSelectingChain)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());

    const auto result = builder.buildForwardKinematics();

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::KinematicChainNotSelected);
    // The other side of the chainError invariant.
    EXPECT_FALSE(result.error().chainError.has_value());
}

TEST(IkEquationBuilderTest, RejectsChainAccessBeforeSelection)
{
    // An accessor reports absence with nullptr, not with an error code -- it
    // performs no operation that could fail in more than one way.
    IkEquationBuilder builder;
    EXPECT_EQ(builder.kinematicChain(), nullptr);

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    EXPECT_EQ(builder.kinematicChain(), nullptr);
}

TEST(IkEquationBuilderTest, RejectsForwardKinematicsAccessBeforeBuild)
{
    IkEquationBuilder builder;
    EXPECT_EQ(builder.forwardKinematics(), nullptr);

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    EXPECT_EQ(builder.forwardKinematics(), nullptr);
}

// --- invalidation ---------------------------------------------------

TEST(IkEquationBuilderTest, LoadingNewRobotClearsChain)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_NE(builder.kinematicChain(), nullptr);

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr4_r600.urdf")).has_value());

    // A chain names links of one specific robot.
    EXPECT_EQ(builder.kinematicChain(), nullptr);
}

TEST(IkEquationBuilderTest, LoadingNewRobotClearsForwardKinematics)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());
    ASSERT_NE(builder.forwardKinematics(), nullptr);

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr4_r600.urdf")).has_value());

    EXPECT_EQ(builder.forwardKinematics(), nullptr);
}

TEST(IkEquationBuilderTest, SelectingNewChainClearsForwardKinematics)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr4_r600.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());
    ASSERT_NE(builder.forwardKinematics(), nullptr);

    // base_link -> base is a different, valid chain of the same robot.
    ASSERT_TRUE(builder.selectChain("base_link", "base").has_value());

    EXPECT_EQ(builder.forwardKinematics(), nullptr);
    ASSERT_NE(builder.kinematicChain(), nullptr);
    EXPECT_EQ(builder.kinematicChain()->joints.size(), 1u);
}

// --- state preserved on failure -------------------------------------

TEST(IkEquationBuilderTest, FailedLoadPreservesPreviousState)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    // Captured before the failing call: asserting only "still not null" would
    // not prove the promise, which is that the object is *untouched*. Pointer
    // identity also pins that a failed operation does not invalidate pointers
    // handed out earlier.
    const auto* chainBefore = builder.kinematicChain();
    const auto* fkBefore = builder.forwardKinematics();
    ASSERT_NE(chainBefore, nullptr);
    ASSERT_NE(fkBefore, nullptr);

    const auto result = builder.loadRobotModel(urdfPath("does_not_exist.urdf"));

    ASSERT_FALSE(result.has_value());
    // Clearing before loading would leave a failed call worse off than no call.
    EXPECT_EQ(builder.kinematicChain(), chainBefore);
    EXPECT_EQ(builder.forwardKinematics(), fkBefore);
}

TEST(IkEquationBuilderTest, FailedChainSelectionPreservesPreviousState)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr4_r600.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    const auto* chainBefore = builder.kinematicChain();
    const auto* fkBefore = builder.forwardKinematics();
    ASSERT_NE(chainBefore, nullptr);
    ASSERT_NE(fkBefore, nullptr);

    const auto result = builder.selectChain("no_such_link", "tool0");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(builder.kinematicChain(), chainBefore);
    EXPECT_EQ(builder.forwardKinematics(), fkBefore);
    EXPECT_EQ(builder.kinematicChain()->joints.size(), 7u);
}

TEST(IkEquationBuilderTest, PropagatesChainBuilderError)
{
    // flange and tool0 both exist in kr4_r600.urdf but hang off link_6 as
    // siblings, so there is no downward path between them. The typed code must
    // survive the facade rather than be flattened into a string.
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr4_r600.urdf")).has_value());

    const auto result = builder.selectChain("flange", "tool0");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::ChainBuildFailed);
    ASSERT_TRUE(result.error().chainError.has_value());
    EXPECT_EQ(*result.error().chainError, KinematicChainError::NoPathFound);
}

TEST(IkEquationBuilderTest, ReportsUrdfLoadFailure)
{
    // The message is all that survives a loader failure -- UrdfModelLoader
    // consumes its structured LoadError and throws a string. So the message
    // being non-empty is the whole diagnostic, and needs pinning.
    IkEquationBuilder builder;

    const auto result = builder.loadRobotModel(urdfPath("does_not_exist.urdf"));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::UrdfLoadFailed);
    EXPECT_FALSE(result.error().message.empty());
    EXPECT_FALSE(result.error().chainError.has_value());
}

// --- reuse ----------------------------------------------------------

TEST(IkEquationBuilderTest, ReusesFacadeForSecondRobot)
{
    // The invalidation tests above check one rule each; this checks that their
    // composition leaves a coherent object rather than a mixture of two robots.
    // It is also the most likely real usage.
    IkEquationBuilder builder;

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr4_r600.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());
    ASSERT_NE(builder.forwardKinematics(), nullptr);

    // Copied out: the pointer itself is invalidated by the next successful
    // call, exactly as the accessor contract says.
    const SymbolicTransform kr4Fk = *builder.forwardKinematics();

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    const auto* kr640Fk = builder.forwardKinematics();
    ASSERT_NE(kr640Fk, nullptr);

    bool differs = false;
    for (std::size_t row = 0; row < 4 && !differs; ++row)
        for (std::size_t column = 0; column < 4 && !differs; ++column)
            differs = !structurallyEqual(kr4Fk(row, column), (*kr640Fk)(row, column));

    EXPECT_TRUE(differs) << "two different robots produced the same transform";
}
```

---

## 5. Uwagi do dwóch korekt

### 5.1 `std::runtime_error`, nie `std::exception`

Moja architektura mówiła jednocześnie „`bad_alloc` powinien lecieć dalej" i „rekomendacja: `std::exception`". To sprzeczność, bo `std::bad_alloc` **jest** `std::exception` — złapalibyśmy brak pamięci i zaraportowali go jako `UrdfLoadFailed` z komunikatem o URDF-ie. Loader rzuca dokładnie `std::runtime_error`, więc tyle łapiemy.

### 5.2 Testy akcesorów sprawdzają `nullptr`

Moja §10.2 twierdziła, że „każdy z czterech testów sprawdza kod błędu". Niemożliwe: dwa z nich dotyczą akcesorów, które kodu błędu nie zwracają. Podział jest teraz jawny — dwa testy operacji sprawdzają `RobotModelNotLoaded` i `KinematicChainNotSelected`, dwa testy akcesorów sprawdzają `nullptr`.

---

## 6. `README.md` — obowiązkowa aktualizacja

Obecny przykład (linie 35–42) ignoruje wyniki operacji i używa referencji. Podmiana:

````markdown
```cpp
IkEquationBuilder builder;

if (const auto result = builder.loadRobotModel("kr640.urdf"); !result)
{
    std::cerr << result.error().message << '\n';
    return;
}

if (const auto result = builder.selectChain("base_link", "tool0"); !result)
{
    // ChainBuildFailed also carries result.error().chainError
    std::cerr << result.error().message << '\n';
    return;
}

if (const auto result = builder.buildForwardKinematics(); !result)
{
    std::cerr << result.error().message << '\n';
    return;
}

const SymbolicTransform* fk = builder.forwardKinematics(); // a symbolic 4x4 transform
```
````

Wersja bez `return` sugerowałaby, że po nieudanym ładowaniu wolno iść dalej. Wyciągnięcie tego do funkcji zwracającej `expected<const SymbolicTransform*, …>` byłoby jeszcze czytelniejsze, ale **byłoby błędem czasu życia**: wskaźnik należałby do lokalnego `builder`. Dlatego przykład zostaje w jednym zakresie życia obiektu.

Zdanie pod przykładem („No caching, no hidden state carried between robots…") zostaje, ale wymaga doprecyzowania: stan **jest** przenoszony między wywołaniami — to jego istota — natomiast nie przeżywa zmiany robota ani łańcucha. Proponowana zmiana: *„State is explicit and cascading: loading a robot clears the chain and the transform, selecting a chain clears the transform. Nothing stale survives."*

---

## 7. `main.cpp` — bez zmian

Przyjmuję argumentację review i wycofuję swoją propozycję z §8 architektury. Problem nie leży w liczbie linii, tylko w tym, że sensowny program wymaga rozstrzygnięcia: skąd bierze ścieżkę URDF, czy `data/urdf` jest zakodowane względem drzewa źródeł, jakie przyjmuje argumenty, jak drukuje błędy i **co robi z symbolicznym FK, skoro nie ma jeszcze printera wyrażeń**. Ostatni punkt jest rozstrzygający — bez printera program mógłby najwyżej powiedzieć „udało się".

Zahardkodowane `data/urdf/kr640.urdf` działałoby wyłącznie z właściwego katalogu roboczego. `main.cpp` zostaje smoke-executable; realny przykład to `examples/build_fk.cpp` albo CLI z `argv`, gdy będzie co pokazywać.

---

## 8. Plan zmian w plikach

**Zmienione:** `src/ik_equations/IkEquationBuilder.hpp`, `.cpp`, `tests/CMakeLists.txt` (jedna linia), `README.md` (§6), `STATUS.md`.

**Dodane:** `tests/test_ik_equation_builder.cpp`.

**Bez zmian — jawnie:** `main.cpp`, główny `CMakeLists.txt` (`IkEquationBuilder.cpp` już jest na liście źródeł), wszystkie komponenty potokowe, cała warstwa symboliczna.

`tests/CMakeLists.txt`:

```cmake
    test_kinematics.cpp
    test_ik_equation_builder.cpp
    test_urdf_model_loader.cpp
```

---

## 9. `STATUS.md` — zmiany

1. **Nagłówek** — `204/204` → `221/221`.
2. **Tabela postępu** — Faza 1 `~95%` → **`100%`**; full vision `~45%` → `~50%`.
3. **Diagram** — `IkEquationBuilder` 🟡 → ✅.
4. **„Done"** — nowy wpis: stan przez `optional`, kaskadowe unieważnianie, gwarancja transakcyjna, jednolity `expected`, akcesory wskaźnikowe z kontraktem unieważniania.
5. **„Not done" → sekcja o Fazie 1 znika**; `IkEquationBuilder` przenosi się z „Not done" do „Done".
6. **Known gaps — nowy wiersz:** `UrdfLoadFailed` niesie wyłącznie `message`, bo `UrdfModelLoader` konsumuje strukturalny `LoadError` i rzuca string; typowany błąd loadera wymaga zmiany w loaderze. Powiązać z istniejącym wpisem o odrzucanym `DiagnosticBag`.
7. **Tabela dokumentów** — architektura `approved`, ten proposal `implemented`.
8. **Next step** — `ConstraintBuilder` i Faza 2; wraz z uwagą, że Faza 1 zamyka się z dwoma zapisanymi ograniczeniami zakresu (walidacja od `KinematicChain`, konwencja RPY z wspólnego przekonania), które **nie znikają** wraz z tagiem.

---

## 10. Ryzyka

| Ryzyko | Ocena | Uwaga |
|---|---|---|
| `RobotDescription` niedomyślnie konstruowalny | niskie | dziś jest zwykłym polem klasy, więc musi być; gdyby nie był, zamiana na `std::optional<RobotDescription> loaded;` to jedna linia |
| `structurallyEqual` na kopii `SymbolicTransform` | niskie | kopia to 16 uchwytów `shared_ptr`; węzły są niemutowalne, więc porównanie po kopiowaniu jest bezpieczne |
| `does_not_exist.urdf` daje inny wyjątek niż `runtime_error` | **średnie** | loader może np. rzucić `std::filesystem::filesystem_error`, który dziedziczy z `system_error` → `runtime_error`, więc złapiemy; ale jeśli rzuci coś spoza tej gałęzi, test `ReportsUrdfLoadFailure` **wywali się przez nieprzechwycony wyjątek**, a nie failem asercji. Zaraportuję to jako ustalenie, nie obejdę rozszerzeniem `catch` |

---

## 11. Jak to zweryfikuję po zatwierdzeniu

```powershell
cmake -B build -G Ninja
cmake --build build --clean-first
ctest --test-dir build --output-on-failure
.\build\tests\kinemaforge_tests.exe --gtest_filter=IkEquationBuilderTest.*
```

Oczekiwane: **204 + 17 = 221 zielonych.**

## Do zatwierdzenia

Kod (§2, §3), testy (§4), `README.md` (§6), `tests/CMakeLists.txt` i `STATUS.md` (§8, §9). Bez commita.
