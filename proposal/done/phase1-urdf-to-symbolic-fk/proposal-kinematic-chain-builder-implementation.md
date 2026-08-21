# Proposal: `KinematicChainBuilder` — implementacja

## Prompt

> [...] API → std::expected po tych zmianach, mozesz przygotowac konkretny proposal implementacujny

Realizacja architektury zatwierdzonej w `proposal-kinematic-chain-builder-architecture.md` (sekcja 14 — wszystkie trzy decyzje zatwierdzone bez zmian: `baseLink == toolLink` → pusty łańcuch/sukces; duplicate parent → `InvalidRobotDescription`; `Continuous` → actuated). Ten dokument jest w standardowym formacie repo: stan obecny + pełny kod zmian, czekający na zatwierdzenie przed naniesieniem na pliki źródłowe.

**Rewizja po review (v2):** pięć uwag z review wdrożone:
1. `KinematicChainError` przeniesiony do `model/KinematicChainError.hpp` — to typ domenowy, nie własność buildera; przyszłe komponenty (`KinematicChainValidator`, `...Optimizer`, `...Serializer`) będą go współdzielić bez zależności od `builders/`.
2. `findPath` przepisany na **iteracyjny** DFS z jawnym stosem — zero ryzyka przepełnienia stosu wywołań, bez kosztu wydajnościowego przy tej skali danych.
3. `knownLinks` (unia `RobotDescription.links` + referencji z jointów) — zachowane, z uzasadnieniem poniżej (sekcja "Odpowiedzi na pytania z review").
4. `childLinksSeen` → `childLinksWithParent` — nazwa mówi wprost, co zbiór reprezentuje.
5. Komentarze skrócone; dopisany komentarz klasowy o braku obliczeń symbolicznych/geometrycznych (dokładnie w brzmieniu z review).
6. `toKinematicJoint` → `copyJointData` (czysta zmiana nazwy, bez zmiany zachowania — `KinematicJoint` zostaje agregatem bez konstruktorów, zgodnie z resztą warstwy `model/`).

**Wdrożone i zweryfikowane (v5):** po naniesieniu na pliki źródłowe i uruchomieniu `ctest`, `BuildsKr4BaseToTool0Chain` faktycznie failowało — nie z powodu buga w `findPath`, tylko błędnego założenia w opisie topologii KR4 (i w architektonicznym proposalu, sekcja 6): `joint_1`'s `parentLink` to `base_link` bezpośrednio, nie `base` — `base` jest ślepą gałęzią (tak jak `flange`), a `base_link` ma dwoje dzieci, tak samo jak `link_6`. Poprawna ścieżka `base_link → tool0` ma 7 jointów, nie 8. Skorygowane w tym pliku, w `test_kinematic_chain_builder.cpp` i w sekcji 6 proposalu architektonicznego. Wszystkie 27 testów (10 istniejących + 17 nowych) zielone po poprawce.

**Rewizja po review (v4):** dwie uwagi o `findPath` wdrożone.
1. `Frame::link` (i `visited`) z `std::string` na `std::string_view` — nazwy linków żyją w `robot`/`baseLink`/`toolLink` przez cały czas trwania `build()`, więc kopiowanie ich do każdego `Frame` było niepotrzebne. Zmieniłem konsekwentnie też typ klucza `childrenOf`/`childLinksWithParent` na `string_view` — inaczej `Frame::link` jako `string_view` i tak wymuszałoby budowę tymczasowego `std::string` przy każdym `childrenOf.find(...)`, czyli ten sam koszt gdzie indziej, bez korzyści.
2. `incomingJointIndex` dla roota — było `0` (poprawny, ale mylący indeks). Teraz `std::optional<std::size_t>`, `nullopt` dla roota — czytelnie widać, że root nie ma przychodzącego jointu, zamiast polegać na tym, że pętla odtwarzająca ścieżkę zaczyna się od `i = 1`.

**Rewizja po review (v3):** odpowiedź na uwagę o złożoności `findPath`.

Słuszny niepokój: w v2 `pathJointIndices` był osobnym wektorem, który trzeba było ręcznie trzymać w synchronie ze stosem — `push` przy każdym zejściu w głąb, `pop` przy każdym backtracku. To faktycznie czwarty stan do pilnowania obok `stack`, `visited`, `nextChildIndex`, dokładnie jak zauważono.

Zamiast tylko dopisać komentarz o tej zależności, usunąłem ją: każdy `Frame` na stosie przechowuje teraz `incomingJointIndex` — indeks jointu, który doprowadził do tego linku. `pathJointIndices` przestaje być czymś synchronizowanym przez cały czas trwania przeszukiwania — odtwarzam go **raz**, dopiero w momencie sukcesu, jednym przejściem po stosie. Backtrack (gałąź `nextChildIndex == childCount`) robi teraz wyłącznie to, co robiłby w najprostszej wersji "czy ścieżka istnieje" bez odtwarzania jej treści: `visited.erase` + `stack.pop_back()`. Zero dodatkowego stanu do synchronizacji podczas samego przeszukiwania.

Efekt dla pytania "czy zmieniałbym rekurencję na iterację": **zostawiam iterację**, bo ten konkretny koszt (który był głównym argumentem za prostotą rekurencji) właśnie zniknął. Bez niego zostaje tylko jawny stos zamiast stosu wywołań — różnica głównie kosmetyczna, a bezpieczeństwo (brak rekurencji) zostaje bez dodatkowej ceny w złożoności.

## Stan obecny

### `src/ik_equations/builders/KinematicChainBuilder.hpp` (cały plik)

```cpp
#pragma once

#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/model/RobotDescription.hpp"

#include <string>

namespace kinemaforge::ik {

class KinematicChainBuilder
{
public:
    KinematicChain build(
        const RobotDescription& robot,
        const std::string& baseLink,
        const std::string& toolLink
    ) const;
};

} // namespace kinemaforge::ik
```

Brak `KinematicChainBuilder.cpp` — niezaimplementowane, nieobecne w `CMakeLists.txt`. Brak `model/KinematicChainError.hpp` — nie istnieje.

### `CMakeLists.txt` (fragment — lista źródeł biblioteki)

```cmake
add_library(kinemaforge_ik STATIC
    src/Kinematics.cpp
    src/kinematics/robot_model.cpp
    src/kinematics/robot_model_loader.cpp
    src/ik_equations/IkEquationBuilder.cpp
    src/ik_equations/UrdfModelLoader.cpp
)
```

### `tests/CMakeLists.txt` (fragment — lista źródeł testów)

```cmake
add_executable(kinemaforge_tests
    test_kinematics.cpp
    test_urdf_model_loader.cpp
)
```

### Struktury danych, których dotyczy implementacja (bez zmian, cytowane dla kontekstu)

`KinematicChain.hpp`: `KinematicJoint{index, name, parentLink, childLink, type, origin, axis, limits, variable}`, `KinematicChain{baseLink, toolLink, joints}` — oba agregaty, bez konstruktorów.
`JointVariable.hpp`: `JointVariable{name, index}`.
`UrdfJoint.hpp`: `JointType{Fixed, Revolute, Continuous, Prismatic}`, `UrdfJoint{name, parentLink, childLink, type, origin, axis, limits}`, `UrdfLink{name}`.
`RobotDescription.hpp`: `RobotDescription{name, links, joints}`.

Żadna z tych struktur się nie zmienia — implementacja tylko je wypełnia.

## Co się zmienia

### 1. `src/ik_equations/model/KinematicChainError.hpp` (nowy plik)

```cpp
#pragma once

namespace kinemaforge::ik {

enum class KinematicChainError
{
    BaseLinkNotFound,
    ToolLinkNotFound,
    NoPathFound,
    InvalidRobotDescription
};

} // namespace kinemaforge::ik
```

Żyje w `model/`, obok `KinematicChain.hpp` — to część słownika domenowego (co może pójść nie tak przy budowaniu łańcucha), nie szczegół implementacyjny jednej klasy w `builders/`.

### 2. `src/ik_equations/builders/KinematicChainBuilder.hpp` (nowa pełna treść)

```cpp
#pragma once

#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/model/KinematicChainError.hpp"
#include "ik_equations/model/RobotDescription.hpp"

#include <expected>
#include <string>

namespace kinemaforge::ik {

// Preserves topology only. Performs no symbolic or geometric computation.
class KinematicChainBuilder
{
public:
    std::expected<KinematicChain, KinematicChainError> build(
        const RobotDescription& robot,
        const std::string& baseLink,
        const std::string& toolLink
    ) const;
};

} // namespace kinemaforge::ik
```

### 3. `src/ik_equations/builders/KinematicChainBuilder.cpp` (nowy plik)

```cpp
#include "ik_equations/builders/KinematicChainBuilder.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace kinemaforge::ik {

namespace {

bool isActuated(JointType type)
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

enum class PathSearchResult
{
    Found,
    NotFound,
    CycleDetected
};

// Iterative DFS (explicit stack, no recursion) over parent -> child edges.
// The URDF is a tree, so at most one path to toolLink can exist.
//
// Each Frame remembers the joint that led into it, so the path is not kept
// in sync with the stack on every push/pop — it's read off the stack once,
// at the single point where toolLink is found. `link` is a view: every name
// it can point to (baseLink/toolLink, or a joint's parentLink/childLink)
// outlives this whole call, so nothing is copied while walking the graph.
PathSearchResult findPath(
    std::string_view baseLink,
    std::string_view toolLink,
    const std::unordered_map<std::string_view, std::vector<std::size_t>>& childrenOf,
    const RobotDescription& robot,
    std::vector<std::size_t>& pathJointIndices)
{
    struct Frame
    {
        std::string_view link;
        std::size_t nextChildIndex = 0;
        std::optional<std::size_t> incomingJointIndex; // nullopt only for the root frame
    };

    std::unordered_set<std::string_view> visited{baseLink};
    std::vector<Frame> stack;
    stack.push_back(Frame{baseLink});

    while (!stack.empty())
    {
        Frame& frame = stack.back();
        auto it = childrenOf.find(frame.link);
        const std::size_t childCount = (it != childrenOf.end()) ? it->second.size() : 0;

        if (frame.nextChildIndex == childCount)
        {
            // No more children to try from here — backtrack.
            visited.erase(frame.link);
            stack.pop_back();
            continue;
        }

        const std::size_t jointIndex = it->second[frame.nextChildIndex];
        ++frame.nextChildIndex;
        const std::string_view child = robot.joints[jointIndex].childLink;

        if (visited.contains(child))
            return PathSearchResult::CycleDetected;

        if (child == toolLink)
        {
            // Root has no incoming joint (nullopt); every other frame does.
            for (std::size_t i = 1; i < stack.size(); ++i)
                pathJointIndices.push_back(*stack[i].incomingJointIndex);
            pathJointIndices.push_back(jointIndex);
            return PathSearchResult::Found;
        }

        visited.insert(child);
        stack.push_back(Frame{child, 0, jointIndex});
    }

    return PathSearchResult::NotFound;
}

KinematicJoint copyJointData(const UrdfJoint& joint, std::size_t index)
{
    KinematicJoint kinematicJoint;
    kinematicJoint.index = index;
    kinematicJoint.name = joint.name;
    kinematicJoint.parentLink = joint.parentLink;
    kinematicJoint.childLink = joint.childLink;
    kinematicJoint.type = joint.type;
    kinematicJoint.origin = joint.origin;
    kinematicJoint.axis = joint.axis;
    kinematicJoint.limits = joint.limits;
    return kinematicJoint;
}

} // namespace

std::expected<KinematicChain, KinematicChainError> KinematicChainBuilder::build(
    const RobotDescription& robot,
    const std::string& baseLink,
    const std::string& toolLink) const
{
    // robot_model_loader.cpp doesn't cross-check <link> declarations against
    // joint parent/child references, so RobotDescription.links alone isn't
    // a reliable "does this link exist" source — union with joint references.
    std::unordered_set<std::string> knownLinks;
    for (auto const& link : robot.links)
        knownLinks.insert(link.name);
    for (auto const& joint : robot.joints)
    {
        knownLinks.insert(joint.parentLink);
        knownLinks.insert(joint.childLink);
    }

    if (!knownLinks.contains(baseLink))
        return std::unexpected(KinematicChainError::BaseLinkNotFound);
    if (!knownLinks.contains(toolLink))
        return std::unexpected(KinematicChainError::ToolLinkNotFound);

    if (baseLink == toolLink)
        return KinematicChain{baseLink, toolLink, {}};

    // Keys are views into robot.joints[i].parentLink/childLink, valid for
    // the lifetime of this call (robot is not mutated here).
    std::unordered_map<std::string_view, std::vector<std::size_t>> childrenOf;
    std::unordered_set<std::string_view> childLinksWithParent;
    for (std::size_t i = 0; i < robot.joints.size(); ++i)
    {
        const auto& joint = robot.joints[i];
        // A link has at most one parent joint in a valid URDF tree.
        if (!childLinksWithParent.insert(joint.childLink).second)
            return std::unexpected(KinematicChainError::InvalidRobotDescription);

        childrenOf[joint.parentLink].push_back(i);
    }

    std::vector<std::size_t> pathJointIndices;
    const auto searchResult = findPath(baseLink, toolLink, childrenOf, robot, pathJointIndices);

    if (searchResult == PathSearchResult::CycleDetected)
        return std::unexpected(KinematicChainError::InvalidRobotDescription);
    if (searchResult == PathSearchResult::NotFound)
        return std::unexpected(KinematicChainError::NoPathFound);

    KinematicChain chain;
    chain.baseLink = baseLink;
    chain.toolLink = toolLink;

    std::size_t variableCount = 0;
    for (std::size_t position = 0; position < pathJointIndices.size(); ++position)
    {
        auto kinematicJoint = copyJointData(robot.joints[pathJointIndices[position]], position);
        if (isActuated(kinematicJoint.type))
        {
            ++variableCount;
            kinematicJoint.variable = JointVariable{"q" + std::to_string(variableCount), variableCount};
        }
        chain.joints.push_back(std::move(kinematicJoint));
    }

    return chain;
}

} // namespace kinemaforge::ik
```

### 4. `CMakeLists.txt` (nowa pełna treść)

```cmake
cmake_minimum_required(VERSION 3.21)

project(KinemaForge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (MINGW)
    # Statically link the MinGW runtime so binaries don't depend on
    # libstdc++-6.dll / libgcc_s_seh-1.dll / libwinpthread-1.dll being
    # found on PATH (other MinGW installs, e.g. Anaconda, can shadow
    # these with incompatible versions and crash with 0xc0000139).
    add_link_options(-static-libgcc -static-libstdc++ -static)
endif()

include(FetchContent)

# --- pugixml (URDF parsing) ---
FetchContent_Declare(
    pugixml
    GIT_REPOSITORY https://github.com/zeux/pugixml.git
    GIT_TAG        v1.14
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(pugixml)

add_library(kinemaforge_ik STATIC
    src/Kinematics.cpp
    src/kinematics/robot_model.cpp
    src/kinematics/robot_model_loader.cpp
    src/ik_equations/IkEquationBuilder.cpp
    src/ik_equations/UrdfModelLoader.cpp
    src/ik_equations/builders/KinematicChainBuilder.cpp
)
target_include_directories(kinemaforge_ik PUBLIC src)
target_link_libraries(kinemaforge_ik PUBLIC pugixml::pugixml)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE kinemaforge_ik)

enable_testing()
add_subdirectory(tests)
```

(jedna dodana linia: `src/ik_equations/builders/KinematicChainBuilder.cpp`)

### 5. `tests/CMakeLists.txt` (nowa pełna treść)

```cmake
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)
# Windows: use the same dynamic CRT as the rest of the project
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(kinemaforge_tests
    test_kinematics.cpp
    test_urdf_model_loader.cpp
    test_kinematic_chain_builder.cpp
)
target_link_libraries(kinemaforge_tests PRIVATE kinemaforge_ik GTest::gtest_main)
target_compile_definitions(kinemaforge_tests PRIVATE
    KINEMAFORGE_URDF_DATA_DIR="${CMAKE_SOURCE_DIR}/data/urdf"
)

include(GoogleTest)
gtest_discover_tests(kinemaforge_tests)
```

(jedna dodana linia: `test_kinematic_chain_builder.cpp`)

### 6. `tests/test_kinematic_chain_builder.cpp` (nowy plik)

Bez zmian względem v1 — API (`kinemaforge::ik::KinematicChainError`) jest teraz dostępne transitywnie przez `KinematicChainBuilder.hpp → model/KinematicChainError.hpp`, dokładnie jak `JointType` jest dziś dostępne transitywnie przez `UrdfModelLoader.hpp → RobotDescription.hpp → UrdfJoint.hpp`.

```cpp
#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"

#include <filesystem>
#include <string>

namespace {

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

} // namespace

using kinemaforge::ik::JointType;
using kinemaforge::ik::KinematicChainBuilder;
using kinemaforge::ik::KinematicChainError;
using kinemaforge::ik::RobotDescription;
using kinemaforge::ik::UrdfJoint;
using kinemaforge::ik::UrdfLink;
using kinemaforge::ik::UrdfModelLoader;

TEST(KinematicChainBuilderTest, BuildsKr640BaseToTool0Chain)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->joints.size(), 7u);
}

TEST(KinematicChainBuilderTest, PreservesJointOrder)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->joints.size(), 7u);

    const char* expectedNames[] = {
        "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a6_to_tool0",
    };
    for (std::size_t i = 0; i < result->joints.size(); ++i)
    {
        SCOPED_TRACE(testing::Message() << "joint index " << i);
        EXPECT_EQ(result->joints[i].name, expectedNames[i]);
    }
}

TEST(KinematicChainBuilderTest, KeepsFixedJoints)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());

    const auto& fixedJoint = result->joints.back();
    EXPECT_EQ(fixedJoint.name, "joint_a6_to_tool0");
    EXPECT_EQ(fixedJoint.type, JointType::Fixed);
}

TEST(KinematicChainBuilderTest, AssignsSymbolsOnlyToActuatedJoints)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->joints.size(), 7u);

    for (std::size_t i = 0; i < 6; ++i)
    {
        SCOPED_TRACE(testing::Message() << "actuated joint index " << i);
        EXPECT_TRUE(result->joints[i].variable.has_value());
    }
    EXPECT_FALSE(result->joints.back().variable.has_value());
}

TEST(KinematicChainBuilderTest, NumbersSymbolsFromQ1)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());

    for (std::size_t i = 0; i < 6; ++i)
    {
        SCOPED_TRACE(testing::Message() << "actuated joint index " << i);
        ASSERT_TRUE(result->joints[i].variable.has_value());
        EXPECT_EQ(result->joints[i].variable->name, "q" + std::to_string(i + 1));
        EXPECT_EQ(result->joints[i].variable->index, i + 1);
    }
}

TEST(KinematicChainBuilderTest, BuildsKr4BaseToTool0Chain)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr4_r600.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");

    ASSERT_TRUE(result.has_value());
    // joint_1..joint_6 (6 actuated) + link6-tool0 (fixed). base_link's other
    // fixed joint (base_link-base -> "base") is a dead-end branch, correctly
    // excluded — mirrors the link_6 -> flange/tool0 branch at the other end.
    EXPECT_EQ(result->joints.size(), 7u);
}

TEST(KinematicChainBuilderTest, PicksCorrectBranchAtLinkSix)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr4_r600.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());

    for (auto const& joint : result->joints)
        EXPECT_NE(joint.name, "link6-flange");

    EXPECT_EQ(result->joints.back().name, "link6-tool0");
}

TEST(KinematicChainBuilderTest, ChainRecordsRequestedBaseAndToolLinkNames)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());

    EXPECT_EQ(result->baseLink, "base_link");
    EXPECT_EQ(result->toolLink, "tool0");
}

TEST(KinematicChainBuilderTest, JointIndexMatchesPositionInChain)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());

    for (std::size_t i = 0; i < result->joints.size(); ++i)
        EXPECT_EQ(result->joints[i].index, i);
}

TEST(KinematicChainBuilderTest, PreservesOriginAndAxisForFixedJoints)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());

    const auto& fixedJoint = result->joints.back();
    ASSERT_EQ(fixedJoint.name, "joint_a6_to_tool0");
    EXPECT_DOUBLE_EQ(fixedJoint.origin.translation.z, 0.290);
}

TEST(KinematicChainBuilderTest, ReturnsBaseLinkNotFoundWhenBaseLinkDoesNotExist)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "does_not_exist", "tool0");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), KinematicChainError::BaseLinkNotFound);
}

TEST(KinematicChainBuilderTest, ReturnsToolLinkNotFoundWhenToolLinkDoesNotExist)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "does_not_exist");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), KinematicChainError::ToolLinkNotFound);
}

TEST(KinematicChainBuilderTest, ReturnsNoPathFoundWhenNoPathExists)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr4_r600.urdf"));

    KinematicChainBuilder builder;
    // "flange" exists but is a dead-end branch off link_6; tool0 hangs off
    // the other branch, so there is no downward path from flange to tool0.
    const auto result = builder.build(robot, "flange", "tool0");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), KinematicChainError::NoPathFound);
}

TEST(KinematicChainBuilderTest, ReturnsEmptyChainWhenBaseEqualsTool)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "base_link");

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->baseLink, "base_link");
    EXPECT_EQ(result->toolLink, "base_link");
    EXPECT_TRUE(result->joints.empty());
}

TEST(KinematicChainBuilderTest, ReturnsInvalidRobotDescriptionOnCyclicInput)
{
    RobotDescription robot;
    robot.name = "cyclic";
    robot.links = {UrdfLink{"A"}, UrdfLink{"B"}, UrdfLink{"C"}};

    UrdfJoint j1;
    j1.name = "j1";
    j1.parentLink = "A";
    j1.childLink = "B";
    j1.type = JointType::Revolute;

    UrdfJoint j2;
    j2.name = "j2";
    j2.parentLink = "B";
    j2.childLink = "A";
    j2.type = JointType::Revolute;

    robot.joints = {j1, j2};

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "A", "C");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), KinematicChainError::InvalidRobotDescription);
}

TEST(KinematicChainBuilderTest, ReturnsInvalidRobotDescriptionOnDuplicateChildLink)
{
    RobotDescription robot;
    robot.name = "duplicate_parent";
    robot.links = {UrdfLink{"base"}, UrdfLink{"mid"}, UrdfLink{"tool"}};

    UrdfJoint j1;
    j1.name = "j1";
    j1.parentLink = "base";
    j1.childLink = "mid";
    j1.type = JointType::Fixed;

    UrdfJoint j2;
    j2.name = "j2";
    j2.parentLink = "tool";
    j2.childLink = "mid"; // "mid" already claimed as a child by j1
    j2.type = JointType::Fixed;

    robot.joints = {j1, j2};

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base", "mid");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), KinematicChainError::InvalidRobotDescription);
}

TEST(KinematicChainBuilderTest, AssignsVariableToContinuousJoint)
{
    // The loader doesn't parse type="continuous" yet (see
    // proposal-loader-test-coverage.md, "Znane luki"), so this is
    // constructed directly — regression guard for the sekcja 7/14
    // decision that Continuous counts as actuated.
    RobotDescription robot;
    robot.name = "continuous_test";
    robot.links = {UrdfLink{"base"}, UrdfLink{"tool"}};

    UrdfJoint joint;
    joint.name = "spin";
    joint.parentLink = "base";
    joint.childLink = "tool";
    joint.type = JointType::Continuous;

    robot.joints = {joint};

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base", "tool");

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->joints.size(), 1u);
    ASSERT_TRUE(result->joints[0].variable.has_value());
    EXPECT_EQ(result->joints[0].variable->name, "q1");
    EXPECT_EQ(result->joints[0].variable->index, 1u);
}
```

17 testów: 9 wymaganych z oryginalnego promptu (nazwy `Throws...` zamienione na `Returns...`-styl) + 8 dodatkowych (7 z v1 + `AssignsVariableToContinuousJoint` z tej rewizji).

## Odpowiedzi na pytania z review

**"Po co `knownLinks`, skoro `RobotDescription.links` powinno wystarczyć?"** — bo dziś nie wystarcza: `robot_model_loader.cpp` (`load_urdf`) parsuje `<link>` i `<joint>` niezależnie i nigdzie nie sprawdza, czy `parentLink`/`childLink` każdego jointu odpowiada faktycznie zadeklarowanemu `<link>`. To nie jest hipoteza — to odczytane wprost z kodu parsera (brak takiego sprawdzenia w `load_urdf`). Dopóki ta luka istnieje w parserze, poleganie wyłącznie na `RobotDescription.links` znaczyłoby: link osiągalny w grafie (bo referencje w jointach istnieją), ale zgłaszany jako `BaseLinkNotFound`/`ToolLinkNotFound`, tylko dlatego że nikt nie dodał dla niego `<link>`. Zgadzam się z kierunkiem "może to parser powinien to gwarantować" — to dokładnie pokrywa się z odłożonym wcześniej `invalid_missing_parent.urdf` w `proposal-loader-test-coverage.md` ("Znane luki / Krok 2"). Jeśli/gdy ta luka w parserze zostanie zamknięta, `knownLinks` w `KinematicChainBuilder` będzie można bezpiecznie uprościć do samego `RobotDescription.links` — ale to zależność w złą stronę do robienia teraz (nie chcę przywiązywać tego proposalu do jeszcze niezrobionego Kroku 2 loadera).

**Dlaczego `knownLinks` zostaje `std::string`, skoro reszta przeszła na `string_view`?** — bo to dwa różne miejsca w kodzie o różnej częstotliwości użycia. `childrenOf`/`visited`/`Frame::link` żyją w pętli DFS, odwiedzanej per joint na ścieżce — tam kopiowanie się liczy. `knownLinks` jest budowane raz i odpytywane dokładnie dwa razy (`baseLink`, `toolLink`) — zamiana na `string_view` nic by tu nie dała poza niespójnością bez powodu.

**Dlaczego nie `KinematicJoint(const UrdfJoint&)` jako konstruktor?** — bo `model/` w tym repo dziś konsekwentnie trzyma same dane (agregaty, zero zachowania — `KinematicJoint`, `KinematicChain`, `UrdfJoint`, `RobotDescription` wszystkie bez konstruktorów). Dodanie konstruktora tylko do `KinematicJoint` byłoby jedynym wyjątkiem od tej konwencji w całej warstwie `model/`. Wolna funkcja `copyJointData` w `builders/` (tam, gdzie mieszka logika) zostawia `model/` czystym.

## Weryfikacja zgodności z zatwierdzoną architekturą

| Decyzja z proposalu architektonicznego | Gdzie w kodzie |
|---|---|
| Własna struktura `KinematicJoint`, nie `UrdfJoint` wprost | `copyJointData()` — jawne kopiowanie pól |
| Kopie, nie referencje | `KinematicChain`/`chain` budowane i zwracane przez wartość |
| Fixed jointy zachowane | `copyJointData()` wywoływane dla każdego jointu na ścieżce, niezależnie od `type` |
| Numeracja `q1..qn` tylko dla actuated, wzdłuż ścieżki | `isActuated()` + `variableCount` rosnący tylko w gałęzi actuated, iteracja po `pathJointIndices` (kolejność ścieżki, nie `RobotDescription.joints`) |
| `Continuous` jako actuated | `isActuated()` obejmuje `Continuous` |
| DFS z backtrackingiem, bez rekurencji | `findPath()` — jawny stos `std::vector<Frame>`, `nextChildIndex` per poziom; ścieżka odtwarzana raz przy sukcesie z `incomingJointIndex`, nie synchronizowana ręcznie podczas przeszukiwania |
| Cykl jako defensywny guard, nie pełna walidacja | `visited` w `findPath()`, ograniczony do bieżącej gałęzi (usuwany przy backtracku) |
| Duplicate parent wykrywany tanio przy budowie grafu | pętla budująca `childrenOf` w `build()`, `childLinksWithParent.insert(...).second` |
| `baseLink == toolLink` → sukces, pusty łańcuch | wczesny `return KinematicChain{baseLink, toolLink, {}}` w `build()` |
| `std::expected<KinematicChain, KinematicChainError>` | sygnatura `build()`, `KinematicChainError` w `model/` |
| Błędy domenowe vs. bug implementacyjny | `KinematicChainError` ma tylko 4 wartości domenowe; żadnego `assert`/`unreachable` nie było potrzeba dodać — algorytm nie ma dziś ścieżki kończącej się sukcesem bez trafienia w `toolLink` |
| Klasa nie liczy nic symbolicznie/geometrycznie | komentarz nad `class KinematicChainBuilder` w nagłówku |

## Jak to zweryfikuję po zatwierdzeniu

```powershell
cmake --build build
ctest --test-dir build --output-on-failure
```

Oczekiwany wynik: wszystkie dotychczasowe testy (10) + 17 nowych = 27 zielonych.

## Do zatwierdzenia

Czekam na Twoje ok przed naniesieniem tych zmian na pliki źródłowe.
