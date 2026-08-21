# Proposal: `ConstraintBuilder` — `PositionOnly` (F2.4), implementacja

**Etap:** F2.4 · **v2 APPROVED — ZAIMPLEMENTOWANE** · 2026-08-20 · część 2 z 2

> **v2, po review.** Dwie poprawki, obie implementacyjne. (1) Dwunasty test —
> `PreservesSystemRejection` — pokrywa trzecią gałąź modelu błędów, która nie
> miała żadnego testu (4.7); gate rośnie do 332/332. (2) Poprawiony opis
> czasu życia `string_view` w 2.1 i w tabeli ryzyk: widoki wskazują wyłącznie na
> `SymbolNode::name` w DAG-u transformacji, nie na `unknowns`. Kod bez zmian.

Architektura: [`proposal-constraint-builder-architecture.md`](proposal-constraint-builder-architecture.md),
**v2 APPROVED**. Ten dokument nie zmienia żadnej decyzji — podaje kod, który z
niej wynika.

Zakres jest zamknięty i **celowo nierozszerzany**, zgodnie z warunkiem review:

```text
SymbolicTransform + KinematicChain + PositionTarget
                       ↓
                 ConstraintBuilder
                       ↓
            3 × Equation: X, Y, Z
                       ↓
              IkEquationSystem

jeden dodatkowy inwariant:  symbols(lhs X,Y,Z) ⊆ nazwy unknowns łańcucha
```

Czego tu **nie ma i nie będzie**, mimo że nasuwa się przy pisaniu kodu:
`SymbolId`/provenance, publicznego `SymbolCollector`, integracji z fasadą,
walidacji kanoniczności transformacji, nowych operacji w `ExpressionFactory`.

## 1. `src/ik_equations/builders/ConstraintBuilder.hpp`

```cpp
#pragma once

#include "ik_equations/model/IkEquationSystem.hpp"
#include "ik_equations/model/IkTarget.hpp"
#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/model/TargetValidation.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <expected>
#include <optional>
#include <string>

namespace kinemaforge::ik {

enum class ConstraintBuilderErrorCode
{
    InvalidTarget,     // the target itself is not usable
    UndeclaredSymbol,  // an equation depends on a symbol that is not an unknown
    SystemRejected     // IkEquationSystem::create refused the assembled system
};

// The nested error is preserved rather than flattened into a string, and the
// invariant is: targetError has a value iff code == InvalidTarget, systemError
// iff code == SystemRejected. Like IkEquationBuilderError, this is a public
// aggregate -- the guarantee is about what this class produces, not about what
// the struct can hold.
struct ConstraintBuilderError
{
    ConstraintBuilderErrorCode code{};
    std::string message;

    std::optional<TargetValidationError> targetError;
    std::optional<IkEquationSystemError> systemError;
};

// Turns "where the tool must be" into equations: the first component that
// formulates an IK problem rather than describing geometry.
//
//     T(0,3)(q) = target.x
//     T(1,3)(q) = target.y
//     T(2,3)(q) = target.z
//
// Equations are stored as lhs = rhs, never as lhs - rhs = 0: subtracting would
// destroy which side came from the robot and which from the task, and Equation
// says so explicitly. Normalisation belongs to the future simplifier.
//
// Two preconditions cannot be checked here, and pretending otherwise would be
// worse than saying it plainly:
//
//   * WHICH transform. This class does not know whether it was handed
//     forwardKinematics() or tcpForwardKinematics(), and must not: the frame a
//     PositionTarget lives in is the caller's contract. Pass the TCP transform
//     when a TCP is set.
//
//   * WHETHER the pair matches. transform and chain are separate arguments and
//     nothing couples them. Symbol names are positional ("q1", "q2", ...), so
//     the FK of one six-axis robot passes the symbol check against the chain of
//     another. What IS guaranteed is narrower and exact: every symbol appearing
//     in the three translation cells is a declared unknown of the chain given.
//     Detecting provenance would need symbol identity stronger than a name --
//     out of scope, recorded in STATUS.md.
//
// An unknown that does NOT appear in the equations is accepted, not an error: a
// wrist roll with no tool offset does not move the TCP position, so q6 is
// legitimately absent from a position-only system. The inclusion holds one way.
class ConstraintBuilder
{
public:
    explicit ConstraintBuilder(ExpressionFactory factory = {});

    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    build(const SymbolicTransform& transform,
          const KinematicChain& chain,
          const PositionTarget& target) const;

private:
    ExpressionFactory factory_;
};

} // namespace kinemaforge::ik
```

## 2. `src/ik_equations/builders/ConstraintBuilder.cpp`

```cpp
#include "ik_equations/builders/ConstraintBuilder.hpp"

#include "ik_equations/model/Equation.hpp"
#include "ik_equations/model/JointVariable.hpp"
#include "ik_equations/symbolic/Expression.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace kinemaforge::ik {
namespace {

// Column 3 of a homogeneous transform, rows 0..2 -- the translation.
constexpr std::size_t kTranslationColumn = 3;

// Symbol names a subtree depends on, in first-encounter order.
//
// Order matters: with several undeclared symbols, the reported one must not
// depend on hash iteration. That is the same trap R2 removed from the loader,
// where the root link was chosen by unordered_map iteration order. Here the
// walk is X, then Y, then Z, each depth-first left to right, so the message is
// reproducible.
//
// The visited set is by node identity, because an FK DAG shares sub-trees
// heavily -- 281 unique nodes against 21 882 counted with multiplicity for
// kr640. Without it the walk is exponential in practice.
//
// string_view is safe here: the nodes are owned by the caller's transform,
// which outlives build(), and nothing in this file stores a view past return.
void collectSymbols(const Expression& expression,
                    std::unordered_set<const ExpressionNode*>& visited,
                    std::vector<std::string_view>& symbols)
{
    if (!visited.insert(&expression.node()).second)
        return;

    std::visit(
        [&](const auto& node) {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, SymbolNode>)
            {
                if (std::ranges::find(symbols, node.name) == symbols.end())
                    symbols.emplace_back(node.name);
            }
            else if constexpr (std::is_same_v<Node, AddNode>
                               || std::is_same_v<Node, SubtractNode>
                               || std::is_same_v<Node, MultiplyNode>
                               || std::is_same_v<Node, DivideNode>)
            {
                collectSymbols(node.lhs, visited, symbols);
                collectSymbols(node.rhs, visited, symbols);
            }
            else if constexpr (std::is_same_v<Node, NegateNode>
                               || std::is_same_v<Node, SinNode>
                               || std::is_same_v<Node, CosNode>)
            {
                collectSymbols(node.operand, visited, symbols);
            }
            else
            {
                // Constant is the only other leaf. A node type added later must
                // decide explicitly whether it has children: falling into
                // "nothing to do" would silently stop the walk above its
                // subtree, and this walk is what the symbol guarantee rests on.
                static_assert(std::is_same_v<Node, ConstantNode>,
                              "Unhandled ExpressionNode type");
            }
        },
        expression.node().value);
}

std::vector<JointVariable> unknownsOf(const KinematicChain& chain)
{
    std::vector<JointVariable> unknowns;
    for (const auto& joint : chain.joints)
        if (joint.variable)
            unknowns.push_back(*joint.variable);
    return unknowns;
}

ConstraintBuilderError invalidTarget(TargetValidationError error)
{
    auto message = "position target rejected: " + error.message;
    return ConstraintBuilderError{ConstraintBuilderErrorCode::InvalidTarget,
                                  std::move(message), std::move(error), std::nullopt};
}

ConstraintBuilderError undeclaredSymbol(std::string_view name)
{
    return ConstraintBuilderError{
        ConstraintBuilderErrorCode::UndeclaredSymbol,
        "equation depends on '" + std::string{name}
            + "', which is not an unknown of the given chain",
        std::nullopt, std::nullopt};
}

ConstraintBuilderError systemRejected(IkEquationSystemError error)
{
    auto message = "equation system rejected: " + error.message;
    return ConstraintBuilderError{ConstraintBuilderErrorCode::SystemRejected,
                                  std::move(message), std::nullopt, std::move(error)};
}

} // namespace

ConstraintBuilder::ConstraintBuilder(ExpressionFactory factory)
    : factory_(std::move(factory))
{
}

std::expected<IkEquationSystem, ConstraintBuilderError>
ConstraintBuilder::build(const SymbolicTransform& transform,
                         const KinematicChain& chain,
                         const PositionTarget& target) const
{
    // First, and the order is load-bearing: the target's coordinates become
    // constants below, and ExpressionFactory::constant has a finite-value
    // precondition. Validating here makes that precondition hold by
    // construction rather than by hope.
    if (auto valid = validate(target); !valid)
        return std::unexpected(invalidTarget(valid.error()));

    auto unknowns = unknownsOf(chain);

    std::unordered_set<const ExpressionNode*> visited;
    std::vector<std::string_view> symbols;
    for (std::size_t row = 0; row < 3; ++row)
        collectSymbols(transform(row, kTranslationColumn), visited, symbols);

    for (const auto& symbol : symbols)
    {
        const auto declared = std::ranges::any_of(
            unknowns, [&symbol](const JointVariable& unknown) { return unknown.name == symbol; });
        if (!declared)
            return std::unexpected(undeclaredSymbol(symbol));
    }

    // X, Y, Z in this order: IkEquationSystem promises position first in this
    // order and rejects UnorderedEquations, so a producer that emits them any
    // other way builds a system its own validation refuses.
    std::vector<Equation> equations;
    equations.reserve(3);
    equations.push_back(Equation{transform(0, kTranslationColumn),
                                 factory_.constant(target.position.x),
                                 PositionEquationSource{CartesianComponent::X}});
    equations.push_back(Equation{transform(1, kTranslationColumn),
                                 factory_.constant(target.position.y),
                                 PositionEquationSource{CartesianComponent::Y}});
    equations.push_back(Equation{transform(2, kTranslationColumn),
                                 factory_.constant(target.position.z),
                                 PositionEquationSource{CartesianComponent::Z}});

    auto system = IkEquationSystem::create(IkTaskKind::Position, std::move(unknowns),
                                           std::move(equations));
    if (!system)
        return std::unexpected(systemRejected(std::move(system.error())));

    return std::move(*system);
}

} // namespace kinemaforge::ik
```

### 2.1 Trzy szczegóły, które wyszły dopiero przy kodzie

**Kolejność zbierania symboli jest kontraktem diagnostyki.** Gdyby `symbols`
było `unordered_set`, komunikat przy dwóch niezadeklarowanych symbolach
zależałby od kolejności hashowania — dokładnie ta klasa błędu, którą R2 usunęło
z wyboru korzenia w loaderze. Stąd wektor z zachowaną kolejnością napotkania
i liniowe `find`: symboli jest tyle, co osi (6), więc koszt jest bez znaczenia,
a komunikat jest odtwarzalny.

**Sprawdzenie symboli musi być przed `std::move(unknowns)`** — ale nie z powodu
czasu życia widoków.

> **Poprawione w v2, po review.** v1 tłumaczyła to tak, jakby `symbols`
> zawierało widoki na `unknown.name`. Nie zawiera. Każdy `string_view` w
> `symbols` powstaje wyłącznie z `symbols.emplace_back(node.name)`, czyli
> wskazuje na `SymbolNode::name` w DAG-u **transformacji**, który żyje przez
> całe wywołanie i należy do wywołującego. `unknown.name` jest tylko chwilowo
> porównywany (`unknown.name == symbol`) i nigdy nie jest zapamiętywany.

Prawdziwy powód jest prostszy: po przekazaniu `unknowns` do `create()` lokalny
wektor jest moved-from, więc nie wolno na nim opierać dalszej walidacji. Stąd
`create()` na końcu funkcji, po całej walidacji — kolejność, nie lifetime.

**Kolektor nie ma `else { return; }`.** Ostatnia gałąź to `static_assert` na
`ConstantNode`, czyli wymóg z architektury 4.6: dziesiąta alternatywa
`ExpressionNode` zatrzyma kompilację zamiast po cichu urwać obchód.

## 3. `CMakeLists.txt`

Jedna linia w liście źródeł `kinemaforge_ik`, po `ForwardKinematicsBuilder.cpp`:

```cmake
    src/ik_equations/builders/ForwardKinematicsBuilder.cpp
    src/ik_equations/builders/ConstraintBuilder.cpp
    src/ik_equations/builders/RigidTransformConstruction.cpp
```

## 4. `tests/test_constraint_builder.cpp` — dwanaście testów

### 4.1 Nagłówek i pomocnicze

```cpp
#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ConstraintBuilder.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <cstddef>
#include <filesystem>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace ik = kinemaforge::ik;

using ik::CartesianComponent;
using ik::ConstraintBuilder;
using ik::ConstraintBuilderErrorCode;
using ik::Expression;
using ik::ExpressionFactory;
using ik::IkTaskKind;
using ik::PositionEquationSource;
using ik::PositionTarget;
using ik::SymbolicTransform;
using ik::TargetValidationErrorCode;
using ik::sameNode;

namespace {

constexpr std::size_t kTranslationColumn = 3;

// A chain that declares the given unknowns and nothing else. The joints carry
// no geometry: every test here feeds the builder a transform built by hand or
// by the real pipeline, so the chain is only ever asked what its unknowns are.
//
// Numbering matches KinematicChainBuilder: "q1" has index 1.
ik::KinematicChain chainDeclaring(std::initializer_list<const char*> names)
{
    ik::KinematicChain chain;
    chain.baseLink = "base_link";
    chain.toolLink = "tool0";

    std::size_t index = 0;
    for (const char* name : names)
    {
        ik::KinematicJoint joint;
        joint.index = index;
        joint.name = std::string{"joint_"} + name;
        joint.type = ik::JointType::Revolute;
        joint.variable = ik::JointVariable{name, index + 1};

        chain.joints.push_back(std::move(joint));
        ++index;
    }
    return chain;
}

// Identity with the translation column replaced by the three given cells.
SymbolicTransform transformWithTranslation(Expression x, Expression y, Expression z)
{
    auto transform = SymbolicTransform::identity();
    transform(0, kTranslationColumn) = std::move(x);
    transform(1, kTranslationColumn) = std::move(y);
    transform(2, kTranslationColumn) = std::move(z);
    return transform;
}

struct LoadedRobot
{
    ik::KinematicChain chain;
    SymbolicTransform fk;
};

std::optional<LoadedRobot> loadRobot(const char* fileName)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(std::filesystem::path{KINEMAFORGE_URDF_DATA_DIR} / fileName);

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

} // namespace
```

`loadRobot` jest bliźniakiem helpera z `test_numeric_fk_validation.cpp` i
`test_expression_factory.cpp` — świadomie duplikowany, ta sama decyzja co w R5
(§9 tamtego proposala): piętnaście linii nie uzasadnia wspólnej infrastruktury
w `tests/support/`, a duplikat trzyma każdy plik testowy czytelnym osobno.

### 4.2 Kształt wyniku

```cpp
TEST(ConstraintBuilderTest, BuildsThreePositionEquations)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q2"), factory.constant(0.5));
    const auto chain = chainDeclaring({"q1", "q2"});

    const auto system = builder.build(transform, chain, PositionTarget{{1.0, 2.0, 3.0}});

    ASSERT_TRUE(system.has_value()) << static_cast<int>(system.error().code);
    EXPECT_EQ(system->taskKind(), IkTaskKind::Position);
    EXPECT_EQ(system->equations().size(), 3u);
}

TEST(ConstraintBuilderTest, EquationsAreOrderedXYZ)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      PositionTarget{{1.0, 2.0, 3.0}});

    ASSERT_TRUE(system.has_value());
    const auto& equations = system->equations();
    EXPECT_EQ(std::get<PositionEquationSource>(equations[0].source).component,
              CartesianComponent::X);
    EXPECT_EQ(std::get<PositionEquationSource>(equations[1].source).component,
              CartesianComponent::Y);
    EXPECT_EQ(std::get<PositionEquationSource>(equations[2].source).component,
              CartesianComponent::Z);
}
```

### 4.3 Skąd bierze się lewa i prawa strona

```cpp
TEST(ConstraintBuilderTest, UsesTcpTranslationColumn)
{
    // The builder reads cells (0..2, 3) of the transform it is given -- which
    // is the TCP transform whenever the caller passes one. It cannot tell.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto x = factory.symbol("q1");
    const auto y = factory.symbol("q2");
    const auto z = factory.constant(0.25);
    const auto transform = transformWithTranslation(x, y, z);

    const auto system = builder.build(transform, chainDeclaring({"q1", "q2"}),
                                      PositionTarget{{0.0, 0.0, 0.0}});

    ASSERT_TRUE(system.has_value());
    EXPECT_TRUE(sameNode(system->equations()[0].lhs, x));
    EXPECT_TRUE(sameNode(system->equations()[1].lhs, y));
    EXPECT_TRUE(sameNode(system->equations()[2].lhs, z));
}

TEST(ConstraintBuilderTest, DoesNotSubtractTargetFromLhs)
{
    // sameNode, not structural equality: the left-hand side must be the very
    // cell of the transform, not a rebuilt or rearranged expression. Equation
    // stores lhs = rhs as given; forming lhs - rhs would erase which side came
    // from the robot and which from the task.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto x = factory.symbol("q1");
    const auto transform = transformWithTranslation(x, x, x);

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      PositionTarget{{7.0, 8.0, 9.0}});

    ASSERT_TRUE(system.has_value());
    for (const auto& equation : system->equations())
    {
        EXPECT_TRUE(sameNode(equation.lhs, x));
        EXPECT_EQ(equation.rhs.type(), ik::ExpressionType::Constant);
    }
}

TEST(ConstraintBuilderTest, UsesTargetValuesOnRightHandSide)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      PositionTarget{{1.5, -2.5, 0.0}});

    ASSERT_TRUE(system.has_value());
    const auto& equations = system->equations();
    EXPECT_DOUBLE_EQ(ik::constantValue(equations[0].rhs), 1.5);
    EXPECT_DOUBLE_EQ(ik::constantValue(equations[1].rhs), -2.5);
    EXPECT_TRUE(ik::isZero(equations[2].rhs));  // 0.0 reaches the shared zero node
}

TEST(ConstraintBuilderTest, PreservesJointSymbols)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    const ConstraintBuilder builder;
    const auto system = builder.build(robot->fk, robot->chain, PositionTarget{{1.0, 0.0, 1.0}});

    ASSERT_TRUE(system.has_value());
    ASSERT_EQ(system->unknowns().size(), 6u);
    for (std::size_t position = 0; position < 6; ++position)
    {
        EXPECT_EQ(system->unknowns()[position].name, "q" + std::to_string(position + 1));
        EXPECT_EQ(system->unknowns()[position].index, position + 1);
    }
}
```

### 4.4 Inwariant symboli — obie strony inkluzji

```cpp
TEST(ConstraintBuilderTest, RejectsUndeclaredSymbol)
{
    // The shape of review issue 10: an equation depending on a symbol the
    // system never declares. Nothing downstream would catch it --
    // IkEquationSystem::create cannot collect symbols.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q999"), factory.symbol("q2"));

    const auto system = builder.build(transform, chainDeclaring({"q1", "q2"}),
                                      PositionTarget{{0.0, 0.0, 0.0}});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::UndeclaredSymbol);
    EXPECT_NE(system.error().message.find("q999"), std::string::npos);
}

TEST(ConstraintBuilderTest, AcceptsUnknownAbsentFromEquations)
{
    // The inclusion holds one way only. A declared unknown that does not appear
    // is normal: a wrist roll with no tool offset does not move the TCP
    // position. The system is then positionally underdetermined, which is a
    // correct answer for PositionOnly, not a fault.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.constant(1.0));

    const auto system = builder.build(transform, chainDeclaring({"q1", "q2"}),
                                      PositionTarget{{0.0, 0.0, 1.0}});

    ASSERT_TRUE(system.has_value());
    EXPECT_EQ(system->unknowns().size(), 2u);
}
```

### 4.5 Cel odrzucony

```cpp
TEST(ConstraintBuilderTest, RejectsNonFinitePositionTarget)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    const PositionTarget target{{0.0, std::numeric_limits<double>::infinity(), 0.0}};

    const auto system = builder.build(transform, chainDeclaring({"q1"}), target);

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::InvalidTarget);
    ASSERT_TRUE(system.error().targetError.has_value());
    EXPECT_EQ(system.error().targetError->code, TargetValidationErrorCode::NonFinitePosition);
    EXPECT_FALSE(system.error().systemError.has_value());
}
```

Ten test pilnuje też kolejności z 2: gdyby walidacja szła po budowie równań,
`factory.constant(inf)` naruszyłby precondition — w `build-werror` (asercje
aktywne) test przerwałby proces, zamiast zwrócić błąd.

### 4.6 End-to-end

```cpp
TEST(ConstraintBuilderTest, BuildsKr640PositionEquationSystem)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    const ConstraintBuilder builder;
    const auto system = builder.build(robot->fk, robot->chain, PositionTarget{{2.0, 0.5, 1.5}});

    ASSERT_TRUE(system.has_value());
    EXPECT_EQ(system->taskKind(), IkTaskKind::Position);
    EXPECT_EQ(system->equations().size(), 3u);
    EXPECT_EQ(system->unknowns().size(), 6u);
    EXPECT_TRUE(sameNode(system->equations()[0].lhs, robot->fk(0, kTranslationColumn)));
}

TEST(ConstraintBuilderTest, BuildsKr4PositionEquationSystem)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    const ConstraintBuilder builder;
    const auto system = builder.build(robot->fk, robot->chain, PositionTarget{{0.4, 0.1, 0.6}});

    ASSERT_TRUE(system.has_value());
    EXPECT_EQ(system->equations().size(), 3u);
    EXPECT_EQ(system->unknowns().size(), 6u);
}
```

Oba robią to, czego nie robi żaden test na ręcznie złożonej macierzy:
przechodzą **całą** drogę URDF → `RobotDescription` → `KinematicChain` → FK →
`IkEquationSystem`, więc łapią rozjazd kontraktów między etapami — na przykład
symbol nazwany inaczej przez `KinematicChainBuilder`, niż zakłada kolektor.

### 4.7 Trzeci kod błędu

> **Dodane w v2, po review.** Model błędów ma trzy przypadki, a v1 testowała
> dwa. `systemRejected()` miała własny kontrakt — `code == SystemRejected`,
> `targetError == nullopt`, `systemError` z konkretnym kodem — i żaden test tej
> gałęzi nie wykonywał. W repo, gdzie `IkEquationSystem` przypina testem
> praktycznie każdy własny kod błędu, to była wyraźna dziura.

```cpp
TEST(ConstraintBuilderTest, PreservesSystemRejection)
{
    // A chain with no joints declares no unknowns, and an identity transform
    // has no symbols in its translation column -- so the symbol check passes
    // vacuously and the refusal comes from the layer below, unchanged.
    const ConstraintBuilder builder;
    const auto transform = SymbolicTransform::identity();
    const ik::KinematicChain chain;

    const auto system = builder.build(transform, chain, PositionTarget{{0.0, 0.0, 0.0}});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::SystemRejected);
    EXPECT_FALSE(system.error().targetError.has_value());

    ASSERT_TRUE(system.error().systemError.has_value());
    EXPECT_EQ(system.error().systemError->code, ik::IkEquationSystemErrorCode::NoUnknowns);
}
```

Ten przypadek trafia dokładnie tam, gdzie ma trafić, i warto powiedzieć dlaczego:

```text
symbole kolumny translacji identyczności   ∅
nazwy unknowns pustego łańcucha            ∅
inwariant symboli                          ∅ ⊆ ∅, przechodzi
IkEquationSystem::create                   3 równania -> NIE NoEquations
                                           0 niewiadomych -> NoUnknowns
```

Czyli nie da się go pomylić z `UndeclaredSymbol` ani z `InvalidTarget` — jest
jedyną drogą do trzeciego kodu. Przy okazji pokazuje decyzję z `unknownsOf()`:
builder nie „naprawia" podanego łańcucha, tylko pozwala odmówić warstwie, która
ma do tego walidację.

## 5. `tests/CMakeLists.txt`

```cmake
    test_forward_kinematics_builder.cpp
    test_constraint_builder.cpp
    test_numeric_fk_validation.cpp
```

`KINEMAFORGE_URDF_DATA_DIR` jest już definiowane dla całego celu testowego —
żadnej innej zmiany w CMake nie potrzeba.

## 6. `STATUS.md`

```text
Pipeline progress    → ConstraintBuilder  ✅ implemented + tested (F2.4)
Tests                320 -> 332
Done                 nowa sekcja "ConstraintBuilder — position constraints"
Known gaps           nowy wiersz: para (transform, chain) nie jest sprawdzana
                     pod kątem pochodzenia — nazwy symboli są pozycyjne, więc
                     FK jednego sześcioosiowego robota przechodzi kontrolę
                     symboli przeciwko łańcuchowi drugiego
Working process      wiersze dla obu proposali F2.4
```

Wiersz w „Known gaps" jest wymagany przez architekturę 4.5 i jest **jedynym**
miejscem, w którym ta luka zostaje zapisana poza proposalem.

## 7. Gate

```text
build (Release)        0 ostrzeżeń
build-werror           0 ostrzeżeń
ctest                  332/332 w obu (320 + 12)
istniejące testy       zero zmienionych asercji, zero zmienionych komentarzy
end-to-end             kr640 i kr4_r600: URDF -> IkEquationSystem, 3 równania,
                       6 niewiadomych
uwaga 10               RejectsUndeclaredSymbol zielony
zakres                 zero zmian w loaderze, chain builderze, budowniczych
                       transformacji, warstwie symbolicznej, modelu równań
                       i fasadzie
```

Ostatni wiersz jest częścią gate'u świadomie: F2.4 ma **dołożyć ogniwo**, nie
przestawić pipeline'u. Każda zmiana w tych plikach oznacza, że komponent
przekroczył swoją granicę.

## 8. Ryzyka implementacyjne

| Ryzyko | Waga | Reakcja |
|---|---|---|
| `string_view` przeżywa swoje napisy | niskie | Widoki wskazują wyłącznie na `SymbolNode::name` w DAG-u transformacji, która należy do wywołującego i żyje przez całe `build()`; nic nie przechowuje ich po powrocie z funkcji — 2.1 |
| Walidacja oparta na moved-from `unknowns` | **realne** | `create()` stoi na końcu funkcji, po całym sprawdzeniu symboli — 2.1 |
| Kolejność komunikatu zależna od hashowania | średnie | Wektor w kolejności napotkania zamiast `unordered_set` — 2.1, lekcja z R2 |
| Rekurencja kolektora | niskie | Głębokość FK: 22 i 24; `ExpressionEvaluator` chodzi po tym samym drzewie rekurencyjnie w produkcji |
| `SymbolicTransform::identity()` w helperze testowym daje `1` w `(3,3)` | znikome | Kolumna translacji to wiersze 0..2; ostatni wiersz nie jest czytany |
| Pokusa rozszerzenia zakresu przy pisaniu kodu | **realne** | Wprost zakazane na początku dokumentu i w gate 7 |

## 9. Do rozstrzygnięcia w review

1. **`std::ranges::find` / `any_of` czy ręczne pętle.** Repo używa obu; wybrałem
   `<algorithm>` dla dwóch liniowych wyszukiwań po zbiorach wielkości „liczba
   osi". Alternatywa to jawne pętle bez dodatkowego include'a.
2. **Czy `kTranslationColumn` ma być stałą w anonimowym namespace `.cpp`**, jak
   wyżej, czy literałem `3` w trzech miejscach. Rekomenduję stałą — nazwa mówi,
   czym jest kolumna 3 w macierzy jednorodnej.
3. **Duplikacja `loadRobot`** — trzeci plik testowy z tym helperem. Rekomenduję
   zostawić duplikat (precedens R5), ale jeśli wolisz, wyniesienie do
   `tests/support/` jest teraz tańsze niż będzie w F2.5.

## 10. Rekomendacja

Zaimplementować po zatwierdzeniu. Po F2.4 projekt po raz pierwszy **formułuje**
IK: z pliku URDF i punktu w przestrzeni powstaje układ równań z zadeklarowanymi
niewiadomymi. Upraszczanie i rozwiązywanie należą do Fazy 3; F2.5 i F2.6
poszerzają rodzinę ograniczeń, a F2.7 domyka Fazę 2 integracją z fasadą.
