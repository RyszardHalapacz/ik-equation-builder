# Proposal: inwariant skończoności w `ExpressionFactory` (R5) — implementacja

**Etap:** F2.3.R, dokument **R5**, część 2 z 2 · **v2 — do review** · 2026-08-18

> **v2, po review.** Kod produkcyjny FULL APPROVE bez zmian. Poprawione pięć
> rzeczy w warstwie testowej i w komentarzach: (1) traversal wyczerpujący przez
> `static_assert` na nieobsłużonym typie węzła (5.2); (2) `EXPECT_FALSE(visited
> .empty())` zastąpione licznikiem odwiedzonych `ConstantNode` (5.2, 5.6);
> (3) komentarz w `ExpressionEvaluator` przestał obiecywać więcej niż
> architektura (3); (4) usunięta sprzeczność „folded / declines" w komentarzu
> istniejącego testu (4); (5) poprawiony matematycznie komentarz przy
> `kHuge`/`kTiny` (5.2). Liczba testów bez zmian: **8**, gate 320/320.

Architektura: [`proposal-expression-factory-finite-invariant-architecture.md`](proposal-expression-factory-finite-invariant-architecture.md),
**v2 FULL APPROVE**. Ten dokument nie zmienia żadnej decyzji projektowej — podaje
kod, który z niej wynika, i nazywa dwie rzeczy znalezione przy jego pisaniu (3, 4).

Kontrakt, którego dotyczy całość, w brzmieniu zatwierdzonym w architekturze §1.1:

```text
skończone ConstantNode na wejściu  +  legalna operacja fabryki
        ↓
fabryka nie utworzy nowego nieskończonego ConstantNode
```

## 1. `src/ik_equations/symbolic/ExpressionFactory.hpp`

Pełna treść po zmianie. Zmienione względem dzisiejszej: `#include <optional>`,
jeden akapit w komentarzu klasy, dwa zdania w komentarzu `constant()`, sekcja
prywatna.

```cpp
#pragma once

#include "ik_equations/symbolic/Expression.hpp"

#include <optional>
#include <string>

namespace kinemaforge::ik {

// The only way to build a composite expression: Expression's node
// constructor is private and befriends this class.
//
// Applies two kinds of local, O(1) normalization while building:
//   * constant folding — sin(c), c1 + c2, ... collapse to a constant
//   * neutral elements — x + 0 -> x, x * 1 -> x, x / 1 -> x, ...
//
// Both look at the immediate operands only. Anything needing traversal,
// operand reordering or equivalence checking belongs to a future
// EquationSimplifier, not here.
//
// Folding happens only when the result is finite. This factory is closed
// over the finite-constant contract: given operands that satisfy it, no
// operation here produces a constant that violates it. An overflow
// declines to fold and builds the operation node instead, which then
// reports NonFiniteResult when evaluated. That is a property of the
// factory's own transformations, not a runtime guard -- constant() still
// trusts its caller.
//
// Deliberately NOT applied: x * 0 -> 0. It would erase domain
// information — (1/q) * 0 is undefined at q = 0, but the folded 0 claims
// it is defined everywhere. That matters once IK equations start
// carrying variable denominators whose roots mark singularities. Folding
// two known finite constants (c * 0 -> 0) stays, since nothing is lost.
class ExpressionFactory
{
public:
    // Precondition: value is finite. NaN and +/-Inf have no legitimate
    // use here, so passing one is a caller bug, caught by assert in
    // diagnostic builds.
    //
    // Constant folding never routes a non-finite result here: an
    // overflowing fold declines and builds the operation node instead, so
    // this precondition can only be violated by a caller.
    //
    // This is a precondition, not a runtime guarantee: an NDEBUG build
    // will happily store whatever it is given.
    //
    // Returns the shared zero node for 0.0.
    Expression constant(double value) const;

    Expression symbol(std::string name) const;

    Expression add(Expression lhs, Expression rhs) const;
    Expression subtract(Expression lhs, Expression rhs) const;
    Expression multiply(Expression lhs, Expression rhs) const;
    Expression divide(Expression lhs, Expression rhs) const;

    Expression negate(Expression operand) const;
    Expression sin(Expression operand) const;
    Expression cos(Expression operand) const;

private:
    // Constant folding, but only when the arithmetic stayed finite.
    //
    // Overflow in a fold is not a caller error: 1e308 and 1e308 are both
    // legitimate constants and multiply() is a legitimate operation, yet
    // their product is not representable. Folding it would put a
    // non-finite leaf in the tree, which the class comment promises this
    // factory never does -- and which the next fold would propagate
    // (inf - inf is NaN).
    //
    // Declining to fold loses nothing: the operation node is built
    // instead, and ExpressionEvaluator reports NonFiniteResult when it is
    // evaluated. The subtree keeps saying what it is rather than
    // pretending to be a number.
    //
    // Underflow is not this function's business: 1e-308 * 1e-308 is 0.0,
    // which is finite, a legitimate constant, and folded like any other.
    //
    // Only the four binary operations route through here. negate, sin and
    // cos cannot produce a non-finite result from a finite operand.
    [[nodiscard]] std::optional<Expression> foldFinite(double value) const;
};

} // namespace kinemaforge::ik
```

Trzy uwagi do tego, co się **nie** zmieniło:

- publiczne metody nadal bez `[[nodiscard]]` — atrybut jest tylko na nowym
  helperze, zgodnie z zatwierdzoną architekturą; dokładanie go do ośmiu
  istniejących metod byłoby osobną zmianą,
- `sin`, `cos`, `negate` bez `foldFinite`, uzasadnienie w komentarzu helpera,
  żeby przy następnej lekturze nie wyglądało to na przeoczenie,
- `<string>` zostaje (`symbol(std::string)`).

## 2. `src/ik_equations/symbolic/ExpressionFactory.cpp`

Pełna treść po zmianie.

```cpp
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <cassert>
#include <cmath>
#include <optional>
#include <utility>

namespace kinemaforge::ik {

Expression ExpressionFactory::constant(double value) const
{
    assert(std::isfinite(value) && "constants must be finite");
    if (value == 0.0)
        return Expression{}; // shared zero node
    return Expression{ExpressionNode{ConstantNode{value}}};
}

std::optional<Expression> ExpressionFactory::foldFinite(double value) const
{
    if (!std::isfinite(value))
        return std::nullopt;
    return constant(value);
}

Expression ExpressionFactory::symbol(std::string name) const
{
    assert(!name.empty() && "symbol names must not be empty");
    return Expression{ExpressionNode{SymbolNode{std::move(name)}}};
}

Expression ExpressionFactory::add(Expression lhs, Expression rhs) const
{
    if (isConstant(lhs) && isConstant(rhs))
        if (auto folded = foldFinite(constantValue(lhs) + constantValue(rhs)))
            return *folded;
    if (isZero(rhs))
        return lhs;
    if (isZero(lhs))
        return rhs;
    return Expression{ExpressionNode{AddNode{std::move(lhs), std::move(rhs)}}};
}

Expression ExpressionFactory::subtract(Expression lhs, Expression rhs) const
{
    if (isConstant(lhs) && isConstant(rhs))
        if (auto folded = foldFinite(constantValue(lhs) - constantValue(rhs)))
            return *folded;
    if (isZero(rhs))
        return lhs;
    return Expression{ExpressionNode{SubtractNode{std::move(lhs), std::move(rhs)}}};
}

Expression ExpressionFactory::multiply(Expression lhs, Expression rhs) const
{
    if (isConstant(lhs) && isConstant(rhs))
        if (auto folded = foldFinite(constantValue(lhs) * constantValue(rhs)))
            return *folded;
    // No x * 0 -> 0 here: see the class comment.
    if (isOne(rhs))
        return lhs;
    if (isOne(lhs))
        return rhs;
    return Expression{ExpressionNode{MultiplyNode{std::move(lhs), std::move(rhs)}}};
}

Expression ExpressionFactory::divide(Expression lhs, Expression rhs) const
{
    assert(!isZero(rhs) && "division by literal zero");
    if (isConstant(lhs) && isConstant(rhs))
        if (auto folded = foldFinite(constantValue(lhs) / constantValue(rhs)))
            return *folded;
    if (isOne(rhs))
        return lhs;
    return Expression{ExpressionNode{DivideNode{std::move(lhs), std::move(rhs)}}};
}

Expression ExpressionFactory::negate(Expression operand) const
{
    if (isConstant(operand))
        return constant(-constantValue(operand));
    return Expression{ExpressionNode{NegateNode{std::move(operand)}}};
}

Expression ExpressionFactory::sin(Expression operand) const
{
    if (isConstant(operand))
        return constant(std::sin(constantValue(operand)));
    return Expression{ExpressionNode{SinNode{std::move(operand)}}};
}

Expression ExpressionFactory::cos(Expression operand) const
{
    if (isConstant(operand))
        return constant(std::cos(constantValue(operand)));
    return Expression{ExpressionNode{CosNode{std::move(operand)}}};
}

} // namespace kinemaforge::ik
```

**Cztery operacje, jedno miejsce sprawdzenia.** Jedyna zmiana w każdej z nich to
zamiana `return constant(a op b);` na próbę: `if (auto folded = foldFinite(a op b))
return *folded;`. Nigdzie w tym pliku nie zostaje bezpośrednie
`constant(<wynik działania na dwóch stałych>)` — to jest punkt 3 z listy
weryfikacyjnej review i widać go w tym pliku wprost: po zmianie `constant()`
wywołują tylko `foldFinite`, `negate`, `sin` i `cos`.

**`<optional>` jest i w nagłówku, i tutaj.** W nagłówku, bo `std::optional`
występuje w sygnaturze — to była wymagana poprawka v2. Tutaj, bo `.cpp` sam
nazywa `std::optional` w definicji, a zasada z R4 (`<utility>` dla `std::move`)
mówi, że plik deklaruje to, czego używa, zamiast liczyć na include tranzytywny —
nawet gdy tranzytywność akurat pochodzi z własnego nagłówka.

**`foldFinite` stoi zaraz pod `constant()`**, a nie na końcu pliku: czyta się je
razem, bo razem tworzą kontrakt stałych.

## 3. `src/ik_equations/symbolic/ExpressionEvaluator.cpp` — komentarz

Zatwierdzone w architekturze §10 pkt 3. Dzisiejszy komentarz przy sprawdzeniu
skończoności opisuje podział odpowiedzialności, który po R5 przestaje być
aktualny:

```cpp
// PRZED
// Checked for every node, leaves included. ExpressionFactory asserts that
// constants are finite, but that assert is gone under NDEBUG, and an
// overflow such as q1 * q2 with both at 1e308 is reachable in any build.
```

```cpp
// PO
// Checked for every node, leaves included. With ExpressionFactory's
// preconditions respected, constant leaves are finite and overflowing
// constant folds remain operation nodes. Finite leaves and finite symbol
// bindings can still produce a non-finite value during evaluation
// (for example q1 * q2 with both bound to 1e308).
//
// The leaf check also keeps evaluation robust if a caller violates
// constant()'s finite-value precondition in an NDEBUG build.
```

> **v2, po review.** v1 pisała „ExpressionFactory keeps constants finite —
> including refusing to fold an overflowing constant expression", co obiecywało
> więcej niż architektura §1.1: `constant(inf)` w Release nadal zbuduje
> `ConstantNode{inf}`, jeśli wywołujący złamie precondition. Nowe brzmienie
> mówi „with preconditions respected" i wprost nazywa drugą rolę tego
> sprawdzenia — odporność evaluatora, gdy kontrakt zostanie złamany.

Sam kod (`if (!std::isfinite(*result)) return makeError(...)`) **bez zmian**, tak
jak żaden `EvaluationErrorCode`.

## 4. Znalezione przy pisaniu: nieaktualny komentarz w istniejącym teście

Nie było tego w architekturze, bo widać to dopiero przy kodzie testów.
`tests/test_expression_evaluator.cpp:277`:

```cpp
TEST(ExpressionEvaluatorTest, ReportsNonFiniteResult)
{
    // Two symbols, not two constants: constant folding routes back through
    // ExpressionFactory::constant, whose isfinite assert would abort the
    // process while building 1e308 * 1e308.
```

Po R5 to zdanie przestaje być prawdziwe — folding nie zawoła `constant()`, tylko
zbuduje `MultiplyNode`. Test zostaje bez zmian (dwa symbole to nadal właściwy
sposób sprawdzenia ścieżki ewaluacji), zmienia się komentarz:

```cpp
    // Two symbols intentionally exercise runtime overflow from finite bindings.
    // Two constants would exercise ExpressionFactory's attempted constant-fold
    // path instead; since R5 that fold is declined and remains a MultiplyNode --
    // see ExpressionFactoryTest.OverflowingFoldEvaluatesToNonFiniteResult.
```

> **v2, po review.** v1 mówiła jednocześnie „two constants would be folded at
> build time" i „since R5 that fold declines" — sprzeczność w dwóch sąsiednich
> zdaniach. Nowe brzmienie nazywa to, czym ta ścieżka faktycznie jest: próbą
> foldu, która po R5 zostaje odrzucona.

To jest jedyna zmiana w istniejącym teście w całym R5, i jest to zmiana
**komentarza**, nie asercji. Gate „312 istniejących testów przechodzi bez edycji"
zostaje spełniony w znaczeniu, które ma sens: żadne oczekiwanie nie jest
poprawiane pod nowy kod.

## 5. `tests/test_expression_factory.cpp` — osiem testów

### 5.1 Nagłówek pliku

Dochodzą include'y i using-i potrzebne dwóm ostatnim testom:

```cpp
#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <numbers>
#include <optional>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <variant>

using kinemaforge::ik::EvaluationErrorCode;
using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionEvaluator;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::ExpressionNode;
using kinemaforge::ik::ExpressionType;
using kinemaforge::ik::SymbolValues;
using kinemaforge::ik::constantValue;
using kinemaforge::ik::isConstant;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::sameNode;
using kinemaforge::ik::structurallyEqual;

namespace ik = kinemaforge::ik;   // file scope, jak w test_numeric_fk_validation.cpp
```

`KINEMAFORGE_URDF_DATA_DIR` jest zdefiniowane dla całego celu `kinemaforge_tests`
(`tests/CMakeLists.txt`), więc **żadna zmiana w CMake nie jest potrzebna**.

### 5.2 Pomocnicze, w anonimowym namespace

```cpp
namespace {

// Large and tiny finite magnitudes chosen so the tested operations overflow
// or underflow while both operands remain legitimate constants.
constexpr double kHuge = 1e308;
constexpr double kTiny = 1e-308;

std::optional<ik::SymbolicTransform> forwardKinematics(const char* fileName)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName);

    ik::KinematicChainBuilder chainBuilder;
    const auto chain = chainBuilder.build(robot, "base_link", "tool0");
    if (!chain)
    {
        ADD_FAILURE() << "chain build failed for " << fileName;
        return std::nullopt;
    }

    const ik::JointTransformBuilder transformBuilder;
    const ik::ForwardKinematicsBuilder fkBuilder;
    return fkBuilder.build(*chain, transformBuilder);
}

// Walks each unique node once. An FK transform shares sub-trees heavily --
// 281 unique nodes against 21 882 counted with multiplicity for kr640 -- so a
// walk without the visited set would revisit the same subexpression thousands
// of times for no additional coverage.
//
// Counts the constant leaves it checked, so the caller can tell "every constant
// was finite" apart from "the walk never reached a constant".
void expectConstantLeavesFinite(const Expression& expression,
                                std::unordered_set<const ExpressionNode*>& visited,
                                std::size_t& constantCount)
{
    if (!visited.insert(&expression.node()).second)
        return;

    std::visit(
        [&](const auto& node) {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, ik::ConstantNode>)
            {
                ++constantCount;
                EXPECT_TRUE(std::isfinite(node.value))
                    << "non-finite constant leaf: " << node.value;
            }
            else if constexpr (std::is_same_v<Node, ik::AddNode>
                               || std::is_same_v<Node, ik::SubtractNode>
                               || std::is_same_v<Node, ik::MultiplyNode>
                               || std::is_same_v<Node, ik::DivideNode>)
            {
                expectConstantLeavesFinite(node.lhs, visited, constantCount);
                expectConstantLeavesFinite(node.rhs, visited, constantCount);
            }
            else if constexpr (std::is_same_v<Node, ik::NegateNode>
                               || std::is_same_v<Node, ik::SinNode>
                               || std::is_same_v<Node, ik::CosNode>)
            {
                expectConstantLeavesFinite(node.operand, visited, constantCount);
            }
            else
            {
                // Symbol is the only leaf with nothing to check. A node type
                // added later must state explicitly whether it has children:
                // without this, it would silently fall into "nothing to do" and
                // the walk would stop above its subtree while still passing.
                static_assert(std::is_same_v<Node, ik::SymbolNode>,
                              "Unhandled ExpressionNode type");
            }
        },
        expression.node().value);
}

} // namespace
```

Rekurencja, nie jawny stos: zmierzona głębokość FK to 22 i 24 węzły
(`STATUS.md`, „Recursion depth scaling unmeasured"), a `ExpressionEvaluator`
chodzi po tym samym drzewie rekurencyjnie w kodzie produkcyjnym. Test nie
wprowadza nowego ryzyka względem tego, co już robi produkcja.

### 5.3 Cztery odmowy foldu

```cpp
// --- finite-constant closure ----------------------------------------

TEST(ExpressionFactoryTest, DoesNotFoldOverflowingSum)
{
    const ExpressionFactory factory;
    const auto sum = factory.add(factory.constant(kHuge), factory.constant(kHuge));

    EXPECT_EQ(sum.type(), ExpressionType::Add);
    EXPECT_FALSE(isConstant(sum));
}

TEST(ExpressionFactoryTest, DoesNotFoldOverflowingDifference)
{
    const ExpressionFactory factory;
    const auto difference =
        factory.subtract(factory.constant(-kHuge), factory.constant(kHuge));

    EXPECT_EQ(difference.type(), ExpressionType::Subtract);
    EXPECT_FALSE(isConstant(difference));
}

TEST(ExpressionFactoryTest, DoesNotFoldOverflowingProduct)
{
    const ExpressionFactory factory;
    const auto product = factory.multiply(factory.constant(kHuge), factory.constant(kHuge));

    EXPECT_EQ(product.type(), ExpressionType::Multiply);
    EXPECT_FALSE(isConstant(product));
}

TEST(ExpressionFactoryTest, DoesNotFoldOverflowingQuotient)
{
    const ExpressionFactory factory;
    const auto quotient = factory.divide(factory.constant(kHuge), factory.constant(kTiny));

    EXPECT_EQ(quotient.type(), ExpressionType::Divide);
    EXPECT_FALSE(isConstant(quotient));
}
```

Sprawdzana jest **struktura**, nie wartość. Przed R5 każdy z tych czterech
testów kończył się `ConstantNode{±inf}` w Release, a `abort()`-em w
`build-werror` (pusty typ builda → asercje aktywne) — czyli faktycznie pinuje
zmianę, a nie stan, który już obowiązywał.

### 5.4 Domknięcie spina się z ewaluacją

```cpp
TEST(ExpressionFactoryTest, OverflowingFoldEvaluatesToNonFiniteResult)
{
    // The other half of declining to fold: the surviving node is not a dead
    // end, it is an expression whose value happens not to be representable,
    // and the existing error model already says so.
    const ExpressionFactory factory;
    const auto product = factory.multiply(factory.constant(kHuge), factory.constant(kHuge));

    ExpressionEvaluator evaluator{SymbolValues{}};
    const auto result = evaluator.evaluate(product);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::NonFiniteResult);
}
```

### 5.5 Czego R5 **nie** wyłączyło

```cpp
TEST(ExpressionFactoryTest, StillFoldsWhenResultIsFinite)
{
    // Explicit regression pin for R5, read together with the four DoesNotFold
    // tests above: declining an overflow must not cost ordinary folding.
    // FoldsConstantArithmetic covers the same values from the other side.
    const ExpressionFactory factory;
    const auto sum = factory.add(factory.constant(2.0), factory.constant(3.0));

    ASSERT_EQ(sum.type(), ExpressionType::Constant);
    EXPECT_DOUBLE_EQ(constantValue(sum), 5.0);
}

TEST(ExpressionFactoryTest, FoldsUnderflowingProductToZero)
{
    // Underflow is not the same as non-finite: 1e-308 * 1e-308 is exactly 0.0,
    // which is finite, foldable, and reaches the shared zero node like any
    // other zero. isfinite is the boundary, not "the result is surprising".
    const ExpressionFactory factory;
    const auto product = factory.multiply(factory.constant(kTiny), factory.constant(kTiny));

    EXPECT_TRUE(isConstant(product));
    EXPECT_TRUE(isZero(product));
    EXPECT_TRUE(sameNode(product, Expression{}));
}
```

### 5.6 Inwariant na prawdziwym pipelinie

```cpp
TEST(ExpressionFactoryTest, AllConstantLeavesRemainFinite)
{
    // Integration smoke test, not a proof: it says that every constant leaf in
    // the FK DAGs the current pipeline builds for these two robots is finite.
    // A future operation that folds without checking would slip past this
    // unless FK uses it and overflows on these fixtures -- the direct
    // DoesNotFold* tests and the single fold site in foldFinite are what
    // actually guard the contract.
    for (const char* fileName : {"kr640.urdf", "kr4_r600.urdf"})
    {
        const auto fk = forwardKinematics(fileName);
        ASSERT_TRUE(fk.has_value()) << fileName;

        std::unordered_set<const ExpressionNode*> visited;
        std::size_t constantCount = 0;
        for (std::size_t row = 0; row < ik::SymbolicTransform::rows; ++row)
            for (std::size_t column = 0; column < ik::SymbolicTransform::columns; ++column)
                expectConstantLeavesFinite((*fk)(row, column), visited, constantCount);

        EXPECT_GT(constantCount, 0u) << fileName;
    }
}
```

> **v2, po review.** v1 kończyła się `EXPECT_FALSE(visited.empty())` z
> komentarzem, że chroni przed „przejściem przez nieprzejście niczego". Nie
> chroniła: `SymbolicTransform` ma zawsze szesnaście komórek, więc pierwsze
> `visited.insert` z definicji czyni zbiór niepustym. Licznik `ConstantNode`
> sprawdza to, o co naprawdę chodziło — że obchód **dotarł do stałych** i je
> zbadał. Dla obu robotów jest ich dużo (ostatni wiersz `[0 0 0 1]`, składowe
> osi, wartości `origin`), więc zero oznaczałoby zerwany obchód.

## 6. Plan zmian w plikach

```text
src/ik_equations/symbolic/ExpressionFactory.hpp     +#include <optional>
                                                    +1 metoda prywatna (+22 linie
                                                     komentarza), 2 akapity zmienione
src/ik_equations/symbolic/ExpressionFactory.cpp     +#include <optional>
                                                    +6 linii (foldFinite)
                                                    4 x zmieniona ścieżka foldu
src/ik_equations/symbolic/ExpressionEvaluator.cpp   komentarz (3), kod bez zmian
tests/test_expression_evaluator.cpp                 komentarz (4), asercje bez zmian
tests/test_expression_factory.cpp                   +8 testów, +2 helpery, include'y
STATUS.md                                           R5 w bloku pipeline'u,
                                                    wiersz o R5 w tabeli proposali
proposal/.../README.md                              R5 → ZAIMPLEMENTOWANE
```

Zero zmian w: `CMakeLists.txt` (definicja `KINEMAFORGE_URDF_DATA_DIR` już jest),
loaderze, `KinematicChainBuilder`, budowniczych transformacji, modelu równań,
`Expression.hpp`, `SymbolicMatrix`.

## 7. Gate

Lista weryfikacyjna z review architektury, punkt po punkcie:

```text
1. kod foldFinite()                     2, sześć linii, jedno miejsce
2. wszystkie 4 operacje przez helper    2, add/subtract/multiply/divide
3. brak constant(a op b) bezpośrednio   2, constant() wołają już tylko
                                        foldFinite, negate, sin, cos
4. struktura 4 węzłów wynikowych        5.3, type() + !isConstant
5. NonFiniteResult z evaluatora         5.4
6. underflow → shared zero              5.5, isZero + sameNode
7. traversal testu FK z visited         5.2, unordered_set<const ExpressionNode*>
8. 320/320 + build-werror               niżej
```

```text
build (Release)        0 ostrzeżeń
build-werror           0 ostrzeżeń
ctest                  320/320 w obu (312 + 8)
istniejące testy       zero zmienionych asercji; jedna zmiana komentarza (4)
sonda z architektury   po zmianie: isConstant=0 dla wszystkich czterech,
2.1                    isConstant=1 value=0 dla underflow
```

Sonda jest przy tym najostrzejszym sprawdzeniem: ten sam plik, ten sam sposób
kompilacji (`-O2 -DNDEBUG`), a wynik ma być inny niż zapisany w architekturze.

## 8. Ryzyka implementacyjne

| Ryzyko | Waga | Reakcja |
|---|---|---|
| `EXPECT_FALSE(isConstant(...))` przechodziłby też przy zupełnie zepsutym foldzie | niskie | Testy sprawdzają **oba**: `type()` i `isConstant()`; do tego `StillFoldsWhenResultIsFinite` z drugiej strony |
| Rekurencja w helperze testowym | niskie | Głębokość FK: 22 i 24; `ExpressionEvaluator` robi to samo w produkcji |
| Duplikacja `forwardKinematics()` względem `test_numeric_fk_validation.cpp` | średnie | 9 do rozstrzygnięcia |
| `-Wfloat-equal` na `value == 0.0` w `constant()` | znikome | Linia niezmieniona, dziś kompiluje się czysto w obu buildach |

## 9. Rozstrzygnięte w review (v2)

| # | Pytanie z v1 | Rozstrzygnięcie |
|---|---|---|
| 1 | Duplikacja `forwardKinematics()` względem `test_numeric_fk_validation.cpp` | **Zduplikować lokalnie.** 15 linii helpera nie uzasadnia tworzenia teraz wspólnej infrastruktury w `tests/support/` |
| 2 | `<optional>` również w `.cpp` | **Tak, w obu.** Nie szkodzi, a jasno deklaruje zależność TU |
| 3 | Komentarz w `test_expression_evaluator.cpp` w zakresie R5 | **Tak.** Po R5 obecny komentarz jest nieprawdziwy |

Otwarte pytania: brak.

## 10. Rekomendacja

Zaimplementować po zatwierdzeniu. Po R5 etap **F2.3.R jest zamknięty**: wszystkie
dziesięć uwag review ma odpowiedź — dziewięć zaimplementowanych, uwaga 10 wchodzi
jako warunek kontraktowy do F2.4 (`ConstraintBuilder` jedynym producentem
`IkEquationSystem`). Gate przed F2.4 się otwiera.
