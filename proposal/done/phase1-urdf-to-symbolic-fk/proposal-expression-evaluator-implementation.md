# Proposal: `ExpressionEvaluator` — implementacja

## Prompt

> Werdykt: APPROVE z drobnymi korektami przed proposalem implementacyjnym. Do proposalu implementacyjnego należy przenieść: poprawną liczbę testów — minimum 24 nowe / 185 łącznie po dodaniu testu kolejności błędów; najlepiej dodatkowy test rozróżniający tożsamość od strukturalnej równości — wtedy 25 nowych / 186 łącznie; komentarz o monotonicznym czasie życia wpisów cache'u; pełne definicje `ExpressionIdentityHash` i `ExpressionIdentityEqual` przed deklaracją `memo_`. Nie potrzeba trzeciej rundy architektury.

Realizacja architektury zatwierdzonej w `proposal-expression-evaluator-architecture.md` (werdykt `APPROVE` po dwóch rundach review).

## Status weryfikacji

**Kod poniżej nie został przeze mnie skompilowany.** Zgodnie z procesem: proposal jest dokumentem, kod trafia na dysk dopiero po zatwierdzeniu.

Review zweryfikowało w poprzedniej rundzie, że proponowany **kształt API** kompiluje się w C++23 z obecnym `Expression.hpp` — pełne zagnieżdżone typy hash/equality, nieskopiowalność i domyślne operacje przenoszenia. Ciała funkcji i testy poniżej nie były weryfikowane przez nikogo; §7 wymienia trzy miejsca, gdzie widzę realne ryzyko.

## Cztery korekty z review — wprowadzone

| # | Korekta | Gdzie |
|---|---|---|
| 1 | liczba testów: **25 nowych, 186 łącznie** — moje 22/183 było zwykłym błędem rachunkowym | §5, §6 |
| 2 | `ReportsLeftOperandErrorFirst` — pinuje kolejność `lhs` przed `rhs` | §5.2 |
| 3 | `DoesNotMergeStructurallyEqualDistinctNodes` — tożsamość ≠ hash-consing | §5.3 |
| 4 | komentarz kontraktowy o monotonicznym wzroście cache'u | §3.1, komentarz klasy |

Plus przeniesione z architektury: pełne definicje `ExpressionIdentityHash` / `ExpressionIdentityEqual` **przed** `memo_` (§5.2 architektury — inaczej `unordered_map` instancjonuje się na typach niezupełnych).

## Cztery korekty z trzeciej rundy review — wprowadzone

Review skompilowało kod z tego dokumentu (GCC 14.2 i Clang 17, C++23, `-Wall -Wextra -Wpedantic -Werror` — czysto) i uruchomiło wszystkie testy pod ASan/UBSan — 25/25, bez zgłoszeń. Zgłoszone poprawki:

| # | Korekta | Gdzie |
|---|---|---|
| 1 | brak `<utility>` w pliku testowym mimo użycia `std::move` — kompiluje się tranzytywnie, ale plik ma być samowystarczalny | §4, lista include'ów |
| 2 | `RejectsNonFiniteSymbolValue` sprawdzał tylko `NaN`; regresja do `std::isnan` przeszłaby ten test | §4 — pętla po `NaN`, `+Inf`, `-Inf` |
| 3 | *(opcjonalna, przyjęta)* `MemoizesSharedLeafAcrossDistinctParents` — pinuje cache'owanie **liścia**, nie tylko poddrzewa | §4, §5.4 — **26 testów, 187 łącznie** |
| 4 | *(sugestia, przyjęta)* zdjąć ręczne `noexcept` z operacji przenoszenia | §2 — kompilator wyprowadza warunek ze składowych; usuwa ryzyko §8.1 |

---

## 1. Stan obecny

### 1.1 Co istnieje

Brak jakiegokolwiek pliku evaluatora. `ExpressionEvaluator` nie jest nigdzie zadeklarowany ani wspomniany w kodzie — tylko w `STATUS.md` jako największa luka.

### 1.2 Co jest dostępne — zweryfikowane w nagłówkach

`Expression` (z `Expression.hpp`):

```cpp
class Expression
{
public:
    Expression();                                   // publiczny — współdzielone zero
    ExpressionType type() const noexcept;
    const ExpressionNode& node() const noexcept;    // publiczne — klucz cache'u
private:
    std::shared_ptr<const ExpressionNode> node_;
};

struct ConstantNode { double value{}; };
struct SymbolNode   { std::string name; };
struct AddNode      { Expression lhs, rhs; };
struct SubtractNode { Expression lhs, rhs; };
struct MultiplyNode { Expression lhs, rhs; };
struct DivideNode   { Expression lhs, rhs; };
struct NegateNode   { Expression operand; };
struct SinNode      { Expression operand; };
struct CosNode      { Expression operand; };

struct ExpressionNode { std::variant< /* dziewięć powyższych */ > value; };

bool sameNode(const Expression& lhs, const Expression& rhs) noexcept;   // O(1)
bool structurallyEqual(const Expression& lhs, const Expression& rhs);
bool isConstant(const Expression&) noexcept;
```

**Evaluator nie wymaga żadnej zmiany w istniejącym API.** `node()` i `sameNode()` są publiczne, `ExpressionNode` jest typem zupełnym w nagłówku.

Kontrakty fabryki istotne dla testów (z `ExpressionFactory.cpp`): dzielenie przez literalne zero asertuje; `constant(-0.0)` zapada się do współdzielonego zera dodatniego; zwijanie stałych wraca przez `constant()`, które asertuje `isfinite`.

### 1.3 Stan testów

161/161. Po tej zmianie oczekiwane **187/187**.

---

## 2. `src/ik_equations/symbolic/ExpressionEvaluator.hpp` (nowy plik)

```cpp
#pragma once

#include "ik_equations/symbolic/Expression.hpp"

#include <cstddef>
#include <expected>
#include <functional>
#include <string>
#include <unordered_map>

namespace kinemaforge::ik {

using SymbolValues = std::unordered_map<std::string, double>;

enum class EvaluationErrorCode
{
    MissingSymbol,
    NonFiniteSymbolValue,
    DivisionByZero,
    NonFiniteResult
};

struct EvaluationError
{
    EvaluationErrorCode code{};

    // Set only for the two symbol errors. Empty otherwise, which is
    // unambiguous: ExpressionFactory rejects empty symbol names.
    std::string symbolName;
};

struct EvaluationStatistics
{
    // Counts the outcome of the cache lookup, not the outcome of the
    // evaluation: a node whose evaluation then fails still counts as a miss.
    std::size_t cacheMisses{};
    std::size_t cacheHits{};
};

// Evaluates an expression DAG numerically under one fixed set of symbol
// values.
//
// One instance is one substitution. All sixteen cells of a forward-kinematics
// transform are meant to be evaluated with the same instance, so they share
// the cache; a different joint configuration needs a new instance. Binding the
// values to the object's lifetime makes it impossible to reuse a cache that
// was filled for different values.
//
// Memoization is mandatory rather than an optimization. A composed FK
// transform measures 281 unique nodes but 21 882 counted with multiplicity for
// kr640.urdf, and 516 vs 153 703 for kr4_r600.urdf.
//
// The cache is keyed by Expression, not by a raw node pointer, so an entry
// keeps its node alive. evaluate() takes a reference that may bind to a
// temporary; with a raw pointer key the node could die, its address be reused
// by the next allocation, and the cache then answer for the wrong node.
//
// Lifetime contract: this is a short-lived session for one symbol binding and
// one related expression DAG. Successfully evaluated nodes stay cached — and
// alive — until the evaluator is destroyed, so the cache grows monotonically.
// Do not use one instance as a long-lived global interpreter.
//
// Not thread-safe: the cache is mutable. The DAG itself is immutable, so
// several evaluators may read the same tree concurrently.
class ExpressionEvaluator
{
public:
    explicit ExpressionEvaluator(SymbolValues values);

    // A session is not a value. Copying would silently duplicate the cache
    // and the statistics, and it is not clear whether that means "snapshot"
    // or "second session" — so it is not offered.
    ExpressionEvaluator(const ExpressionEvaluator&) = delete;
    ExpressionEvaluator& operator=(const ExpressionEvaluator&) = delete;

    // No explicit noexcept: std::unordered_map's move constructor is only
    // conditionally noexcept, so letting the compiler derive the specifier
    // from the members states the truth instead of asserting a stronger
    // guarantee that would turn a throwing move into std::terminate.
    ExpressionEvaluator(ExpressionEvaluator&&) = default;
    ExpressionEvaluator& operator=(ExpressionEvaluator&&) = default;

    // Evaluates both operands of every binary node; nothing is skipped based
    // on the value of the other operand. In particular a zero factor does not
    // short-circuit a Multiply -- that is what keeps (1/q) * 0 reporting
    // DivisionByZero at q = 0 instead of quietly yielding 0, which is the
    // whole reason ExpressionFactory has no x * 0 -> 0 rule.
    //
    // Evaluation does stop at the first error, and operands are visited left
    // to right, so the reported error is deterministic.
    [[nodiscard]] std::expected<double, EvaluationError>
    evaluate(const Expression& expression);

    EvaluationStatistics statistics() const noexcept;

private:
    // Both types must be COMPLETE here: declaring memo_ instantiates
    // unordered_map, which stores Hash and KeyEqual as subobjects. A forward
    // declaration would make the member declaration ill-formed.
    struct ExpressionIdentityHash
    {
        std::size_t operator()(const Expression& expression) const noexcept
        {
            return std::hash<const ExpressionNode*>{}(&expression.node());
        }
    };

    struct ExpressionIdentityEqual
    {
        bool operator()(const Expression& lhs, const Expression& rhs) const noexcept
        {
            return sameNode(lhs, rhs);
        }
    };

    using Memo = std::unordered_map<Expression, double,
                                    ExpressionIdentityHash, ExpressionIdentityEqual>;

    std::expected<double, EvaluationError> evaluateNode(const Expression& expression);

    SymbolValues values_;
    Memo memo_;
    EvaluationStatistics statistics_{};
};

} // namespace kinemaforge::ik
```

### 2.1 Dlaczego `evaluate` deleguje do prywatnego `evaluateNode`

Dziś oba mają identyczne ciało, więc rozdzielenie może wyglądać na zbędne. Zostawiam je, bo publiczna metoda jest miejscem na wszystko, co ma się dziać **raz na wywołanie**, a nie raz na węzeł — dziś nic takiego nie ma, ale pierwsza rzecz tego rodzaju (limit głębokości, licznik wywołań) nie powinna wymagać przeprojektowania rekurencji.

---

## 3. `src/ik_equations/symbolic/ExpressionEvaluator.cpp` (nowy plik)

```cpp
#include "ik_equations/symbolic/ExpressionEvaluator.hpp"

#include <cmath>
#include <type_traits>
#include <utility>
#include <variant>

namespace kinemaforge::ik {

namespace {

std::unexpected<EvaluationError> makeError(EvaluationErrorCode code,
                                           std::string symbolName = {})
{
    return std::unexpected(EvaluationError{code, std::move(symbolName)});
}

} // namespace

ExpressionEvaluator::ExpressionEvaluator(SymbolValues values)
    : values_(std::move(values))
{
}

EvaluationStatistics ExpressionEvaluator::statistics() const noexcept
{
    return statistics_;
}

std::expected<double, EvaluationError>
ExpressionEvaluator::evaluate(const Expression& expression)
{
    return evaluateNode(expression);
}

std::expected<double, EvaluationError>
ExpressionEvaluator::evaluateNode(const Expression& expression)
{
    if (const auto cached = memo_.find(expression); cached != memo_.end())
    {
        ++statistics_.cacheHits;
        return cached->second;
    }
    ++statistics_.cacheMisses;

    const auto result = std::visit(
        [this](const auto& node) -> std::expected<double, EvaluationError> {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, ConstantNode>)
            {
                return node.value;
            }
            else if constexpr (std::is_same_v<Node, SymbolNode>)
            {
                const auto binding = values_.find(node.name);
                if (binding == values_.end())
                    return makeError(EvaluationErrorCode::MissingSymbol, node.name);

                // Bindings are validated on read, not in the constructor: an
                // unused NaN binding cannot affect any result, so it is not
                // an error. See AcceptsNonFiniteBindingForUnusedSymbol.
                if (!std::isfinite(binding->second))
                    return makeError(EvaluationErrorCode::NonFiniteSymbolValue, node.name);

                return binding->second;
            }
            else if constexpr (requires { node.lhs; node.rhs; })
            {
                // Left before right, unconditionally. Both are visited unless
                // the left one already failed; neither is skipped because of
                // the other's value.
                const auto lhs = evaluateNode(node.lhs);
                if (!lhs) return std::unexpected(lhs.error());

                const auto rhs = evaluateNode(node.rhs);
                if (!rhs) return std::unexpected(rhs.error());

                if constexpr (std::is_same_v<Node, AddNode>)
                    return *lhs + *rhs;
                else if constexpr (std::is_same_v<Node, SubtractNode>)
                    return *lhs - *rhs;
                else if constexpr (std::is_same_v<Node, MultiplyNode>)
                    return *lhs * *rhs;
                else
                {
                    static_assert(std::is_same_v<Node, DivideNode>);
                    // Exact comparison, no tolerance -- consistent with the
                    // rest of the symbolic layer. Catches -0.0 for free,
                    // since -0.0 == 0.0.
                    if (*rhs == 0.0)
                        return makeError(EvaluationErrorCode::DivisionByZero);
                    return *lhs / *rhs;
                }
            }
            else
            {
                const auto operand = evaluateNode(node.operand);
                if (!operand) return std::unexpected(operand.error());

                if constexpr (std::is_same_v<Node, NegateNode>)
                    return -*operand;
                else if constexpr (std::is_same_v<Node, SinNode>)
                    return std::sin(*operand);
                else
                {
                    static_assert(std::is_same_v<Node, CosNode>);
                    return std::cos(*operand);
                }
            }
        },
        expression.node().value);

    if (!result)
        return result;

    // Checked for every node, leaves included. ExpressionFactory asserts that
    // constants are finite, but that assert is gone under NDEBUG, and an
    // overflow such as q1 * q2 with both at 1e308 is reachable in any build.
    if (!std::isfinite(*result))
        return makeError(EvaluationErrorCode::NonFiniteResult);

    memo_.emplace(expression, *result);
    return result;
}

} // namespace kinemaforge::ik
```

### 3.1 Trzy właściwości warte zaprotokołowania

**Cache przechowuje wyłącznie sukcesy.** Ścieżka błędna nie jest ścieżką wydajnościową tego komponentu; powtórne wywołanie dla tego samego błędnego poddrzewa policzy je ponownie i to jest akceptowalne. Cache'owanie `std::expected` byłoby poprawne (wartości symboli są niezmienne przez sesję, więc błąd jest deterministyczny), ale komplikuje typ wpisu bez zysku tam, gdzie zależy nam na wydajności.

**Cache rośnie monotonicznie.** Nie ma `clearCache()` — evaluator jest sesją, nie długowiecznym interpreterem, i tak jest to zapisane w komentarzu klasy. Dla FK to 281–516 węzłów.

**Miss jest liczony przed próbą ewaluacji.** Węzeł, którego ewaluacja następnie zawiedzie, liczy się jako miss i nie trafia do cache'u. Wynika to wprost z nazwy licznika i nie wymaga osobnej reguły.

---

## 4. `tests/test_expression_evaluator.cpp` (nowy plik) — 26 testów

```cpp
#include <gtest/gtest.h>

#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <utility>

using kinemaforge::ik::EvaluationErrorCode;
using kinemaforge::ik::EvaluationStatistics;
using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionEvaluator;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::SymbolValues;
using kinemaforge::ik::sameNode;
using kinemaforge::ik::structurallyEqual;

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Most node-coverage tests need at least one symbolic operand: a purely
// constant expression is folded by the factory at build time, so the test
// would be exercising constant folding rather than evaluation.
Expression symbolic(const ExpressionFactory& factory, const char* name)
{
    return factory.symbol(name);
}

} // namespace

// --- node coverage --------------------------------------------------

TEST(ExpressionEvaluatorTest, EvaluatesConstant)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{}};

    const auto result = evaluator.evaluate(factory.constant(2.5));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 2.5);
}

TEST(ExpressionEvaluatorTest, EvaluatesSymbol)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 1.5}}};

    const auto result = evaluator.evaluate(symbolic(factory, "q1"));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 1.5);
}

TEST(ExpressionEvaluatorTest, EvaluatesAdd)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 1.0}}};

    const auto result =
        evaluator.evaluate(factory.add(symbolic(factory, "q1"), factory.constant(2.0)));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 3.0);
}

TEST(ExpressionEvaluatorTest, EvaluatesSubtract)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 1.0}}};

    const auto result =
        evaluator.evaluate(factory.subtract(symbolic(factory, "q1"), factory.constant(2.0)));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, -1.0);
}

TEST(ExpressionEvaluatorTest, EvaluatesMultiply)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 2.0}}};

    const auto result =
        evaluator.evaluate(factory.multiply(symbolic(factory, "q1"), factory.constant(3.0)));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 6.0);
}

TEST(ExpressionEvaluatorTest, EvaluatesDivide)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 2.0}}};

    const auto result =
        evaluator.evaluate(factory.divide(symbolic(factory, "q1"), factory.constant(4.0)));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 0.5);
}

TEST(ExpressionEvaluatorTest, EvaluatesNegate)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 2.0}}};

    const auto result = evaluator.evaluate(factory.negate(symbolic(factory, "q1")));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, -2.0);
}

TEST(ExpressionEvaluatorTest, EvaluatesSin)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", kPi / 2.0}}};

    const auto result = evaluator.evaluate(factory.sin(symbolic(factory, "q1")));

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 1.0, 1e-15);
}

TEST(ExpressionEvaluatorTest, EvaluatesCos)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.0}}};

    const auto result = evaluator.evaluate(factory.cos(symbolic(factory, "q1")));

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 1.0, 1e-15);
}

TEST(ExpressionEvaluatorTest, EvaluatesNestedExpression)
{
    // sin(q1)^2 + cos(q1)^2 == 1 for any q1 -- a nested expression whose
    // expected value does not depend on getting the arithmetic of a
    // particular constant right.
    const ExpressionFactory factory;
    const Expression q1 = symbolic(factory, "q1");
    const Expression sine = factory.sin(q1);
    const Expression cosine = factory.cos(q1);

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.7}}};

    const auto result = evaluator.evaluate(
        factory.add(factory.multiply(sine, sine), factory.multiply(cosine, cosine)));

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 1.0, 1e-15);
}

// --- errors and domain ----------------------------------------------

TEST(ExpressionEvaluatorTest, ReportsMissingSymbol)
{
    // Never defaults to zero: a silent zero would produce a plausible-looking
    // but wrong robot pose.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 1.0}}};

    const auto result =
        evaluator.evaluate(factory.add(symbolic(factory, "q1"), symbolic(factory, "q3")));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::MissingSymbol);
    EXPECT_EQ(result.error().symbolName, "q3");
}

TEST(ExpressionEvaluatorTest, RejectsNonFiniteSymbolValue)
{
    // All three non-finite values, not just NaN: the contract is isfinite,
    // and a regression to isnan would pass a NaN-only test.
    const ExpressionFactory factory;

    const double nonFinite[] = {kNaN, kInfinity, -kInfinity};

    for (const double value : nonFinite)
    {
        SCOPED_TRACE(testing::Message() << "binding = " << value);

        ExpressionEvaluator evaluator{SymbolValues{{"q1", value}}};
        const auto result = evaluator.evaluate(factory.sin(symbolic(factory, "q1")));

        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, EvaluationErrorCode::NonFiniteSymbolValue);
        EXPECT_EQ(result.error().symbolName, "q1");
    }
}

TEST(ExpressionEvaluatorTest, ReportsLeftOperandErrorFirst)
{
    // The visiting order is a publicly documented contract, so it needs a
    // test: without one the implementation could visit right-to-left and the
    // reported error would silently change.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{}};

    const auto result = evaluator.evaluate(
        factory.add(symbolic(factory, "missing_left"), symbolic(factory, "missing_right")));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::MissingSymbol);
    EXPECT_EQ(result.error().symbolName, "missing_left");
}

TEST(ExpressionEvaluatorTest, ReportsDivisionByZero)
{
    // The denominator has to be a symbol: ExpressionFactory::divide asserts
    // on a literal zero denominator, so that expression cannot be built.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.0}}};

    const auto result =
        evaluator.evaluate(factory.divide(factory.constant(1.0), symbolic(factory, "q1")));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::DivisionByZero);
}

TEST(ExpressionEvaluatorTest, ReportsDivisionByNegativeZero)
{
    // Negative zero can only enter through a binding: constant(-0.0) collapses
    // to the shared positive zero node, because -0.0 == 0.0.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", -0.0}}};

    const auto result =
        evaluator.evaluate(factory.divide(factory.constant(1.0), symbolic(factory, "q1")));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::DivisionByZero);
}

TEST(ExpressionEvaluatorTest, PreservesDomainWhenZeroIsRightOperand)
{
    // (1/q1) * 0 at q1 = 0 must report DivisionByZero, not yield 0. This is
    // the end-to-end check that dropping x * 0 -> 0 from the factory buys
    // something real.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.0}}};

    const Expression reciprocal =
        factory.divide(factory.constant(1.0), symbolic(factory, "q1"));
    const auto result =
        evaluator.evaluate(factory.multiply(reciprocal, factory.constant(0.0)));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::DivisionByZero);
}

TEST(ExpressionEvaluatorTest, PreservesDomainWhenZeroIsLeftOperand)
{
    // The mirror image. Not hypothetical: SymbolicMatrix::multiply folds
    // products left to right, so Constant(0) really does appear as the left
    // operand inside FK cells.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.0}}};

    const Expression reciprocal =
        factory.divide(factory.constant(1.0), symbolic(factory, "q1"));
    const auto result =
        evaluator.evaluate(factory.multiply(factory.constant(0.0), reciprocal));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::DivisionByZero);
}

TEST(ExpressionEvaluatorTest, ReportsNonFiniteResult)
{
    // Two symbols, not two constants: constant folding routes back through
    // ExpressionFactory::constant, whose isfinite assert would abort the
    // process while building 1e308 * 1e308.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 1e308}, {"q2", 1e308}}};

    const auto result = evaluator.evaluate(
        factory.multiply(symbolic(factory, "q1"), symbolic(factory, "q2")));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::NonFiniteResult);
}

// --- session and memoization ----------------------------------------

TEST(ExpressionEvaluatorTest, MemoizesSharedSubexpression)
{
    // add(x, x) over the SAME handle: Add, Cos and Symbol are each computed
    // once, and the second occurrence of x is a hit.
    const ExpressionFactory factory;
    const Expression x = factory.cos(symbolic(factory, "q1"));

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.5}}};
    ASSERT_TRUE(evaluator.evaluate(factory.add(x, x)).has_value());

    const EvaluationStatistics statistics = evaluator.statistics();
    EXPECT_EQ(statistics.cacheMisses, 3u);
    EXPECT_EQ(statistics.cacheHits, 1u);
}

TEST(ExpressionEvaluatorTest, DoesNotMergeStructurallyEqualDistinctNodes)
{
    // Memoization is by node identity, not by structural equality. Two
    // separately built but structurally identical trees stay separate
    // entries -- this is not hash-consing.
    const ExpressionFactory factory;
    const Expression lhs = factory.cos(symbolic(factory, "q1"));
    const Expression rhs = factory.cos(symbolic(factory, "q1"));

    ASSERT_FALSE(sameNode(lhs, rhs));
    ASSERT_TRUE(structurallyEqual(lhs, rhs));

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.5}}};
    ASSERT_TRUE(evaluator.evaluate(factory.add(lhs, rhs)).has_value());

    // Add + two Cos + two Symbol, all distinct nodes.
    const EvaluationStatistics statistics = evaluator.statistics();
    EXPECT_EQ(statistics.cacheMisses, 5u);
    EXPECT_EQ(statistics.cacheHits, 0u);
}

TEST(ExpressionEvaluatorTest, MemoizesSharedLeafAcrossDistinctParents)
{
    // The architecture states that Constant and Symbol nodes are cached too,
    // not only composite ones. Here one Symbol leaf is reached through two
    // different parents, so the hit can only come from caching the leaf.
    const ExpressionFactory factory;
    const Expression q1 = symbolic(factory, "q1");

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.5}}};
    ASSERT_TRUE(evaluator.evaluate(factory.add(factory.sin(q1), factory.cos(q1))).has_value());

    // Add, Sin, Symbol, Cos are misses; the second visit to Symbol is a hit.
    const EvaluationStatistics statistics = evaluator.statistics();
    EXPECT_EQ(statistics.cacheMisses, 4u);
    EXPECT_EQ(statistics.cacheHits, 1u);
}

TEST(ExpressionEvaluatorTest, SharesMemoizationAcrossMultipleRootExpressions)
{
    // The reason the evaluator is a session rather than a free function: the
    // sixteen cells of an FK matrix are separate roots over one shared DAG.
    const ExpressionFactory factory;
    const Expression shared = factory.cos(symbolic(factory, "q1"));

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.5}}};

    ASSERT_TRUE(evaluator.evaluate(factory.add(shared, factory.constant(1.0))).has_value());
    const EvaluationStatistics afterFirst = evaluator.statistics();
    EXPECT_EQ(afterFirst.cacheHits, 0u);

    ASSERT_TRUE(
        evaluator.evaluate(factory.multiply(shared, factory.constant(2.0))).has_value());
    const EvaluationStatistics afterSecond = evaluator.statistics();

    EXPECT_EQ(afterSecond.cacheHits, 1u);
    EXPECT_GT(afterSecond.cacheMisses, afterFirst.cacheMisses);
}

TEST(ExpressionEvaluatorTest, DifferentEvaluatorsUseDifferentSymbolValues)
{
    // The same expression handle under two sessions. If a cache ever leaked
    // between instances, this is what would catch it.
    const ExpressionFactory factory;
    const Expression expression = factory.multiply(symbolic(factory, "q1"), factory.constant(2.0));

    ExpressionEvaluator first{SymbolValues{{"q1", 1.0}}};
    ExpressionEvaluator second{SymbolValues{{"q1", 3.0}}};

    const auto firstResult = first.evaluate(expression);
    const auto secondResult = second.evaluate(expression);

    ASSERT_TRUE(firstResult.has_value());
    ASSERT_TRUE(secondResult.has_value());
    EXPECT_DOUBLE_EQ(*firstResult, 2.0);
    EXPECT_DOUBLE_EQ(*secondResult, 6.0);
}

TEST(ExpressionEvaluatorTest, IgnoresUnusedExtraBindings)
{
    const ExpressionFactory factory;
    const Expression expression = factory.add(symbolic(factory, "q1"), factory.constant(1.0));

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 2.0}, {"unused", 99.0}}};

    const auto result = evaluator.evaluate(expression);

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 3.0);
}

TEST(ExpressionEvaluatorTest, AcceptsNonFiniteBindingForUnusedSymbol)
{
    // Bindings are validated on read, not eagerly. An unused NaN cannot
    // affect any result, so "an unused binding does not change the outcome"
    // holds without exception.
    const ExpressionFactory factory;
    const Expression expression = factory.add(symbolic(factory, "q1"), factory.constant(1.0));

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 2.0}, {"unused", kNaN}}};

    const auto result = evaluator.evaluate(expression);

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 3.0);
}

// --- cache ownership ------------------------------------------------

TEST(ExpressionEvaluatorTest, KeepsEvaluatedNodesAliveAcrossTemporaries)
{
    // Every expression here is a temporary that dies at the end of its full
    // expression. Because the cache is keyed by Expression rather than by a
    // raw node pointer, each entry keeps its own node alive, so an address
    // cannot be recycled for a different node while it is still a key.
    //
    // This test can only fail when the allocator actually reuses an address,
    // which is implementation behaviour -- so a green run is not a proof.
    // The guarantee comes from the key type; this is a regression net.
    const ExpressionFactory factory;

    SymbolValues values;
    for (int i = 0; i < 64; ++i)
        values.emplace("q" + std::to_string(i), static_cast<double>(i));

    ExpressionEvaluator evaluator{std::move(values)};

    for (int i = 0; i < 64; ++i)
    {
        SCOPED_TRACE(testing::Message() << "symbol q" << i);
        const auto result = evaluator.evaluate(factory.symbol("q" + std::to_string(i)));
        ASSERT_TRUE(result.has_value());
        EXPECT_DOUBLE_EQ(*result, static_cast<double>(i));
    }
}
```

---

## 5. Uwagi do trzech nowych testów

### 5.1 `KeepsEvaluatedNodesAliveAcrossTemporaries` — czego nie dowodzi

Zgodnie z uwagą review §4, **nie należy raportować tego testu jako dowodu poprawnej własności cache'u.** Właściwe sformułowanie:

> Test wykrywa wcześniej zaobserwowany scenariusz ponownego użycia adresu; gwarancję własności daje typ klucza (`Expression`, nie `const ExpressionNode*`).

Nie próbuję wymuszać reuse setkami alokacji ani przeplatać ich zwalnianiem. Pętla po 64 symbolach jest tania i wystarcza, żeby scenariusz w ogóle mógł zajść; jej rozdmuchiwanie sugerowałoby, że test jest dowodem, którym nie jest.

### 5.2 `ReportsLeftOperandErrorFirst` — dlaczego jest potrzebny

Kontrakt „lewy przed prawym" jest **publicznie obserwowalny** przez treść błędu. Bez tego testu implementacja mogłaby odwiedzać operandy w odwrotnej kolejności, a dokumentacja nadal deklarowałaby determinizm, którego nic nie sprawdza. Oba symbole są nieznane, więc test rozróżnia wyłącznie kolejność.

### 5.3 `DoesNotMergeStructurallyEqualDistinctNodes` — co pinuje

Pozostałe testy memoizacji dowodzą, że **współdzielony uchwyt** trafia w cache. Żaden nie dowodzi, że dwa osobno zbudowane, strukturalnie identyczne drzewa **pozostają odrębne**. Ten test pinuje granicę:

```
memoizacja po tożsamości   ≠   strukturalny hash-consing
```

Gdyby ktoś kiedyś zamienił `sameNode` na `structurallyEqual` w `ExpressionIdentityEqual` — co wygląda jak „ulepszenie" — testy poprawnościowe nadal by przechodziły, a ten failuje. Zmiana byłaby przy okazji katastrofalna wydajnościowo: `structurallyEqual` jest O(rozmiar drzewa), więc każde porównanie w bucket'cie kosztowałoby przejście po poddrzewie.

---

## 6. CMake — po jednej linii

`CMakeLists.txt`:

```cmake
    src/ik_equations/symbolic/ExpressionFactory.cpp
    src/ik_equations/symbolic/ExpressionEvaluator.cpp
    src/ik_equations/symbolic/SymbolicTransform.cpp
)
```

`tests/CMakeLists.txt`:

```cmake
    test_expression_factory.cpp
    test_expression_evaluator.cpp
    test_symbolic_matrix.cpp
    test_symbolic_transform.cpp
)
```

---

## 7. `STATUS.md` — zmiany

1. **Nagłówek** — `161/161` → `186/186`.
2. **Diagram pipeline'u** — `ExpressionEvaluator` 🟡 → ✅; dopisać `numeric FK validation` ⬜ jako następną pozycję.
3. **Sekcja „Done"** — nowy wpis: sesja, memoizacja po tożsamości z kluczem `Expression`, cztery kody błędów, leniwa walidacja wiązań, zakaz skracania na podstawie wartości, kolejność `lhs` przed `rhs`.
4. **Known gaps — wiersz `No ExpressionEvaluator` znika**, ale **nie znika luka**. Zastępuje go wiersz o treści: evaluator istnieje, natomiast FK nadal **nie jest zweryfikowane numerycznie** — to krok 4. Do tego czasu zdanie „spójny błąd znaku przeszedłby każdy test" pozostaje prawdziwe.
5. **Known gaps — nowy wiersz** o monotonicznym wzroście cache'u i o tym, że evaluator jest sesją, nie globalnym interpreterem.
6. **Known gaps — nowy wiersz**: skalowanie głębokości drzewa z długością łańcucha jest **niezmierzone**; rekurencja jest bezpieczna dla 22–24, ale przed dopuszczeniem bardzo długich łańcuchów albo po zmianach w simplifierze trzeba to zmierzyć.
7. **Tabela dokumentów** — architektura `approved (v2)`, ten proposal `implemented`.
8. **Next step** — numeryczna walidacja FK dla KR4 i KR640, z własnym proposalem (wybór konfiguracji `q`, źródło macierzy odniesienia, uzasadnienie tolerancji wobec znanego szumu `1e-16`).

---

## 8. Ryzyka — co może wyjść dopiero przy budowaniu

### 8.1 `noexcept` na operacjach przenoszenia — **usunięte**

Ryzyko zniknęło wraz z korektą 4. Ręczne `noexcept` przy `= default` znaczyło „gwarantuję, że składowe przenoszą się bez wyjątku" — czego standard dla `std::unordered_map` nie obiecuje (specyfikacja jest warunkowa), więc rzucający move wywołałby `std::terminate`. Bez ręcznego `noexcept` kompilator wyprowadza właściwy warunek ze składowych, co jest **prawdziwe z definicji**.

To lepsze rozwiązanie niż moje pierwotne: zamiast dokumentować założenie, usuwamy potrzebę jego przyjmowania.

### 8.2 Liczby w testach statystyk — ryzyko średnie

`MemoizesSharedSubexpression` (3/1) i `DoesNotMergeStructurallyEqualDistinctNodes` (5/0) zakładają, że fabryka **nie zwinie** budowanych wyrażeń. Sprawdziłem reguły: `add(x, x)` nie ma reguły zwijającej, `cos(symbol)` też nie. Ale to jest wyprowadzenie z lektury, nie pomiar.

Jeżeli któraś liczba nie zgodzi się przy pierwszym uruchomieniu, **nie poprawiam jej po cichu** — raportuję rzeczywisty przebieg i wyjaśniam, skąd bierze się różnica, bo rozbieżność oznaczałaby, że nie rozumiem kształtu budowanego drzewa.

### 8.3 `std::visit` z `requires` na lambdzie generycznej — ryzyko niskie

Ten sam wzorzec działa już w `test_joint_transform_builder.cpp` i `test_forward_kinematics_builder.cpp` (`containsSymbol`). Tutaj dochodzi rekurencyjne wywołanie metody składowej z wnętrza lambdy przechwytującej `this` — nic nietypowego, ale to pierwsze takie miejsce w kodzie produkcyjnym.

---

## 9. Zgodność z zatwierdzoną architekturą

| Decyzja (architektura v2) | Gdzie w kodzie |
|---|---|
| evaluator jako sesja dla jednego podstawienia | konstruktor przyjmuje `SymbolValues`, brak przeciążenia statycznego |
| kopiowanie zabronione, przenoszenie dozwolone | `= delete` / `noexcept = default` |
| cache kluczowany przez `Expression` | `Memo` z `ExpressionIdentityHash` / `ExpressionIdentityEqual` |
| hash/equality po tożsamości węzła | `&expression.node()` oraz `sameNode` |
| pełne definicje typów przed `memo_` | zagnieżdżone struktury zdefiniowane w całości |
| cache przechowuje też `Constant` i `Symbol` | `memo_.emplace` na końcu `evaluateNode`, bez rozróżniania typu |
| błędy nie są cache'owane | `memo_` przechowuje `double`; `emplace` po sprawdzeniu `result` |
| `std::expected`, cztery kody | `EvaluationErrorCode` |
| leniwa walidacja wiązań | `isfinite` przy odczycie symbolu, nie w konstruktorze |
| dokładne `denominator == 0.0` | gałąź `DivideNode`, bez tolerancji |
| `isfinite` po każdym węźle | jedno sprawdzenie po `std::visit` |
| brak skracania na podstawie wartości | oba operandy zawsze odwiedzane, brak testu na zero |
| kolejność `lhs` przed `rhs` | gałąź binarna; pinowane `ReportsLeftOperandErrorFirst` |
| statystyki `cacheMisses` / `cacheHits` | inkrementy przy sprawdzeniu cache'u |
| kontrakt czasu życia cache'u | komentarz klasy |
| brak zależności od `SymbolicTransform` | nagłówek włącza tylko `Expression.hpp` |
| rekurencja | `evaluateNode` woła siebie |

---

## 10. Jak to zweryfikuję po zatwierdzeniu

```powershell
cmake -B build -G Ninja
cmake --build build --clean-first
ctest --test-dir build --output-on-failure
.\build\tests\kinemaforge_tests.exe --gtest_filter=ExpressionEvaluatorTest.*
```

Oczekiwane: **161 + 26 = 187 zielonych.** Liczba policzona z treści tego dokumentu (10 pokrycie węzłów + 8 błędy i dziedzina + 7 sesja i memoizacja + 1 własność cache'u), nie oszacowana — po korekcie rachunkowej z drugiej rundy i dodaniu testu liścia z trzeciej.

Zamierzam dodatkowo sprawdzić statystyki na **rzeczywistym drzewie FK** dla `kr640.urdf`: liczba missów powinna odpowiadać zmierzonym 281 unikalnym węzłom, a liczba hitów — różnicy wobec 21 882 odwiedzin. To niezależne potwierdzenie, że memoizacja działa na danych, a nie tylko na trzyweęzłowym przykładzie. Wynik zaraportuję niezależnie od tego, czy się zgodzi.

## Do zatwierdzenia

Kod produkcyjny (§2, §3) i testowy (§4) po korektach z obu rund review — łącznie **26 nowych testów, 187 oczekiwanych**.
