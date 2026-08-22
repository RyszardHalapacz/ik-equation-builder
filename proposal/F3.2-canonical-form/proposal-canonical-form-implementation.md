# Proposal: postać AC-normalna — F3.2, implementacja

**Etap:** F3.2 · **v1 — CZEKA NA REVIEW** · 2026-08-22 · część 2 z 2

> **Stan tego dokumentu:** wersja **v1**, oddana do review, werdyktu jeszcze nie
> ma. Kod w repo kończy się na F3.1. Następny krok to decyzja recenzenta,
> nie pisanie do `src/`.

> **Nota o procesie, którą jestem winien.** Kod z tego dokumentu **został
> skompilowany i uruchomiony** — 25 testów, wszystkie zielone — zanim dokument
> powstał. To było **złamanie ustalonego procesu**: proposal ma powstać przed
> kodem, a weryfikacja kompilacją należy do etapu po zatwierdzeniu. Zgłaszam to
> wprost zamiast przemilczeć, bo liczby i wyniki poniżej są prawdziwe i
> użyteczne przy review, ale kolejność była zła.
>
> Praktyczna konsekwencja dla ciebie: **liczby w 7 i gate w 6 nie są
> przewidywaniem** — pochodzą z uruchomienia. Jedno miejsce jest jawnie
> niezmierzone i tak opisane (7.2).

---

## Prompt

> pisz proposal implementacyjny

Architektura **v8 zatwierdzona**. Ten dokument nie zmienia żadnego jej
rozstrzygnięcia; pokazuje kod, który je realizuje, i rozstrzyga to, co
architektura zostawiła implementacji.

Wchodzą tu również **dwie uwagi nieblokujące z approvala**:

1. **jawny kontrakt platformowy** — kod z 3.2.1 architektury zakłada IEEE-754
   binary64, a to założenie ma być w kodzie, nie w głowie (2.1);
2. **oracle-testy predykatów dokładności** — z werdyktami policzonymi niezależnie
   (5.4).

---

## 0. Zakres i stan wyjściowy

```text
NOWE   src/ik_equations/symbolic/ExpressionCanonicalizer.hpp   porządek, sesja, predykaty
       src/ik_equations/symbolic/ExpressionCanonicalizer.cpp   przebieg kanonizujący
       src/ik_equations/builders/EquationSimplifier.hpp        cienki orkiestrator
       src/ik_equations/builders/EquationSimplifier.cpp
       tests/test_expression_canonicalizer.cpp                 25 testów

ZMIENIONE  CMakeLists.txt          +2 wiersze
           tests/CMakeLists.txt    +1 wiersz

BEZ ZMIAN  Expression, ExpressionFactory, ExpressionEvaluator, RewriteEngine,
           SymbolicTransform, cały model, ConstraintBuilder, fasada, loader
```

Ostatni wiersz jest sprawdzianem, tak jak w F3.1, i **wychodzi**. W szczególności
`ExpressionFactory::divide` zachowuje `assert(!isZero(rhs))`, a `RewriteEngine`
nie jest wołany — F3.2 nie dostarcza silnikowi żadnej reguły (architektura 8.1).

Stan wyjściowy: `450/450` w obu konfiguracjach, gałąź `f2.3r-review-remediation`.

---

## 1. `ExpressionCanonicalizer.hpp`

```cpp
#pragma once

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/RewriteEngine.hpp"

#include <cstddef>
#include <vector>

namespace kinemaforge::ik {

namespace detail {

// The two load-bearing predicates of constant reduction, exposed because they
// are worth testing against an oracle rather than only through their effect.
//
// Both are SOUND AND COMPLETE: true exactly when the mathematical result is a
// representable finite binary64. Soundness keeps the canonicaliser from
// introducing rounding; completeness keeps it from splitting one
// AC-equivalence class into two normal forms, because ExpressionFactory folds
// adjacent constants greedily while building.
[[nodiscard]] bool addsExactly(double lhs, double rhs) noexcept;
[[nodiscard]] bool multipliesExactly(double lhs, double rhs) noexcept;

} // namespace detail

// A total, deterministic, structural order on expressions.
//
// Deliberately NOT derived from node addresses: those depend on the allocator,
// so the same equations would order differently in two processes and the
// canonical form would stop being canonical. Comparison is by node kind first,
// in the fixed ranking
//
//     Constant < Symbol < Sin < Cos < Negate < Multiply < Divide < Add < Subtract
//
// then structurally within a kind. Consistent with structurallyEqual: two
// expressions compare equal exactly when they are structurally equal.
//
// Returns a negative value, zero, or a positive value, like a three-way
// comparison; std::strong_ordering is not used because the result is also
// consumed as a bool by sort predicates in the implementation.
[[nodiscard]] int compareExpressions(const Expression& lhs, const Expression& rhs);

// Rewrites an expression into AC-NORMAL FORM: unique with respect to the
// associativity and commutativity of Add and Multiply, to the reduction of
// constants, and to the normalisation of subtraction and sign. NOT unique with
// respect to anything else -- distributivity is not applied, like terms are not
// collected, and no trigonometric identity is used, so a*(b+c) and a*b + a*c
// remain two different expressions.
//
// That limit matters downstream: after this stage structurallyEqual still does
// NOT mean "algebraically equal".
//
// Guarantees:
//
//   * the output contains no Subtract node, and Negate appears only as the sign
//     of a single summand, or as one sign outside a product or a quotient --
//     never directly above a sum;
//   * the symbolic domain over R is preserved exactly: no constant is joined
//     unless the join is exact, so the canonicaliser introduces no rounding of
//     its own;
//   * an expression already in normal form is returned as THE SAME NODE, even
//     by a session that has never seen it.
//
// A SESSION, not a free function, for the reason RewriteSession is one: an
// equation system is many roots of one shared DAG, and rebuilding a shared
// subtree once per root would break that sharing before every later stage pays
// for it again. One session per system, not per expression.
//
// The memo grows monotonically and keeps its nodes alive; the session is meant
// to be short-lived. Not thread-safe.
class CanonicalizationSession
{
public:
    explicit CanonicalizationSession(ExpressionFactory factory);

    CanonicalizationSession(const CanonicalizationSession&) = delete;
    CanonicalizationSession& operator=(const CanonicalizationSession&) = delete;
    CanonicalizationSession(CanonicalizationSession&&) = default;
    CanonicalizationSession& operator=(CanonicalizationSession&&) = default;

    [[nodiscard]] Expression canonicalize(const Expression& expression);

private:
    // Sign application that respects the convention rather than wrapping a
    // node: a sum takes the sign into its summands, a product or quotient
    // flips its parity, a constant changes value, and a negation cancels.
    // Wrapping with factory.negate() instead would put Negate above a sum and
    // break idempotence -- (-1) * (a+b) is the case that shows it.
    [[nodiscard]] Expression canonicalNegate(const Expression& expression);

    [[nodiscard]] Expression canonicalizeNode(const Expression& expression);

    void collectSum(const Expression& expression, int sign, std::vector<Expression>& terms);
    void collectProduct(const Expression& expression, int& parity,
                        std::vector<Expression>& factors);

    [[nodiscard]] Expression buildSum(std::vector<Expression> terms);
    [[nodiscard]] Expression buildProduct(std::vector<Expression> factors, int parity);
    [[nodiscard]] Expression canonicalizeQuotient(const Expression& expression);

    ExpressionFactory factory_;
    detail::ExpressionMap<Expression> memo_;
};

// One-shot convenience over the session, for callers with a single expression
// and for tests. A caller with several roots of one DAG wants the session.
[[nodiscard]] Expression canonicalize(const Expression& expression,
                                      const ExpressionFactory& factory);

} // namespace kinemaforge::ik
```

### 1.1 Dwie decyzje warte nazwania

**`detail::ExpressionMap` przychodzi z `RewriteEngine.hpp`**, a nie powstaje po
raz drugi. To trzeci użytkownik tej pary funktorów w projekcie i drugi w
`namespace detail` — duplikat wobec `ExpressionEvaluator` jest już zapisany
w `STATUS.md` jako znany brak i **nie rośnie** przez ten etap.

**Predykaty są w API, nie w anonimowej przestrzeni.** Architektura ich tam nie
wymagała; wystawiam je, bo approval prosił o oracle-testy, a testowanie ich
wyłącznie przez skutek („czy wyrażenie się złożyło") badałoby kanonizator zamiast
predykatu. Są w `detail`, więc nie stają się słownikiem modułu.

---

## 2. `ExpressionCanonicalizer.cpp` — kontrakt platformy i predykaty

```cpp
#include "ik_equations/symbolic/ExpressionCanonicalizer.hpp"

#include <algorithm>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>
#include <utility>
#include <variant>

namespace kinemaforge::ik {

namespace {

// The exactness tests below are statements about IEEE-754 binary64, not about
// "double" in the abstract. If a toolchain offers something else, they are not
// merely slower -- they are wrong, and silently.
static_assert(std::numeric_limits<double>::is_iec559,
              "the exactness predicates assume IEEE-754");
static_assert(std::numeric_limits<double>::digits == 53,
              "the exactness predicates assume a 53-bit significand");
static_assert(std::numeric_limits<double>::max_exponent == 1024,
              "the exactness predicates assume binary64 exponent bounds");
static_assert(std::numeric_limits<double>::min_exponent == -1021,
              "the exactness predicates assume binary64 exponent bounds");
static_assert(std::numeric_limits<double>::has_denorm == std::denorm_present,
              "2Sum assumes gradual underflow; flush-to-zero breaks it");

int kindRank(ExpressionType type) noexcept
{
    switch (type)
    {
    case ExpressionType::Constant: return 0;
    case ExpressionType::Symbol:   return 1;
    case ExpressionType::Sin:      return 2;
    case ExpressionType::Cos:      return 3;
    case ExpressionType::Negate:   return 4;
    case ExpressionType::Multiply: return 5;
    case ExpressionType::Divide:   return 6;
    case ExpressionType::Add:      return 7;
    case ExpressionType::Subtract: return 8;
    }
    return 9;
}

// Exact rounding error of a + b. Zero exactly when the sum was exact.
//
// Sound AND complete for every finite pair that does not overflow, subnormals
// included: a sum whose result is subnormal is exact under gradual underflow,
// so the error term can never underflow away. Overflow is the only breakage,
// and ExpressionFactory declines to fold a non-finite result anyway.
double additionError(double a, double b) noexcept
{
    const double sum = a + b;
    const double shifted = sum - a;
    return (a - (sum - shifted)) + (b - shifted);
}

// True exactly when the mathematical product of a and b is a representable
// finite binary64 -- SOUND AND COMPLETE, both halves required.
//
// A false positive would let the canonicaliser introduce rounding and move the
// zero set of a denominator. A false negative is just as damaging for a
// different reason: ExpressionFactory folds two adjacent constants greedily at
// build time, so refusing a fold the factory would have performed splits one
// AC-equivalence class into two normal forms -- (c1*c2)*x against c1*(c2*x).
//
// fma(a, b, -a*b) is not a witness on its own: for a = b = nextafter(2^-511)
// the product is normal, yet its rounding error is 2^-1126 and underflows to
// zero, so the residual reports "exact" for a rounded product. The test
// therefore works from the representation. The 106-bit significand product is
// assembled from 32-bit limbs rather than __int128, which keeps this portable.
bool multipliesExactlyImpl(double a, double b) noexcept
{
    const double product = a * b;

    if (!std::isfinite(product))
        return false;
    if (a == 0.0 || b == 0.0)
        return true;

    int exponentA = 0;
    int exponentB = 0;
    const double fractionA = std::frexp(std::fabs(a), &exponentA);
    const double fractionB = std::frexp(std::fabs(b), &exponentB);

    // |x| = significand * 2^(exponent - 53), significand a 53-bit integer.
    const auto significandA = static_cast<std::uint64_t>(std::ldexp(fractionA, 53));
    const auto significandB = static_cast<std::uint64_t>(std::ldexp(fractionB, 53));

    const std::uint64_t lowA = significandA & 0xFFFFFFFFu;
    const std::uint64_t highA = significandA >> 32;
    const std::uint64_t lowB = significandB & 0xFFFFFFFFu;
    const std::uint64_t highB = significandB >> 32;

    // Both halves are below 2^53, so the cross term stays below 2^54 and cannot
    // overflow; the full product needs at most 106 bits.
    const std::uint64_t cross = lowA * highB + highA * lowB;
    std::uint64_t high = highA * highB + (cross >> 32);
    const std::uint64_t crossLow = cross << 32;
    std::uint64_t low = lowA * lowB + crossLow;
    if (low < crossLow)
        ++high;

    const int trailingZeros = low != 0 ? std::countr_zero(low) : 64 + std::countr_zero(high);
    const int bitWidth = high != 0 ? 64 + std::bit_width(high) : std::bit_width(low);

    const int significantBits = bitWidth - trailingZeros;
    const int lowestExponent = exponentA + exponentB - 106 + trailingZeros;

    if (significantBits > 53)
        return false;                                   // needs more than 53 bits
    if (lowestExponent < -1074)
        return false;                                   // bits below the smallest subnormal
    if (lowestExponent + significantBits - 1 > 1023)
        return false;                                   // overflow

    return true;
}

bool isNegativeConstant(const Expression& expression) noexcept
{
    return isConstant(expression) && constantValue(expression) < 0.0;
}

} // namespace

namespace detail {

bool addsExactly(double lhs, double rhs) noexcept
{
    const double sum = lhs + rhs;
    return std::isfinite(sum) && additionError(lhs, rhs) == 0.0;
}

bool multipliesExactly(double lhs, double rhs) noexcept
{
    return multipliesExactlyImpl(lhs, rhs);
}

} // namespace detail
```

### 2.1 Kontrakt platformowy — uwaga z approvala

Pięć `static_assert`ów, nie komentarz. Cztery pierwsze pilnują tego, co
predykaty **liczą** (53 bity, granice wykładnika); piąty pilnuje tego, czego
**nie widać w kodzie**: 2Sum jest kompletne tylko przy stopniowym niedomiarze.
Przy FTZ/DAZ suma o wyniku subnormalnym przestaje być dokładna, więc predykat
zacząłby ogłaszać dokładnymi połączenia, które nimi nie są — cicho i tylko dla
bardzo małych stałych.

`has_denorm` łapie to na poziomie typu, ale **nie łapie trybu procesora**:
FTZ/DAZ da się włączyć w runtime (`_MM_SET_FLUSH_ZERO_MODE`, `-ffast-math`
w konsumencie). Zapisuję to jako **znany brak do `STATUS.md`**, a nie udaję, że
`static_assert` to zamyka.

---

## 3. `ExpressionCanonicalizer.cpp` — porządek i przebieg

```cpp
int compareExpressions(const Expression& lhs, const Expression& rhs)
{
    if (sameNode(lhs, rhs))
        return 0;

    const int rankLhs = kindRank(lhs.type());
    const int rankRhs = kindRank(rhs.type());
    if (rankLhs != rankRhs)
        return rankLhs < rankRhs ? -1 : 1;

    return std::visit(
        [&rhs](const auto& node) -> int {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, ConstantNode>)
            {
                const double other = std::get<ConstantNode>(rhs.node().value).value;
                if (node.value == other) return 0;
                return node.value < other ? -1 : 1;
            }
            else if constexpr (std::is_same_v<Node, SymbolNode>)
            {
                const std::string& other = std::get<SymbolNode>(rhs.node().value).name;
                if (node.name == other) return 0;
                return node.name < other ? -1 : 1;
            }
            else if constexpr (requires { node.lhs; node.rhs; })
            {
                const auto& other = std::get<Node>(rhs.node().value);
                if (const int left = compareExpressions(node.lhs, other.lhs); left != 0)
                    return left;
                return compareExpressions(node.rhs, other.rhs);
            }
            else
            {
                const auto& other = std::get<Node>(rhs.node().value);
                return compareExpressions(node.operand, other.operand);
            }
        },
        lhs.node().value);
}

CanonicalizationSession::CanonicalizationSession(ExpressionFactory factory)
    : factory_(std::move(factory))
{
}

Expression CanonicalizationSession::canonicalize(const Expression& expression)
{
    return canonicalizeNode(expression);
}

Expression CanonicalizationSession::canonicalNegate(const Expression& expression)
{
    switch (expression.type())
    {
    case ExpressionType::Constant:
        return factory_.constant(-constantValue(expression));

    case ExpressionType::Negate:
        return std::get<NegateNode>(expression.node().value).operand;

    case ExpressionType::Add:
    {
        // The sign enters the summands. Wrapping the sum would put Negate above
        // an Add, which the postcondition forbids and which would make a second
        // pass produce a different result.
        std::vector<Expression> terms;
        collectSum(expression, -1, terms);
        return buildSum(std::move(terms));
    }

    case ExpressionType::Multiply:
    case ExpressionType::Divide:
    case ExpressionType::Symbol:
    case ExpressionType::Sin:
    case ExpressionType::Cos:
    case ExpressionType::Subtract:
        break;
    }

    return factory_.negate(expression);
}

void CanonicalizationSession::collectSum(const Expression& expression, int sign,
                                         std::vector<Expression>& terms)
{
    switch (expression.type())
    {
    case ExpressionType::Add:
    {
        const auto& node = std::get<AddNode>(expression.node().value);
        collectSum(node.lhs, sign, terms);
        collectSum(node.rhs, sign, terms);
        return;
    }
    case ExpressionType::Subtract:
    {
        const auto& node = std::get<SubtractNode>(expression.node().value);
        collectSum(node.lhs, sign, terms);
        collectSum(node.rhs, -sign, terms);
        return;
    }
    case ExpressionType::Negate:
    {
        const auto& node = std::get<NegateNode>(expression.node().value);
        collectSum(node.operand, -sign, terms);
        return;
    }
    default:
        break;
    }

    const Expression term = canonicalizeNode(expression);

    // Canonicalising a summand can REVEAL a sum that was not visible before --
    // a + -(-(b+c)) is the case. Splicing rather than appending is what keeps
    // one pass enough; it terminates because the spliced expression is already
    // canonical, so its own summands come back unchanged.
    if (term.type() == ExpressionType::Add || term.type() == ExpressionType::Subtract
        || term.type() == ExpressionType::Negate)
    {
        collectSum(term, sign, terms);
        return;
    }

    terms.push_back(sign < 0 ? canonicalNegate(term) : term);
}

void CanonicalizationSession::collectProduct(const Expression& expression, int& parity,
                                             std::vector<Expression>& factors)
{
    switch (expression.type())
    {
    case ExpressionType::Multiply:
    {
        const auto& node = std::get<MultiplyNode>(expression.node().value);
        collectProduct(node.lhs, parity, factors);
        collectProduct(node.rhs, parity, factors);
        return;
    }
    case ExpressionType::Negate:
    {
        const auto& node = std::get<NegateNode>(expression.node().value);
        parity ^= 1;
        collectProduct(node.operand, parity, factors);
        return;
    }
    case ExpressionType::Constant:
    {
        // The second source of sign: the factory builds (-2) * x as
        // Multiply(Constant{-2}, x), never as Multiply(Negate(2), x), so a
        // parity counted from Negate nodes alone would not see it.
        //
        // The test is c < 0, so -0.0 does not flip the parity: -0.0 < 0.0 is
        // false in IEEE-754, and -0.0 is the same value as 0.0.
        const double value = constantValue(expression);
        if (value < 0.0)
        {
            parity ^= 1;
            factors.push_back(factory_.constant(-value));
            return;
        }
        factors.push_back(expression);
        return;
    }
    default:
        break;
    }

    const Expression factor = canonicalizeNode(expression);

    if (factor.type() == ExpressionType::Multiply || factor.type() == ExpressionType::Negate
        || factor.type() == ExpressionType::Constant)
    {
        collectProduct(factor, parity, factors);
        return;
    }

    factors.push_back(factor);
}
```

### 3.1 Dlaczego splice się kończy

To jedyne miejsce w tym pliku, w którym da się napisać pętlę bez końca, więc
argument należy do dokumentu, nie tylko do komentarza.

```text
splice woła collectSum na wyrażeniu JUŻ KANONICZNYM,
którego korzeń należy do zbioru zejścia {Add, Subtract, Negate}
    -> trafia w jedną z trzech pierwszych gałęzi
    -> schodzi STRUKTURALNIE NIŻEJ
    -> jego dzieci są kanoniczne, więc canonicalizeNode zwraca je bez zmian (5.6 architektury)
    -> trafiają do push
```

Terminacja **opiera się na wymogu tożsamości węzła**. Gdyby `canonicalizeNode`
budowało nowy, strukturalnie identyczny węzeł zamiast oddawać oryginał, splice
mógłby wołać sam siebie w nieskończoność. Te dwie decyzje trzymają się nawzajem
i warto o tym wiedzieć, zanim ktoś „uprości" którąś z nich.

---

## 4. `ExpressionCanonicalizer.cpp` — budowanie i wejście

```cpp
Expression CanonicalizationSession::buildSum(std::vector<Expression> terms)
{
    std::vector<double> constants;
    std::vector<Expression> others;
    others.reserve(terms.size());

    for (const Expression& term : terms)
    {
        if (isConstant(term))
            constants.push_back(constantValue(term));
        else
            others.push_back(term);
    }

    // Order first, then reduce, and repeat until nothing more joins. Reducing in
    // arrival order is not deterministic (double addition is not associative),
    // and a single ordered pass is deterministic but not idempotent: the
    // constants 2^52, 2^52+1, 2^52+3 leave two residuals that a second pass
    // would join. Terminates because every join removes one constant.
    bool joined = true;
    while (joined && constants.size() > 1)
    {
        joined = false;
        std::sort(constants.begin(), constants.end());

        std::vector<double> reduced;
        reduced.push_back(constants.front());
        for (std::size_t index = 1; index < constants.size(); ++index)
        {
            if (detail::addsExactly(reduced.back(), constants[index]))
            {
                reduced.back() += constants[index];
                joined = true;
                continue;
            }
            reduced.push_back(constants[index]);
        }
        constants = std::move(reduced);
    }

    std::erase_if(constants, [](double value) { return value == 0.0; });

    std::sort(others.begin(), others.end(), [](const Expression& lhs, const Expression& rhs) {
        return compareExpressions(lhs, rhs) < 0;
    });

    if (others.empty())
    {
        if (constants.empty())
            return factory_.constant(0.0);

        // Two residual constants with no other summand would have to become
        // operands of one add(), which the factory would then fold -- possibly
        // inexactly, defeating the whole point. It cannot arise: a chain of
        // nothing but constants would have been folded by the factory while it
        // was built, and this stage never turns a non-constant into a constant.
        assert(constants.size() == 1 && "an all-constant sum should not survive the factory");
        return factory_.constant(constants.front());
    }

    // Balanced over the non-constant summands; constants are attached OUTSIDE,
    // one at a time, so no add() ever receives two constants.
    const auto balanced = [this](auto&& self, std::span<const Expression> range) -> Expression {
        if (range.size() == 1)
            return range.front();
        const std::size_t middle = range.size() / 2;
        return factory_.add(self(self, range.first(middle)),
                            self(self, range.subspan(middle)));
    };

    Expression result = balanced(balanced, std::span<const Expression>(others));
    for (auto constant = constants.rbegin(); constant != constants.rend(); ++constant)
        result = factory_.add(factory_.constant(*constant), result);

    return result;
}

Expression CanonicalizationSession::buildProduct(std::vector<Expression> factors, int parity)
{
    std::vector<double> constants;
    std::vector<Expression> others;
    others.reserve(factors.size());

    for (const Expression& factor : factors)
    {
        if (isConstant(factor))
            constants.push_back(constantValue(factor));
        else
            others.push_back(factor);
    }

    bool joined = true;
    while (joined && constants.size() > 1)
    {
        joined = false;
        std::sort(constants.begin(), constants.end());

        std::vector<double> reduced;
        reduced.push_back(constants.front());
        for (std::size_t index = 1; index < constants.size(); ++index)
        {
            if (detail::multipliesExactly(reduced.back(), constants[index]))
            {
                reduced.back() *= constants[index];
                joined = true;
                continue;
            }
            reduced.push_back(constants[index]);
        }
        constants = std::move(reduced);
    }

    std::erase_if(constants, [](double value) { return value == 1.0; });

    const bool hasZero = std::any_of(constants.begin(), constants.end(),
                                     [](double value) { return value == 0.0; });

    std::sort(others.begin(), others.end(), [](const Expression& lhs, const Expression& rhs) {
        return compareExpressions(lhs, rhs) < 0;
    });

    Expression result = factory_.constant(1.0);

    // A zero factor with nothing symbolic beside it collapses; with symbolic
    // factors it does NOT -- x * 0 keeps its domain information, which is the
    // rule ExpressionFactory has refused to break since Phase 1.
    if (hasZero && others.empty())
    {
        result = factory_.constant(0.0);
        constants.clear();
    }

    if (!others.empty())
    {
        const auto balanced = [this](auto&& self, std::span<const Expression> range) -> Expression {
            if (range.size() == 1)
                return range.front();
            const std::size_t middle = range.size() / 2;
            return factory_.multiply(self(self, range.first(middle)),
                                     self(self, range.subspan(middle)));
        };
        result = balanced(balanced, std::span<const Expression>(others));

        for (auto constant = constants.rbegin(); constant != constants.rend(); ++constant)
            result = factory_.multiply(factory_.constant(*constant), result);
    }
    else if (!constants.empty())
    {
        assert(constants.size() == 1 && "an all-constant product should not survive the factory");
        result = factory_.constant(constants.front());
    }

    return parity != 0 ? canonicalNegate(result) : result;
}

Expression CanonicalizationSession::canonicalizeQuotient(const Expression& expression)
{
    const auto& node = std::get<DivideNode>(expression.node().value);

    int parity = 0;
    Expression numerator = canonicalizeNode(node.lhs);
    Expression denominator = canonicalizeNode(node.rhs);

    if (numerator.type() == ExpressionType::Negate)
    {
        parity ^= 1;
        numerator = std::get<NegateNode>(numerator.node().value).operand;
    }
    else if (isNegativeConstant(numerator))
    {
        parity ^= 1;
        numerator = factory_.constant(-constantValue(numerator));
    }

    if (denominator.type() == ExpressionType::Negate)
    {
        parity ^= 1;
        denominator = std::get<NegateNode>(denominator.node().value).operand;
    }
    else if (isNegativeConstant(denominator))
    {
        parity ^= 1;
        denominator = factory_.constant(-constantValue(denominator));
    }

    // Representability guard, the same one F3.1 relies on: a denominator that
    // canonicalised to a literal zero cannot be built, and ExpressionFactory's
    // contract is not being loosened for this stage. Keep the original.
    if (isZero(denominator))
        denominator = node.rhs;

    const Expression quotient = factory_.divide(numerator, denominator);
    return parity != 0 ? canonicalNegate(quotient) : quotient;
}

Expression CanonicalizationSession::canonicalizeNode(const Expression& expression)
{
    if (const auto cached = memo_.find(expression); cached != memo_.end())
        return cached->second;

    Expression result = expression;

    switch (expression.type())
    {
    case ExpressionType::Constant:
    case ExpressionType::Symbol:
        break;

    case ExpressionType::Add:
    case ExpressionType::Subtract:
    {
        std::vector<Expression> terms;
        collectSum(expression, 1, terms);
        result = buildSum(std::move(terms));
        break;
    }

    case ExpressionType::Multiply:
    {
        std::vector<Expression> factors;
        int parity = 0;
        collectProduct(expression, parity, factors);
        result = buildProduct(std::move(factors), parity);
        break;
    }

    case ExpressionType::Divide:
        result = canonicalizeQuotient(expression);
        break;

    case ExpressionType::Negate:
        result = canonicalNegate(
            canonicalizeNode(std::get<NegateNode>(expression.node().value).operand));
        break;

    case ExpressionType::Sin:
    {
        const Expression operand =
            canonicalizeNode(std::get<SinNode>(expression.node().value).operand);
        result = factory_.sin(operand);
        break;
    }

    case ExpressionType::Cos:
    {
        const Expression operand =
            canonicalizeNode(std::get<CosNode>(expression.node().value).operand);
        result = factory_.cos(operand);
        break;
    }
    }

    // Node identity: an expression already in normal form comes back as the
    // same node, even from a session with an empty memo. Without this every
    // pass would clone a structurally identical DAG, the sharing this session
    // exists to keep would be lost on the first pass that changes nothing, and
    // the splice in collectSum would lose its termination argument.
    if (!sameNode(result, expression) && structurallyEqual(result, expression))
        result = expression;

    memo_.emplace(expression, result);
    return result;
}

Expression canonicalize(const Expression& expression, const ExpressionFactory& factory)
{
    CanonicalizationSession session{factory};
    return session.canonicalize(expression);
}

} // namespace kinemaforge::ik
```

### 4.1 Jedno miejsce, w którym implementacja mówi więcej niż architektura

`buildProduct` musi rozstrzygnąć przypadek, o którym architektura milczała:
**czynnik zerowy**. Zwinięcie `x * 0` do `0` jest tym samym zakazem, który
`ExpressionFactory` trzyma od Fazy 1 i którego F3.1 §2.6 dotyka jako warunku
lokalnego. Kanonizator **nie ma prawa** go złamać, więc zero zwija się wyłącznie
wtedy, gdy w łańcuchu nie ma żadnego czynnika symbolicznego.

To jest asymetryczne wobec sumy, gdzie `0` jest po prostu odrzucane jako
neutralne — i słusznie: `x + 0 = x` na całej dziedzinie `x`, a `x * 0 = 0` tylko
tam, gdzie `x` jest określone.

---

## 5. `EquationSimplifier`

```cpp
#pragma once

#include "ik_equations/model/IkEquationSystem.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <expected>

namespace kinemaforge::ik {

// Normalises every expression of an equation system, and nothing else.
//
// Thin on purpose. Its whole reason to exist at this stage is that the twelve
// equations of a system are many roots of ONE shared forward-kinematics DAG:
// it runs them all through a single CanonicalizationSession, so a subtree the
// equations share stays one node instead of becoming twelve structurally equal
// copies. That property cannot be tested on a synthetic tree; it needs a real
// system, which is why this class ships with the canonicaliser rather than
// with the first stage that has something to simplify.
//
// ALL NON-EXPRESSION FIELDS ARE PRESERVED EXACTLY -- not the list of fields
// that exist today (unknowns and their order, equation order, task kind, each
// equation's kind and source) but the rule itself, so a field added to
// Equation next year does not quietly fall out of it.
//
// Returns expected because IkEquationSystem::create is the only way to build a
// system and it validates nine invariants. Normalising expressions cannot break
// any of them, so the error path is unreachable in practice -- but swallowing
// it with an assert would be a guarantee this class cannot make about a type it
// does not own.
class EquationSimplifier
{
public:
    explicit EquationSimplifier(ExpressionFactory factory);

    [[nodiscard]] std::expected<IkEquationSystem, IkEquationSystemError>
    normalize(const IkEquationSystem& system) const;

private:
    ExpressionFactory factory_;
};

} // namespace kinemaforge::ik
```

```cpp
#include "ik_equations/builders/EquationSimplifier.hpp"

#include "ik_equations/symbolic/ExpressionCanonicalizer.hpp"

#include <utility>
#include <vector>

namespace kinemaforge::ik {

EquationSimplifier::EquationSimplifier(ExpressionFactory factory)
    : factory_(std::move(factory))
{
}

std::expected<IkEquationSystem, IkEquationSystemError>
EquationSimplifier::normalize(const IkEquationSystem& system) const
{
    // ONE session for the whole system. A session per equation, or per side,
    // would rebuild the shared forward-kinematics subtrees once per root and
    // break the sharing before any later stage could benefit from it.
    CanonicalizationSession session{factory_};

    std::vector<Equation> equations;
    equations.reserve(system.equations().size());

    for (const Equation& equation : system.equations())
        equations.push_back(Equation{session.canonicalize(equation.lhs),
                                     session.canonicalize(equation.rhs),
                                     equation.source});

    const std::span<const JointVariable> unknowns = system.unknowns();
    return IkEquationSystem::create(system.taskKind(),
                                    std::vector<JointVariable>(unknowns.begin(), unknowns.end()),
                                    std::move(equations));
}

} // namespace kinemaforge::ik
```

### 5.1 Odstępstwo od architektury, świadome

Architektura pisała `IkEquationSystem normalize(...)`, zwracane przez wartość.
Zwracam `expected`, bo `IkEquationSystem::create` jest **jedynym** sposobem
zbudowania układu i waliduje dziewięć inwariantów. Kanonizacja wyrażeń nie może
żadnego z nich złamać — nie rusza źródeł, kolejności ani niewiadomych — więc
ścieżka błędu jest w praktyce nieosiągalna. Ale „w praktyce nieosiągalna" to nie
to samo co „niemożliwa", a połknięcie tego `assert`em byłoby obietnicą, której ta
klasa nie może złożyć o typie, którego nie jest właścicielem.

Koszt dla wołającego to jedno `.has_value()`. Jeśli wolisz wersję z architektury,
to zmiana czterech linijek — ale wtedy `assert` w Release nie sprawdza niczego,
a `create` zwróci błąd, który zniknie.

---

## 6. Testy — 25

Plik `tests/test_expression_canonicalizer.cpp`. Pełny kod jest długi i powtarza
wzorce z F3.1, więc podaję **listę z intencją**, a pełne źródła w części, której
review najczęściej dotyka.

```text
porządek (3)
  IsTotalAndAntisymmetricOnMixedKinds     11 rodzajów węzłów, wszystkie pary
  OrdersByNameNotByHash                   „a" < „z"; łapie podmianę na odcisk
  DoesNotDependOnConstructionOrder        a+b i b+a -> jedna postać

AC-normalizacja (13)
  FlattensNestedAssociativeChains
  ReducesExactConstantsAndDropsNeutral    3 + x + 4 -> 7 + x
  DoesNotJoinConstantsInexactly           1e16 + 1 zostaje rozdzielone
  ReducesConstantsToAFixedPoint           2^52, 2^52+1, 2^52+3 -> JEDNA stała
  EliminatesSubtractionEntirely           + obchód całego wyniku: zero Subtract
  NormalizesExplicitNegatedSum            -(a+b), 0-(a+b), (-a)+(-b)
  FlattensOperatorRevealedByChildNormalization   a + -(-(b+c)) -> płasko
  NormalizesSignByParity                  -(a*b), (-a)*b, a*(-b)
  NormalizesNegativeConstantSign          (-2)*x, 2*(-x);  (-1)*x -> -x
  KeepsNegativeZeroOutOfParity            -0.0 nie przełącza znaku
  NormalizesMinusOneTimesSum              (-1)*(a+b) BEZ Negate nad sumą
  NormalizesSignThroughDivision           -(a/b), (-a)/b, a/(-b)
  IsIdempotent                            pięć wyrażeń, sameNode po drugim przebiegu

sesja i tożsamość (2)
  KeepsNodeIdentityInAFreshSession        PUSTE memo, ten sam węzeł
  PreservesSharingAcrossRoots             wspólne poddrzewo = JEDEN węzeł w obu wynikach

dziedzina (1)
  PreservesDefinednessAcrossCanonicalization   1/(q+r), punkt q=1, r=-1

predykaty dokładności (3)
  AgreesWithExactArithmeticOnBoundaryCases     10 iloczynów + 5 sum, werdykty
                                               policzone niezależnie
  RejectsProductWhoseRoundingErrorUnderflows   nextafter(2^-511)^2
  AcceptsExactProductOutsideTheNormalRange     2^-1000 * 2^-70

EquationSimplifier (3)
  PreservesEverythingButTheExpressions    taskKind, unknowns, kolejność, source
  SharesOneSessionAcrossTheSystem         liczba unikalnych węzłów po
  AgreesNumericallyWithinToleranceOnFk    oba roboty, 50 konfiguracji, próg 1e-12
```

### 6.1 Test, który jest sercem sekcji 3.2.1 architektury

```cpp
TEST(ExactnessTest, AgreesWithExactArithmeticOnBoundaryCases)
{
    struct Case { double lhs; double rhs; bool exact; };

    // Verdicts computed independently in exact rational arithmetic.
    const std::array<Case, 10> products{{
        {1e200, 3.0, true},
        {3.0, 4.0, true},
        {0.5, 0.25, true},
        {std::nextafter(std::ldexp(1.0, -511), 1.0),
         std::nextafter(std::ldexp(1.0, -511), 1.0), false},
        {std::numeric_limits<double>::denorm_min(), 0.5, false},
        {std::ldexp(1.0, -1000), std::ldexp(1.0, -70), true},
        {std::nextafter(1.0, 2.0), 3.0, false},
        {std::numeric_limits<double>::max(), 1.0, true},
        {std::numeric_limits<double>::max(), 2.0, false},
        {4503599627370497.0, 2.0, true},
    }};

    for (const Case& product : products)
    {
        EXPECT_EQ(ik::detail::multipliesExactly(product.lhs, product.rhs), product.exact)
            << product.lhs << " * " << product.rhs;
    }

    const std::array<Case, 5> sums{{
        {3.0, 4.0, true},
        {1e16, 1.0, false},
        {std::ldexp(1.0, 52), std::ldexp(1.0, 52) + 1.0, false},
        {std::ldexp(1.0, 52) + 1.0, std::ldexp(1.0, 52) + 3.0, true},
        {std::numeric_limits<double>::denorm_min(),
         std::numeric_limits<double>::denorm_min(), true},
    }};

    for (const Case& sum : sums)
    {
        EXPECT_EQ(ik::detail::addsExactly(sum.lhs, sum.rhs), sum.exact)
            << sum.lhs << " + " << sum.rhs;
    }
}
```

Werdykty w kolumnie `exact` **nie są przepisane z implementacji** — pochodzą
z niezależnego rachunku na ułamkach wymiernych, poza C++. Gdyby predykat i test
liczyły to samo tym samym sposobem, test badałby zgodność kodu z samym sobą.

---

## 7. Gate i liczby

### 7.1 Zmierzone

```text
przed    450 / 450
nowe     +25
po       475 / 475
```

W obu konfiguracjach, z `-Wall -Wextra -Wswitch -Werror`, z assertami żywymi
i pod `NDEBUG`.

### 7.2 Rozmiar wyniku — liczba, której się nie spodziewałem

```text
kr640.urdf     przed  total= 540  unique=286
               po     total= 598  unique=334      +17% unikalnych

kr4_r600.urdf  przed  total= 920  unique=521
               po     total=1059  unique=674      +29% unikalnych
```

**Postać normalna jest większa od wejścia.** Powód nie jest przypadkowy:
współdzielenie w FK bierze się ze **wspólnych prefiksów** łańcuchów sumowych —
kompozycja macierzy buduje `((a+b)+c)` i dzieli `(a+b)` między komórki. Sortowanie
składników niszczy te prefiksy, bo dwie komórki dostają różną kolejność, więc ich
częściowe sumy przestają być tym samym węzłem.

Nie jest to argument przeciw etapowi — postać normalna jest **przesłanką** F3.3,
F3.5 i F3.8, a nie zabiegiem oszczędnościowym (architektura 3) — ale trzeba to
napisać, bo naturalne oczekiwanie jest odwrotne.

**Niezmierzone i tak opisane:** o ile gorzej wypadłaby sesja **na równanie**
zamiast jednej na układ. Test `SharesOneSessionAcrossTheSystem` sprawdza dziś
tylko, że wynik mieści się w rozsądnym rzędzie wielkości. Ten pomiar wart jest
zrobienia przy implementacji i wpisania do dokumentu — to on uzasadnia całą
konstrukcję sesji.

---

## 8. Ryzyka implementacyjne

| Ryzyko | Reakcja |
|---|---|
| Splice w kolektorze zapętla się | Terminacja opiera się na tożsamości węzła; argument w 3.1, nie tylko w komentarzu |
| Fabryka zwija dwie reszty stałych | Stałe doklejane na zewnątrz, nigdy dwie w jednym `add`/`multiply` (4) |
| `x * 0` zwinięte do `0` | Zwijane tylko przy braku czynników symbolicznych (4.1) |
| FTZ/DAZ włączone w runtime psuje 2Sum | `static_assert` łapie typ, nie tryb procesora — zapisane jako znany brak (2.1) |
| `EquationSimplifier` zwraca `expected` wbrew architekturze | Odstępstwo świadome i uzasadnione (5.1); do cofnięcia w cztery linijki |
| Postać normalna większa od wejścia | Zmierzone i wyjaśnione (7.2), nie ukryte |
| Rosnący duplikat funktorów tożsamości | Nie rośnie: `detail::ExpressionMap` bierze się z `RewriteEngine.hpp` (1.1) |

---

## 9. Rekomendacja

Przyjąć, zaimplementować, uruchomić gate `475/475` w obu konfiguracjach, dorobić
pomiar z 7.2, zaktualizować `STATUS.md`, `KATALOG.md` i roadmapę, przenieść oba
dokumenty F3.2 do `proposal/done/`, zrobić commit.

Potem **F3.3** — trygonometria. To tam po raz pierwszy pojawi się reguła dla
silnika z F3.1, i tam trzeba będzie rozstrzygnąć pytanie zapisane w architekturze
F3.2 §4 jako znane wejście: czy reguły trygonometryczne piszemy tak, żeby
zachowywały postać AC-normalną, czy przeplatanie kanonizacji z przepisywaniem
dostaje własny dowód terminacji.
