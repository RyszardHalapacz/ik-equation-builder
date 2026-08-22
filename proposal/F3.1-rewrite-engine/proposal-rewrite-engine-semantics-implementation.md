# Proposal: silnik przepisywania — F3.1, implementacja

**Etap:** F3.1 · **v2 — do review** · 2026-08-22 · część 2 z 2

> **v2, po review.** Jeden blocker, trzy wzmocnienia i dwa drobiazgi:
>
> 1. **`collectGuards` nie zatrzymywało builda przy nowym typie węzła.**
>    Generyczne `requires { node.operand; }` potraktowałoby `SqrtNode` jak
>    operator totalny i policzyło `guards(sqrt(x)) = guards(x)`, gubiąc `x >= 0`.
>    Visitor jest teraz **wyliczający**, z `static_assert` w galeęzi domyślnej (3).
> 2. **Test „odrzucona reguła nie blokuje kolejnych" nie testował tego, co
>    deklaruje** — dowodził rzeczy o traversalu, nie o dwóch regułach na jednym
>    węźle. Rozdzielone na dwa testy (6.4).
> 3. **Równoważność behawioralna sprawdzana była tylko dla rewrite'u
>    odrzuconego.** Dochodzi test dla rewrite'u **przyjętego** (6.6).
> 4. **Kontrakt determinizmu wzmocniony** do niezależności od przypadkowego
>    współdzielenia DAG-a (1.3).
> 5. Cache'owa maszyneria schowana w `namespace detail`, `<utility>` w teście.
>
> Liczba testów rośnie z 26 do **28**, gate `450/450`. Architektura wymienia 26 —
> różnica to dwa testy dodane w tym review, nie zmiana zakresu etapu.
>
> **Jedna uwaga z review się nie potwierdziła** i opisuję to w 6.5, bo wpływa na
> to, czemu wierzyć przy następnym takim przypadku.

> Architektura **v6 zatwierdzona**. Ten dokument nie zmienia żadnego jej
> rozstrzygnięcia — pokazuje kod, który je realizuje, i rozstrzyga to, co
> architektura świadomie zostawiła implementacji.
>
> Wchodzą tu również **dwie uwagi nieblokujące** z approvala, obie jako zmiany
> w treści kontraktów, nie w konstrukcji:
>
> 1. **„ten sam replacement" znaczy strukturalnie, nie tożsamościowo** — fabryka
>    nie robi hash-consingu, więc czysta reguła wywołana dwa razy legalnie
>    zbuduje dwa różne węzły o identycznej strukturze (1.2).
> 2. **„punkt stały" znaczy punkt stały całej polityki przepisywania**, razem
>    z kontekstowym representability guardem i kolejnością reguł — a nie
>    „żaden potomek nie dałby się przepisać w izolacji" (2.4).

---

## Prompt

> implementuj, potem zrob aktualizacje danych, potem zrob commita, potem pisz
> kolejny proposal

Dokument zawiera pełny kod trzech nowych plików, 28 testów i dwie linijki
w CMake. Nic z istniejącego kodu nie jest modyfikowane.

---

## 0. Zakres i stan wyjściowy

```text
NOWE   src/ik_equations/symbolic/RewriteRule.hpp     typy i kontrakty
       src/ik_equations/symbolic/RewriteEngine.hpp   API + sesja
       src/ik_equations/symbolic/RewriteEngine.cpp   analizator, sesja, pętla
       tests/test_rewrite_engine.cpp                 28 testów

ZMIENIONE  CMakeLists.txt          +1 wiersz
           tests/CMakeLists.txt    +1 wiersz

BEZ ZMIAN  Expression, ExpressionFactory, ExpressionEvaluator, SymbolicMatrix,
           SymbolicTransform, cały model, builders, fasada, loader
```

Ostatni wiersz jest sprawdzianem z §9 architektury i **wychodzi**: silnik
dokłada warstwę i nie potrzebuje ani jednej zmiany pod spodem. W szczególności
`ExpressionFactory::divide` zachowuje swój `assert(!isZero(rhs))` —
representability guard istnieje właśnie po to.

Stan wyjściowy: `422/422` w obu konfiguracjach, gałąź `f2.3r-review-remediation`.

---

## 1. `RewriteRule.hpp`

### 1.1 Pełny plik

```cpp
#pragma once

#include "ik_equations/symbolic/Expression.hpp"

#include <cstddef>
#include <optional>
#include <string>

namespace kinemaforge::ik {

class ExpressionFactory;

// A rewrite rule: given an expression, either produce a replacement or decline.
//
// nullopt means "does not apply", NOT "applies and changes nothing". A rule
// that returned its input unchanged would spin the fixed-point loop, so the
// engine treats an unchanged result as a defect.
//
// THREE CONTRACTS, all owed by the rule's author and none checkable by the
// engine on its own:
//
//   (E)         the replacement is algebraically equal to the input ON THE
//               INPUT'S DOMAIN. The engine cannot verify this and does not try.
//
//   measure     the replacement has strictly fewer nodes counted WITH
//               MULTIPLICITY. This one is asserted in diagnostic builds.
//
//   determinism apply() must be deterministic with respect to the MEANING of
//               its arguments: for STRUCTURALLY EQUIVALENT inputs it makes the
//               same decision and produces STRUCTURALLY EQUIVALENT replacements.
//               External mutable state must not affect either.
//
// On the third, two halves, and both matter.
//
// "Structurally equivalent replacement", not "the same node": this factory does
// no hash-consing, so building multiply(constant(2), q) twice yields two nodes
// that are structurallyEqual and not sameNode. A rule is pure when its output
// means the same thing every time, not when it hands back the same allocation
// -- requiring the latter would demand something ExpressionFactory never
// promised.
//
// "Structurally equivalent INPUTS", not "the same input node": Add(x, x) with
// both operands the same node and Add(x1, x2) with two structurally equal but
// separately built operands mean the same thing, so a rule must answer the same
// way for both. Node identity may be used as a FAST PATH -- sameNode(lhs, rhs)
// short-circuits structurallyEqual -- but never as the decision:
//
//     if (sameNode(lhs, rhs)) simplify(); else decline();   // FORBIDDEN
//
// A rule shaped like that makes the normal form depend on how the DAG happened
// to be allocated earlier, which is a difference no caller controls and no test
// reproduces reliably. F3.2 wants canonical forms, not forms that remember
// their allocation history.
//
// Diagnostic side effects are allowed as long as they cannot reach the result.
// Counting how often a rule was consulted is fine, and the engine's own tests
// rely on it; declining the first time and applying the second is not.
//
// Determinism is not decoration. The session memoises results across roots, so
// a rule consulted once may never be consulted again for that node; a rule
// whose answer depends on when it was asked makes the memo hold a claim the
// rules would no longer agree with.
struct RewriteRule
{
    // An owning std::string, not a string_view: the session outlives the call
    // that built its rule array, so a view would point into storage the session
    // does not control. A dozen rules copied once at construction cost nothing.
    std::string name;

    std::optional<Expression> (*apply)(const Expression&, const ExpressionFactory&);
};

// A defensive operational budget, NOT a bound derived from the size of the
// input DAG. Termination rests on each rule's own decreasing measure; this
// limit exists as an independent net for the case where that contract is
// broken. Sanity check only: the largest forward-kinematics DAG in this project
// measures 153 703 nodes with multiplicity (kr4_r600), so a million leaves
// ample headroom over anything the pipeline builds today.
inline constexpr std::size_t kDefaultRewriteStepLimit = 1'000'000;

// The result of one rewrite() call. Both flags answer DIFFERENT questions and
// are deliberately independent, because a session-wide memo makes them diverge:
//
//   changed   the output differs from the input OF THIS CALL
//   steps     new rule applications accepted DURING THIS CALL; a memo hit is
//             not a step
//
// A rule can fire while the output stays identical (the representability guard
// drops the replacement at the parent): changed = false, steps = 1. And the
// output can differ while no rule fires here (the memo already held the
// result): changed = true, steps = 0.
//
// Consumers driving repeated passes must test `changed`, never `steps > 0`.
//
// reachedLimit is NOT an error, which is why this is not an expected<>: the
// expression is valid, has the same domain, and is equivalent -- it is merely
// not canonical.
struct RewriteResult
{
    Expression expression;
    bool changed{};
    std::size_t steps{};
    bool reachedLimit{};
};

} // namespace kinemaforge::ik
```

### 1.2 Dlaczego „strukturalnie", a nie „ten sam węzeł"

Uwaga z approvala, przyjęta w brzmieniu recenzenta. Kontrakt v6 mówił „ten sam
replacement", co przy dosłownym czytaniu znaczy `sameNode`. Tego **fabryka nie
obiecuje**:

```cpp
const Expression first  = factory.multiply(factory.constant(2.0), q);
const Expression second = factory.multiply(factory.constant(2.0), q);

sameNode(first, second)          // false -- brak hash-consingu
structurallyEqual(first, second) // true
```

Reguła całkowicie czysta dostałaby więc etykietę niedeterministycznej, a autor
reguły pisząc jej test wymagałby czegoś, czego nie da się spełnić bez zmiany
fabryki. Kontrakt mówi zatem o **znaczeniu wyniku**, nie o alokacji.

Silnikowi to nie przeszkadza: memo trafia w wynik, zanim reguła zostanie
zapytana po raz drugi, więc rozjazd tożsamości nie ma jak się ujawnić.

### 1.3 Druga połowa: to samo dotyczy **wejścia**

Wzmocnienie z review, i słusznie — wersja v1 mówiła „ten sam `Expression`", co
dla memo po tożsamości węzła wystarcza, ale dla F3.2 już nie. Reguła może
zobaczyć dwie postacie tego samego znaczenia:

```text
Add(x, x)       obie gałęzie to TEN SAM węzeł
Add(x1, x2)     x1 i x2 structurallyEqual, ale NIE sameNode
```

Nic w projekcie nie gwarantuje, która z nich powstanie — to zależy od tego, czy
budowniczy akurat współdzielił poddrzewo. Reguła napisana tak:

```cpp
if (sameNode(lhs, rhs)) simplify(); else decline();      // ZAKAZANE
```

dałaby postać normalną zależną od **historii alokacji**: to samo wyrażenie
matematyczne raz uproszczone, raz nie, bez żadnej różnicy widocznej dla
wołającego. Kontrakt mówi więc o **znaczeniu wejścia**, a `sameNode` wolno użyć
wyłącznie jako fast-path — zresztą `structurallyEqual` robi to skrócenie sam,
w pierwszej linijce.

Silnik tego nie egzekwuje i nie ma jak; to trzeci kontrakt autora reguły, obok
(E) i miary. Warto go mieć **teraz**, zanim F3.2 napisze pierwsze dwadzieścia
reguł pod słabsze brzmienie.

---

## 2. `RewriteEngine.hpp`

### 2.1 Pełny plik

```cpp
#pragma once

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/RewriteRule.hpp"

#include <cstddef>
#include <functional>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

namespace kinemaforge::ik {

// Cache machinery, not vocabulary: in `detail` because nothing outside this
// engine has a reason to name it, and `kinemaforge::ik::ExpressionMap` would
// read as a type the module offers on purpose.
//
// Hashing and equality by NODE IDENTITY, with the key owning its node: an entry
// keeps its node alive, so a cache filled while rewriting a temporary cannot be
// answered for a different node that later reuses the address.
//
// ExpressionEvaluator carries a private copy of this pair for its own memo.
// Promoting one shared definition means editing a file this stage otherwise
// does not touch, so the duplication is recorded in STATUS.md as a known gap
// rather than fixed in passing.
namespace detail {

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

template <typename Value>
using ExpressionMap =
    std::unordered_map<Expression, Value, ExpressionIdentityHash, ExpressionIdentityEqual>;

} // namespace detail

// The syntactic domain guards of an expression: the divisor of every Divide
// node, deduplicated STRUCTURALLY. Complete for the nine node kinds that exist
// today -- a new partial operator (sqrt, log, asin) would need a richer model,
// not one more entry here.
//
// Structural deduplication, not by identity, because hasSameDomainGuards()
// compares sizes and then inclusion; that is a valid set comparison only when
// neither side holds two equivalent entries.
[[nodiscard]] std::vector<Expression> divisorsOf(const Expression& expression);

// Compares those guards AS SETS -- order-independent, because divisorsOf walks
// the tree and two guards may come out in either order. A plain vector ==
// vector would reject a rewrite for reordering its own output.
//
// Deliberately NOT called preservesDomain: it decides a syntactic question, and
// only LOCALLY -- at the point a rule fires, between the rule's input and its
// output. Applying it to whole trees before and after a rewrite is wrong and
// would reject correct rewrites (a/-(-q) and a/q have different divisors and
// the same domain); the whole-tree property follows by induction instead.
//
// These two are the API and the tests' tool, not the path the engine walks:
// called directly they traverse the DAG from scratch. RewriteSession uses the
// cached form, over the SAME implementation -- two independent domain analysers
// is exactly the shape in which a safety condition quietly stops meaning what
// its test says.
[[nodiscard]] bool hasSameDomainGuards(const Expression& before, const Expression& after);

// Rewrites expressions to a fixed point under one fixed, ordered set of rules.
//
// A SESSION, not a free function, for the same reason ExpressionEvaluator is
// one: the sixteen cells of a transform and the two sides of twelve equations
// are many roots of ONE shared DAG, and the caches work between them.
//
// RULE ORDER IS SEMANTICS. The session preserves the order given to the
// constructor and the first admissible rule wins. The engine promises neither
// confluence nor a unique normal form -- two correct rules may lead to two
// different fixed points, and which one is reached depends on their order.
// Choosing the set and its order is the caller's job.
//
// FIXED POINT means a fixed point of the complete rewrite policy: the rules in
// their declared order, the local domain condition, and the contextual
// representability guard. It does NOT mean that no descendant would be
// rewriteable in isolation. a/(q-q) is a fixed point of this engine even with
// `x - x -> 0` in the set, because the guard refuses to put a literal zero in a
// denominator -- the subexpression matches, the policy still declines.
//
// The caches grow monotonically and one session is one rule set; a different
// set needs a new session, which is cheap.
//
// Not thread-safe: the caches are mutable. The DAG itself is immutable, so
// several sessions may read the same tree concurrently.
class RewriteSession
{
public:
    // Takes a span and COPIES it: the session owns its rules.
    RewriteSession(std::span<const RewriteRule> rules, ExpressionFactory factory,
                   std::size_t stepLimit = kDefaultRewriteStepLimit);

    // A session is not a value, for the same reason ExpressionEvaluator is not:
    // copying would duplicate the caches with no clear meaning.
    RewriteSession(const RewriteSession&) = delete;
    RewriteSession& operator=(const RewriteSession&) = delete;

    // No explicit noexcept: unordered_map's move constructor is only
    // conditionally noexcept, so the compiler-derived specifier states the
    // truth instead of turning a throwing move into std::terminate.
    RewriteSession(RewriteSession&&) = default;
    RewriteSession& operator=(RewriteSession&&) = default;

    [[nodiscard]] RewriteResult rewrite(const Expression& expression);

private:
    using Memo = detail::ExpressionMap<Expression>;
    using Guards = detail::ExpressionMap<std::vector<Expression>>;

    // Everything one rewrite() call owns. New cache entries land here and are
    // committed to the session only if the call reaches a fixed point, so a
    // call stopped by the step limit leaves nothing behind -- neither an
    // unfinished result the next root could pick up, nor the temporary DAGs a
    // guard entry would keep alive through its key.
    struct Call
    {
        Memo memo;
        Guards guards;
        std::size_t steps{};
        bool reachedLimit{};
    };

    Expression rewriteNode(const Expression& expression, Call& call);
    Expression rewriteChildren(const Expression& expression, Call& call);
    std::optional<Expression> firstAdmissibleRule(const Expression& expression, Call& call);

    const Expression* lookup(const Expression& expression, const Call& call) const;
    const std::vector<Expression>& domainGuards(const Expression& expression, Call& call);

    std::vector<RewriteRule> rules_;
    ExpressionFactory factory_;
    std::size_t stepLimit_;

    Memo memo_;      // proven fixed points only
    Guards guards_;  // domain guards, committed with the memo

    // Node measures, used ONLY by the rule-contract assert. Stays empty in a
    // build with NDEBUG defined -- which the Release configuration does -- and
    // that is the honest statement of the measure contract's enforcement: it is
    // checked where asserts are live and nowhere else.
    detail::ExpressionMap<std::size_t> measures_;
};

} // namespace kinemaforge::ik
```

### 2.2 Dlaczego funktory tożsamości wychodzą z klasy — i dlaczego do `detail`

`ExpressionEvaluator` trzyma je prywatnie, bo był jedynym użytkownikiem. Tutaj
potrzebuje ich **i** sesja (pola), **i** wolne funkcje z `.cpp` (lokalne cache)
— a dublowanie ich po raz trzeci, tym razem w jednym pliku dwa razy, byłoby
gorsze niż wypuszczenie nazwy.

Wypuszczam je jednak do `detail`, nie do `kinemaforge::ik`: to mechanizm
cache'a, a nie typ, który moduł oferuje. `kinemaforge::ik::ExpressionMap`
czytałoby się jak słownik warstwy symbolicznej i pierwszy konsument wziąłby go
za zaproszenie.

Nie ruszam przy tym evaluatora. Wspólny nagłówek dla tej pary to refaktoryzacja
pliku, którego ten etap nie dotyka; **zapisuję ją w `STATUS.md` jako znany
brak**, zamiast dokleić do etapu o czym innym.

### 2.3 Kolejność reguł w komentarzu klasy

Architektura ustaliła to w 4.4; tutaj trafia w miejsce, gdzie zobaczy to
autor kodu wołającego, a nie tylko czytelnik proposala. Zdanie
„zmiana kolejności może zmienić postać wyniku" musi stać przy typie, bo za trzy
etapy zobaczy je ktoś, kto tego dokumentu nie przeczyta.

### 2.4 Co znaczy „punkt stały" — druga uwaga z approvala

Wpisana do komentarza klasy w brzmieniu recenzenta, z przykładem, bo bez
przykładu brzmi jak wykręt:

```text
a / (q - q)   z regułą  x - x -> 0  w zestawie

   podwyrażenie q-q PASUJE do reguły
   polityka i tak odmawia: guard nie wpuszcza literalnego zera do mianownika
   => a/(q-q) JEST punktem stałym tego silnika
```

To nie jest luka w definicji, tylko jej treść: punkt stały jest punktem stałym
**polityki** — reguł w zadanej kolejności, warunku dziedziny i kontekstowego
representability guarda — a nie stwierdzeniem, że żaden potomek nie dałby się
przepisać w izolacji.

---

## 3. `RewriteEngine.cpp` — analizator dziedziny

Plik dzielę na trzy części, bo trzy razy chodzi o coś innego. Kolejność w pliku
jest taka jak tu.

```cpp
#include "ik_equations/symbolic/RewriteEngine.hpp"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <type_traits>
#include <utility>
#include <variant>

namespace kinemaforge::ik {

namespace {

using GuardCache = detail::ExpressionMap<std::vector<Expression>>;

// The default branch of an exhaustive if-constexpr chain needs a condition that
// is dependent on the template parameter; a plain `static_assert(false)` fires
// while merely parsing the template.
template <typename>
inline constexpr bool alwaysFalse = false;

// Structural, not by identity: `a/x + b/x` built from two separately
// constructed `x` nodes must yield ONE guard, or the size-plus-inclusion test
// in sameGuards() stops being a set comparison.
void appendUnique(std::vector<Expression>& guards, const Expression& candidate)
{
    const auto equivalent = [&candidate](const Expression& present) {
        return structurallyEqual(present, candidate);
    };
    if (std::none_of(guards.begin(), guards.end(), equivalent))
        guards.push_back(candidate);
}

void appendAll(std::vector<Expression>& guards, const std::vector<Expression>& more)
{
    for (const Expression& guard : more)
        appendUnique(guards, guard);
}

// THE domain analyser. There is exactly one, and both the free functions and
// RewriteSession go through it.
//
// Computed compositionally rather than by scanning from scratch:
//
//     guards(Constant | Symbol) = {}
//     guards(op(a, b))          = guards(a) u guards(b)
//     guards(op(a))             = guards(a)
//     guards(Divide(a, b))      = guards(a) u guards(b) u { b }
//
// so each unique node is computed once. That bounds the NUMBER of computations,
// not the total cost: each union deduplicates structurally, so the cost of one
// entry grows with the number of DISTINCT guards below it. For everything this
// project builds today that number is zero -- no builder calls
// ExpressionFactory::divide, so no FK DAG contains a single Divide node.
//
// `committed` is the session-wide cache when a session calls this and nullptr
// when a free function does; `pending` always receives the new entries.
//
// Returns a reference INTO `pending`. std::unordered_map keeps references to
// its elements valid across rehashing -- only iterators are invalidated -- so
// the recursive calls below cannot dangle what this returns.
const std::vector<Expression>& collectGuards(const Expression& expression,
                                             GuardCache& pending,
                                             const GuardCache* committed)
{
    if (const auto found = pending.find(expression); found != pending.end())
        return found->second;

    if (committed != nullptr)
        if (const auto found = committed->find(expression); found != committed->end())
            return found->second;

    std::vector<Expression> guards;

    // EVERY node kind spelled out, on purpose. A generic
    // `else if constexpr (requires { node.operand; })` would silently absorb a
    // future SqrtNode and report guards(sqrt(x)) = guards(x), losing x >= 0 --
    // and it would compile. Here a tenth node kind lands in the default branch
    // and STOPS THE BUILD with a message naming the decision that has to be
    // made. This is a safety boundary; explicit duplication is the feature.
    std::visit(
        [&](const auto& node) {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, ConstantNode> ||
                          std::is_same_v<Node, SymbolNode>)
            {
                // No children, no guards.
            }
            else if constexpr (std::is_same_v<Node, AddNode> ||
                               std::is_same_v<Node, SubtractNode> ||
                               std::is_same_v<Node, MultiplyNode>)
            {
                appendAll(guards, collectGuards(node.lhs, pending, committed));
                appendAll(guards, collectGuards(node.rhs, pending, committed));
            }
            else if constexpr (std::is_same_v<Node, DivideNode>)
            {
                appendAll(guards, collectGuards(node.lhs, pending, committed));
                appendAll(guards, collectGuards(node.rhs, pending, committed));

                // The only place in the language that restricts the domain.
                appendUnique(guards, node.rhs);
            }
            else if constexpr (std::is_same_v<Node, NegateNode> ||
                               std::is_same_v<Node, SinNode> ||
                               std::is_same_v<Node, CosNode>)
            {
                appendAll(guards, collectGuards(node.operand, pending, committed));
            }
            else
            {
                static_assert(alwaysFalse<Node>,
                              "unhandled Expression node in domain analysis: decide whether "
                              "it restricts the domain before adding it to the language");
            }
        },
        expression.node().value);

    return pending.emplace(expression, std::move(guards)).first->second;
}

// Set equality. Both sides are structurally deduplicated by construction, so
// equal sizes plus one-way inclusion is enough.
bool sameGuards(const std::vector<Expression>& before, const std::vector<Expression>& after)
{
    if (before.size() != after.size())
        return false;

    return std::all_of(before.begin(), before.end(), [&after](const Expression& guard) {
        return std::any_of(after.begin(), after.end(), [&guard](const Expression& other) {
            return structurallyEqual(guard, other);
        });
    });
}

} // namespace

std::vector<Expression> divisorsOf(const Expression& expression)
{
    GuardCache cache;
    return collectGuards(expression, cache, nullptr);
}

bool hasSameDomainGuards(const Expression& before, const Expression& after)
{
    GuardCache cache;

    // Both references stay valid while the second call inserts -- see the note
    // on reference stability in collectGuards.
    const std::vector<Expression>& beforeGuards = collectGuards(before, cache, nullptr);
    const std::vector<Expression>& afterGuards = collectGuards(after, cache, nullptr);

    return sameGuards(beforeGuards, afterGuards);
}
```

### 3.1 Dlaczego akurat tu żadnego `requires`

Architektura obiecała, że dziesiąty typ węzła **zatrzyma build** i wymusi
decyzję o dziedzinie. v1 tej obietnicy nie dotrzymywała i to jest ta poprawka.

```text
struct SqrtNode { Expression operand; };     // dorzucony kiedyś do wariantu

v1:  requires { node.operand; }  łapie go jako operator TOTALNY
     guards(sqrt(x)) = guards(x)             kompiluje się, gubi x >= 0
v2:  gałąź domyślna, static_assert         build staje, z komunikatem
```

Kontrargument, który sam sobie postawiłem i który **nie broni** v1: przecież
`rewriteChildren` też by się wywaliło, bo kończy `static_assert`ami na
`DivideNode` i `CosNode`. Owszem — ale to znaczy tylko tyle, że autor nowego
węzła **naprawi przebudowę** i pójdzie dalej. Analizator dziedziny przepuści go
w milczeniu, a to jedyne miejsce, w którym pomyłka nie daje błędu kompilacji ani
złego wyniku, tylko **cichą utratę bezpieczeństwa**.

`measureOf` z 4 zostaje generyczne i to jest świadoma różnica: miara jest
własnością **strukturalną**, więc `1 + measure(operand)` jest poprawne dla
każdego przyszłego węzła jednoargumentowego, cokolwiek by liczył. Dziedzina
poprawna „z definicji" nie jest.

Uwaga o stabilności referencji nie jest ozdobą: bez niej `collectGuards`
zwracające `const&` do mapy, do której samo wstawia w rekurencji, byłoby
błędem. `std::unordered_map` jest kontenerem węzłowym i unieważnia przy rehashu
**iteratory**, nie referencje — dlatego ten kształt jest legalny i dlatego
komentarz o tym stoi w kodzie.

---

## 4. `RewriteEngine.cpp` — kontrakt reguły

```cpp
namespace {

#ifndef NDEBUG

// Node count WITH MULTIPLICITY, memoised by node identity. Memoising the count
// does not turn it into a count of unique nodes: the parent adds a shared
// child's total once per slot it occupies, which is exactly what "with
// multiplicity" means.
std::size_t measureOf(const Expression& expression, ExpressionMap<std::size_t>& cache)
{
    if (const auto found = cache.find(expression); found != cache.end())
        return found->second;

    std::size_t total = 1;

    std::visit(
        [&](const auto& node) {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, ConstantNode> || std::is_same_v<Node, SymbolNode>)
            {
            }
            else if constexpr (requires { node.lhs; node.rhs; })
            {
                total += measureOf(node.lhs, cache);
                total += measureOf(node.rhs, cache);
            }
            else
            {
                total += measureOf(node.operand, cache);
            }
        },
        expression.node().value);

    cache.emplace(expression, total);
    return total;
}

// Exists only to be called from an assert(), like isActuated in
// JointTransformBuilder -- and with the same caveat stated plainly: a build
// with NDEBUG defined enforces NEITHER of these contracts.
//
// A plain assert() cannot name the offending rule, and "some rule broke the
// measure contract" is a diagnosis nobody can act on when the set has a dozen
// entries. Hence the message on stderr before returning false.
[[maybe_unused]] bool ruleContractHolds(const RewriteRule& rule,
                                        const Expression& input,
                                        const Expression& replacement,
                                        ExpressionMap<std::size_t>& measures)
{
    if (sameNode(replacement, input))
    {
        std::fprintf(stderr,
                     "rewrite rule '%s' returned its input; nullopt means "
                     "\"does not apply\"\n",
                     rule.name.c_str());
        return false;
    }

    const std::size_t before = measureOf(input, measures);
    const std::size_t after = measureOf(replacement, measures);

    if (after >= before)
    {
        std::fprintf(stderr,
                     "rewrite rule '%s' did not decrease the measure: %zu -> %zu\n",
                     rule.name.c_str(), before, after);
        return false;
    }

    return true;
}

#endif // NDEBUG

} // namespace
```

`measureOf` i `ruleContractHolds` żyją pod `#ifndef NDEBUG`, bo `assert()` i tak
usuwa wywołanie, a bez tego zostałyby zdefiniowane i nieużywane. `measures_`
zostaje polem bezwarunkowo — pusta mapa w Release jest tańsza niż warunkowe
pole, które zmienia układ klasy między konfiguracjami.

---

## 5. `RewriteEngine.cpp` — sesja

```cpp
RewriteSession::RewriteSession(std::span<const RewriteRule> rules, ExpressionFactory factory,
                               std::size_t stepLimit)
    : rules_(rules.begin(), rules.end())
    , factory_(std::move(factory))
    , stepLimit_(stepLimit)
{
}

RewriteResult RewriteSession::rewrite(const Expression& expression)
{
    Call call;
    Expression result = rewriteNode(expression, call);

    // Commit, or discard everything this call built. Both caches, one moment.
    // merge() moves the nodes across; a key already present in the session keeps
    // the session's value, which by the determinism contract says the same thing.
    if (!call.reachedLimit)
    {
        memo_.merge(call.memo);
        guards_.merge(call.guards);
    }

    // `changed` is computed BEFORE the move: elements of a braced-init-list are
    // evaluated in order, so writing sameNode(expression, result) as the second
    // element would read a moved-from Expression.
    const bool changed = !sameNode(expression, result);
    return RewriteResult{std::move(result), changed, call.steps, call.reachedLimit};
}

const Expression* RewriteSession::lookup(const Expression& expression, const Call& call) const
{
    // Call-local first: both tiers hold fixed points, so the order is a
    // performance detail, and the local map is the smaller one.
    if (const auto found = call.memo.find(expression); found != call.memo.end())
        return &found->second;

    if (const auto found = memo_.find(expression); found != memo_.end())
        return &found->second;

    return nullptr;
}

const std::vector<Expression>& RewriteSession::domainGuards(const Expression& expression,
                                                           Call& call)
{
    return collectGuards(expression, call.guards, &guards_);
}

Expression RewriteSession::rewriteNode(const Expression& expression, Call& call)
{
    // THE single point of step-limit propagation. Once the budget is gone every
    // deeper call returns its input untouched, so the recursion unwinds by
    // rebuilding the tree from whatever was already normalised: the result is a
    // valid, partially rewritten expression and no caller needs a special case.
    // Nothing is memoised on this path -- the commit in rewrite() is skipped for
    // the whole call, and the emplace below is never reached.
    if (call.reachedLimit)
        return expression;

    if (const Expression* cached = lookup(expression, call))
        return *cached;

    Expression current = expression;

    while (true)
    {
        // Back to the CHILDREN after every firing, not just to the rule attempt:
        // a rule receives the factory and may build a subtree that was never in
        // the input, and calling that a fixed point without normalising it would
        // be a lie. For children already at their fixed point this costs one
        // memo hit.
        Expression rebuilt = rewriteChildren(current, call);
        if (call.reachedLimit)
            return rebuilt;

        const std::optional<Expression> replacement = firstAdmissibleRule(rebuilt, call);
        if (!replacement)
        {
            current = std::move(rebuilt);
            break;
        }

        if (call.steps >= stepLimit_)
        {
            // Admissible, but there is no budget to take it. The last form that
            // was actually built is returned; the replacement is dropped.
            call.reachedLimit = true;
            return rebuilt;
        }

        ++call.steps;
        current = *replacement;
    }

    // Written AFTER the loop, keyed by this call's INPUT and holding its fixed
    // point. The intermediate values of `current` are states in progress, not
    // results, and never reach the cache -- which is what makes "every
    // committed entry is a fixed point" true by construction rather than by
    // one condition somewhere inside the loop.
    call.memo.emplace(expression, current);
    return current;
}

Expression RewriteSession::rewriteChildren(const Expression& expression, Call& call)
{
    return std::visit(
        [&](const auto& node) -> Expression {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, ConstantNode> || std::is_same_v<Node, SymbolNode>)
            {
                return expression;
            }
            else if constexpr (requires { node.lhs; node.rhs; })
            {
                Expression lhs = rewriteNode(node.lhs, call);
                Expression rhs = rewriteNode(node.rhs, call);

                if constexpr (std::is_same_v<Node, DivideNode>)
                {
                    // Representability guard. A denominator that simplified to a
                    // literal zero cannot be built -- ExpressionFactory::divide
                    // asserts against it -- and that contract is NOT being
                    // loosened for this engine: the simplification is dropped and
                    // the original denominator kept. The node stays exactly as
                    // undefined as it was, which is the honest outcome.
                    if (isZero(rhs))
                        rhs = node.rhs;
                }

                // Identity preservation: if nothing actually entered the parent,
                // return the ORIGINAL node. Rebuilding a structurally identical
                // node would make `changed` always true and would break the
                // sharing this engine exists to preserve.
                if (sameNode(lhs, node.lhs) && sameNode(rhs, node.rhs))
                    return expression;

                if constexpr (std::is_same_v<Node, AddNode>)
                    return factory_.add(std::move(lhs), std::move(rhs));
                else if constexpr (std::is_same_v<Node, SubtractNode>)
                    return factory_.subtract(std::move(lhs), std::move(rhs));
                else if constexpr (std::is_same_v<Node, MultiplyNode>)
                    return factory_.multiply(std::move(lhs), std::move(rhs));
                else
                {
                    static_assert(std::is_same_v<Node, DivideNode>);
                    return factory_.divide(std::move(lhs), std::move(rhs));
                }
            }
            else
            {
                Expression operand = rewriteNode(node.operand, call);

                if (sameNode(operand, node.operand))
                    return expression;

                if constexpr (std::is_same_v<Node, NegateNode>)
                    return factory_.negate(std::move(operand));
                else if constexpr (std::is_same_v<Node, SinNode>)
                    return factory_.sin(std::move(operand));
                else
                {
                    static_assert(std::is_same_v<Node, CosNode>);
                    return factory_.cos(std::move(operand));
                }
            }
        },
        expression.node().value);
}

std::optional<Expression> RewriteSession::firstAdmissibleRule(const Expression& expression,
                                                             Call& call)
{
    // Declaration order is semantics: the first admissible rule wins. A rule
    // rejected by the domain condition does not abort the pass and does not
    // block the rules after it.
    for (const RewriteRule& rule : rules_)
    {
        std::optional<Expression> replacement = rule.apply(expression, factory_);
        if (!replacement)
            continue;

        assert(ruleContractHolds(rule, expression, *replacement, measures_) &&
               "rewrite rule broke its contract");

        // The domain condition, and it is LOCAL: between this rule's input and
        // its output, never between whole trees.
        const std::vector<Expression>& before = domainGuards(expression, call);
        const std::vector<Expression>& after = domainGuards(*replacement, call);

        if (!sameGuards(before, after))
            continue;

        return replacement;
    }

    return std::nullopt;
}

} // namespace kinemaforge::ik
```

### 5.1 Propagacja limitu — miejsce, o które prosiło review

Recenzent zapowiedział, że będzie tego pilnował, więc rozliczam to wprost.
Wymagane zachowanie i to, co je realizuje:

```text
limit osiągnięty głęboko w dziecku
    -> natychmiast propaguje w górę      jeden `if` na wejściu rewriteNode:
                                          każde głębsze wywołanie staje się no-opem
    -> nie commitujemy memo               `if (!call.reachedLimit)` w rewrite()
    -> nie commitujemy guardów            ta sama gałąź, te same nawiasy
    -> zwracamy poprawny Expression       rozwijanie stosu składa drzewo z tego,
                                          co zdążyło się znormalizować
    -> reachedLimit = true                pole Call, przepisane do wyniku
```

Nie ma tu ani jednego `if (call.reachedLimit)` u wołających `rewriteNode` —
i to jest cała sztuczka. Strażnik na **wejściu** zamienia każde głębsze
wywołanie w zwrot argumentu, więc `rewriteChildren` składa węzeł z części
przepisanych i nieprzepisanych, nie wiedząc nawet, że coś się stało. Drugi
`if`, w pętli po `rewriteChildren`, istnieje po to, żeby po wyczerpaniu budżetu
nie próbować już reguł.

Miejsce, w którym łatwo napisać subtelnie zły kod, jest dokładnie jedno:
`call.memo.emplace` po pętli. Gdyby pętla mogła się z niej wyrwać z ustawionym
`reachedLimit`, wpis trafiłby do memo lokalnego — i choć commit i tak by go
odrzucił, inwariant „memo lokalne zawiera punkty stałe" przestałby być
prawdziwy w środku wywołania. Dlatego **oba** wyjścia awaryjne to `return`,
nie `break`.

### 5.2 Czego tu nie ma

Nie ma `EquationSimplifier`, nie ma ani jednej reguły produkcyjnej i nie ma
wołania silnika z `ConstraintBuilder`. F3.1 dostarcza mechanizm; reguły
i wpięcie w pipeline to F3.2 i dalej.

---

## 6. Testy — 28

Plik `tests/test_rewrite_engine.cpp`. Reguły testowe są zwykłymi funkcjami,
bo `RewriteRule::apply` jest wskaźnikiem na funkcję.

### 6.1 Wspólna infrastruktura

```cpp
#include <gtest/gtest.h>

#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/RewriteEngine.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace ik = kinemaforge::ik;

using ik::Expression;
using ik::ExpressionFactory;
using ik::RewriteResult;
using ik::RewriteRule;
using ik::RewriteSession;
using ik::sameNode;
using ik::structurallyEqual;

namespace {

const ExpressionFactory kFactory;

// Diagnostic counters. Legal under the determinism contract precisely because
// nothing below reads them to decide anything -- they are written, never
// branched on.
std::size_t g_subtractionConsultations = 0;

void resetCounters()
{
    g_subtractionConsultations = 0;
}

// x - x -> 0. Domain-safe when x carries no division, rejected when it does.
std::optional<Expression> selfSubtraction(const Expression& expression,
                                          const ExpressionFactory& factory)
{
    const auto* node = std::get_if<ik::SubtractNode>(&expression.node().value);
    if (node == nullptr)
        return std::nullopt;

    ++g_subtractionConsultations;

    if (!structurallyEqual(node->lhs, node->rhs))
        return std::nullopt;

    return factory.constant(0.0);
}

// -(-x) -> x
std::optional<Expression> doubleNegation(const Expression& expression, const ExpressionFactory&)
{
    const auto* outer = std::get_if<ik::NegateNode>(&expression.node().value);
    if (outer == nullptr)
        return std::nullopt;

    const auto* inner = std::get_if<ik::NegateNode>(&outer->operand.node().value);
    if (inner == nullptr)
        return std::nullopt;

    return inner->operand;
}

// x * 0 -> 0. THE unsafe one: correct wherever x is defined, and defined in
// more places than x is.
std::optional<Expression> multiplyByZero(const Expression& expression,
                                         const ExpressionFactory& factory)
{
    const auto* node = std::get_if<ik::MultiplyNode>(&expression.node().value);
    if (node == nullptr)
        return std::nullopt;

    if (!ik::isZero(node->lhs) && !ik::isZero(node->rhs))
        return std::nullopt;

    return factory.constant(0.0);
}

// x / x -> 1. Also unsafe: it drops the guard x != 0.
std::optional<Expression> selfDivision(const Expression& expression,
                                       const ExpressionFactory& factory)
{
    const auto* node = std::get_if<ik::DivideNode>(&expression.node().value);
    if (node == nullptr || !structurallyEqual(node->lhs, node->rhs))
        return std::nullopt;

    return factory.constant(1.0);
}

// The counterexample from the architecture: a rule that BUILDS material the
// input never contained. (q+q) + (r-r) = 2q = 2 * -(-q); measure 7 -> 5.
Expression counterexampleInput()
{
    const Expression q = kFactory.symbol("q");
    const Expression r = kFactory.symbol("r");
    return kFactory.add(kFactory.add(q, q), kFactory.subtract(r, r));
}

std::optional<Expression> introducesDoubleNegation(const Expression& expression,
                                                   const ExpressionFactory& factory)
{
    if (!structurallyEqual(expression, counterexampleInput()))
        return std::nullopt;

    const Expression q = factory.symbol("q");
    return factory.multiply(factory.constant(2.0), factory.negate(factory.negate(q)));
}

// Same input, different admissible result: (q+q) + (r-r) -> q + q. Measure 7 -> 3.
std::optional<Expression> collapsesToPlainSum(const Expression& expression,
                                              const ExpressionFactory& factory)
{
    if (!structurallyEqual(expression, counterexampleInput()))
        return std::nullopt;

    const Expression q = factory.symbol("q");
    return factory.add(q, q);
}

// (a/b) * 0 -> 0/b. Equal to the input ON THE SAME DOMAIN -- both are 0 wherever
// b != 0 and both are undefined at b == 0 -- and measure 5 -> 3. It exists to be
// the SECOND rule tried at a node where the first one was rejected.
std::optional<Expression> zeroFactorIntoNumerator(const Expression& expression,
                                                  const ExpressionFactory& factory)
{
    const auto* node = std::get_if<ik::MultiplyNode>(&expression.node().value);
    if (node == nullptr || !ik::isZero(node->rhs))
        return std::nullopt;

    const auto* ratio = std::get_if<ik::DivideNode>(&node->lhs.node().value);
    if (ratio == nullptr)
        return std::nullopt;

    // ratio->rhs cannot be a literal zero: the factory would never have built
    // that Divide in the first place.
    return factory.divide(factory.constant(0.0), ratio->rhs);
}

RewriteRule rule(std::string name, std::optional<Expression> (*apply)(const Expression&,
                                                                     const ExpressionFactory&))
{
    return RewriteRule{std::move(name), apply};
}

RewriteSession sessionOf(std::vector<RewriteRule> rules,
                         std::size_t stepLimit = ik::kDefaultRewriteStepLimit)
{
    return RewriteSession{rules, kFactory, stepLimit};
}

} // namespace
```

### 6.2 Strażnicy dziedziny — 5

```cpp
TEST(RewriteEngineTest, CollectsDivisorsFromNestedExpression)
{
    const Expression a = kFactory.symbol("a");
    const Expression x = kFactory.symbol("x");
    const Expression y = kFactory.symbol("y");

    // a/x + (a/x)/y -- x appears twice and must be reported once.
    const Expression nested = kFactory.add(kFactory.divide(a, x),
                                           kFactory.divide(kFactory.divide(a, x), y));

    const std::vector<Expression> divisors = ik::divisorsOf(nested);

    ASSERT_EQ(divisors.size(), 2u);
    EXPECT_TRUE(structurallyEqual(divisors[0], x) || structurallyEqual(divisors[1], x));
    EXPECT_TRUE(structurallyEqual(divisors[0], y) || structurallyEqual(divisors[1], y));
}

TEST(RewriteEngineTest, RejectsRewriteThatWidensDomain)
{
    const Expression q = kFactory.symbol("q");
    const Expression unsafe = kFactory.multiply(kFactory.divide(kFactory.constant(1.0), q),
                                                kFactory.constant(0.0));

    RewriteSession session = sessionOf({rule("x*0", multiplyByZero)});
    const RewriteResult result = session.rewrite(unsafe);

    EXPECT_FALSE(result.changed);
    EXPECT_TRUE(sameNode(result.expression, unsafe));
}

TEST(RewriteEngineTest, AllowsSameRewriteWhenNoDivisionPresent)
{
    const Expression q = kFactory.symbol("q");
    const Expression safe = kFactory.multiply(q, kFactory.constant(0.0));

    RewriteSession session = sessionOf({rule("x*0", multiplyByZero)});
    const RewriteResult result = session.rewrite(safe);

    EXPECT_TRUE(result.changed);
    EXPECT_TRUE(ik::isZero(result.expression));
}

TEST(RewriteEngineTest, RejectsCancellationOfSelfDivision)
{
    const Expression q = kFactory.symbol("q");
    const Expression ratio = kFactory.divide(q, q);

    RewriteSession session = sessionOf({rule("x/x", selfDivision)});
    const RewriteResult result = session.rewrite(ratio);

    EXPECT_FALSE(result.changed);
}

// The test that would fail under a GLOBAL comparison of divisors: guards go
// from { -(-q) } to { q }, which are different sets, and the rewrite is still
// correct. The condition is local, so this passes.
TEST(RewriteEngineTest, RewritesInsideDenominator)
{
    const Expression a = kFactory.symbol("a");
    const Expression q = kFactory.symbol("q");
    const Expression awkward = kFactory.divide(a, kFactory.negate(kFactory.negate(q)));

    RewriteSession session = sessionOf({rule("-(-x)", doubleNegation)});
    const RewriteResult result = session.rewrite(awkward);

    EXPECT_TRUE(result.changed);
    EXPECT_TRUE(structurallyEqual(result.expression, kFactory.divide(a, q)));
}
```

### 6.3 Representability guard — 2

```cpp
TEST(RewriteEngineTest, KeepsOriginalDenominatorWhenRewriteYieldsZero)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);
    const Expression ratio = kFactory.divide(kFactory.constant(1.0), shared);

    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});
    const RewriteResult result = session.rewrite(ratio);

    EXPECT_TRUE(sameNode(result.expression, ratio));
}

TEST(RewriteEngineTest, StillRewritesSameSubtreeElsewhere)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);

    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});
    const RewriteResult result = session.rewrite(kFactory.add(shared, kFactory.constant(5.0)));

    // add(0, 5) folds in the factory, so the whole thing collapses to 5.
    EXPECT_TRUE(result.changed);
    EXPECT_TRUE(ik::isConstant(result.expression));
    EXPECT_DOUBLE_EQ(ik::constantValue(result.expression), 5.0);
}
```

### 6.4 Silnik — 11

```cpp
TEST(RewriteEngineTest, ReportsChangedWhenARuleFires)
{
    const Expression q = kFactory.symbol("q");
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});

    const RewriteResult result = session.rewrite(kFactory.subtract(q, q));

    EXPECT_TRUE(result.changed);
    EXPECT_EQ(result.steps, 1u);
    EXPECT_FALSE(result.reachedLimit);
}

TEST(RewriteEngineTest, ReportsUnchangedOnFixedPoint)
{
    const Expression q = kFactory.symbol("q");
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});

    const RewriteResult result = session.rewrite(kFactory.add(q, kFactory.symbol("r")));

    EXPECT_FALSE(result.changed);
    EXPECT_EQ(result.steps, 0u);
}

// The architecture's blocker, as a test: after the first rule fires, the fresh
// -(-q) it built must be normalised before anything is called a fixed point.
TEST(RewriteEngineTest, RewritesFreshDescendantsIntroducedByRule)
{
    RewriteSession session = sessionOf({rule("A->2*-(-q)", introducesDoubleNegation),
                                        rule("-(-x)", doubleNegation)});

    const RewriteResult result = session.rewrite(counterexampleInput());

    const Expression expected = kFactory.multiply(kFactory.constant(2.0), kFactory.symbol("q"));
    EXPECT_TRUE(structurallyEqual(result.expression, expected));
    EXPECT_EQ(result.steps, 2u);
}

TEST(RewriteEngineTest, StopsAtStepLimitAndSaysSo)
{
    const Expression q = kFactory.symbol("q");
    const Expression r = kFactory.symbol("r");
    const Expression twoPairs = kFactory.add(kFactory.subtract(q, q), kFactory.subtract(r, r));

    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)}, 1);
    const RewriteResult result = session.rewrite(twoPairs);

    EXPECT_TRUE(result.reachedLimit);
    EXPECT_EQ(result.steps, 1u);

    // The partial result is still a valid expression with the same value.
    ik::ExpressionEvaluator evaluator{{{"q", 0.3}, {"r", -1.25}}};
    const auto value = evaluator.evaluate(result.expression);
    ASSERT_TRUE(value.has_value());
    EXPECT_DOUBLE_EQ(*value, 0.0);
}

// root1 from the architecture's worked example: a rule fires, the guard drops
// it at the parent, the output is identical to the input.
TEST(RewriteEngineTest, ReportsUnchangedWhenGuardRejectsInDenominator)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);
    const Expression root1 = kFactory.divide(kFactory.constant(1.0), shared);

    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});
    const RewriteResult result = session.rewrite(root1);

    EXPECT_FALSE(result.changed);
    EXPECT_EQ(result.steps, 1u);
}

// root2 from the same example, in the SAME session: the output differs while no
// rule fires here, because the memo already holds the answer.
TEST(RewriteEngineTest, ReportsChangedWithoutStepsOnMemoHit)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);

    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});

    const RewriteResult first = session.rewrite(kFactory.divide(kFactory.constant(1.0), shared));
    ASSERT_EQ(first.steps, 1u);

    const RewriteResult second = session.rewrite(kFactory.add(shared, kFactory.constant(5.0)));

    EXPECT_TRUE(second.changed);
    EXPECT_EQ(second.steps, 0u);
}

TEST(RewriteEngineTest, KeepsNodeIdentityWhenNothingApplies)
{
    const Expression q = kFactory.symbol("q");
    const Expression tree = kFactory.add(kFactory.sin(q), kFactory.cos(q));

    RewriteSession session = sessionOf({rule("-(-x)", doubleNegation)});
    const RewriteResult result = session.rewrite(tree);

    EXPECT_TRUE(sameNode(result.expression, tree));
    EXPECT_FALSE(result.changed);
}

// Innermost order, measured by step count: rewriting q-q first lets the factory
// fold both negations away in one step. Outermost order would take two.
TEST(RewriteEngineTest, AppliesRulesInnermostFirst)
{
    const Expression q = kFactory.symbol("q");
    const Expression nested = kFactory.negate(kFactory.negate(kFactory.subtract(q, q)));

    RewriteSession session = sessionOf({rule("-(-x)", doubleNegation),
                                        rule("x-x", selfSubtraction)});
    const RewriteResult result = session.rewrite(nested);

    EXPECT_TRUE(ik::isZero(result.expression));
    EXPECT_EQ(result.steps, 1u);
}

// The rejected rule is tried FIRST at this node; the second rule must still get
// its turn AT THE SAME NODE. This is the claim the architecture makes, and the
// traversal test below does not prove it.
TEST(RewriteEngineTest, ContinuesWithNextRuleAfterDomainRejection)
{
    const Expression q = kFactory.symbol("q");
    const Expression unsafe = kFactory.multiply(kFactory.divide(kFactory.constant(1.0), q),
                                                kFactory.constant(0.0));

    RewriteSession session = sessionOf({rule("x*0", multiplyByZero),
                                        rule("(a/b)*0 -> 0/b", zeroFactorIntoNumerator)});
    const RewriteResult result = session.rewrite(unsafe);

    EXPECT_TRUE(result.changed);
    EXPECT_EQ(result.steps, 1u);
    EXPECT_TRUE(structurallyEqual(result.expression,
                                  kFactory.divide(kFactory.constant(0.0), q)));
}

// A rule rejected at one node does not abort the pass: the engine keeps
// rewriting elsewhere in the same tree.
TEST(RewriteEngineTest, RejectedRuleDoesNotStopTheTraversal)
{
    const Expression q = kFactory.symbol("q");
    const Expression r = kFactory.symbol("r");
    const Expression unsafe = kFactory.multiply(kFactory.divide(kFactory.constant(1.0), q),
                                                kFactory.constant(0.0));
    const Expression tree = kFactory.add(unsafe, kFactory.subtract(r, r));

    RewriteSession session = sessionOf({rule("x*0", multiplyByZero),
                                        rule("x-x", selfSubtraction)});
    const RewriteResult result = session.rewrite(tree);

    EXPECT_EQ(result.steps, 1u);
    EXPECT_TRUE(structurallyEqual(result.expression, unsafe));
}

TEST(RewriteEngineTest, UsesFirstAdmissibleRuleInDeclaredOrder)
{
    const Expression q = kFactory.symbol("q");

    RewriteSession firstWins = sessionOf({rule("A->2*-(-q)", introducesDoubleNegation),
                                          rule("A->q+q", collapsesToPlainSum)});
    const RewriteResult a = firstWins.rewrite(counterexampleInput());

    RewriteSession reversed = sessionOf({rule("A->q+q", collapsesToPlainSum),
                                         rule("A->2*-(-q)", introducesDoubleNegation)});
    const RewriteResult b = reversed.rewrite(counterexampleInput());

    EXPECT_TRUE(structurallyEqual(
        a.expression,
        kFactory.multiply(kFactory.constant(2.0), kFactory.negate(kFactory.negate(q)))));
    EXPECT_TRUE(structurallyEqual(b.expression, kFactory.add(q, q)));
    EXPECT_FALSE(structurallyEqual(a.expression, b.expression));
}
```

### 6.5 DAG — 7

```cpp
TEST(RewriteEngineTest, PreservesSharingAcrossRewrite)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.negate(kFactory.negate(q));
    const Expression tree = kFactory.add(shared, shared);

    RewriteSession session = sessionOf({rule("-(-x)", doubleNegation)});
    const RewriteResult result = session.rewrite(tree);

    const auto* node = std::get_if<ik::AddNode>(&result.expression.node().value);
    ASSERT_NE(node, nullptr);
    EXPECT_TRUE(sameNode(node->lhs, node->rhs));
}

TEST(RewriteEngineTest, RewritesSharedSubtreeOnce)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);

    // Eight occurrences of one node.
    Expression tree = shared;
    for (int level = 0; level < 3; ++level)
        tree = kFactory.add(tree, tree);

    resetCounters();
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});
    const RewriteResult result = session.rewrite(tree);

    EXPECT_TRUE(ik::isZero(result.expression));
    EXPECT_EQ(g_subtractionConsultations, 1u);
    EXPECT_EQ(result.steps, 1u);
}

TEST(RewriteEngineTest, SharesMemoAcrossRootsInOneSession)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);

    resetCounters();
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});

    const RewriteResult first = session.rewrite(kFactory.add(shared, kFactory.symbol("a")));
    ASSERT_FALSE(first.reachedLimit);
    ASSERT_TRUE(first.changed);
    ASSERT_EQ(first.steps, 1u);
    ASSERT_EQ(g_subtractionConsultations, 1u);

    const RewriteResult second = session.rewrite(kFactory.multiply(shared, kFactory.symbol("b")));

    EXPECT_EQ(g_subtractionConsultations, 1u);
    EXPECT_EQ(second.steps, 0u);
    EXPECT_TRUE(second.changed);
}

// The memo key owns its node, so an entry made while rewriting a temporary
// cannot be answered for a different node that later reuses the address.
//
// Stated honestly: this cannot deterministically FAIL for a raw-pointer key --
// it depends on the allocator reusing an address. It exercises the pattern,
// churns allocations to make reuse likely, and the actual protection is the
// key type, not this test.
TEST(RewriteEngineTest, SurvivesRewriteOfTemporaryExpression)
{
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});

    for (int iteration = 0; iteration < 200; ++iteration)
    {
        const Expression q = kFactory.symbol("q" + std::to_string(iteration));
        const RewriteResult result = session.rewrite(
            kFactory.add(kFactory.subtract(q, q), kFactory.constant(iteration)));

        ASSERT_TRUE(ik::isConstant(result.expression)) << iteration;
        ASSERT_DOUBLE_EQ(ik::constantValue(result.expression), iteration) << iteration;
    }
}

// A call stopped by the limit must leave NOTHING in the session: the next root
// has to consult the rules again rather than pick up an unfinished result.
TEST(RewriteEngineTest, DoesNotReuseIncompleteMemoAfterStepLimit)
{
    const Expression q = kFactory.symbol("q");
    const Expression r = kFactory.symbol("r");
    const Expression shared = kFactory.subtract(q, q);

    resetCounters();
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)}, 1);

    const RewriteResult interrupted =
        session.rewrite(kFactory.add(shared, kFactory.subtract(r, r)));
    ASSERT_TRUE(interrupted.reachedLimit);

    const std::size_t afterInterrupted = g_subtractionConsultations;

    RewriteResult second = session.rewrite(shared);

    EXPECT_GT(g_subtractionConsultations, afterInterrupted);
    EXPECT_TRUE(ik::isZero(second.expression));
    EXPECT_EQ(second.steps, 1u);
}

TEST(RewriteEngineTest, CommitsMemoOnlyAfterReachingFixedPoint)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);

    resetCounters();
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});

    const RewriteResult complete = session.rewrite(kFactory.add(shared, kFactory.symbol("a")));
    ASSERT_FALSE(complete.reachedLimit);

    const std::size_t afterComplete = g_subtractionConsultations;
    const RewriteResult second = session.rewrite(shared);

    EXPECT_EQ(g_subtractionConsultations, afterComplete);
    EXPECT_EQ(second.steps, 0u);
}

// The session copies its rules. Overwriting the caller's array afterwards must
// not change what the session does -- with a span it would.
TEST(RewriteEngineTest, OutlivesTheRuleArrayItWasBuiltFrom)
{
    const Expression q = kFactory.symbol("q");

    std::vector<RewriteRule> rules{rule("x-x", selfSubtraction)};
    RewriteSession session{rules, kFactory};

    rules[0] = rule("-(-x)", doubleNegation);
    rules.clear();

    const RewriteResult result = session.rewrite(kFactory.subtract(q, q));

    EXPECT_TRUE(ik::isZero(result.expression));
    EXPECT_EQ(result.steps, 1u);
}
```

### 6.6 Równoważność z dziedziną — 3

```cpp
// Definedness, not just value -- and with the pole IN the point set by name.
TEST(RewriteEngineTest, AgreesOnDefinednessAndValue)
{
    const Expression q1 = kFactory.symbol("q1");
    const Expression denominator = kFactory.subtract(q1, kFactory.constant(0.5));
    const Expression original = kFactory.multiply(kFactory.divide(kFactory.symbol("a"), denominator),
                                                  kFactory.constant(0.0));

    RewriteSession session = sessionOf({rule("x*0", multiplyByZero)});
    const RewriteResult result = session.rewrite(original);

    // Deterministic boundary points first, then a spread. 0.5 EXACTLY is the
    // only point at which the rejected rewrite would differ, so it is not left
    // to sampling.
    const std::array<double, 9> points{0.5,   0.5 - 1e-9, 0.5 + 1e-9, 0.5 - 1e-6, 0.5 + 1e-6,
                                       -2.0,  0.0,        1.25,       3.5};

    for (const double point : points)
    {
        ik::ExpressionEvaluator before{{{"q1", point}, {"a", 2.0}}};
        ik::ExpressionEvaluator after{{{"q1", point}, {"a", 2.0}}};

        const auto originalValue = before.evaluate(original);
        const auto rewrittenValue = after.evaluate(result.expression);

        ASSERT_EQ(originalValue.has_value(), rewrittenValue.has_value()) << point;
        if (originalValue.has_value())
            EXPECT_NEAR(*originalValue, *rewrittenValue, 1e-12) << point;
    }

    // ...and the point set has teeth: at the pole the REJECTED replacement
    // would have disagreed. Without this the test above could pass for an
    // engine that accepted the unsafe rewrite.
    ik::ExpressionEvaluator atPole{{{"q1", 0.5}, {"a", 2.0}}};
    ik::ExpressionEvaluator atPoleFolded{{{"q1", 0.5}, {"a", 2.0}}};
    EXPECT_FALSE(atPole.evaluate(original).has_value());
    EXPECT_TRUE(atPoleFolded.evaluate(kFactory.constant(0.0)).has_value());
}

// The other half: an ACCEPTED rewrite must preserve definedness too. Guards go
// from { -(-q) } to { q } -- different sets, same domain -- which is exactly the
// case a global comparison of divisors would have rejected.
TEST(RewriteEngineTest, AgreesOnDefinednessAcrossAnAcceptedRewrite)
{
    const Expression a = kFactory.symbol("a");
    const Expression q = kFactory.symbol("q");
    const Expression awkward = kFactory.divide(a, kFactory.negate(kFactory.negate(q)));

    RewriteSession session = sessionOf({rule("-(-x)", doubleNegation)});
    const RewriteResult result = session.rewrite(awkward);

    ASSERT_TRUE(result.changed);
    ASSERT_TRUE(structurallyEqual(result.expression, kFactory.divide(a, q)));

    // Zero EXACTLY, then either side of it: the pole is where a widened domain
    // would show, and it is in the list by name rather than by sampling.
    const std::array<double, 5> points{0.0, -1e-9, 1e-9, -1.0, 1.0};

    for (const double point : points)
    {
        ik::ExpressionEvaluator before{{{"q", point}, {"a", 3.0}}};
        ik::ExpressionEvaluator after{{{"q", point}, {"a", 3.0}}};

        const auto originalValue = before.evaluate(awkward);
        const auto rewrittenValue = after.evaluate(result.expression);

        ASSERT_EQ(originalValue.has_value(), rewrittenValue.has_value()) << point;
        if (originalValue.has_value())
            EXPECT_NEAR(*originalValue, *rewrittenValue, 1e-12) << point;
    }
}

// End-to-end on the real 21 882-node DAG: each cell wrapped in a double
// negation the engine must peel off, returning the ORIGINAL nodes.
//
// The rule fires exactly at the twelve wrapped roots and nowhere else, which
// is a measured fact and not an assumption: both FK DAGs contain six Negate
// nodes each and NOT ONE Negate(Negate(...)). Four cells per robot are
// constants -- the [0 0 0 1] row -- and those fold at build time.
TEST(RewriteEngineTest, RewritesRealForwardKinematicsAndKeepsItsNodes)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / "kr640.urdf");

    ik::KinematicChainBuilder chainBuilder;
    const auto chain = chainBuilder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(chain.has_value());

    const ik::JointTransformBuilder transformBuilder;
    const ik::ForwardKinematicsBuilder fkBuilder;
    const ik::SymbolicTransform fk = fkBuilder.build(*chain, transformBuilder);

    RewriteSession session = sessionOf({rule("-(-x)", doubleNegation)});

    for (std::size_t row = 0; row < ik::SymbolicTransform::rows; ++row)
        for (std::size_t column = 0; column < ik::SymbolicTransform::columns; ++column)
        {
            const Expression& cell = fk(row, column);
            const RewriteResult result =
                session.rewrite(kFactory.negate(kFactory.negate(cell)));

            EXPECT_TRUE(structurallyEqual(result.expression, cell)) << row << ',' << column;

            // Constant cells fold at build time -- negate(constant) collapses --
            // so the wrap never existed and the identity claim does not apply.
            // Everywhere else the engine must hand back the very node it was
            // given, not a rebuilt copy of it.
            if (!ik::isConstant(cell))
                EXPECT_TRUE(sameNode(result.expression, cell)) << row << ',' << column;
        }
}
```

### 6.7 Jedna uwaga z review, która się nie potwierdziła

Review przewidywało, że nieużywana zmienna `first` w
`SharesMemoAcrossRootsInOneSession` wywali `build-werror` na
`-Werror=unused-but-set-variable`, więc deklarowany gate „w tej postaci nie może
przejść". Sprawdziłem to, zanim zmieniłem test, bo od tego zależy, czy gate z v1
był zmyślony:

```text
g++ -std=c++23 -Wall -Wextra -Wswitch -Werror

const unsigned long n = g + 1;   // skalar        -> error: unused variable 'n'
const Result first = make();     // typ klasowy   -> BEZ ostrzeżenia
```

GCC nie ostrzega o nieużywanej zmiennej typu z nietrywialnym konstruktorem
i destruktorem — a `RewriteResult` trzyma `Expression`, czyli `shared_ptr`.
`first` nie zatrzymałoby więc builda i gate `448/448` z v1 był osiągalny.

Zmianę i tak wprowadzam, bo proponowana wersja — sprawdzenie `reachedLimit`,
`changed` i `steps` — czyni test **mocniejszym**, a to jest lepszy powód niż
nieistniejące ostrzeżenie. Zapisuję to jednak wprost, bo przy następnym takim
przewidywaniu warto pamiętać, że ta konkretna diagnostyka nie dotyczy typów
klasowych.

### 6.8 Uczciwie o dwóch testach

`SurvivesRewriteOfTemporaryExpression` **nie potrafi zawieść deterministycznie**
przy kluczu na surowym wskaźniku — zależy od tego, czy allocator odda ten sam
adres. Testuje wzorzec i zwiększa szansę, ale prawdziwym zabezpieczeniem jest
typ klucza, nie ten test. To samo zastrzeżenie stoi w komentarzu przy nim.

Test na FK stoi na pomiarze, nie na założeniu. Zanim powstał, sprawdziłem, czy
reguła nie zadziała też **w środku** FK — wtedy asercja tożsamości byłaby fałszywa,
a test wyglądałby na zły silnik:

```text
kr640.urdf       unique=281  Negate=6  Negate(Negate)=0  komorki stale=4
kr4_r600.urdf    unique=516  Negate=6  Negate(Negate)=0  komorki stale=4
```

Zero podwójnych negacji w obu robotach, więc reguła trafia dokładnie dwanaście
razy — w opakowane korzenie komórek niestałych — i w żadne poddrzewo. Cztery
komórki stale to wiersz `[0 0 0 1]`.

**Dwa testy zamiast jednego — poprawka z review.** v1 miała jeden test o nazwie
`DiscardsUnsafeRuleAndContinues`, który dowodził czegoś innego, niż obiecywał.
Odrzucona reguła siedziała przy `Multiply`, przyjęta przy `Subtract`, więc test
mówił „traversal nie zatrzymuje się po odrzuceniu" — a architektura twierdzi coś
mocniejszego: **kolejna reguła dostaje swóją kolej na tym samym węźle**.

Napisałem wtedy, że wersja jednowęzłowa byłaby „tak sztuczna, że przestałaby
cokolwiek mówić o silniku". To było wygodne i nieprawdziwe — wystarczyła druga
reguła, która **zachowuje** dziedzinę tam, gdzie pierwsza ją rozszerza:

```text
(1/q) * 0

R1:  x * 0 -> 0        guardy {q} -> {}      ODRZUCONA
R2:  (a/b)*0 -> 0/b    guardy {q} -> {q}     PRZYJĘTA,  miara 5 -> 3
```

Obie pasują do tego samego węzła, obie są poprawne algebraicznie tam, gdzie
wejście jest określone, i różni je dokładnie to, o co chodzi w 2.3. Testy są
teraz dwa: `ContinuesWithNextRuleAfterDomainRejection` (jeden węzeł, dwie reguły)
i `RejectedRuleDoesNotStopTheTraversal` (dwa węzły).

Nie ma testu na **iteracyjność** pętli po zastosowaniu reguły. Żeby wersja
rekurencyjna przewróciła się na stosie, potrzeba dziesiątek tysięcy trafień z
rzędu, a taki test kosztuje sekundy i zależy od rozmiaru stosu na maszynie CI.
Wymóg jest strukturalny i pilnowany przy review kodu.

Nie ma też testu na **determinizm reguły**. Nie da się go napisać: silnik nie
odróżnia reguły nieczystej od czystej, a test, który by próbował, sprawdzałby
własną regułę testową. Tak samo jak (E) — kontrakt, nie asercja.

---

## 7. CMake

```cmake
# CMakeLists.txt, do listy źródeł kinemaforge_ik
    src/ik_equations/symbolic/RewriteEngine.cpp

# tests/CMakeLists.txt, do listy źródeł kinemaforge_tests
    test_rewrite_engine.cpp
```

---

## 8. Gate

```text
przed    422 / 422
nowe     +28
po       450 / 450
```

Architektura wymienia 26 testów. Różnica to dwa dodane w tym review —
`ContinuesWithNextRuleAfterDomainRejection` i
`AgreesOnDefinednessAcrossAnAcceptedRewrite` — a nie zmiana zakresu etapu.

W **obu** konfiguracjach: `build/` (Release, `NDEBUG`, asserty martwe) oraz
`build-werror/` (`-Wall -Wextra -Wswitch -Werror`, asserty żywe). Ta druga jest
tu ważniejsza niż zwykle: kontrakt miary i zakaz zwracania wejścia są
sprawdzane **wyłącznie** tam.

---

## 9. Ryzyka implementacyjne

| Ryzyko | Reakcja |
|---|---|
| `changed` liczone po `std::move(result)` | Policzone do lokalnej zmiennej przed konstrukcją wyniku; komentarz mówi dlaczego (5) |
| Referencja z `collectGuards` unieważniona przez rekurencyjne wstawienie | `unordered_map` jest węzłowy — rehash unieważnia iteratory, nie referencje; zapisane w komentarzu |
| Wpis do memo lokalnego przy przerwanym wywołaniu | Oba wyjścia awaryjne to `return`, nie `break`; `emplace` jest za pętlą (5.1) |
| `measureOf` liczone od zera przy każdym trafieniu reguły | Memoizacja po tożsamości węzła; całość i tak tylko pod `assert` |
| Dedup strażników po tożsamości zamiast strukturalnie | Porównanie „rozmiar + zawieranie" byłoby wtedy niepoprawne; dedup strukturalny, uzasadniony w komentarzu (3) |
| Duplikat funktorów tożsamości wobec `ExpressionEvaluator` | Świadomy: wspólny nagłówek to zmiana pliku spoza zakresu etapu; zapisane w `STATUS.md` jako znany brak (2.2) |
| Nowy typ węzła przechodzi przez analizator dziedziny w milczeniu | Visitor wyliczający, `static_assert` w gałęzi domyślnej; jedyne miejsce, gdzie pomyłka nie daje ani błędu kompilacji, ani złego wyniku, tylko cichą utratę bezpieczeństwa (3.1) |
| Reguła decyduje na podstawie `sameNode` zamiast struktury | Kontrakt determinizmu mówi o **znaczeniu** wejścia; `sameNode` tylko jako fast-path (1.3) |
| Test FK zawodzi na komórkach stałych | `negate(constant)` zwija się przy budowie, więc opakowanie nie istnieje; asercja tożsamości ograniczona do komórek niestałych, z komentarzem (6.6) |

---

## 10. Rekomendacja

Przyjąć w tym brzmieniu, zaimplementować, uruchomić gate `450/450` w obu
konfiguracjach, zaktualizować `STATUS.md`, `KATALOG.md` i roadmapę, przenieść
oba dokumenty F3.1 do `proposal/done/`, zrobić commit.

Potem **F3.2** — pierwszy prawdziwy zestaw reguł. To tam po raz pierwszy ktoś
będzie musiał dotrzymać (E), tam okaże się, czy representability guard był
potrzebny, i tam kolejność reguł przestanie być tematem teoretycznym.
