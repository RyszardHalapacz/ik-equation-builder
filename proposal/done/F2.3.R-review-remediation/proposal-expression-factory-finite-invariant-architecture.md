# Proposal: inwariant skończoności w `ExpressionFactory` (R5) — architektura

**Etap:** F2.3.R, dokument **R5**, część 1 z 2 (architektura + implementacja) ·
**waga:** MEDIUM · **v2 — do review** · 2026-08-18

> **v2, po review.** Cztery wymagane poprawki: (1) precyzyjnie nazwany
> inwariant — własność domknięcia fabryki, a nie runtime enforcement przeciw
> `constant(inf)`, nowa sekcja 1.1; (2) `<optional>` przeniesione do nagłówka
> (4.1, 8); (3) usunięty `NeutralElementSurvivesDeclinedFold` jako przypadek
> logicznie pusty (7.1); (4) osłabiony claim o `AllConstantLeavesRemainFinite`
> — smoke test aktualnego pipeline'u FK, nie dowód globalny (7.2, 9). Sam
> algorytm zatwierdzony bez zmian. Nowa liczba testów: **8**, gate 320/320.

## Prompt

> Review kodu, uwaga 7:
> `ExpressionFactory` obiecuje, że stałe są skończone (`assert` w `constant()`),
> ale constant folding może samo wyprodukować `inf` z dwóch legalnych operandów.
> W Release `assert` znika i `ConstantNode{inf}` powstaje.
>
> Rekomendacja review: foldować tylko wtedy, gdy wynik jest skończony; w
> przeciwnym razie zbudować normalny węzeł operacji i pozwolić
> `ExpressionEvaluator` zwrócić `NonFiniteResult` — czyli wykorzystać istniejący
> model błędów zamiast dodawać nowy.

**Zakres szerszy niż w review**, ustalone w roadmapie: dotyczy `add`,
`subtract`, `multiply` **i** `divide`, nie tylko `multiply`. `divide` ma już
`assert(!isZero(rhs))`, ale `1e308 / 1e-308` też daje `inf`. Poprawka musi być
jednolita dla wszystkich czterech, inaczej inwariant zostaje połowiczny.

R5 nie dotyka loadera, więc mógł iść równolegle do R2–R4. Idzie po nich, bo
R2 i R4 były na ścieżce krytycznej merge'a.

## Status weryfikacji

### Odtworzone eksperymentalnie, nie z lektury

Sonda skompilowana **z flagami Release** (`-O2 -DNDEBUG`, czyli dokładnie tak jak
`build/`), linkowana z `libkinemaforge_ik.a`:

```text
build: NDEBUG (Release) -- asserts compiled out

multiply(1e308, 1e308)       isConstant=1  value=inf   finite=0
add(1e308, 1e308)            isConstant=1  value=inf   finite=0
subtract(-1e308, 1e308)      isConstant=1  value=-inf  finite=0
divide(1e308, 1e-308)        isConstant=1  value=inf   finite=0
multiply(1e-308, 1e-308)     isConstant=1  value=0     finite=1
```

Cztery pierwsze wiersze to uwaga 7, potwierdzona dla **wszystkich czterech**
operacji binarnych — nie tylko dla `multiply`, jak zgłaszało review.

Ostatni wiersz jest równie ważny: **underflow zwija się do zera i tak ma
zostać.** `0` jest skończone, jest legalną stałą i trafia do współdzielonego
węzła zerowego. R5 nie może go przypadkiem objąć zakazem.

### Czego sonda **nie** pokazuje

Nie ma dziś ścieżki z prawdziwego URDF do tego stanu. Wartości w plikach
`data/urdf` to metry i radiany, a `JointTransformBuilder` mnoży je przez
składowe osi jednostkowej — nic tam nie zbliża się do `1e308`. Dlatego uwaga 7
jest MEDIUM, a nie HIGH: **osiągalna z ręcznie budowanych wyrażeń i z każdego
przyszłego producenta wyrażeń, nieosiągalna z dzisiejszego wejścia.**

To jest argument za naprawą teraz, a nie przeciw: `ExpressionFactory` jest
fundamentem, na którym stanie symplifikator i solver, i to one będą produkować
wyrażenia o wartościach, których nikt nie oglądał w pliku URDF.

## 1. Cel i zakres

**W zakresie:**

- ścieżki constant foldingu w `add`, `subtract`, `multiply`, `divide`,
- treść kontraktu `constant()` i komentarza klasy — dziś obiecują coś, czego kod
  nie dotrzymuje.

**Poza zakresem:**

- `sin`, `cos`, `negate` — patrz 2.3, nie mogą wyprodukować wartości
  nieskończonej ze skończonego operandu,
- `x * 0 → 0` — świadomie niezwijane, decyzja z warstwy symbolicznej stoi,
- model błędów `ExpressionEvaluator` — R5 celowo **nie dodaje** żadnego kodu
  błędu, patrz 6,
- przyszły `EquationSimplifier` — R5 jest warunkiem, żeby mógł ufać inwariantowi,
  ale sam go nie dotyczy.

### 1.1 Jaki inwariant R5 ustanawia — i jaki nie

> **Dodane w v2.** v1 mówiła miejscami tak, jakby po R5 obowiązywało
> bezwarunkowo „każdy `ConstantNode` w dowolnym `Expression` jest skończony".
> To nieprawda i tego proposal nie obiecuje.

**Czego R5 NIE ustanawia.** `constant(double)` zostaje publiczne i chronione
wyłącznie asercją, więc w Release nadal działa:

```cpp
factory.constant(std::numeric_limits<double>::infinity());   // -> ConstantNode{inf}
```

Zmiana tego wymagałaby `std::expected` albo wyjątku w publicznym API fabryki —
warianty B i C, odrzucone w 3.1 i 3.2. R5 **nie jest** walidacją nieufnego
wejścia runtime.

**Co R5 ustanawia — własność domknięcia:**

```text
skończone ConstantNode na wejściu
        +
legalna operacja fabryki
        ↓
każdy NOWO utworzony ConstantNode też skończony
```

Sformułowane kontraktowo: **`ExpressionFactory` zachowuje precondition
skończonych stałych podczas własnych transformacji** — dla wywołań spełniających
publiczne preconditions. Fabryka przestaje łamać własny kontrakt **sama, bez
udziału wywołującego**, i to była treść uwagi 7.

To jest design by contract, spójny z tym, co już dziś stoi w `Expression.hpp:47`:

```cpp
// Private on purpose: every composite node goes through
// ExpressionFactory, so the contracts it enforces (finite constants,
// non-empty symbol names, no division by literal zero) hold for the
// whole graph rather than only where callers remembered to use it.
```

Ten komentarz też opisuje kontrakt, nie walidację. Różnica przed R5 była taka,
że jedno ogniwo tego łańcucha — folding — łamało kontrakt bez niczyjego błędu.
Po R5 łańcuch jest spójny: kontrakt może naruszyć **tylko wywołujący**, i to
jest dokładnie to, o czym mówi `assert`.

Wszystkie sformułowania w tym dokumencie należy czytać w tym znaczeniu.
„Inwariant zamknięty" znaczy „fabryka jest względem niego domknięta", nie
„fabryka go egzekwuje przeciwko dowolnemu wejściu".

## 2. Na czym dokładnie polega błąd

### 2.1 Inwariant, którego pilnuje wyłącznie `assert`

`constant()` jest jedyną bramą do `ConstantNode`, i dziś brzmi tak:

```cpp
Expression ExpressionFactory::constant(double value) const
{
    assert(std::isfinite(value) && "constants must be finite");
    if (value == 0.0)
        return Expression{};              // shared zero node
    return Expression{ExpressionNode{ConstantNode{value}}};
}
```

Komentarz nad metodą mówi wprost, że to zabezpieczenie obejmuje też folding:

> „Folded results route back through this method, so an overflow in constant
> folding trips the same assert instead of quietly producing Inf."

Zdanie jest prawdziwe **tylko w buildzie z asercjami**. `build/` jest Release
(`-DNDEBUG`), więc `assert` znika i zostaje dokładnie to, czemu miał zapobiec:
cicho wyprodukowany `Inf`. Sonda z „Statusu weryfikacji" pokazuje to na czterech
operacjach.

### 2.2 Dlaczego `assert` jest tu **złym narzędziem**, nie tylko wyłączonym

To jest sedno tego proposala i powód, dla którego wariant „włączmy asercje w
Release" nie jest odpowiedzią.

`assert` wyraża **defekt programisty**: warunek, który przy poprawnym kodzie nie
może zajść. Tak jest, gdy wywołujący podaje `inf` do `constant()` — to błąd w
kodzie wywołującego. Ale overflow w foldingu jest czym innym:

```text
constant(1e308)    legalne wejście, skończone
constant(1e308)    legalne wejście, skończone
multiply(...)      legalna operacja
      →            inf
```

Żaden uczestnik tej sekwencji nie zrobił nic błędnego. To **właściwość
arytmetyki zmiennoprzecinkowej**, a nie pomyłka. Reagowanie na nią `assert`-em
oznacza, że poprawny program przerywa działanie — albo, po skompilowaniu z
`NDEBUG`, że nie reaguje wcale. Oba wyjścia są złe, i to jest powód, dla którego
naprawa musi być zmianą **zachowania**, a nie wzmocnieniem asercji.

### 2.3 Dlaczego `sin`, `cos` i `negate` są poza zakresem

Nie jest to przeoczenie, tylko własność tych funkcji:

```text
negate(x)   |x| skończone  →  |-x| skończone         (ten sam moduł)
sin(x)      x skończone    →  |sin(x)| <= 1
cos(x)      x skończone    →  |cos(x)| <= 1
```

Operand jest skończony na mocy preconditions (1.1), a nie na mocy sprawdzenia w
kodzie. `std::sin(inf)` daje `NaN`, ale żeby tam trafić, trzeba by już mieć w
drzewie stałą nieskończoną — czyli albo naruszone precondition `constant()`,
albo folding łamiący kontrakt. To drugie R5 usuwa; pierwsze zostaje po stronie
wywołującego, świadomie.

Wniosek: **wystarczy uszczelnić cztery operacje binarne**, żeby fabryka była
domknięta względem inwariantu — przy wywołaniach spełniających preconditions.
Rozszerzanie `foldFinite` na `sin`/`cos`/`negate` „dla symetrii" dołożyłoby kod
bez osiągalnego przypadku.

### 2.4 Co konkretnie psuje `ConstantNode{inf}` w drzewie

Nie chodzi o estetykę. Trzy konsekwencje:

1. **Kolejne foldy kłamią dalej.** `inf` jest `isConstant`, więc bierze udział w
   następnym foldzie: `subtract(inf, inf)` → `NaN`, i to `NaN` też wchodzi do
   drzewa jako stała.
2. **`isZero`/`isOne` przestają być pełnym opisem stałych.** Neutralne elementy
   nadal działają poprawnie, ale przyszły symplifikator, który zacznie rozumować
   o wartościach stałych (porównania, dziedziny, dzielenie przez wyrażenie), robi
   to na drzewie, o którym komentarz klasy zapewnia, że jest skończone.
3. **Struktura zaczyna zależeć od buildu.** W buildzie z asercjami program
   przerywa działanie, w Release buduje węzeł. To samo wejście, dwa różne
   zachowania, i różnicy nie widać w testach — bo testy chodzą w obu
   konfiguracjach i w obu dziś przechodzą.

Punkt 3 jest tym, co czyni to problemem **teraz**, a nie „kiedy pojawi się
symplifikator": to jest ta sama klasa błędu co uwaga 9 z R1 — komentarz
obiecujący ochronę, której kod nie realizuje.

## 3. Warianty

| # | Wariant | Ocena |
|---|---|---|
| **A** | Foldować tylko wtedy, gdy wynik jest skończony; inaczej zbudować normalny węzeł operacji | **Wybrany** (rekomendacja review) |
| B | `ExpressionFactory` zwraca `std::expected<Expression, ...>` | Odrzucony — 3.1 |
| C | Rzucać wyjątek przy niefoldowalnym wyniku | Odrzucony — 3.2 |
| D | Zostawić `assert` i włączyć asercje w Release | Odrzucony — 3.3 |

### 3.1 Dlaczego nie B (`std::expected` z fabryki)

Sygnatura `Expression add(Expression, Expression)` jest używana w każdym
budowniku i w `multiply()` dla macierzy. Zamiana na `std::expected` kaskaduje
na `JointTransformBuilder`, `RigidTransformConstruction`, `SymbolicMatrix`,
`ForwardKinematicsBuilder` — każde mnożenie macierzy 4×4 stałoby się ciągiem
sprawdzeń błędu, dla sytuacji, która przy prawdziwym URDF nie zachodzi.

Co gorsza, **byłby to zły model**: overflow w foldzie nie oznacza, że wyrażenia
nie da się zbudować. Da się — po prostu nie da się go w tym miejscu uprościć.
Błąd pojawia się dopiero przy **ewaluacji**, i tam już jest reprezentowany.

### 3.2 Dlaczego nie C (wyjątek)

Ten sam argument co wyżej, plus: warstwa symboliczna nie rzuca dziś niczego, a
`IkEquationBuilder` łapie `std::runtime_error` z zupełnie innego powodu
(loader). Wyjątek z fabryki wyrażeń wjechałby w ten sam `catch` i zostałby
zaraportowany jako zły URDF.

### 3.3 Dlaczego nie D (asercje w Release)

Nie usuwa problemu, tylko zamienia cichy `inf` na `abort()` w produkcyjnym
buildzie — patrz 2.2: to nie jest defekt programisty, więc nie zasługuje na
przerwanie programu. Osobno: czy `build/` ma zostać Release, skoro kontrakty
warstwy budowniczej opierają się na asercjach, jest **otwartym pytaniem z R1** i
zostaje otwarte. R5 celowo nie jest od niego zależne — poprawka ma działać w
każdej konfiguracji.

## 4. Proponowana zmiana

### 4.1 Prywatny helper

`std::optional` pojawia się w **sygnaturze w nagłówku**, więc `<optional>` należy
do `ExpressionFactory.hpp`, nie do `.cpp`:

```cpp
#include "ik_equations/symbolic/Expression.hpp"

#include <optional>
#include <string>
```

> **v2, po review.** v1 umieszczała `<optional>` w planie zmian po stronie
> `.cpp`. To błąd tej samej klasy co brakujący `<utility>` w R4: nagłówek musi
> sam deklarować wszystko, czego używa w swoim interfejsie, a nie liczyć na
> include tranzytywny.

```cpp
private:
    // Constant folding, but only when the arithmetic stayed finite.
    //
    // Overflow in a fold is not a caller error: 1e308 and 1e308 are both
    // legitimate constants and multiply() is a legitimate operation, yet the
    // product is not representable. Folding it would put a non-finite leaf in
    // the tree, which the class comment promises cannot happen -- and which the
    // next fold would then propagate (inf - inf is NaN).
    //
    // Declining to fold is not a loss: the operation node is built instead, and
    // ExpressionEvaluator reports NonFiniteResult when it is evaluated. The
    // subtree keeps saying what it is rather than pretending to be a number.
    [[nodiscard]] std::optional<Expression> foldFinite(double value) const;
```

```cpp
std::optional<Expression> ExpressionFactory::foldFinite(double value) const
{
    if (!std::isfinite(value))
        return std::nullopt;
    return constant(value);
}
```

Helper, a nie czterokrotnie powtórzony `if`, z jednego powodu: uzasadnienie ma
istnieć w jednym miejscu. Czterokrotnie skopiowany warunek to czterokrotna
okazja, żeby jeden z nich został przy następnej zmianie pominięty — dokładnie ta
klasa błędu, którą R3 usuwało z parsowania limitów.

### 4.2 Cztery operacje binarne

```cpp
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
```

Zmiana jest w każdym przypadku ta sama: fold **przestaje być bezwarunkowy** i
staje się próbą. Gdy próba się nie powiedzie, sterowanie leci dalej tą samą
ścieżką, którą leci dla operandów niestałych.

Kolejność sprawdzeń zostaje bez zmian i to jest istotne: neutralne elementy są
sprawdzane **po** foldzie. Gdy fold odmówi, `isZero`/`isOne` nadal mają szansę
zadziałać — ale nie mogą wtedy zmienić wyniku, bo jeśli jeden z dwóch stałych
operandów jest zerem lub jedynką, żadna z czterech operacji nie przepełni się.
Sprawdzenia zostają dokładnie tam, gdzie są, żeby nie wprowadzać różnicy między
ścieżką „obie stałe" a „jedna stała".

### 4.3 `assert` w `constant()` **zostaje**

Po zmianie z 4.2 jest to znowu to, czym `assert` ma być: warunek wstępny, który
może naruszyć **wyłącznie kod wywołujący**, bo folding już przez niego
nieskończoności nie przepuszcza. Sprawdzone dzisiejsze wywołania spoza fabryki:

```text
JointTransformBuilder.cpp     constant(1.0), constant(unitAxis.x/y/z),
                              constant(component), constant(0.0)
RigidTransformConstruction    constant(rpy.x/y/z), constant(value.x/y/z)
SymbolicMatrix.hpp            constant(1.0)
```

Wszystkie biorą albo literał, albo wartość, której skończoność **gwarantuje
loader** (`RobotDescription` ma kanoniczną geometrię: skończone origin i rpy,
oś znormalizowana). Czyli `assert` opisuje realny kontrakt na realnej granicy.

Zostaje przy tym świadoma asymetria: `constant(inf)` w Release nadal przejdzie i
utworzy `ConstantNode{inf}`. **To nie jest luka R5, tylko granica kontraktu** —
patrz 1.1. R5 odpowiada za to, żeby fabryka nie produkowała takiego węzła sama
z legalnych wejść; za jawne podanie `inf` odpowiada wywołujący.

### 4.4 Komentarze do poprawienia

Nad `constant()` znika zdanie, które przestaje być prawdą:

```text
USUNĄĆ:  „Folded results route back through this method, so an overflow in
          constant folding trips the same assert instead of quietly producing
          Inf."

WSTAWIĆ: „Constant folding never routes a non-finite result here: an overflow
          declines to fold and builds the operation node instead, so this
          precondition can only be violated by a caller."
```

W komentarzu klasy, przy opisie constant foldingu, dochodzi jedno zdanie:
fold zachodzi tylko wtedy, gdy wynik jest skończony, a niefoldowalne działanie
zostaje węzłem operacji i jest zgłaszane dopiero przy ewaluacji.

### 4.5 `divide()` — `assert` zostaje, i co się dzieje przy jego złamaniu

> **Dodane w v2**, żeby zapisać rozróżnienie ustalone w review.

`assert(!isZero(rhs))` zostaje bez zmian. Dla programu spełniającego
preconditions dzielenie ma dwa wyjścia i oba są obsłużone: wynik skończony →
fold, wynik nieskończony → węzeł operacji.

Przy **złamanym** preconditionie zachowanie różni się między buildami:

```text
divide(constant(0), constant(0))

  Debug     assert -> abort
  Release   foldFinite(NaN) odmawia foldu -> DivideNode
            -> ewaluacja -> DivisionByZero (evaluator ma osobny kod)
```

Ta różnica jest dopuszczalna i jest **fundamentalnie innym przypadkiem niż uwaga
7**: tam różniło się zachowanie dla wywołania w pełni legalnego
(`multiply(constant(1e308), constant(1e308))`), tu — dla wywołania, które łamie
udokumentowany warunek wstępny. Pierwsze jest błędem projektu, drugie jest
normalnym znaczeniem `assert`.

## 5. Co się dzieje z niezwiniętym węzłem

To jest jedyna obserwowalna zmiana zachowania, więc warto ją nazwać wprost.

```text
przed:  multiply(constant(1e308), constant(1e308))  →  ConstantNode{inf}
po:     multiply(constant(1e308), constant(1e308))  →  MultiplyNode{
                                                          ConstantNode{1e308},
                                                          ConstantNode{1e308} }
```

Konsekwencje:

- `isConstant()` na tym węźle daje **`false`**, więc otaczające foldy się na nim
  zatrzymują. To jest poprawne, nie szkodliwe: poddrzewo, którego nie da się
  policzyć skończenie, **nie ma prawa udawać liczby**.
- Drzewo jest o dwa węzły większe. Dla realnych robotów: bez zmian, bo ta
  ścieżka nie zachodzi (2.1 „czego sonda nie pokazuje").
- Ewaluacja tego poddrzewa zwraca `NonFiniteResult` — bez żadnej nowej linijki
  kodu, patrz 6.

## 6. Model błędów — bez zmian

`ExpressionEvaluator` sprawdza skończoność **każdego węzła, liście włącznie**
(`ExpressionEvaluator.cpp:120`), z komentarzem, który już dziś przewiduje ten
scenariusz:

```cpp
// Checked for every node, leaves included. ExpressionFactory asserts that
// constants are finite, but that assert is gone under NDEBUG, and an
// overflow such as q1 * q2 with both at 1e308 is reachable in any build.
if (!std::isfinite(*result))
    return makeError(EvaluationErrorCode::NonFiniteResult);
```

Czyli mechanizm zgłaszania **już istnieje i jest przetestowany**. R5 dokłada do
niego jedynie nową drogę dojścia. Żadnego nowego `EvaluationErrorCode`, żadnej
zmiany w `ExpressionEvaluator` — to jest dokładnie to, o co prosiło review
(„wykorzystaj istniejący model błędów zamiast dodawać nowy").

Drobiazg do odnotowania przy implementacji: powyższy komentarz zawiera zdanie
„`ExpressionFactory` asserts that constants are finite, but that assert is gone
under NDEBUG" — po R5 pierwsza część zostaje prawdziwa, druga przestaje mieć
znaczenie dla foldingu. Komentarz warto doprecyzować, żeby nie sugerował, że to
evaluator jest ostatnią linią obrony przed niefoldowalnymi stałymi.

## 7. Testy — **osiem**, nie dziewięć

```text
z roadmapy
  DoesNotFoldOverflowingSum                    add(1e308, 1e308)      -> AddNode
  DoesNotFoldOverflowingDifference             subtract(-1e308, 1e308)-> SubtractNode
  DoesNotFoldOverflowingProduct                multiply(1e308, 1e308) -> MultiplyNode
  DoesNotFoldOverflowingQuotient               divide(1e308, 1e-308)  -> DivideNode
  OverflowingFoldEvaluatesToNonFiniteResult    ewaluacja -> NonFiniteResult
  AllConstantLeavesRemainFinite                obchód DAG-u FK dla obu robotów

dołożone w tym proposalu
  StillFoldsWhenResultIsFinite                 regresja: 2+3 nadal -> ConstantNode{5}
  FoldsUnderflowingProductToZero               1e-308 * 1e-308 -> zero, NIE węzeł
```

Testy sprawdzają **strukturę**, nie tylko wartość: `isConstant(result) == false`
plus rozpoznanie typu węzła. Sprawdzanie samej wartości nie odróżniłoby
poprawki od stanu sprzed niej, bo `constantValue` na `inf` nadal zwraca `inf`.

### 7.1 Usunięty: `NeutralElementSurvivesDeclinedFold`

> **v2, po review.** v1 proponowała ten test dla „kolejności sprawdzeń z 4.2".
> Nie ma dla niego przypadku testowego, i wynika to wprost z akapitu w 4.2,
> który sam to zauważa.

Test miałby pinować sekwencję: obie strony stałe → fold odrzucony z powodu
nieskończoności → **następnie** działa neutralny element. Ta sekwencja nie
istnieje:

```text
a + 0     a - 0     a * 1     a / 1     dla skończonego a nie przepełnia się
```

Czyli fold nigdy nie zostaje odrzucony w przypadku, w którym neutralny element
miałby coś do zrobienia. A gdy jeden operand jest symboliczny — `add(x, 0)` —
fold w ogóle nie wchodzi w grę (`isConstant(lhs) && isConstant(rhs)` jest
`false`), więc to zwykła ścieżka neutralnego elementu, pinowana już dziś przez
`DropsAdditiveIdentity`, `DropsSubtractiveIdentity`,
`DropsMultiplicativeIdentity` i `DropsDivisionByOne`.

Test usunięty bez zamiennika. Liczba osiem nie jest celem sama w sobie.

### 7.2 `AllConstantLeavesRemainFinite` — czym jest, a czym nie

> **v2, po review.** v1 nazywała go „jedynym testem, który pilnuje inwariantu
> globalnie" i twierdziła, że złapie nową operację foldującą bez sprawdzenia.
> To za mocny claim.

Test buduje FK dla `kr640` i `kr4_r600`, schodzi rekurencyjnie po
`Expression::node()` i sprawdza, że każdy napotkany `ConstantNode` jest
skończony. Co z tego wynika naprawdę:

> wszystkie DAG-i wyrażeń generowane przez **aktualny pipeline FK** dla **tych
> dwóch fixture'ów** spełniają inwariant.

To jest wartościowy test integracyjny i zostaje. Ale **nie jest dowodem dla
wszystkich wyjść `ExpressionFactory`**. Kontrprzykład: przyszłe
`Expression pow(Expression, Expression)` z błędnym `return constant(std::pow(a, b));`
przejdzie ten test, jeśli FK w ogóle nie używa `pow` albo używa go na małych
wartościach.

Realną ochroną są więc, w tej kolejności:

```text
1. centralizacja foldu w foldFinite()   jedno miejsce do przeoczenia zamiast czterech
2. cztery bezpośrednie testy foldów     pinują dokładnie zmienioną ścieżkę
3. AllConstantLeavesRemainFinite        smoke test całego dzisiejszego pipeline'u
```

### 7.3 Nakładanie się z istniejącymi testami

`StillFoldsWhenResultIsFinite` jest częściowo pokryty przez istniejący
`FoldsConstantArithmetic`, który testuje wszystkie cztery normalne foldy. Zostaje
mimo to — jako jawny pin „R5 nie wyłączyło poprawnego foldowania", czytany razem
z czterema testami `DoesNotFold*`. Redundancja świadoma i tania.

## 8. Plan zmian w plikach

```text
src/ik_equations/symbolic/ExpressionFactory.hpp   +1 metoda prywatna,
                                                  #include <optional>,
                                                  komentarz klasy i constant()
src/ik_equations/symbolic/ExpressionFactory.cpp   +5 linii (foldFinite),
                                                  4 x zmieniona ścieżka foldu
src/ik_equations/symbolic/ExpressionEvaluator.cpp  doprecyzowanie komentarza (6)
tests/test_expression_factory.cpp                 +8 testów
STATUS.md                                         wiersz o R5
```

Zero zmian w: loaderze, `KinematicChainBuilder`, budowniczych transformacji, FK,
TCP, modelu równań, `ExpressionEvaluator` (poza komentarzem).

## 9. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Fold przestaje działać tam, gdzie działał poprawnie | niskie | `StillFoldsWhenResultIsFinite` + 312 istniejących testów; warunek jest `!isfinite`, więc dotyka wyłącznie przypadków, które dziś produkują `inf`/`NaN` |
| Większe drzewa | znikome | Tylko dla wyrażeń, które i tak nie mają skończonej wartości; dla realnych robotów liczba węzłów bez zmian, pinowane przez istniejące testy rozmiaru FK |
| `NaN` z innego źródła niż overflow | realne | `foldFinite` odrzuca też `NaN` (`isfinite` pokrywa oba), np. `0.0/0.0` — choć `divide` ma na to osobny `assert` |
| Inwariant naruszony w przyszłości przez nową operację | **realne, nie w pełni pokryte** | Centralizacja w `foldFinite()` sprawia, że jest jedno miejsce do przeoczenia zamiast czterech, a komentarz nad helperem mówi dlaczego. `AllConstantLeavesRemainFinite` złapie to tylko wtedy, gdy nowa operacja trafi do pipeline'u FK i przepełni się na tych fixture'ach — patrz 7.2. Pełnej ochrony automatem nie ma; jest dyscyplina review |
| Ktoś czyta „inwariant" jako gwarancję runtime | realne | 1.1 nazywa to wprost, komentarz klasy po zmianie też |

## 10. Rozstrzygnięte w review (v2)

| # | Pytanie z v1 | Rozstrzygnięcie |
|---|---|---|
| 1 | `foldFinite` jako prywatna metoda czy wolna funkcja w anonimowym namespace | **Prywatna metoda.** Nie poszerza publicznego API, trzyma mechanizm przy fabryce i pokazuje, że jest częścią jej polityki foldingu. Wolna funkcja musiałaby brać fabrykę parametrem bez realnej korzyści. Koszt: `#include <optional>` w nagłówku (4.1) |
| 2 | Czy `AllConstantLeavesRemainFinite` w wersji na FK obu robotów | **Tak, zostaje**, ale z osłabionym claimem — smoke test aktualnego pipeline'u, nie dowód globalny (7.2) |
| 3 | Czy komentarz w `ExpressionEvaluator.cpp` należy do R5 | **Tak.** Po zmianie fabryki stary komentarz opisywałby nieaktualny podział odpowiedzialności. Poza tym evaluator zostaje nietknięty — zero nowych kodów błędu |

Otwarte pytania: brak.

## 11. Rekomendacja

Przyjąć wariant A. Po zatwierdzeniu tej architektury — proposal implementacyjny
(część 2 z 2) z pełnym kodem obu plików i ośmioma testami.

Gate:

```text
build (Release)        0 ostrzeżeń
build-werror           0 ostrzeżeń
ctest                  320/320 w obu (312 + 8)
sonda z 2.1            po zmianie: isConstant=0 dla wszystkich czterech,
                       isConstant=1 value=0 dla underflow
```

Po R5 etap F2.3.R jest zamknięty i gate przed F2.4 (`ConstraintBuilder`) się
otwiera — z uwagą 10 wchodzącą tam jako warunek kontraktowy, zgodnie z
`README.md` etapu.
