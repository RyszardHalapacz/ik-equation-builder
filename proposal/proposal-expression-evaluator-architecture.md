# Proposal: `ExpressionEvaluator` — architektura (v2)

## Prompt

> `ExpressionEvaluator` jest teraz dokładnie następnym etapem. Bez niego builder generuje symboliczną macierz FK, ale nadal nie potrafimy udowodnić, że jej wartości są poprawne. Na wejściu `Expression` + wartości symboli, na wyjściu `double`. Evaluator jako sesja, memoizacja obowiązkowa, model błędów przez `std::expected`, test dziedziny `(1/q1) · 0`.

**Rewizja v2** po werdykcie `REQUEST CHANGES`. Review zatwierdziło kierunek i zgłosiło sześć uwag, w tym **jeden blocker o charakterze bezpieczeństwa pamięci**. Wszystkie sześć przyjmuję. Co się zmieniło: §16.

## Status weryfikacji

**Ten dokument nie zawiera kodu produkcyjnego i niczego nie kompilowałem.**

Ustalenia w §4 pochodzą z przeczytania `ExpressionFactory.cpp` i `Expression.hpp`. Liczby w §5.4 pochodzą z pomiarów zapisanych w `STATUS.md`. Dwie rzeczy są **niezweryfikowane przeze mnie** i tak oznaczone: skalowanie głębokości (§9) oraz reuse adresu węzła — ten drugi punkt zweryfikowało review i to on wymusił zmianę projektu (§5.1).

---

## 1. Cel

Zamknąć największą lukę projektu. `STATUS.md` nazywa ją wprost: FK jest zweryfikowane **strukturalnie**, więc spójny błąd znaku w konwencji RPY, w Rodriguesie albo w `p = p_a + R_a · p_b` przeszedłby dziś każdy test w repo.

```
Expression + wartości symboli  →  double
```

Evaluator sam jeszcze niczego nie dowodzi — jest narzędziem, którym w kroku 4 porównamy symboliczne FK po podstawieniu `q` z niezależnie policzoną macierzą numeryczną.

## 2. Zakres i non-goals

**W zakresie:** ewaluacja wszystkich dziewięciu typów węzłów (`Constant`, `Symbol`, `Add`, `Subtract`, `Multiply`, `Divide`, `Negate`, `Sin`, `Cos`), memoizacja po tożsamości węzła, model błędów, obserwowalność memoizacji.

**Poza zakresem — jawnie:** upraszczanie wyrażeń, modyfikowanie DAG-u, algebra symboliczna, zgadywanie brakujących symboli, tolerancja przy dzieleniu, globalny cache, mieszanie ewaluacji z budowaniem FK, numeryczna walidacja FK (krok 4, osobny proposal).

To ma być **mały interpreter nad niezmiennym DAG-iem** i nic więcej.

---

## 3. Evaluator jako sesja

```cpp
explicit ExpressionEvaluator(SymbolValues values);
```

Jedna instancja = **jedno konkretne podstawienie** `q1..qn`. Wszystkie 16 komórek FK liczy się jedną instancją, współdzieląc cache; dla innej konfiguracji tworzy się nowy evaluator.

Odrzucone statyczne `evaluate(expression, values)` — z dwóch powodów, przy czym drugi jest ważniejszy:

1. **Wydajność** — nowy cache na każde wywołanie zniweczyłby memoizację między komórkami, a właśnie tam jest największe współdzielenie.
2. **Bezpieczeństwo** — cache i wartości symboli są związane czasem życia tego samego obiektu, więc użycie cache'u z innym podstawieniem jest **niewyrażalne**, a nie tylko niezalecane.

`evaluate` **nie jest `const`**: mutuje cache i statystyki, a statystyki są obserwowalne.

### 3.1 Kopiowanie sesji — rozstrzygnięcie (review §5)

Przy domyślnych metodach specjalnych klasa byłaby kopiowalna, a kopia dostałaby kopię wiązań, cache'u i statystyk. Technicznie poprawne, semantycznie mylące dla obiektu reprezentującego sesję: nie jest jasne, czy kopia to „snapshot", czy druga sesja.

**Przyjęte: kopiowanie zabronione, przenoszenie dozwolone.**

```cpp
ExpressionEvaluator(const ExpressionEvaluator&) = delete;
ExpressionEvaluator& operator=(const ExpressionEvaluator&) = delete;
ExpressionEvaluator(ExpressionEvaluator&&) noexcept = default;
ExpressionEvaluator& operator=(ExpressionEvaluator&&) noexcept = default;
```

Nie zostawiamy tego przypadkowemu generowaniu metod specjalnych. Przenoszenie zostaje, żeby evaluator dało się zwrócić z funkcji fabrykującej albo trzymać w kontenerze — jeżeli walidacja z kroku 4 będzie przemiatać wiele konfiguracji `q`, przyda się `std::vector<ExpressionEvaluator>`.

---

## 4. Cztery ustalenia wymuszone przez obecną fabrykę

Przeczytałem `ExpressionFactory.cpp`. **Trzy testy z listy w prompcie nie dają się zbudować** w zaproponowanej formie — nie failują, tylko **przerywają proces asercją**.

### 4.1 Dzielenie przez literalne zero jest niewyrażalne

```cpp
assert(!isZero(rhs) && "division by literal zero");
```

`ReportsDivisionByZero` musi użyć **mianownika symbolicznego**: `divide(constant(1.0), symbol("q1"))` z `q1 = 0`. To nie obejście, tylko poprawny opis tego, kiedy dzielenie przez zero może w tym projekcie w ogóle wystąpić: wyłącznie w wyniku podstawienia.

### 4.2 Ujemne zero nie istnieje jako stała

```cpp
if (value == 0.0) return Expression{}; // shared zero node
```

`-0.0 == 0.0`, więc `constant(-0.0)` zwraca węzeł zera **dodatniego**. Ujemne zero wchodzi tylko przez podstawienie. Potwierdza to również poprawność `denominator == 0.0` bez tolerancji — ten warunek łapie `-0.0` za darmo.

### 4.3 Przepełnienie na stałych też jest niewyrażalne

Zwijanie stałych wraca przez `constant()`, które asertuje `isfinite`, więc `multiply(constant(1e308), constant(1e308))` abortuje. `ReportsNonFiniteResult` musi użyć **dwóch symboli** z `q1 = q2 = 1e308`.

### 4.4 Wniosek ogólny

> **Każde poddrzewo stałe zostało już policzone przy budowaniu, a fabryka gwarantuje jego skończoność.** `DivisionByZero` i `NonFiniteResult` mogą więc powstać **wyłącznie** w poddrzewach zależnych od symboli.

Sprawdzenia zostają (są tanie i chronią przed buildem `NDEBUG`), ale klasa błędów jest węższa, niż sugeruje lista kodów.

---

## 5. Memoizacja

### 5.1 Blocker z review: klucz musi utrzymywać węzeł przy życiu

**v1 tego dokumentu proponowała cache, który jest niebezpieczny pamięciowo.** Przyjmuję to jako realny defekt.

```cpp
std::unordered_map<const ExpressionNode*, double> memo_;   // BŁĄD
```

`evaluate(const Expression&)` przyjmuje referencję, którą wolno związać z tymczasowym:

```cpp
evaluator.evaluate(factory.symbol("q1"));   // tymczasowy ginie po wywołaniu
evaluator.evaluate(factory.symbol("q2"));   // nowy węzeł, potencjalnie ten sam adres
```

Jeżeli nic innego nie trzyma pierwszego węzła, `shared_ptr` zwalnia go po powrocie z `evaluate`, a w `memo_` zostaje **wiszący adres**. Alokator może natychmiast oddać ten sam adres kolejnemu węzłowi — i cache uzna `q2` za wcześniej policzone `q1`, zwracając **cichy błędny wynik**.

Review zweryfikowało to na obecnej implementacji `Expression`: kolejny symbol dostał dokładnie ten sam adres. **Nie jest to ryzyko teoretyczne.**

Co gorsza, jest to najgorszy możliwy rodzaj błędu dla tego akurat komponentu: evaluator ma być narzędziem, którym **dowodzimy poprawności FK**. Narzędzie pomiarowe, które potrafi po cichu skłamać, jest gorsze niż jego brak.

**Przyjęte rozwiązanie — `Expression` jako klucz, z tożsamością węzła jako hash i equality:**

```cpp
std::unordered_map<Expression, double,
                   ExpressionIdentityHash, ExpressionIdentityEqual> memo_;
```

gdzie hash to `std::hash<const ExpressionNode*>{}(&expression.node())`, a equality to istniejące `sameNode(lhs, rhs)`.

Klucz trzyma `shared_ptr`, więc **wpis cache'u utrzymuje swój węzeł przy życiu**, a adres nie może zostać ponownie użyty, dopóki wpis istnieje. Jednocześnie porównanie pozostaje O(1) po tożsamości — `structurallyEqual` nie jest wołane nigdy.

Wariant `unordered_map<const ExpressionNode*, MemoEntry{Expression owner; double value;}>` też jest poprawny, ale wariant z `Expression` jako kluczem **wyraża własność w typie**, zamiast polegać na tym, że ktoś nie usunie pola `owner` jako pozornie nieużywanego.

**Odrzucone: udokumentowanie preconditionu** „wszystkie ewaluowane drzewa muszą przeżyć evaluator". Byłby łatwy do złamania, niemożliwy do asertowania, a jego złamanie daje ciche złe wyniki zamiast diagnozowalnego crasha. Zgadzam się z review w całości.

**Świadomy koszt:** evaluator przypina cały odwiedzony DAG na czas swojego życia. Dla FK to 281–516 węzłów, a sesje są krótkotrwałe — koszt bez znaczenia wobec ceny alternatywy.

### 5.2 Uwaga do zapisania dla proposalu implementacyjnego

Szkic z review deklaruje typy pomocnicze jako niepełne:

```cpp
private:
    struct ExpressionIdentityHash;      // deklaracja
    struct ExpressionIdentityEqual;
    std::unordered_map<Expression, double,
                       ExpressionIdentityHash, ExpressionIdentityEqual> memo_;
```

To się **nie skompiluje**: deklaracja składowej instancjonuje `unordered_map`, a `Hash` i `KeyEqual` muszą być wtedy typami zupełnymi (kontener trzyma je jako podobiekty). Oba typy trzeba **zdefiniować w całości powyżej** deklaracji `memo_`.

To ten sam rodzaj pułapki, którą `Expression.hpp` już raz w tym projekcie rozbroił komentarzem „declaration order below is load-bearing". Odnotowuję, żeby nie wróciła.

### 5.3 Memoizacja po tożsamości, nie po równoważności

Fabryka współdzieli **wyłącznie** węzeł zera — `constant(1.0)` zbudowane dwa razy daje dwa węzły. Cache trafia więc tylko w **rzeczywiste współdzielenie**. To właściwe zachowanie: odzwierciedla realny kształt danych (`multiplyTransforms` masowo kopiuje te same uchwyty), a gdyby fabryka kiedyś dostała hash-consing, współczynnik trafień wzrósłby bez żadnej zmiany w evaluatorze.

Cache przechowuje **wszystkie** poprawnie policzone węzły, także `Constant` i `Symbol` — dla `Symbol` oszczędza to hashowanie stringa przy każdym wystąpieniu.

### 5.4 Skala problemu

| Robot | węzły unikalne | odwiedziny bez memoizacji |
|---|---:|---:|
| `kr640.urdf` | 281 | 21 882 |
| `kr4_r600.urdf` | 516 | 153 703 |

Trzy rzędy wielkości dla KR4. Zaobserwowałem to również przypadkiem: program napisany na potrzeby tego dokumentu chodził po DAG-u bez memoizacji i **zawiesił się** na dłuższym łańcuchu.

### 5.5 Błędy nie są cache'owane — poprawiony argument (review §6)

**v1 uzasadniała to błędnie**, twierdząc, że błędne poddrzewo policzy się najwyżej 16 razy. To nieprawda: publiczne API pozwala wołać `evaluate` dowolną liczbę razy, więc liczba powtórzeń jest nieograniczona.

Poprawne uzasadnienie:

> Błędy nie są cache'owane, ponieważ ścieżka błędna **nie jest ścieżką wydajnościową** tego komponentu. Powtórne wywołanie dla tego samego błędnego poddrzewa policzy je ponownie, i to jest akceptowalne.

Cache'owanie `std::expected<double, EvaluationError>` byłoby poprawne — wartości symboli są niezmienne przez całą sesję, więc błąd jest deterministyczny — ale komplikuje typ wpisu bez zysku tam, gdzie zależy nam na wydajności. **Do rewizji tylko wtedy**, gdyby powstał scenariusz masowo ewaluujący wyrażenia poza dziedziną.

---

## 6. Model błędów

`std::expected` jest tu uzasadniony, w przeciwieństwie do `ForwardKinematicsBuilder`: błędy nie oznaczają złamanych inwariantów, tylko niepoprawne dane podstawienia albo dziedzinę wyrażenia.

```cpp
enum class EvaluationErrorCode { MissingSymbol, NonFiniteSymbolValue,
                                 DivisionByZero, NonFiniteResult };

struct EvaluationError { EvaluationErrorCode code; std::string symbolName; };
```

| Sytuacja | Kod | Rozstrzygnięcie |
|---|---|---|
| symbol bez wiązania | `MissingSymbol` | **nigdy** domyślne zero — ciche zero dałoby poprawnie wyglądającą, błędną pozycję robota |
| wiązanie `NaN`/`±Inf`, **użyte** | `NonFiniteSymbolValue` | sprawdzane przy odczycie symbolu — §6.1 |
| mianownik `== 0.0` | `DivisionByZero` | porównanie dokładne, bez tolerancji; łapie `-0.0` |
| wynik operacji nieskończony | `NonFiniteResult` | `std::isfinite` po każdym węźle |

`symbolName` puste dla błędów niesymbolicznych — dopuszczalne, bo fabryka zabrania pustych nazw symboli, więc pusty string jest jednoznacznym „nie dotyczy".

### 6.1 Walidacja wiązań leniwa — zatwierdzone w review

Nieużywane wiązanie `NaN`/`Inf` **nie jest błędem**; użyte daje `NonFiniteSymbolValue`. Utrzymuje prosty konstruktor i spójną zasadę „nieużywane wiązanie nie wpływa na wynik". Pinowane testem `AcceptsNonFiniteBindingForUnusedSymbol`.

---

## 7. Zakaz skracania — doprecyzowany kontrakt (review §3)

Sformułowanie z v1 („zawsze ewaluuje oba operandy") było **nieprecyzyjne**: przy `std::expected` naturalna implementacja zwraca błąd lewego operandu bez dotykania prawego, i to jest poprawne.

**Obowiązujący kontrakt:**

> Evaluator **nie pomija żadnego operandu na podstawie wartości** drugiego operandu. W szczególności zero w `Multiply` **nie** powoduje skrócenia. Ewaluacja może natomiast zakończyć się natychmiast po napotkaniu **pierwszego błędu**.
>
> Kolejność jest ustalona: **lewy operand przed prawym.** Gdy oba poddrzewa są błędne, zwracany jest błąd lewego — deterministycznie.

Ustalenie kolejności ma znaczenie praktyczne: bez niego test z dwoma błędnymi poddrzewami byłby zależny od implementacji, a komunikat błędu — nieprzewidywalny.

### 7.1 Dlaczego to jest inwariant, a nie mikrooptymalizacja

```
(1 / q1) · 0     dla q1 = 0     →     DivisionByZero,  nie 0
```

Evaluator skracający mnożenie po zobaczeniu zera dałby `0` i **cicho unieważnił** decyzję warstwy symbolicznej o braku reguły `x · 0 → 0`. Cała ta decyzja istnieje po to, żeby nie zgubić informacji o dziedzinie; evaluator, który skraca, gubi ją z powrotem na końcu potoku.

### 7.2 Symetria — poprawka z review §2

**v1 testowała tylko zero po prawej stronie.** To nie wykryłoby implementacji skracającej wyłącznie `0 · x`. Warstwa symboliczna świadomie zachowuje **oba** kształty, więc evaluator musi respektować dziedzinę symetrycznie. Stąd dwa testy zamiast jednego:

| Test | Wyrażenie |
|---|---|
| `PreservesDomainWhenZeroIsRightOperand` | `multiply(divide(constant(1), q1), constant(0))` |
| `PreservesDomainWhenZeroIsLeftOperand` | `multiply(constant(0), divide(constant(1), q1))` |

Oba dla `q1 = 0` muszą dać `DivisionByZero`.

Zwracam uwagę, że drugi przypadek nie jest hipotetyczny w tym projekcie: `SymbolicMatrix::multiply` zwija iloczyny lewostronnie, więc `Constant(0)` **realnie występuje jako lewy operand** w komórkach macierzy FK.

---

## 8. Statystyki — nazewnictwo doprecyzowane (review §4)

`evaluatedNodes` było **niejednoznaczne**: nie rozstrzygało, czy węzeł zakończony błędem jest liczony. Przyjmuję nazwy jednoznaczne semantycznie:

```cpp
struct EvaluationStatistics
{
    std::size_t cacheMisses;   // brak wpisu przed próbą ewaluacji
    std::size_t cacheHits;     // wpis znaleziony
};
```

Semantyka wynika teraz z samej nazwy i nie wymaga dopisku o błędach: licznik dotyczy **wyniku sprawdzenia cache'u**, a nie powodzenia ewaluacji.

Przykład — `x = cos(q1)`, wyrażenie `add(x, x)`:

```
Add    → miss      Cos → miss      Symbol → miss      Cos (ten sam węzeł) → hit
cacheMisses = 3     cacheHits = 1
```

Statystyki są **kumulatywne dla sesji**, bez resetu — to właśnie pozwala napisać `SharesMemoizationAcrossMultipleRootExpressions`. Włączone zawsze, bez `#ifdef`: warunkowa kompilacja oznaczałaby, że testujemy inną konfigurację niż wysyłamy. Koszt to dwa inkrementy `size_t`.

Bez tej obserwowalności `MemoizesSharedSubexpression` nie odróżniłby implementacji z cache'em od implementacji bez — sprawdziłby tylko, że wynik jest poprawny.

---

## 9. Rekurencja — zatwierdzona, z zastrzeżeniem

Zmierzona głębokość FK: **22** (`kr640`), **24** (`kr4_r600`), po 7 jointów. Bezpieczne dla rekurencji z ogromnym zapasem.

**Nie zmierzyłem** skalowania głębokości z długością łańcucha. Szacuję wzrost o kilka poziomów na złożony joint, czyli setki dla łańcucha 100-jointowego — nadal bezpiecznie — ale **to jest szacunek, nie pomiar**, i tak go oznaczam. Do zapisania w `STATUS.md`: gdyby przyszły simplifier pogłębiał drzewa albo pojawiły się łańcuchy o setkach jointów, decyzję trzeba zrewidować, zaczynając od tego pomiaru.

Wariant iteracyjny po jawnym stosie jest tu wyraźnie brzydszy niż w `KinematicChainBuilder`, bo wymaga ręcznego odtwarzania kolejności „policz oba operandy, potem połącz".

---

## 10. Projekt API — całość po poprawkach

```cpp
using SymbolValues = std::unordered_map<std::string, double>;

enum class EvaluationErrorCode { MissingSymbol, NonFiniteSymbolValue,
                                 DivisionByZero, NonFiniteResult };

struct EvaluationError { EvaluationErrorCode code; std::string symbolName; };

struct EvaluationStatistics { std::size_t cacheMisses; std::size_t cacheHits; };

class ExpressionEvaluator
{
public:
    explicit ExpressionEvaluator(SymbolValues values);

    ExpressionEvaluator(const ExpressionEvaluator&) = delete;
    ExpressionEvaluator& operator=(const ExpressionEvaluator&) = delete;
    ExpressionEvaluator(ExpressionEvaluator&&) noexcept = default;
    ExpressionEvaluator& operator=(ExpressionEvaluator&&) noexcept = default;

    [[nodiscard]] std::expected<double, EvaluationError>
    evaluate(const Expression& expression);

    EvaluationStatistics statistics() const noexcept;

private:
    // Both must be COMPLETE types before memo_ — see §5.2.
    struct ExpressionIdentityHash { /* &expression.node() */ };
    struct ExpressionIdentityEqual { /* sameNode(lhs, rhs) */ };

    SymbolValues values_;
    std::unordered_map<Expression, double,
                       ExpressionIdentityHash, ExpressionIdentityEqual> memo_;
    EvaluationStatistics statistics_{};
};
```

Umiejscowienie: `src/ik_equations/symbolic/ExpressionEvaluator.{hpp,cpp}`. Nie zna `SymbolicTransform` ani `KinematicChain`.

**Świadomie brak** przeciążenia `evaluate(const SymbolicTransform&)`. Pętla po 16 komórkach należy do wołającego; dodanie jej tutaj wprowadziłoby zależność evaluatora od reprezentacji transformacji. Jeżeli w kroku 4 okaże się potrzebna, jej miejsce jest w kodzie walidacji.

**Wątkowość:** evaluator **nie jest** bezpieczny wątkowo (mutowalny cache). Sam DAG jest niemutowalny, więc wiele evaluatorów może czytać to samo drzewo równolegle — istotne, gdyby walidacja przemiatała wiele konfiguracji `q`. Do zapisania w komentarzu klasy.

---

## 11. Plan zmian w plikach

**Dodane:** `ExpressionEvaluator.hpp`, `.cpp`, `tests/test_expression_evaluator.cpp`.

**Zmienione:** `CMakeLists.txt`, `tests/CMakeLists.txt` (po jednej linii), `STATUS.md`.

**Bez zmian — jawnie:** `Expression`, `ExpressionFactory`, `SymbolicMatrix`, `SymbolicTransform`, wszystkie buildery. Evaluator **nie wymaga zmiany w istniejącym API**: `node()` i `sameNode()` są publiczne, co potwierdziłem w nagłówku. Review potwierdza tę ocenę również po naprawie własności cache'u.

---

## 12. Plan testów — 22 pozycje

### 12.1 Pokrycie węzłów (10)

`EvaluatesConstant`, `EvaluatesSymbol`, `EvaluatesAdd`, `EvaluatesSubtract`, `EvaluatesMultiply`, `EvaluatesDivide`, `EvaluatesNegate`, `EvaluatesSin`, `EvaluatesCos`, `EvaluatesNestedExpression`

Uwaga konstrukcyjna: większości **nie da się** zbudować na samych stałych — fabryka zwinie je w `Constant` już przy budowaniu i test sprawdzi zwijanie zamiast ewaluacji. Każdy musi mieć co najmniej jeden operand symboliczny.

### 12.2 Błędy i dziedzina (7)

| Test | Konstrukcja — po korekcie z §4 i §7.2 |
|---|---|
| `ReportsMissingSymbol` | wyrażenie z `q3`, wiązania bez `q3` |
| `RejectsNonFiniteSymbolValue` | `q1 = NaN`, **użyte** |
| `ReportsDivisionByZero` | mianownik symboliczny, `q1 = 0` (§4.1) |
| `ReportsDivisionByNegativeZero` | wiązanie `q1 = -0.0` (§4.2) |
| `PreservesDomainWhenZeroIsRightOperand` | `(1/q1) · 0`, `q1 = 0` |
| `PreservesDomainWhenZeroIsLeftOperand` | **nowy** — `0 · (1/q1)`, `q1 = 0` (§7.2) |
| `ReportsNonFiniteResult` | dwa symbole, `q1 = q2 = 1e308` (§4.3) |

### 12.3 Sesja i memoizacja (5)

| Test | Co pinuje |
|---|---|
| `MemoizesSharedSubexpression` | `add(x, x)` → `cacheMisses == 3`, `cacheHits == 1` |
| `SharesMemoizationAcrossMultipleRootExpressions` | dwa korzenie nad wspólnym poddrzewem; drugie `evaluate` ma trafienia |
| `DifferentEvaluatorsUseDifferentSymbolValues` | dwie instancje, te same wyrażenia, różne wyniki |
| `IgnoresUnusedExtraBindings` | nadmiarowe wiązanie nie zmienia wyniku |
| `AcceptsNonFiniteBindingForUnusedSymbol` | leniwa walidacja (§6.1) |

### 12.4 Własność cache'u (nowa grupa, review §1)

Blocker z §5.1 **musi mieć własny test**, inaczej naprawa jest niepinowana i ktoś ją kiedyś „uprości" z powrotem do surowego wskaźnika.

| Test | Co pinuje |
|---|---|
| `KeepsEvaluatedNodesAliveAcrossTemporaries` | ciąg `evaluate` na **tymczasowych** wyrażeniach: `evaluate(factory.symbol("q1"))`, potem `evaluate(factory.symbol("q2"))`, itd. Każdy wynik musi odpowiadać właściwemu symbolowi. Przy cache'u na surowym wskaźniku ten test daje wynik `q1` dla `q2`, gdy alokator odda ten sam adres. |

**Uczciwe zastrzeżenie do zapisania w proposalu implementacyjnym:** ten test wykrywa defekt **tylko wtedy**, gdy alokator faktycznie ponownie użyje adresu — to zachowanie zależne od implementacji, więc test nie jest deterministycznym dowodem. Review zaobserwowało reuse natychmiast, więc w praktyce działa; nie należy jednak twierdzić, że jego zielony wynik **dowodzi** poprawności własności. Dowodem jest typ klucza (`Expression`, nie `const ExpressionNode*`), a test jest siecią bezpieczeństwa przeciw regresji.

### 12.5 Czego tu nie ma

Numerycznej walidacji FK dla KR4/KR640 — to **krok 4**, z własnym proposalem: wybór konfiguracji `q` (zero, kilka niezerowych, wiele jointów naraz, wartości blisko limitów), źródło niezależnej macierzy odniesienia oraz tolerancja porównania. `STATUS.md` odnotowuje już szum zwijania trygonometrii rzędu `1e-16`, więc tolerancja **będzie** potrzebna i będzie wymagała uzasadnienia.

Oczekiwany stan po tym komponencie: **161 + 22 = 183**.

---

## 13. Ryzyka

| Ryzyko | Ocena | Reakcja |
|---|---|---|
| regresja własności cache'u | **było blockerem**, teraz niskie | typ klucza wymusza poprawność; `KeepsEvaluatedNodesAliveAcrossTemporaries` jako sieć bezpieczeństwa (§12.4) |
| głębokość rekurencji przy długich łańcuchach | niskie, **niezmierzone** | §9 — zapis w `STATUS.md`, pomiar przed ewentualną zmianą |
| evaluator przypina DAG na czas życia | niskie | 281–516 węzłów, sesje krótkotrwałe; świadomy koszt naprawy z §5.1 |
| hashowanie adresów w `unordered_map` | niskie | przy tej liczbie węzłów bez znaczenia |
| tolerancja w kroku 4 stanie się sporna | **średnie** | znany, udokumentowany szum `1e-16`; decyzja należy do proposalu walidacji |

---

## 14. Stan decyzji

**Zatwierdzone w review v1:** evaluator jako sesja, `std::expected`, leniwa walidacja wiązań, dokładne `denominator == 0.0`, memoizacja po tożsamości węzła, statystyki w API, rekurencja, brak skracania na podstawie wartości, brak zależności od `SymbolicTransform`.

**Zmienione w v2 na żądanie review:** własność cache'u (§5.1), symetryczny test dziedziny (§7.2), doprecyzowany kontrakt skracania wraz z kolejnością `lhs` przed `rhs` (§7), nazwy statystyk (§8), kopiowanie sesji (§3.1), argument o niecache'owaniu błędów (§5.5).

**Dodane przeze mnie w v2:** test własności cache'u wraz z zastrzeżeniem o jego niedeterminizmie (§12.4) oraz ostrzeżenie o niepełnych typach `Hash`/`KeyEqual` (§5.2).

---

## 15. Rekomendacja końcowa

Zatwierdzić w kształcie: **evaluator jako nieskopiowalna sesja z jednym podstawieniem, cache kluczowany przez `Expression` z tożsamością węzła jako hash/equality, `std::expected` z czterema kodami błędów, leniwa walidacja wiązań, statystyki `cacheMisses`/`cacheHits`, rekurencja, zakaz skracania na podstawie wartości przy ustalonej kolejności `lhs` przed `rhs`.**

Komponent jest mały — dziewięć typów węzłów i jeden cache. Wartość leży gdzie indziej: jest to pierwsze narzędzie w projekcie zdolne **obalić** wcześniejszą decyzję. Dlatego blocker z §5.1 był poważniejszy, niż wynikałoby z jego lokalności — narzędzie pomiarowe, które potrafi po cichu skłamać, jest gorsze niż jego brak.

---

## 16. Co zmieniła rewizja v2

| # review | Zarzut | Werdykt | Gdzie |
|---|---|---|---|
| 1 | cache z surowym wskaźnikiem nie utrzymuje węzła przy życiu | **przyjęty — blocker, realny błąd v1** | §5.1, §5.2, §12.4 |
| 2 | brak symetrycznego testu `0 · (1/q)` | **przyjęty — luka w teście v1** | §7.2, §12.2 |
| 3 | „zawsze oba operandy" nieprecyzyjne przy `std::expected` | przyjęty | §7 |
| 4 | `evaluatedNodes` niejednoznaczne przy błędach | przyjęty | §8 |
| 5 | kopiowanie sesji zostawione metodom domyślnym | przyjęty | §3.1 |
| 6 | argument „najwyżej 16 razy" nieprawdziwy | **przyjęty — błąd v1** | §5.5 |

Trzy z sześciu to realne błędy, nie preferencje, i warto nazwać, co je łączy: **wszystkie trzy dotyczą tego, co dzieje się poza szczęśliwą ścieżką.** Cache projektowałem dla drzew trzymanych przez macierz FK, nie dla tymczasowych. Test dziedziny napisałem dla kształtu, który akurat miałem w głowie, nie dla obu. Koszt ponownej ewaluacji policzyłem dla pętli 16 komórek, nie dla dowolnego użycia publicznego API.

Wspólny mianownik: rozumowałem o **zamierzonym** sposobie użycia zamiast o **dopuszczonym przez API**. Przy komponencie, który ma być narzędziem dowodowym, to zbyt wąska rama.

Po zatwierdzeniu: `proposal-expression-evaluator-implementation.md` z pełnym kodem produkcyjnym i testowym.
