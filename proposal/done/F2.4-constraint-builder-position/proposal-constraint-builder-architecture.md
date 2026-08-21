# Proposal: `ConstraintBuilder` — `PositionOnly` (F2.4), architektura

**Etap:** F2.4 · **v2 APPROVED** · 2026-08-20 · część 1 z 2

> **v2, po review.** Jeden blocker i trzy poprawki. Blocker: v1 twierdziła, że
> kolektor symboli wykryje sparowanie FK jednego robota z łańcuchem drugiego —
> nie wykryje, bo `SymbolNode` niesie samą nazwę, a oba roboty w `data/urdf`
> dają `q1..q6`. Gwarancja zawężona do tego, co reprezentacja naprawdę udowadnia
> (4.2, 4.5), test `RejectsTransformFromDifferentChain` zastąpiony przez
> `RejectsUndeclaredSymbol` (6), dopisany wymóg wyczerpalności kolektora w
> czasie kompilacji (4.6), fasada przesunięta na F2.7 zgodnie z roadmapą (8),
> poprawione zdanie o solverze (10). Liczba testów bez zmian: **11**.

Poprzednik: [`analysis-ik-pipeline-constraint-builder.md`](../../analysis-ik-pipeline-constraint-builder.md)
(analiza, gdzie w pipelinie żyją ograniczenia — bez kodu i bez API).
Ten dokument rozstrzyga API, model błędów i granice komponentu dla **wyłącznie
pozycyjnego** przypadku. Orientacja to F2.5.

## Prompt

> Roadmapa, F2.4 — pierwszy realny builder równań IK:
>
> ```text
> T_base_tcp(q) + target_position  →  3 równania
>
> T(0,3)(q) = target_x
> T(1,3)(q) = target_y
> T(2,3)(q) = target_z
> ```
>
> Minimalne testy: `BuildsThreePositionEquations`, `UsesTcpTranslationColumn`,
> `PreservesJointSymbols`, `UsesTargetValuesOnRightHandSide`,
> `RejectsNonFinitePositionTarget`, `BuildsKr4PositionEquationSystem`,
> `BuildsKr640PositionEquationSystem`.
>
> Gate: projekt potrafi automatycznie zbudować układ równań pozycyjnego IK, ale
> go nie rozwiązuje.

Plus dług kontraktowy przeniesiony tu z review kodu z 2026-08-17:

> **Uwaga 10.** `IkEquationSystem::create()` przyjmuje równania używające
> symbolu spoza `unknowns` — nie ma publicznego kolektora symboli, więc nie może
> tego sprawdzić. Warunek etapu F2.4: **`ConstraintBuilder` musi być jedynym
> producentem `IkEquationSystem` i sam gwarantować spójność zbioru symboli.**

Termin „jedyny producent" jest cytatem z review i **doprecyzowano go w 9.1**:
chodzi o produkcyjny pipeline formułowania IK na etapie F2.4, nie o formalną
wyłączność API.

## Status: co już istnieje

Ten etap jest nietypowy — **wyjście jest zaprojektowane wcześniej niż
producent**. F2.3 zbudowało cały model docelowy, więc `ConstraintBuilder` nie ma
tu nic do wymyślenia, tylko kontrakt do spełnienia:

```text
Equation                lhs, rhs, source     — lhs = rhs, PRZECHOWYWANE JAK PODANE
EquationSource          PositionEquationSource{X|Y|Z} | OrientationEquationSource
IkEquationSystem        create() waliduje 9 inwariantów; kolejność równań:
                        pozycja X, Y, Z, potem orientacja row-major
IkTaskKind              Position | Pose
PositionTarget          Vector3 position [m]
validate(PositionTarget) -> NonFinitePosition
```

Wejście również: `SymbolicTransform` (4×4, `ForwardKinematicsBuilder` albo
`IkEquationBuilder::tcpForwardKinematics()`) i `KinematicChain`, w którym
`KinematicJoint::variable` niesie `JointVariable{name, index}` dla jointów
sterowanych.

Czego **nie** ma i nie powstanie w tym etapie: publicznego kolektora symboli,
uproszczeń, rozwiązywania, ograniczeń przestrzeni złączowej (limity `q` to
nierówności — trafiają wprost do solvera, analiza §4).

## 1. Cel i zakres

**W zakresie:**

- nowy komponent `ConstraintBuilder` w `src/ik_equations/builders/`,
- budowa `IkEquationSystem` z trzema równaniami pozycji dla `PositionTarget`,
- gwarancja z uwagi 10: każdy symbol w równaniach jest zadeklarowaną niewiadomą,
- model błędów komponentu.

**Poza zakresem:**

- `PoseTarget` i równania orientacji — F2.5 (2.2 mówi, jak wejdą bez przebudowy),
- integracja z fasadą `IkEquationBuilder` — osobny etap **F2.7** wg roadmapy (8),
- upraszczanie równań, `EquationSimplifier`,
- limity złączowe,
- nowe operacje w `ExpressionFactory` — pozycja ich nie potrzebuje.

## 2. API

### 2.1 Kształt

```cpp
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
```

Wzorzec jest ten sam co w `ForwardKinematicsBuilder`: bezstanowy obiekt z
fabryką po wartości, jedna metoda `build`, `const`. Zgodne z „Factory ownership
is value-semantic" ze `STATUS.md` — dopóki `ExpressionFactory` nie ma stanu,
kopia niczego nie kosztuje.

### 2.2 Dlaczego `PositionTarget`, a nie `IkTarget`

To jest główna decyzja tego dokumentu.

| # | Wariant | Ocena |
|---|---|---|
| **A** | `build(..., const PositionTarget&)`; F2.5 dokłada przeciążenie na `PoseTarget` | **Wybrany** |
| B | `build(..., const IkTarget&)` + kod błędu `UnsupportedTaskKind` dla `PoseTarget` | Odrzucony |

Wariant B tworzy błąd runtime dla czegoś, co typ potrafi wyrazić w czasie
kompilacji — a repo konsekwentnie robi odwrotnie (`OrientationEquationSource::create`
zamyka zakres 0..2, `IkTarget` jest wariantem właśnie po to, żeby „cel, który
niczego nie ogranicza" był niewyrażalny). `UnsupportedTaskKind` byłby też kodem
o krótkim życiu: zniknąłby w F2.5, a kody błędów raz wystawione publicznie
zostają.

Konsekwencja dla F2.5: dochodzi **przeciążenie**, nie zmiana istniejącej
sygnatury:

```cpp
build(const SymbolicTransform&, const KinematicChain&, const PositionTarget&) const;
build(const SymbolicTransform&, const KinematicChain&, const PoseTarget&) const;   // F2.5
```

a rozpakowanie wariantu `IkTarget` żyje **u wywołującego**, gdzie `std::visit` z
jawnym zestawem przeciążeń wymusza kompletność w czasie kompilacji — dokładnie
tak, jak `detail::EquationKindVisitor` w `Equation.hpp`.

### 2.3 Dlaczego transformacja **i** łańcuch

`SymbolicTransform` niesie wyrażenia, ale nie wie, czyimi symbolami są `q1..qn`
— to zwykła macierz 4×4. `KinematicChain` niesie `JointVariable{name, index}`,
czyli dokładnie to, czego wymaga `IkEquationSystem::unknowns()` (posortowane
rosnąco po `index`, ściśle rosnąco).

Wariant „bierz sam łańcuch i policz FK w środku" jest odrzucony: powiela pracę
F2.2 i niszczy reużywalność, o którą chodziło w analizie §3 — jedno FK ma
obsługiwać wiele zadań bez przeliczania.

Ceną jest para argumentów, którą **da się rozjechać** (transformacja z jednego
robota, łańcuch z drugiego). To nie jest hipotetyczne — i **nie zostaje
wykryte**. Dzisiejsza reprezentacja symboli tego nie umożliwia: `SymbolNode`
niesie samą nazwę, a `KinematicChainBuilder` nadaje nazwy pozycyjnie (`"q" +
std::to_string(...)`), więc `q1` z KR640 i `q1` z KR4 są tym samym symbolem.
Szczegóły i konsekwencje: 4.5. Zgodność pary jest **nieweryfikowanym warunkiem
wstępnym wywołującego**, tak samo jak wybór między FK a TCP FK w 2.4, i tak jest
udokumentowana.

### 2.4 Której transformacji użyć — kontrakt wywołującego

`ConstraintBuilder` **nie wie i nie ma prawa wiedzieć**, czy dostał
`forwardKinematics()` czy `tcpForwardKinematics()`. Tak stanowi komentarz przy
`PositionTarget`:

> „A point expressed in the base frame of whatever `SymbolicTransform` the
> constraint builder is given. […] Pairing the right target with the right
> transform is the caller's responsibility, and no type here can check it."

To jest znany, świadomie otwarty brak, opisany w `STATUS.md` („A target does not
know its frame"). F2.4 go **nie zamyka** i nie udaje, że zamyka: zamknięcie
wymaga typu transformacji otagowanego ramką, czyli warstwy konwersji, której
nikt jeszcze nie potrzebuje. Dokumentacja `build()` mówi to wprost, a test
`UsesTcpTranslationColumn` pilnuje tego, co da się przypiąć: że budowane są
komórki *podanej* transformacji.

## 3. Algorytm

```text
1. validate(target)                       -> InvalidTarget
2. unknowns = joints łańcucha z variable, w kolejności łańcucha
3. symbole użyte w kolumnie translacji ⊆ nazwy unknowns    -> UndeclaredSymbol
4. equations = [
       { lhs: transform(0,3), rhs: constant(target.position.x), src: X },
       { lhs: transform(1,3), rhs: constant(target.position.y), src: Y },
       { lhs: transform(2,3), rhs: constant(target.position.z), src: Z } ]
5. IkEquationSystem::create(Position, unknowns, equations)  -> SystemRejected
```

Pięć kroków, każdy z własnym kodem błędu, sprawdzane w tej kolejności — jak w
`IkEquationSystem::create` i w loaderze po R2/R3.

### 3.1 `lhs = rhs`, nigdy `lhs - rhs`

Równanie ma postać `T(i,3)(q) = target_i`, a nie `T(i,3)(q) - target_i = 0`.
Nie jest to kwestia gustu — `Equation` zakazuje tego wprost:

> „The model never performs `lhs - rhs` implicitly: subtracting would destroy
> which side came from the robot and which from the task, and that boundary
> cannot be recovered without algebraic decomposition — which this project does
> not have."

Więc `rhs` to `factory.constant(target.position.x)` i nic więcej. Test
`DoesNotSubtractTargetFromLhs` pilnuje, że `lhs` jest **tą samą** komórką (przez
`sameNode`), a nie wyrażeniem pochodnym.

### 3.2 Kolejność X, Y, Z jest kontraktem, nie zwyczajem

`IkEquationSystem` obiecuje: „Position X, Y, Z first; then orientation cells in
row-major order", a `create()` odrzuca `UnorderedEquations`. Producent musi więc
emitować dokładnie w tej kolejności — inaczej sam wyprodukuje układ, którego
własna walidacja nie przyjmie.

### 3.3 Stałe targetu przechodzą przez `ExpressionFactory`

`factory.constant()` ma precondition skończoności (po R5 domknięte na foldach).
Krok 1 już odrzucił nieskończone współrzędne przez `validate()`, więc w kroku 4
precondition jest spełnione **z konstrukcji**, a nie z nadziei. Kolejność 1 → 4
jest z tego powodu wymuszona, nie kosmetyczna.

Uboczna, przyjemna własność: `target = 0` trafia do współdzielonego węzła zera.

## 4. Uwaga 10: gwarancja spójności symboli

### 4.1 Czego dokładnie wymaga review

`IkEquationSystem::create()` sprawdza dziewięć inwariantów, ale **nie** ten:
równanie może używać symbolu, którego nie ma w `unknowns`. Komentarz klasy
tłumaczy dlaczego — nie ma publicznego kolektora symboli. Review nie kazało go
dodawać; kazało, żeby **producent** gwarantował spójność.

### 4.2 Inwariant, dokładnie

> **Przeformułowane w v2.** v1 opisywała ten sam mechanizm, ale przypisywała mu
> silniejszą własność, niż ma — patrz 4.5.

Gwarancja `ConstraintBuilder` brzmi **dokładnie tak**:

```text
każdy symbol występujący w zbudowanych równaniach ma nazwę obecną w unknowns
```

I nic ponadto. W szczególności **nie**: „transformacja pochodzi z tego samego
`KinematicChain`". To dwa różne zdania i tylko pierwsze jest sprawdzalne
dzisiejszą reprezentacją.

| # | Wariant | Ocena |
|---|---|---|
| A | Gwarancja „z konstrukcji", bez kodu — sam warunek wstępny w dokumentacji | Odrzucony |
| **B** | Aktywne sprawdzenie: zebrać symbole z trzech komórek i porównać z nazwami `unknowns` | **Wybrany** |
| C | Publiczny kolektor symboli w warstwie symbolicznej | Odrzucony — poszerza API dla jednego konsumenta |

Wariant A odpada, bo `SymbolicTransform` jest zwykłą macierzą wyrażeń i nie ma
w niej nic, co czyniłoby „z konstrukcji" prawdą. Wyrażenie może przyjść
skądkolwiek — z ręcznie złożonej macierzy w teście, z przyszłego producenta,
z warstwy, która jeszcze nie istnieje. Uwaga 10 opisuje dokładnie ten kształt:
`unknowns = {q1}` przy równaniu zależnym od `q999`. Takie równanie po wariancie
A przechodzi do `IkEquationSystem`, którego `create()` też go nie złapie — i
solver dostaje układ z symbolem, o którym nikt mu nie powiedział.

Wariant B to łapie, bo to jest pytanie o **nazwy**, a nie o pochodzenie.

### 4.3 Koszt

Obchód DAG-u trzech komórek z zbiorem odwiedzonych — ta sama technika co w
teście R5. Rząd wielkości: całe FK to 281 unikalnych węzłów (kr640) i 516
(kr4_r600), a kolumna translacji to jego podzbiór. Raz na `build()`, przy
budowie układu równań, który i tak jest operacją offline.

### 4.4 Zawieranie, nie równość

Sprawdzenie brzmi: **symbole użyte ⊆ nazwy niewiadomych**. Odwrotna inkluzja
byłaby błędem merytorycznym: niewiadoma może legalnie nie wystąpić w równaniach
pozycji. Kanoniczny przykład to nadgarstek sferyczny z zerowym offsetem TCP —
ostatni obrót `q6` nie zmienia **położenia** TCP, więc `q6` w tych trzech
równaniach nie wystąpi. Układ jest wtedy pozycyjnie niedookreślony i to jest
poprawny, spodziewany wynik dla zadania `PositionOnly`, a nie usterka.

Test `AcceptsUnknownAbsentFromEquations` przypina tę interpretację, żeby nikt
później nie „naprawił" jej na równość.

### 4.5 Czego to sprawdzenie **nie** dowodzi

> **Dodane w v2, po review — to był blocker v1.**

v1 obiecywała, że kolektor wykryje `build(*kr640Fk, kr4Chain, target)`. Nie
wykryje, i wynika to wprost z reprezentacji:

```cpp
struct SymbolNode { std::string name; };                      // Expression.hpp

kinematicJoint.variable = JointVariable{"q" + std::to_string(variableCount),
                                        variableCount};       // KinematicChainBuilder.cpp
```

Nazwa jest **pozycyjna**, nie tożsamościowa. Oba roboty w `data/urdf` mają po
sześć jointów sterowanych, więc:

```text
symbole FK KR640   = {q1, q2, q3, q4, q5, q6}
nazwy unknowns KR4 = {q1, q2, q3, q4, q5, q6}
zawieranie         = spełnione
```

Sprawdzenie przechodzi, a układ jest bez sensu: równania opisują geometrię
KR640, `unknowns` opisują jointy KR4. **Ten przypadek zostaje niewykryty i F2.4
tego nie zmienia.**

Co byłoby potrzebne, żeby go wykryć: tożsamość symbolu silniejsza niż nazwa —
`SymbolId` z tokenem pochodzenia, `SymbolicTransform` związany typem z
łańcuchem, albo przekazywanie pary jako jednego obiektu. Każde z tych rozwiązań
dotyka warstwy symbolicznej albo modelu, czyli jest osobną zmianą o zasięgu
większym niż cały F2.4. **Świadomie poza zakresem**; do zapisania w `STATUS.md`
jako znany brak przy implementacji tego etapu, obok pokrewnego „A target does not
know its frame", z którym dzieli przyczynę: para argumentów, której typ nie
spina.

Rozdzielenie tych dwóch rzeczy ma też wpływ na nazwę kodu błędu — 5.

### 4.6 Kolektor musi być wyczerpujący w czasie kompilacji

> **Dodane w v2, po review.**

`ExpressionNode` to zamknięty `std::variant` z dziewięcioma alternatywami, a
`Expression::node()` jest publiczne, więc kolektor da się napisać lokalnie w
`.cpp` buildera, bez dotykania warstwy symbolicznej. Wymóg dla proposala
implementacyjnego: **dodanie dziesiątej alternatywy ma zatrzymać kompilację
kolektora**, dopóki ktoś jawnie nie zdecyduje, czy nowy węzeł ma dzieci do
obejścia.

Czyli żadnego `else { return; }` na końcu — zamiast tego łańcuch `if constexpr`
zakończony `static_assert` na jedynym typie bez dzieci, jak w
`AllConstantLeavesRemainFinite` z R5. Powód jest ten sam co tam: cicho
niezejście do poddrzewa nowego węzła nie zepsułoby żadnego testu, tylko po cichu
osłabiło gwarancję, której ten komponent ma pilnować.

Obchód po tożsamości węzła ze zbiorem odwiedzonych — jak w R5, DAG jest mocno
współdzielony.

## 5. Model błędów

```cpp
enum class ConstraintBuilderErrorCode
{
    InvalidTarget,      // validate() odrzuciło cel
    UndeclaredSymbol,   // 4.2 — równanie używa symbolu spoza unknowns
    SystemRejected      // IkEquationSystem::create() odmówiło
};

struct ConstraintBuilderError
{
    ConstraintBuilderErrorCode code{};
    std::string message;

    // Zachowane, nie spłaszczone do stringa.
    std::optional<TargetValidationError> targetError;      // iff InvalidTarget
    std::optional<IkEquationSystemError> systemError;      // iff SystemRejected
};
```

Trzy kody, nie więcej — każdy odpowiada jednemu krokowi algorytmu, który może
odmówić. Czego celowo **nie ma**:

- `NoUnknowns` / `NoEquations` — `create()` już je ma; duplikowanie kodu w dwóch
  warstwach oznacza dwa miejsca do rozminięcia się. Łańcuch bez jointów
  sterowanych wraca jako `SystemRejected` z zachowanym `IkEquationSystemError`,
- `NonCanonicalTransform` — kolumna translacji to komórki `(0..2, 3)` niezależnie
  od ostatniego wiersza; sprawdzanie kanoniczności byłoby ceremonią bez wpływu
  na wynik. `hasCanonicalHomogeneousLastRow` zostaje tam, gdzie jest naprawdę
  potrzebne, czyli w `multiplyTransforms`,
- `UnsupportedTaskKind` — patrz 2.2, nie istnieje z założenia.

Nazwa `UndeclaredSymbol` jest po 4.5 jeszcze lepiej uzasadniona, niż była w v1.
Odrzucona alternatywa `TransformChainMismatch` opisywałaby przyczynę, której ten
komponent **nie potrafi stwierdzić**: builder nie wie, czy doszło do rozjazdu
łańcucha — wie tylko, że napotkał symbol niezadeklarowany jako niewiadoma. Kod
błędu ma mówić to, co komponent naprawdę ustalił.

**Struktura błędu powtarza wzorzec z `IkEquationBuilderError`**: opcjonalny błąd
warstwy niższej obok kodu, z inwariantem „ustawiony wtedy i tylko wtedy, gdy kod
mu odpowiada". To agregat, więc wywołujący może zbudować niespójną wartość — tak
jak tam, gwarancja dotyczy tego, co ten komponent **produkuje**.

## 6. Testy

Siedem z roadmapy plus cztery wynikające z decyzji tego dokumentu:

```text
z roadmapy
  BuildsThreePositionEquations        3 równania, źródła X, Y, Z
  UsesTcpTranslationColumn            lhs to komórki (0..2, 3) PODANEJ transformacji
  PreservesJointSymbols               unknowns == JointVariable łańcucha, w kolejności
  UsesTargetValuesOnRightHandSide     rhs to stałe o wartościach targetu
  RejectsNonFinitePositionTarget      InvalidTarget + zachowany TargetValidationError
  BuildsKr4PositionEquationSystem     end-to-end z URDF
  BuildsKr640PositionEquationSystem   end-to-end z URDF

dołożone
  DoesNotSubtractTargetFromLhs        3.1 — lhs to sameNode z komórką FK
  EquationsAreOrderedXYZ              3.2 — kontrakt kolejności
  RejectsUndeclaredSymbol             4.2 — q999 w translacji, unknowns {q1, q2}
  AcceptsUnknownAbsentFromEquations   4.4 — zawieranie, nie równość
```

`RejectsUndeclaredSymbol` odtwarza kształt uwagi 10 wprost: ręcznie złożona
`SymbolicTransform`, w której komórka translacji zależy od `q999`, przy łańcuchu
deklarującym `q1` i `q2`. Oczekiwanie: `UndeclaredSymbol`. To jest test tej
gwarancji, którą F2.4 faktycznie daje — nie testu pochodzenia transformacji,
który po 4.5 nie miałby prawa przejść.

`AcceptsUnknownAbsentFromEquations` robi to samo od drugiej strony i równie
tanio: transformacja, której translacja zawiera tylko `q1`, przy łańcuchu
deklarującym `q1` i `q2`. `SymbolicMatrix::operator()` pozwala wstawić dowolne
`Expression`, więc żadna specjalna geometria dwóch jointów nie jest potrzebna.

Oba testy budują macierz ręcznie i to jest zaleta, nie skrót: sprawdzają
**logikę ograniczeń**, nie poprawność wyprowadzenia geometrii — dokładnie ten
argument o testowalności, dla którego analiza §3 chciała osobnego komponentu.

## 7. Plan zmian w plikach

```text
src/ik_equations/builders/ConstraintBuilder.hpp    NOWY
src/ik_equations/builders/ConstraintBuilder.cpp    NOWY (kolektor symboli w
                                                   anonimowym namespace)
CMakeLists.txt                                     +1 plik źródłowy
tests/test_constraint_builder.cpp                  NOWY, 11 testów
tests/CMakeLists.txt                               +1 plik testowy
STATUS.md                                          F2.4, licznik testów
```

Zero zmian w: loaderze, `KinematicChainBuilder`, `JointTransformBuilder`,
`ForwardKinematicsBuilder`, warstwie symbolicznej, modelu równań, fasadzie.
To jest ta sama własność, którą analiza obiecywała w §3.3: nowy etap **dokłada
się**, niczego nie przestawiając.

## 8. Fasada — świadomie poza zakresem

`IkEquationBuilder` kończy się dziś na `tcpForwardKinematics()`. Żeby wystawić
F2.4, potrzebowałby drugiego wejścia — specyfikacji zadania — i nowego węzła w
grafie stanu.

**Rozstrzygnięte: fasada wchodzi w F2.7**, i nie jest to decyzja tego dokumentu
— roadmapa ma osobny etap „F2.7 — integracja z fasadą i milestone Fazy 2".

> **v2, po review.** v1 pisała „dopiero po F2.5". To było za wcześnie: po F2.5
> API zadania wciąż nie jest pełne, bo F2.6 dokłada constrainty kierunkowe.
> Projektowanie wejścia fasady zanim rodzina ograniczeń się domknie oznaczałoby
> przeprojektowanie go dwa razy — razem z kodami błędów, które byłyby już
> publiczne.

Komponent jest w pełni testowalny bez fasady: testy end-to-end składają
loader → chain → FK wprost, jak `test_numeric_fk_validation.cpp`.

## 9. Rozstrzygnięte w review (v2)

| # | Pytanie z v1 | Rozstrzygnięcie |
|---|---|---|
| 1 | Czy fasada dostaje F2.4 od razu, czy po F2.5 | **Ani jedno, ani drugie — F2.7.** Pytanie było źle postawione: roadmapa ma osobny etap integracji z fasadą, a F2.6 nadal rozszerza rodzinę ograniczeń (8) |
| 2 | Czy zamknąć `IkEquationSystem::create()` przez `friend` | **Nie.** `create()` jest walidującą bramą i ma wartość dla testów modelu, które nie powinny budować całego FK |
| 3 | Nazwa `UndeclaredSymbol` | **Zostaje.** Po 4.5 tym bardziej: `TransformChainMismatch` nazywałby przyczynę, której builder nie potrafi stwierdzić |

Otwarte pytania: brak.

### 9.1 „Jedyny producent" — doprecyzowanie

> **Dodane w v2, po review.**

Wymaganie z uwagi 10 brzmi w tym dokumencie kilka razy jako „`ConstraintBuilder`
musi być jedynym producentem `IkEquationSystem`". Wobec rozstrzygnięcia 2 to
skrót myślowy i warto go rozwinąć:

> `ConstraintBuilder` jest jedynym producentem `IkEquationSystem` **w
> produkcyjnym pipelinie formułowania IK na etapie F2.4**.

Formalnie jedynym producentem nie jest i nie ma być: `create()` zostaje
publiczne, a testy modelu korzystają z niego bez buildera. Z czasem własne
systemy będzie produkował `EquationSimplifier` — „jedyny producent" jako wieczny
inwariant całej biblioteki byłby obietnicą nie do utrzymania.

## 10. Rekomendacja

Przyjąć API z 2 i sprawdzenie z 4. Po zatwierdzeniu — proposal implementacyjny z
pełnym kodem obu plików i jedenastoma testami.

Gate etapu:

```text
build (Release)        0 ostrzeżeń
build-werror           0 ostrzeżeń
ctest                  331/331 (320 + 11)
end-to-end             kr640 i kr4_r600: URDF -> IkEquationSystem z 3 równaniami
                       pozycji i 6 niewiadomymi
uwaga 10               RejectsUndeclaredSymbol zielony, czyli gwarancja
                       symbols ⊆ unknowns jest sprawdzeniem, nie deklaracją
```

Po F2.4 projekt **formułuje** IK — pierwszy raz produkuje układ równań, a nie
opis geometrii. Nie rozwiązuje ich: upraszczanie i rozwiązywanie należą do
**Fazy 3**. F2.5 i F2.6 nadal tylko poszerzają rodzinę ograniczeń (orientacja,
constrainty kierunkowe), a F2.7 domyka Fazę 2 integracją z fasadą.
