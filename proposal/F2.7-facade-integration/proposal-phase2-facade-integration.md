# Proposal: integracja z fasadą i milestone Fazy 2 (F2.7), architektura

**Etap:** F2.7 · **v2 — do review** · 2026-08-21 · część 1 z 2

> **v2, po review.** Wariant A zatwierdzony; trzy poprawki merytoryczne i jedna
> rachunkowa:
>
> 1. **„Cel jest w ramce TCP" było semantycznie błędne** i mogło skłonić caller'a
>    do przetransformowania celu do układu TCP — czyli do zbudowania złego
>    ograniczenia. Wszystkie wielkości celu są i pozostają **w bazie**; TCP wybiera
>    wyłącznie, **który frame robota** jest ograniczany. Przepisane 3.1, 4 i 7.
> 2. **Test po zmianie łańcucha opisywał stan niemożliwy.** `selectChain` czyści
>    TCP, więc błędem jest `ForwardKinematicsNotBuilt`, a nie
>    `TcpForwardKinematicsNotBuilt`; po samym `buildForwardKinematics` zapytanie
>    **przechodzi** ścieżką końca łańcucha. Przepisane (8).
> 3. **Pytanie roadmapy dotyczyło celu I TCP**, a v1 rozstrzygała jawnie tylko cel.
>    Asymetria jest zamierzona i teraz jest uzasadniona wprost (2.4).
> 4. Lista testów miała 13 pozycji przy zapowiedzianych 12 — dwa scalone w jeden,
>    gate `422/422` (8).

Ostatni etap Fazy 2. Po nim `IkEquationBuilder` przyjmuje URDF, TCP i cel, a
oddaje **jawny układ równań IK** — czyli to, co roadmapa zapisała jako Definition
of Done całej fazy.

Poprzednik: [`../done/F2.6-directional-constraints/proposal-directional-constraints-architecture.md`](../done/F2.6-directional-constraints/proposal-directional-constraints-architecture.md)
(F2.6, zaimplementowane w dwóch połowach 2026-08-21).

## Prompt

> Roadmapa, F2.7 — docelowy przepływ, kandydat:
>
> ```cpp
> builder.loadRobotModel(...);
> builder.selectChain("base_link", "tool0");
> builder.buildForwardKinematics();
> builder.setTcp(...);
> builder.buildTcpForwardKinematics();
> builder.buildConstraints(target);
>
> const IkEquationSystem* equations = builder.equationSystem();
> ```
>
> **Proposal integracyjny ma rozstrzygnąć, czy API pozostaje stanowe, czy target
> i TCP są argumentami operacji.**
>
> Testy end-to-end: `BuildsKr4PositionIkEquationsFromUrdf`,
> `BuildsKr640PositionIkEquationsFromUrdf`, `BuildsFullPoseEquationsWithTcp`,
> `ChangingTargetInvalidatesEquationSystem`, `ChangingTcpInvalidatesEquationSystem`,
> `ChangingChainInvalidatesTcpAndEquationSystem`,
> `ReusesFacadeForDifferentRobotAndTarget`.
>
> DoD Fazy 2: `URDF + TCP + target → poprawny symboliczny IkEquationSystem`.
> Release: `v0.2-ik-equations`.

---

## 0. Stan wyjściowy

```text
IkEquationBuilder      pięć operacji, cztery akcesory, graf stanu
ConstraintBuilder      sześć rodzajów zadań, nie wpięty w fasadę
IkTarget               sześć alternatyw
410/410 testów w obu konfiguracjach
```

Fasada kończy się dziś na `tcpForwardKinematics()`. `ConstraintBuilder` jest
**celowo** poza nią od F2.4: fasada zaprojektowana przed domknięciem rodziny
ograniczeń byłaby projektowana pod ruchomy cel. Rodzina jest zamknięta od F2.6,
więc jest co wystawić.

---

## 1. Cel i zakres

**W zakresie:**

```text
jedna nowa operacja na fasadzie: URDF + TCP + cel -> IkEquationSystem
rozstrzygnięcie pytania roadmapy: stan czy argument
wybór transformacji (TCP albo koniec łańcucha) i jego widoczność
model błędów: brakujące kroki i błędy buildera
testy end-to-end, DoD Fazy 2, release
```

**Poza zakresem, świadomie:**

```text
rozwiązywanie równań                         -> Faza 3
upraszczanie                                 -> F3.1-F3.3
podstawianie wartości / ewaluacja przez fasadę -> caller ma ExpressionEvaluator
typowany błąd loadera                         -> osobna zmiana, w STATUS.md
frame-tagged transform / SymbolId             -> 7, świadomie niezamykane
```

---

## 2. Pytanie roadmapy: stan czy argument

To jest **jedyna** decyzja architektoniczna tego etapu i roadmapa słusznie
zostawiła ją tutaj.

### 2.1 Wariant A — cel jako argument, układ zwracany ✅ **rekomendowany**

```cpp
[[nodiscard]] std::expected<IkEquationSystem, IkEquationBuilderError>
buildEquationSystem(const IkTarget& target) const;
```

Bez nowego węzła stanu, bez akcesora, bez reguł unieważniania. Metoda jest
`const`: niczego w obiekcie nie zmienia.

### 2.2 Wariant B — cel jako stan, akcesor (kandydat z roadmapy)

```cpp
[[nodiscard]] std::expected<void, IkEquationBuilderError>
buildConstraints(const IkTarget& target);

[[nodiscard]] const IkEquationSystem* equationSystem() const noexcept;
```

Spójne z pozostałymi pięcioma krokami — i to jest jego jedyny mocny argument.

### 2.3 Dlaczego A, a nie B

**Graf stanu istnieje z konkretnego powodu, którego cel nie spełnia.** Każdy
dzisiejszy węzeł jest **drogi i wielokrotnego użytku**: sparsowany robot,
łańcuch, symboliczne FK (281 unikalnych węzłów dla `kr640`, 21 882 liczone z
krotnościami). Trzyma się je, bo policzenie ich drugi raz kosztuje. Cel to
przeciwieństwo: **jedno pytanie**, a odpowiedź jest tania — `IkEquationSystem`
mówi o sobie wprost, że kopiowanie duplikuje uchwyty `Expression`, nigdy drzewa.

**Szósty węzeł istniałby po to, żeby być unieważnianym przez pozostałe pięć.**
Zmiana robota, łańcucha, FK, TCP albo TCP FK musiałaby czyścić układ równań —
pięć nowych reguł, pięć nowych wierszy w kontrakcie akcesorów i pięć nowych
testów pilnujących, że nic nie zostało nieświeże. Wariant A nie ma czego
unieważniać: caller dostaje **migawkę**, która z definicji nie zestarzeje się w
obiekcie, bo jej tam nie ma.

**Faza 3 będzie pytać wielokrotnie, nie raz.** Solver i przyszły generator kodu
przechodzą po *wielu* celach na jednym FK — punkty trajektorii to dokładnie ten
kształt. W wariancie B każde pytanie mutuje obiekt:

```cpp
for (const auto& target : path)          // WARIANT B
{
    if (!builder.buildConstraints(target)) { ... }
    consume(*builder.equationSystem());   // i tak trzeba skopiować, zanim padnie następne
}
```

```cpp
for (const auto& target : path)          // WARIANT A
    if (const auto system = builder.buildEquationSystem(target))
        consume(*system);
```

Wariant A czyta się jak zapytanie, bo nim jest. Wariant B zmusza do mutowania
obiektu w trakcie czegoś, co pojęciowo jest odczytem — i odbiera `const`
wszystkim, którzy chcieliby dzielić jedną fasadę.

**Spójność z pozostałymi krokami jest pozorna.** Tamte pięć operacji buduje
**model robota**; ta jedna zadaje mu **pytanie**. Trzymanie obu rodzajów rzeczy w
jednym grafie stanu nie jest spójnością, tylko zatarciem różnicy.

### 2.4 Druga połowa pytania: TCP zostaje **stanem**

Prompt pyta o „target i TCP". v1 rozstrzygała jawnie tylko cel, a TCP zostawał
stanem niejako przy okazji — review słusznie to wytknęło, bo to jest **osobna
decyzja** i zasługuje na własne uzasadnienie.

```text
robot / łańcuch / FK / TCP / TCP-FK   ->  stan modelu
cel                                   ->  argument zapytania
IkEquationSystem                      ->  migawka wyniku
```

Asymetria jest zamierzona i wynika z tego samego kryterium, które rozstrzygnęło
punkt 2.3: **czy rzecz jest droga i współdzielona przez wiele zapytań.** TCP jest
konfiguracją narzędzia — jedną dla całej operacji technologicznej — a
`T_base_tcp(q)` liczy się raz i służy tysiącom celów:

```cpp
builder.setTcp(tcp);
builder.buildTcpForwardKinematics();     // raz

for (const auto& target : path)          // wiele razy
    consume(builder.buildEquationSystem(target));
```

Gdyby TCP był argumentem zapytania — `buildEquationSystem(target, tcp)` — zostałyby
dwa wyjścia, oba gorsze: albo `T_base_tcp(q)` przelicza się **przy każdym celu**
(kompozycja całego łańcucha na wyrzucenie), albo dochodzi cache po wartości TCP —
czyli stan, tylko ukryty i bez kontraktu.

Kryterium jest więc jedno i rozdziela te dwie rzeczy czysto: **cel jest pytaniem,
TCP jest częścią modelu, którą wiele pytań współdzieli.**

**Zmiana wobec roadmapy jest jawna.** Kandydat z promptu pokazuje wariant B.
Ten proposal go odrzuca i — zgodnie z precedensem F2.5 — **roadmapa zostaje
zmieniona**, a nie po cichu ominięta (9). Znikają też z niej dwa testy
end-to-end, które przy wariancie A nie mają czego sprawdzać (8).

---

## 3. Która transformacja — jedyne realne ryzyko tego etapu

Fasada trzyma **dwie** transformacje: `forwardKinematics()` (koniec łańcucha) i
`tcpForwardKinematics()`. `ConstraintBuilder` przyjmuje jedną i **nie wie**,
którą dostał — to jest zapisany brak („cel nie zna swojej ramki"), a pomyłka daje
układ błędny dokładnie o offset narzędzia: kompiluje się, liczy i jest nieprawdą.

### 3.1 Reguła — i czego ona **nie** znaczy

```text
TCP ustawiony        -> ograniczany jest frame TCP;    używane tcpForwardKinematics()
TCP nieustawiony     -> ograniczany jest koniec łańcucha; używane forwardKinematics()

w OBU przypadkach    -> wielkości celu są wyrażone w BAZIE
```

Ostatni wiersz jest poprawką z v2 i jest ważniejszy niż dwa pierwsze. v1 pisała
„cel jest w ramce TCP", co jest **nieprawdą** i mogłoby kosztować caller'a
poprawność: model mówi wprost, że `PoseTarget` to `T_base_target`, a
`ToolDirectionTarget::direction` to kierunek **w bazie**. Równanie brzmi
`T(0,3)(q) = target.x`, gdzie lewa strona to współrzędna TCP **w bazie** — więc
prawa też nią jest.

TCP odpowiada na pytanie **który frame robota ograniczamy**, a nie **w jakim
układzie zapisany jest cel**. Różnica jest praktyczna: caller, który uwierzyłby
v1, przeliczyłby cel do układu TCP i zbudował ograniczenie błędne o pełną
transformację narzędzia.

Najjaśniej widać to na rodzinie kierunkowej:

```text
ToolDirectionTarget{direction, ToolAxis::Z}   przy ustawionym TCP

ToolAxis::Z   -> oś Z ramki TCP          (który frame i która jego oś)
direction     -> kierunek w BAZIE         (dokąd ta oś ma patrzeć)
```

Dwie różne ramki w jednym celu, i żadnej z nich nie wolno pomylić z drugą.

Sam wybór transformacji jest **niejawny, ale nie cichy**: wynika ze stanu, który
caller sam ustawił, i jest opisany w kontrakcie metody. Parametr `TargetFrame`
byłby zmuszaniem go do powtórzenia informacji, którą już przekazał wołając
`setTcp`.

### 3.2 Przypadek, który naprawdę zabija, i jego rozstrzygnięcie

Niebezpieczna nie jest sama reguła, tylko stan pośredni:

```text
setTcp(...)                       TCP jest
buildTcpForwardKinematics()       NIE zostało wywołane
buildEquationSystem(target)       ???
```

Cichy odwrót do `forwardKinematics()` dałby tu układ błędny o offset narzędzia —
czyli dokładnie ten błąd, przed którym cały ten rozdział ma chronić. Dlatego:

```text
TCP ustawiony, TCP FK niezbudowane  ->  TcpForwardKinematicsNotBuilt
```

**Nigdy odwrotu.** Ta sama zasada, którą fasada stosuje już dziś, raportując
pierwszy brakujący krok zamiast ostatniego.

Symetrycznie, przy braku TCP: jeśli `forwardKinematics()` nie zostało zbudowane,
błędem jest `ForwardKinematicsNotBuilt` — kod, który już istnieje.

---

## 4. API

```cpp
// URDF + TCP + target -> an explicit system of IK equations.
//
// WHICH FRAME OF THE ROBOT is constrained follows from the facade's own state,
// and this is the one place in the project where that ambiguity can be closed
// rather than documented:
//
//     a TCP is set      -> the TCP frame is constrained, and
//                          tcpForwardKinematics() is used
//     no TCP is set     -> the chain-tip frame is constrained, and
//                          forwardKinematics() is used
//
// What does NOT change either way: the target's own quantities are expressed in
// the BASE frame, always. A PoseTarget is T_base_target, a direction is a
// direction in the base frame, and the equations read T(0,3)(q) = target.x with
// both sides in base coordinates. Setting a TCP does not mean "re-express the
// target in tool coordinates" -- doing that would produce a constraint wrong by
// the whole tool transform. It means "constrain the tool frame instead of the
// chain tip".
//
// For a ToolDirectionTarget both frames appear at once and neither may be
// confused for the other: the axis names an axis OF THE CONSTRAINED FRAME, while
// the direction it must point along is given in the base frame.
//
// If a TCP is set but its transform has not been built, this FAILS with
// TcpForwardKinematicsNotBuilt. It never falls back to the chain-tip transform:
// that would return a system wrong by exactly the tool offset, which compiles,
// runs and lies.
//
// const, and returning by value rather than storing: a target is a question
// asked of the model, not a part of it. The returned system is a snapshot -- it
// cannot go stale, because nothing in the facade owns it. Copying one duplicates
// Expression handles, never trees.
[[nodiscard]] std::expected<IkEquationSystem, IkEquationBuilderError>
buildEquationSystem(const IkTarget& target) const;
```

Jedna metoda, przyjmująca **wariant**, a nie sześć przeciążeń. Rozdział na
konkretne przeciążenia `ConstraintBuilder` robi wewnątrz jawny zestaw przeciążeń
— ten sam mechanizm, który w F2.6 dwa razy zatrzymał build przy dokładaniu
alternatywy, i który zrobi to znowu, gdy dojdzie siódma.

`ConstraintBuilder` dochodzi do składowych fasady jako `constraintBuilder_`.

---

## 5. Graf stanu — bez zmian

To jest najmocniejszy skutek wariantu A i wart osobnego zdania:

```text
              RobotDescription
                     |
               KinematicChain
                 +---+---+
                 |       |
     ForwardKinematics   TCP
                 |       |
                 +---+---+
                     |
              TcpForwardKinematics
```

**Graf nie rośnie.** Kontrakt akcesorów nie rośnie. Tabela „co jest nieświeże po
czym" nie rośnie. Jedyna nowa operacja jest `const` i nie ma węzła.

---

## 6. Model błędów

Dwa nowe kody i jedno nowe pole:

```cpp
enum class IkEquationBuilderErrorCode
{
    // ... siedem dotychczasowych ...
    TcpForwardKinematicsNotBuilt,   // TCP jest, ale T_base_tcp(q) nie
    ConstraintBuildFailed           // ConstraintBuilder odrzucił cel albo układ
};

struct IkEquationBuilderError
{
    IkEquationBuilderErrorCode code{};
    std::string message;
    std::optional<KinematicChainError> chainError;
    std::optional<ConstraintBuilderError> constraintError;   // nowe
};
```

Inwariant rośnie o zdanie symetryczne do dotychczasowego: `constraintError` ma
wartość **wtedy i tylko wtedy**, gdy `code == ConstraintBuildFailed`.

Błąd jest **zachowywany, nie spłaszczany** — tak samo jak `chainError` od F2.3.
Caller, który poda cel z niefinitą pozycją, dostaje:

```text
IkEquationBuilderErrorCode::ConstraintBuildFailed
  -> ConstraintBuilderErrorCode::InvalidTarget
     -> TargetValidationErrorCode::NonFinitePosition
```

Trzy warstwy, żadna nie zgubiona. To jest ten sam wzorzec, który F2.4 wprowadził
w `ConstraintBuilderError`, a F2.5Q obronił przy kwaternionie.

Kolejność sprawdzania prerekwizytów, jak dziś: **pierwszy brakujący krok**.

```text
1. brak łańcucha                    -> KinematicChainNotSelected
2. brak FK                          -> ForwardKinematicsNotBuilt
3. TCP jest, brak TCP FK            -> TcpForwardKinematicsNotBuilt
4. ConstraintBuilder odmówił        -> ConstraintBuildFailed
```

Krok 2 jest przed 3 świadomie: bez FK nie ma i tak z czego zbudować TCP FK, więc
raportowanie od razu kroku 3 kazałoby callerowi zrobić rzecz, której jeszcze nie
może.

---

## 7. Co ten etap zamyka z listy znanych braków — i czego nie

Dwa wiersze z `STATUS.md` dotykają dokładnie tego, co robi F2.7. Warto powiedzieć
precyzyjnie, co się z nimi dzieje, bo łatwo tu przesadzić w obie strony.

**„Cel nie zna swojej ramki"** — przy czym „ramka" znaczy tu **który frame robota
jest ograniczany** (TCP czy koniec łańcucha), a nie układ współrzędnych wartości
celu; te są zawsze w bazie (3.1). Fasada **usuwa okazję**, nie usuwa braku.
`IkTarget` nadal nie wie, który frame ogranicza, i nadal nie może wiedzieć. Ale
caller fasady nie ma już czym się pomylić: nie podaje transformacji, więc nie
poda złej. Brak zostaje otwarty dla użytkownika `ConstraintBuilder` wprost — i
tak ma zostać, bo zamknięcie go w typach wymaga transformacji otagowanej ramką,
czyli warstwy konwersji, której nikt jeszcze nie potrzebuje.

**„Para (transformacja, łańcuch) nie jest sprawdzana pod kątem pochodzenia".**
To samo: fasada **posiada** oba, więc para jest zgodna z konstrukcji. Symbole
nadal są pozycyjne (`q1`, `q2`, …), więc dwa sześcioosiowe roboty nadal by się
przez ten test przemknęły — u kogoś, kto woła builder bezpośrednio.

Jedno zdanie do zapamiętania i do wpisania w `STATUS.md`: **F2.7 zmniejsza
powierzchnię błędu, nie zmienia systemu typów.** Obietnica mocniejsza byłaby
nieprawdziwa.

---

## 8. Testy

```text
z roadmapy
  BuildsKr4PositionIkEquationsFromUrdf      end-to-end, bez TCP
  BuildsKr640PositionIkEquationsFromUrdf    end-to-end, bez TCP
  BuildsFullPoseEquationsWithTcp            end-to-end, z TCP, 12 równań
  ChangingChainClearsTcpAndRequiresFreshForwardKinematics   patrz niżej
  ReusesFacadeForDifferentRobotAndTarget    dwa roboty, dwa cele, jedna fasada

wybór transformacji -- sedno etapu
  UsesTcpTransformWhenTcpIsSet              lhs to sameNode z tcpForwardKinematics()
  UsesChainTipTransformWhenNoTcpIsSet       lhs to sameNode z forwardKinematics()
  RejectsTargetWhenTcpForwardKinematicsMissing   TCP jest, TCP FK nie -> błąd,
                                            NIE cichy odwrót do FK
  ChangingTcpChangesTheEquationSystem       ten sam cel przed i po setTcp daje
                                            inne lhs -- offset narzędzia widoczny

prerekwizyty i błędy
  ReportsFirstMissingStep                   trzy stany, trzy kody, kolejność z 6
  PreservesConstraintBuilderError           InvalidTarget + zagnieżdżony
                                            TargetValidationErrorCode, inwariant
                                            pól błędu ORAZ nietknięte akcesory

rodzaje zadań przez fasadę
  BuildsDirectionSystemThroughFacade        jeden z rodzajów F2.6, żeby wariant
                                            IkTarget był naprawdę przepuszczony
```

Razem **dwanaście** testów: 5 + 4 + 2 + 1. v1 miała na liście trzynaście pozycji
przy zapowiedzianych dwunastu — `DoesNotChangeStateOnFailure` jest teraz drugim
blokiem `PreservesConstraintBuilderError`, i to jest naturalne miejsce: metoda
jest `const`, więc sprawdzenie tożsamości wskaźników akcesorów robi się przy tym
samym nieudanym wywołaniu, które bada zagnieżdżony błąd.

**Dwa testy z roadmapy znikają i to jest konsekwencja wariantu A**, nie
przeoczenie:

- `ChangingTargetInvalidatesEquationSystem` — nie ma czego unieważniać. Cel nie
  jest stanem, a zwrócony układ jest migawką callera. Test sprawdzałby zdanie,
  które przy tym API jest tautologią.
- `ChangingTcpInvalidatesEquationSystem` — zastąpiony przez
  `ChangingTcpChangesTheEquationSystem`, który sprawdza rzecz **mocniejszą**: nie
  że coś zostało wyzerowane, tylko że drugi układ naprawdę odnosi się do innego
  punktu.

`ChangingChainClearsTcpAndRequiresFreshForwardKinematics` — v1 opisywała ten
test przez **stan, który nie może zaistnieć**. `selectChain` czyści TCP razem z
całą resztą poddrzewa, więc po nim nie ma żadnego TCP, a zatem nie ma też powodu
żądać TCP FK. Prawdziwy przebieg:

```text
po selectChain
    kinematicChain()        nowy
    forwardKinematics()     nullptr
    tcp()                   nullptr
    tcpForwardKinematics()  nullptr

buildEquationSystem(target)      -> ForwardKinematicsNotBuilt
                                    (NIE TcpForwardKinematicsNotBuilt)

po buildForwardKinematics()
buildEquationSystem(target)      -> OK, ścieżką końca łańcucha
```

Ostatnie dwa wiersze są tu najważniejsze: **brak TCP nie jest brakiem
prerekwizytu**, tylko innym, równie legalnym trybem pracy. Caller, który chce
z powrotem TCP, musi jawnie zawołać `setTcp` i `buildTcpForwardKinematics` —
i to jest dokładnie ten kontrakt, który fasada miała od F2.1.

---

## 9. Plan zmian w plikach

```text
src/ik_equations/IkEquationBuilder.hpp   +2 kody, +1 pole błędu, +1 metoda,
                                          +ConstraintBuilder w składowych
src/ik_equations/IkEquationBuilder.cpp   +buildEquationSystem, +wizytor wariantu
tests/test_ik_equation_builder.cpp        +12 testów
proposal/KinemaForge_Phase2_Phase3_Roadmap.md   ZMIANA: wariant A zamiast B w
                                          „docelowym przepływie", dwa testy
                                          end-to-end zastąpione (8)
STATUS.md                                 pipeline, licznik testów, dwa wiersze
                                          znanych braków przeformułowane (7),
                                          DoD Fazy 2
README.md                                 przykład użycia kończy się układem
                                          równań, nie macierzą FK
```

**Zero zmian** w: `ConstraintBuilder`, całym modelu, warstwie symbolicznej,
loaderze i budowniczych transformacji. Fasada wyłącznie **składa** to, co już
istnieje — i jeśli okaże się, że musi cokolwiek z tego zmienić, znaczy to, że
któryś z poprzednich etapów zostawił dziurę.

---

## 10. Definition of Done Fazy 2 i release

```text
URDF + TCP + cel  ->  poprawny symboliczny IkEquationSystem
```

Konkretnie, po tym etapie:

```text
[x] stała transformacja TCP, numerycznie zweryfikowana        F2.1, F2.2
[x] model równań i celów                                       F2.3
[x] dziesięć uwag review z 2026-08-17 zamkniętych              F2.3.R
[x] pozycja, pełna poza                                        F2.4, F2.5
[x] orientacja jako macierz albo kwaternion                    F2.5Q
[x] kierunek osi narzędzia i kąt do kierunku odniesienia       F2.6
[ ] jedno wejście: URDF + TCP + cel -> IkEquationSystem        F2.7
```

Release **`v0.2-ik-equations`** po zamknięciu ostatniego wiersza.

Zdanie, które musi paść w `STATUS.md` przy ogłaszaniu DoD, żeby nie obiecać za
dużo: **projekt umie w tym momencie formułować IK, ale nie umie go rozwiązywać.**
Wyprowadzenie `q` to Faza 3 i jest to praca cięższa niż wszystko dotychczas
razem wzięte.

---

## 11. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Cichy odwrót do FK przy ustawionym TCP | **realne, to jest główne ryzyko etapu** | `TcpForwardKinematicsNotBuilt` zamiast fallbacku (3.2) + `RejectsTargetWhenTcpForwardKinematicsMissing` |
| Fasada obiecuje, że zamknęła brak „cel nie zna ramki" | **realne** | 7: usuwa okazję, nie brak; zdanie wprost w `STATUS.md` |
| Wariant A odbiega od roadmapy po cichu | realne | Roadmapa **zmieniana jawnie** (9), precedens F2.5 |
| Sześć rodzajów zadań, jedno wejście — łatwo zgubić alternatywę | niskie | Jawny zestaw przeciążeń przy `std::visit`; siódma alternatywa zatrzyma build |
| Błąd buildera spłaszczony do stringa | niskie | `constraintError` obok `chainError`, inwariant jak w F2.3 (6) |
| Kopiowanie `IkEquationSystem` przy zwrocie | **pozorne** | Kopia duplikuje uchwyty `Expression`, nie drzewa — typ mówi to w komentarzu klasy |
| Caller przelicza cel do układu TCP | **realne, złapane w review v1** | Kontrakt mówi wprost, że wartości celu są w bazie, a TCP wybiera ograniczany frame (3.1, 4) |
| Brak TCP mylony z brakiem prerekwizytu | realne | 8: po `selectChain` ścieżka końca łańcucha jest legalna i ma test |

---

## 12. Otwarte pytania do review

**12.1 Nazwa metody.** `buildEquationSystem(target)` czy `equationSystemFor(target)`?
Pierwsza mówi, że coś powstaje (bo powstaje), druga mocniej sugeruje zapytanie.
Rekomendacja: pierwsza — „build" jest w tym projekcie czasownikiem od
budowniczych, a metoda faktycznie buduje.

**12.2 Czy fasada ma wystawiać użytą transformację?** Np.
`const SymbolicTransform* transformUsedForTargets() const`. Argument za:
diagnostyka. Przeciw: to jest wywnioskowalne ze stanu (`tcp() != nullptr`), a
każdy dodatkowy akcesor to kolejny wiersz w kontrakcie nieświeżości.
Rekomendacja: **nie** — dopóki ktoś nie pokaże przypadku, w którym nie da się
tego wywnioskować.

**12.3 Czy `buildEquationSystem` ma być `const`?** Rekomendacja: tak, i to jest
część wartości wariantu A. Warunek: `ConstraintBuilder` musi być wołalny z
`const` fasady — jest, bo jego `build()` jest `const`.

---

## 13. Rekomendacja

Przyjąć **wariant A** (2.1) — cel jako argument, układ zwracany przez wartość,
metoda `const`, graf stanu bez zmian. Przyjąć regułę wyboru transformacji z 3.1
wraz z **zakazem odwrotu** z 3.2, model błędów z 6 i sformułowanie z 7 o tym, co
ten etap naprawdę zamyka.

Przyjąć też jawnie drugą połowę pytania roadmapy (2.4): **cel jest argumentem,
TCP zostaje stanem**.

Po zatwierdzeniu — proposal implementacyjny z pełnym kodem i dwunastoma testami,
gate `422/422` (410 + 12).

Po F2.7: **Definition of Done Fazy 2**, release `v0.2-ik-equations` i przejście
do Fazy 3, która zaczyna się od `F3.1` — semantyki rewrite engine'u.
