# Proposal: `ConstraintBuilder` — `FullPose` (F2.5), architektura

**Etap:** F2.5 · **v2 — do review** · 2026-08-20 · część 1 z 2

> **v2, po review.** Dwa blockery i dwie korekty. (1) Kwaternion nie „wypada" z
> etapu w proposalu — **roadmapa została zmieniona**, testy przeszły do nowego,
> nazwanego etapu `F2.5Q` (5). (2) Kontrakt „9 równań, 3 DOF" był bezwarunkowy i
> jest nieprawdziwy: tolerancyjna walidacja przyjmuje cel, który jest tylko
> **przybliżeniem** SO(3), więc dziewięć dokładnych równań może nie mieć
> dokładnego rozwiązania — przepisane 3.4, dodane 3.5 z policzonym przykładem.
> (3) Wzmocniony argument przeciw wariantowi D: problemem jest globalna
> degeneracja, nie brak `acos` (3.3). (4) Doprecyzowany sposób porównania w
> `PositionEquationsMatchPositionOnlyBuild` (6). Wariant **A zostaje**.

Poprzednik: [`proposal-constraint-builder-architecture.md`](../F2.4-constraint-builder-position/proposal-constraint-builder-architecture.md)
(F2.4, `PositionOnly`, zaimplementowane). Ten dokument rozstrzyga **jak wygląda
ograniczenie orientacji**, czyli jedyne pytanie, którego F2.4 nie dotykało.

## Prompt

> Roadmapa, F2.5 — rozszerzyć target o orientację:
>
> ```text
> 3 równania pozycji + niezależny zestaw równań orientacji
> ```
>
> Nie należy automatycznie zakładać dziewięciu niezależnych równań macierzy
> rotacji. Proposal musi rozstrzygnąć: które elementy macierzy wybrać; czy użyć
> pełnych dziewięciu równań z informacją o redundancji; czy przejść przez błąd
> orientacji; jak późniejszy solver ma rozpoznawać strukturę.
>
> Minimalne testy: `BuildsFullPoseEquationSystem`,
> `IncludesPositionAndOrientationConstraints`, `RejectsNonOrthogonalTargetRotation`,
> `RejectsReflectionTargetRotation`, `AcceptsValidQuaternionTarget`,
> `ProducesEquivalentTargetsForQuaternionAndMatrix`.
>
> Gate: pełna poza → jawny `IkEquationSystem`; target rotation jest poprawnym
> obrotem; brak ukrytej konwersji zależnej od MotionBridge.

## Status: model już czeka na tę decyzję

F2.3 zbudowało model tak, żeby zostawić to pytanie otwarte — i zapisało to w
kodzie wprost:

```cpp
case IkTaskKind::Pose:
    // How MANY orientation equations is F2.5's decision; presence is not.
    if (orientationCount == 0)
        return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                         "a pose task needs at least one orientation equation");
```

Co model **już** wymusza i czego F2.5 nie może obejść:

```text
OrientationEquationSource::create(row, column)   0..2, zamknięte
kolejność        pozycja X, Y, Z, potem orientacja ściśle rosnąco row-major
duplikaty        dwa równania na tę samą komórkę -> DuplicateEquationSource
zawartość        Pose wymaga kompletnej pozycji ORAZ >= 1 równania orientacji
walidacja celu   validate(PoseTarget): skończoność, ortogonalność (1e-8),
                 wyznacznik +1
```

Czyli mechanika jest gotowa. Do rozstrzygnięcia zostaje **liczba i wybór komórek**
oraz **czy `PoseTarget` przyjmuje kwaterniony**.

## 1. Cel i zakres

**W zakresie:**

- przeciążenie `build(transform, chain, const PoseTarget&)`,
- wybór reprezentacji ograniczenia orientacji (3),
- rozszerzenie sprawdzenia symboli na blok rotacji,
- kontrakt „ile równań ≠ ile więzów", zapisany tam, gdzie konsument go zobaczy.

**Poza zakresem:**

- **cel kwaternionowy** — wydzielony do etapu `F2.5Q` jawną zmianą roadmapy (5),
- constrainty kierunkowe i częściowa orientacja — F2.6,
- integracja z fasadą — F2.7,
- nowe operacje w `ExpressionFactory` — 3.3 pokazuje, że wybrana reprezentacja
  ich nie potrzebuje,
- upraszczanie i rozwiązywanie — Faza 3.

## 2. Co się **nie** zmienia względem F2.4

Warto powiedzieć wprost, bo to jest miara tego, czy F2.4 miało dobrą granicę:

```text
API buildera            dochodzi przeciążenie, nic się nie zmienia
model błędów            te same trzy kody, bez zmian
lhs = rhs               bez ukrytego odejmowania, bez zmian
kolejność X, Y, Z       identyczna, te same trzy równania pozycji
kolektor symboli        ta sama funkcja, więcej komórek na wejściu
gwarancja symboli       symbols ⊆ unknowns, bez zmian
```

Trzy równania pozycji dla `PoseTarget{position, orientation}` mają być
**identyczne** z tymi, które daje `PositionTarget{position}` — łącznie z
tożsamością węzłów `lhs`. To jest test, nie deklaracja: 6.

## 3. Reprezentacja ograniczenia orientacji

### 3.1 Warianty

| # | Wariant | Równań | Ocena |
|---|---|---|---|
| **A** | Wszystkie dziewięć komórek `R(q)(i,j) = R_target(i,j)` | 9 | **Wybrany** |
| B | Dwie kolumny; trzecia wynika z iloczynu wektorowego | 6 | Odrzucony — 3.2 |
| C | Stały podzbiór trzech „niezależnych" komórek | 3 | Odrzucony — 3.2 |
| D | Wektor błędu orientacji (część antysymetryczna, oś-kąt) | 3 | Odrzucony — 3.3 |

### 3.2 Dlaczego nie B i nie C — redundancja to nie to samo co nadmiarowość

Rotacja właściwa ma trzy stopnie swobody, więc dziewięć równań komórkowych jest
**redundantne**. To prawda i proposal jej nie ukrywa. Ale z redundancji nie
wynika, że któreś równanie jest **fałszywe** — każde z dziewięciu jest prawdziwym
zdaniem o celu. Usunięcie sześciu nie czyni układu „bardziej poprawnym"; czyni go
**uboższym o informację**, której ten etap nie ma prawa wyrzucić.

**Wariant C upada na tym, że nie istnieje stały niezależny podzbiór trzech
komórek.** Które trzy są niezależne, zależy od samego obrotu: dla `R = I`
pochodne komórek diagonalnych względem kątów znikają w pierwszym rzędzie, więc
podzbiór dobry dla obrotu ogólnego jest zdegenerowany dla obrotu zerowego. Wybór
„na sztywno" byłby więc poprawny dla większości celów i cicho zły dla niektórych
— najgorszy możliwy rodzaj błędu w warstwie, która ma **formułować** zadanie.

**Wariant B jest uczciwszy** — trzecia kolumna to iloczyn wektorowy dwóch
pierwszych, o ile obie macierze są obrotami właściwymi, a to `validate()` dla
celu sprawdza, i `R(q)` spełnia z konstrukcji (iloczyn obrotów). Odrzucam go z
dwóch powodów:

1. **Implikacja żyje poza układem równań.** Konsument, który dostanie sześć
   równań, nie ma jak odczytać z `IkEquationSystem`, że trzecia kolumna jest
   wymuszona — musiałby wiedzieć, że producent zakładał właściwość obrotu obu
   stron. Układ przestaje być samowystarczalny, a to jest dokładnie ta cecha, dla
   której `IkEquationSystem` niesie własne `unknowns` zamiast kazać je odtwarzać.
2. **Założenie o `R(q)` jest globalne, nie lokalne.** „FK jest obrotem" wynika z
   tego, jak zbudowano cały łańcuch. Gdyby kiedykolwiek podano transformację
   spoza tego pipeline'u — a `build()` przyjmuje dowolną `SymbolicTransform`,
   patrz F2.4 §2.3 — sześć równań przestałoby wystarczać, cicho.

Wariant A nie ma żadnego z tych problemów: dziewięć równań mówi wszystko, co
trzeba, bez odwoływania się do własności, których układ nie niesie.

### 3.3 Dlaczego nie D — degeneracja, nie brak funkcji

> **Poprawione w v2.** v1 twierdziła, że trójskładnikowy błąd orientacji wymaga
> `acos`/`atan2`/pierwiastka. To nieprawda dla wszystkich takich reprezentacji i
> osłabiało argument.

Część z nich da się zbudować **wyłącznie** z tego, co fabryka ma dzisiaj
(`+`, `-`, `*`) — na przykład część antysymetryczna:

```text
vee(R_target^T · R(q) − R(q)^T · R_target)
```

albo iloczyny wektorowe kolumn. Problem leży gdzie indziej i jest poważniejszy
niż koszt implementacji: **te trzy składowe nie są globalnie równoważne
`R(q) = R_target`**. Część antysymetryczna zeruje się nie tylko przy zerowym
błędzie, ale i przy błędzie obrotu o `π` — sinus kąta błędu jest wtedy zerem.
Układ „rozwiązany" mógłby więc odpowiadać orientacji odwróconej o pół obrotu,
i to bez żadnego sygnału dla warstwy wyżej.

Reprezentacje wolne od tej degeneracji (oś-kąt z `acos`, kwaternion błędu z
pierwiastkiem) wymagają operacji, których `ExpressionFactory` nie ma, a ich
dołożenie niesie własne pytania o dziedzinę — `acos` jest określone na
`[-1, 1]`, a wyrażenie symboliczne nie ma jak tego obiecać. To ta sama klasa
problemu, dla której projekt nie zwija `x * 0 → 0`.

Do tego dochodzi argument o strukturze: `atan2` z wnętrznością zależną od `q`
jest dla `IkPatternDetector` ścianą, a `R(q)(0,2) = 0.5` — nie jest.

Wniosek: wariant D nie jest „za trudny", tylko **albo globalnie niepoprawny,
albo w złej warstwie**. Redukcja do trzech liczb jest krokiem *rozwiązywania*,
a Faza 2 formułuje.

### 3.4 Kontrakt: liczba równań nie jest liczbą więzów

> **Przepisane w v2.** v1 mówiła bezwarunkowo „dziewięć równań orientacji
> opisuje trzy stopnie swobody". To zdanie jest prawdziwe tylko przy założeniu,
> którego układ równań nie niesie — patrz 3.5.

Wariant A zobowiązuje do kontraktu, powiedzianego w komentarzu `build()`, w
komentarzu `IkEquationSystem` i w `STATUS.md`:

> Dla transformacji będącej sztywnym FK dziewięć równań komórkowych reprezentuje
> **pełną orientację**, której przestrzeń geometryczna ma trzy stopnie swobody.
>
> `IkEquationSystem` **nie koduje jednak rzędu ani niezależności** równań. W
> szczególności cel przyjęty przez tolerancyjny walidator może być numerycznie
> tylko **przybliżeniem** SO(3), więc dziewięć dokładnych równości nie musi mieć
> dokładnego rozwiązania.
>
> Solver nie może zatem utożsamiać liczby równań z liczbą niezależnych więzów.
> Dobór niezależnego podproblemu oraz numeryczna weryfikacja pozostałych równań
> należą do **Fazy 3**.

Dzięki `OrientationEquationSource` każde równanie ma tożsamość `(row, column)`,
więc konsument może wybierać: `IkPatternDetector` szukający nadgarstka sferycznego
dostaje pełną macierz, a nie sześć równań mniej. To jest właśnie ten podział —
F2.5 **zachowuje wszystko**, Faza 3 **wybiera i weryfikuje**.

### 3.5 Tolerancyjna walidacja kontra dokładne równania

> **Dodane w v2 — drugi blocker review.**

`TargetValidation` przyjmuje cel z tolerancją:

```text
|(R^T R − I)ij| <= 1e-8        |det(R) − 1| <= 1e-8
```

a `Equation` nie ma żadnej tolerancji — `lhs = rhs` znaczy równość. Te dwie
rzeczy się nie spotykają i trzeba to powiedzieć wprost.

Policzone dla `Rz(0.7)` z komórkami zaokrąglonymi do dziewięciu miejsc, czyli
dla całkowicie realistycznego wejścia z pliku konfiguracyjnego:

```text
cos 0.7648421872844885  ->  0.764842187
sin 0.6442176872376910  ->  0.644217687

|R^T R − I|max = 7.41e-10        |det − 1| = 7.41e-10
tolerancja 1e-8                  ->  CEL PRZYJĘTY
```

Ta macierz **nie jest** dokładnie ortogonalna, więc nie istnieje żaden dokładny
obrót odpowiadający wszystkim dziewięciu liczbom naraz. Precyzyjnie: nawet jeśli
idealna, niezaokrąglona poza jest dla tego robota osiągalna, samo zaokrąglenie
celu może zostawić residual rzędu `1e-9` i uczynić dziewięć dokładnych równości
**algebraicznie niespełnialnymi**. Osobną sprawą, której to nie dotyczy, jest
osiągalność samej pozy — residual może mieć również tamto źródło.

To nie jest usterka do naprawienia w F2.5 i nie jest powód, żeby zaostrzać
`kOrientationTolerance` (próg zmierzono na 500 000 losowych obrotach; zaostrzenie
odrzucałoby poprawne dane). To jest **własność, którą trzeba zapisać**, bo:

- konsument liczący „9 równań, 9 więzów" jest w błędzie podwójnie: raz przez
  redundancję, raz przez niespełnialność co do ostatnich cyfr,
- solver Fazy 3 i tak musi kończyć round-tripem `q → FK(q) → porównanie z celem`,
  z własną tolerancją — i roadmapa już to przewiduje.

`STATUS.md` ma już pokrewny wiersz o nieortogonalnym celu czyniącym układ cicho
niespełnialnym; F2.5 dokłada do niego przypadek celu **przyjętego**, nie
odrzuconego.

## 4. API

```cpp
// F2.4, bez zmian
[[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
build(const SymbolicTransform& transform,
      const KinematicChain& chain,
      const PositionTarget& target) const;

// F2.5
[[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
build(const SymbolicTransform& transform,
      const KinematicChain& chain,
      const PoseTarget& target) const;
```

Przeciążenie, nie `IkTarget` z rozpakowaniem w środku — decyzja z F2.4 §2.2,
podtrzymana. Wywołujący, który trzyma `IkTarget`, robi `std::visit` z jawnym
zestawem przeciążeń i dostaje kompletność sprawdzoną przez kompilator.

Algorytm jest tym samym algorytmem z jednym rozszerzonym krokiem:

```text
1. validate(target)                    -> InvalidTarget  (teraz 4 kody walidacji)
2. unknowns = jointy z variable
3. symbole 12 komórek ⊆ nazwy unknowns -> UndeclaredSymbol
4. 3 równania pozycji + 9 orientacji, w kolejności wymuszonej przez model
5. IkEquationSystem::create(Pose, ...) -> SystemRejected
```

## 5. Cel kwaternionowy — wydzielony do `F2.5Q`, z **jawną zmianą roadmapy**

> **Poprawione w v2 — pierwszy blocker review.** v1 przenosiła dwa testy poza
> etap, zostawiając roadmapę bez zmian. To dawałoby etap zamknięty jako gotowy
> mimo niespełnionego własnego DoD — proposal nie może po cichu przycinać
> zakresu, który roadmapa nadal ogłasza.
>
> **Roadmapa została zmieniona** (`KinemaForge_Phase2_Phase3_Roadmap.md`,
> 2026-08-20): oba testy wypadły z listy minimalnej F2.5 i tworzą osobny,
> nazwany etap **`F2.5Q` — target kwaternionowy**, z własnym celem, zakresem,
> czterema testami i gate'em. Etap jest na liście etapów Fazy 2 i musi być
> zamknięty przed ogłoszeniem DoD Fazy 2. Nie zniknął — dostał termin.

Powody wydzielenia, z których ostatni jest rozstrzygający:

1. **`PoseTarget` jest zatwierdzonym modelem z F2.3** i niesie `RotationMatrix3`.
   Dołożenie alternatywy to zmiana modelu, czyli zakres F2.3, nie F2.5.
2. **Konwersja ma własną semantykę walidacji.** Kwaternion wymaga normalizacji z
   własną tolerancją, niezależną od `kOrientationTolerance`, które zostało
   zmierzone dla macierzy (500 000 losowych obrotów, najgorsze odchylenie
   1.68e-9). Wrzucenie tego do F2.5 oznacza dwa różne progi w jednym proposalu.
3. **Kwaterniony w tym repo są celowo kodem wyłącznie testowym**, i ich wartość
   znika w chwili, gdy produkt zacznie ich używać. `tests/support/NumericForwardKinematics.hpp`
   mówi to wprost:

   > „Deliberately NOT part of `kinemaforge_ik`: the only value this code has is
   > being a second implementation, and that value disappears the moment
   > anything in the product starts calling it. Independence is by
   > representation, not by rewording."

   Konwersja kwaternion → macierz w produkcie **musiałaby** zostać napisana
   drugi raz, żeby nie unieważnić krzyżowej walidacji FK. To realna praca i
   realne ryzyko — nie doklejka do etapu o równaniach orientacji.

Gate roadmapy „brak ukrytej konwersji zależnej od MotionBridge" jest przy tym
spełniony w najostrzejszej postaci: **nie ma żadnej konwersji**, bo wejściem jest
macierz, którą `validate()` sprawdza jako obrót właściwy. Warunek „konwersja
produkcyjna nie dzieli kodu z `tests/support`" przeszedł do gate'u `F2.5Q`, gdzie
jest realny.

## 6. Testy

```text
z roadmapy
  BuildsFullPoseEquationSystem              12 równań, IkTaskKind::Pose
  IncludesPositionAndOrientationConstraints 3 pozycyjne + 9 orientacyjnych
  RejectsNonOrthogonalTargetRotation        InvalidTarget + NonOrthogonalOrientation
  RejectsReflectionTargetRotation           InvalidTarget + InvalidOrientationDeterminant

dołożone
  OrientationEquationsAreRowMajor           (0,0), (0,1), ..., (2,2)
  UsesRotationBlockCells                    lhs to sameNode z komórkami (0..2, 0..2)
  PositionEquationsMatchPositionOnlyBuild   te same trzy równania co PositionTarget
                                            (lhs sameNode, source ==, rhs po wartości)
  RejectsUndeclaredSymbolInRotationBlock    q999 wyłącznie w R(q), nie w translacji
  BuildsKr640FullPoseEquationSystem         end-to-end
  BuildsKr4FullPoseEquationSystem           end-to-end

przeniesione poza F2.5 (patrz 5)
  AcceptsValidQuaternionTarget
  ProducesEquivalentTargetsForQuaternionAndMatrix
```

Dwa testy zasługują na komentarz.

`RejectsReflectionTargetRotation` nie może oczekiwać kodu „reflection" — takiego
nie ma i to jest świadome. `TargetValidation` odrzuca odbicie przez wyznacznik, a
komentarz przy nim tłumaczy, dlaczego nazwa jest neutralna: przy tolerowanej
ortogonalności jednorodne skalowanie też przechodzi krok 2 i wpada dopiero na
wyznaczniku, nie będąc żadnym odbiciem. Test celuje więc w
`InvalidOrientationDeterminant`.

`RejectsUndeclaredSymbolInRotationBlock` istnieje, bo rozszerzenie z 3 na 12
komórek to jedyne miejsce, w którym gwarancja symboli z F2.4 mogłaby zostać
rozszerzona niekompletnie — kolektor puszczony po samej translacji nadal
przepuściłby wszystkie testy F2.4 i cały `PositionOnly`.

`PositionEquationsMatchPositionOnlyBuild` porównuje trzema różnymi miarami i to
jest istotne implementacyjnie: `lhs` przez `sameNode` (ta sama komórka, nie
przebudowane wyrażenie), `source` przez `==`, a `rhs` **po wartości stałej**, nie
przez `sameNode`. Powód: `ExpressionFactory::constant(1.5)` tworzy nowy węzeł
przy każdym wywołaniu — tylko zero trafia do węzła współdzielonego. Test
oczekujący `sameNode` dla niezerowego celu przechodziłby wyłącznie przypadkiem.

## 7. Plan zmian w plikach

```text
src/ik_equations/builders/ConstraintBuilder.hpp    +1 przeciążenie, komentarz 3.4
src/ik_equations/builders/ConstraintBuilder.cpp    +wspólna część prywatna,
                                                   +9 równań orientacji
src/ik_equations/model/IkEquationSystem.hpp        +akapit kontraktu z 3.4
                                                   (komentarz klasy, zero kodu)
tests/test_constraint_builder.cpp                  +10 testów
proposal/KinemaForge_Phase2_Phase3_Roadmap.md       ZROBIONE w v2: F2.5Q, lista
                                                   etapów, testy minimalne F2.5
STATUS.md                                          F2.5, licznik testów, wiersz
                                                   o rzędzie i tolerancji (3.4, 3.5)
```

Zero zmian w **API i modelu danych** — `Equation`, `IkTarget`,
`IkEquationSystem`, walidacja celu, warstwa symboliczna, loader, budowniczowie
transformacji, fasada. W `IkEquationSystem.hpp` zmienia się **wyłącznie
dokumentacja kontraktu**: akapit z 3.4 w komentarzu klasy, zero kodu.
`TargetValidation` już obsługuje `PoseTarget` w komplecie — F2.5 tylko go woła.

## 8. Rozstrzygnięte w review (v2)

| # | Pytanie z v1 | Rozstrzygnięcie |
|---|---|---|
| 1 | Czy kwaternion wypada z F2.5 | **Tak, ale nie po cichu.** Roadmapa zmieniona, powstał nazwany etap `F2.5Q` z własnym gate'em (5) |
| 2 | Wariant A (9 komórek) czy B (6, dwie kolumny) | **A.** Nic z `R_target` nie jest wyrzucane; redukcja należy do warstwy, która zna strukturę rozwiązania |
| 3 | Czy kontrakt z 3.4 ma iść też do `IkEquationSystem` | **Tak, w obu miejscach.** Czyta go solver, czyli konsument systemu, nie buildera — stąd akapit w komentarzu klasy (7) |

Otwarte pytania: brak.

## 9. Rekomendacja

Przyjąć wariant A i przeciążenie z 4. Po zatwierdzeniu — proposal implementacyjny
z pełnym kodem i dziesięcioma testami, gate 342/342 (332 + 10).

Po F2.5 zostaje **F2.5Q** (target kwaternionowy, wydzielony w tym review),
**F2.6** (constrainty kierunkowe, największy etap tej fazy) i **F2.7** (fasada,
DoD Fazy 2, release `v0.2-ik-equations`).
