# KinemaForge / IkEquationBuilder — roadmapa Fazy 2 i Fazy 3

**Data:** 2026-08-03 · **ostatnia aktualizacja:** 2026-08-21  
**Zakres:** rozwój po zakończeniu Fazy 1 (`URDF → zweryfikowane symboliczne FK`)  
**Założenie wejściowe:** `IkEquationBuilder` jest zaimplementowany, pełny clean build przechodzi, a Faza 1 jest oznaczona jako zakończona.

---

## ▶ Gdzie jesteśmy teraz

```text
F2.1  transformacja TCP — architektura          ✅ zaimplementowane
F2.2  transformacja TCP — implementacja         ✅ zaimplementowane
F2.3  model równań i targetów                   ✅ zaimplementowane
F2.3.R  remediacja po review kodu               ✅ zaimplementowane
F2.4  ConstraintBuilder: PositionOnly           ✅ zaimplementowane
F2.5  ConstraintBuilder: FullPose               ✅ zaimplementowane
F2.5Q target kwaternionowy                      ✅ zaimplementowane
F2.6  constrainty kierunkowe                    ✅ zaimplementowane (a + b)
F2.7  integracja z fasadą + milestone           ✅ zaimplementowane

FAZA 2 ZAMKNIĘTA — URDF + opcjonalny TCP + cel -> IkEquationSystem
Faza 3, F3.1 (semantyka rewrite engine)          ◀ TUTAJ JESTEŚMY
```

Proposale etapów zamkniętych leżą w `proposal/done/<etap>/` — przeniesienie tam
jest ostatnim krokiem etapu, po clean buildzie, pełnych testach i aktualizacji
`STATUS.md`. W `proposal/` zostaje wyłącznie to, co jest w toku.

Etap `F2.3.R` **nie był planowany** w pierwotnej roadmapie — powstał w reakcji
na pełne review kodu z 2026-08-17, które wykazało dwa błędy o wysokiej wadze w
warstwie loadera. Numeracja `F2.3.R` jest celowa: nie przenumerowuje F2.4–F2.7,
do których odwołuje się już `STATUS.md`.

---

# 1. Punkt startowy

Po Fazie 1 projekt potrafi:

```text
URDF
→ RobotDescription
→ KinematicChain
→ SymbolicTransform T_base_tool(q)
→ podstawienie q
→ numerycznie zweryfikowana macierz FK
```

Publicznym punktem wejścia jest fasada `IkEquationBuilder`.

Dalszy rozwój dzielimy na:

```text
Faza 2 — budowanie układu równań IK
Faza 3 — rozwiązywanie analityczne i generowanie kodu
```

Ważne rozróżnienie:

```text
Faza 2:
    tworzy równania IK

Faza 3:
    upraszcza i rozwiązuje te równania
```

---

# 2. Faza 2 — IK Equation Builder

## 2.1 Cel Fazy 2

Doprowadzić projekt do stanu:

```text
URDF
+ definicja TCP
+ zadanie ruchowe / target
→ symboliczny układ równań IK
```

Przykład dla pozycji:

```text
Px(q1...q6) = target_x
Py(q1...q6) = target_y
Pz(q1...q6) = target_z
```

Przykład dla pełnej pozy:

```text
p(q) = p_target
R(q) = R_target
```

Przykład dla kierunku dyszy:

```text
Z_tcp(q) = desired_direction
```

Faza 2 **nie rozwiązuje** równań. Jej wynikiem jest poprawny, jawny model `IkEquationSystem`.

## 2.2 Struktura Fazy 2

Faza 2 ma **2 główne komponenty**:

1. obsługa `tool0 → TCP`,
2. `ConstraintBuilder`.

Realizacyjnie dzielimy ją na **8 etapów** — siedem planowanych pierwotnie plus
`F2.5Q` (target kwaternionowy), wydzielony z F2.5 w review architektury
2026-08-20. Osobno, poza tą numeracją, doszedł `F2.3.R` — etap remediacyjny po
review kodu z 2026-08-17.

---

## F2.1 — architektura stałej transformacji `tool0 → TCP`

### Cel

Zaprojektować reprezentację i sposób dołączenia stałej transformacji narzędzia:

```text
T_base_tcp(q)
=
T_base_tool0(q)
· T_tool0_tcp
```

### Decyzje do podjęcia

- model danych TCP:
  - translacja + RPY,
  - czy osobny `TcpTransform`,
  - czy ogólniejszy `FixedRigidTransform`;
- miejsce walidacji wartości skończonych;
- konwencja RPY — zgodna z URDF:
  ```text
  R = Rz(yaw) · Ry(pitch) · Rx(roll)
  ```
- nazwa komponentu:
  - `TcpTransformBuilder`,
  - `ToolTransformBuilder`,
  - funkcja w warstwie symbolicznej;
- użycie istniejącego `multiplyTransforms`;
- integracja z `IkEquationBuilder`;
- reguły unieważniania po zmianie TCP;
- model błędów;
- tożsamościowy TCP jako przypadek domyślny.

### Non-goals

- YAML i konfiguracja MotionBridge;
- dynamiczne narzędzie;
- kalibracja TCP;
- constrainty;
- solver IK.

### Dokument

```text
proposal-tcp-transform-architecture.md
```

---

## F2.2 — implementacja i numeryczna walidacja TCP

### Zakres

- implementacja modelu TCP;
- zbudowanie stałego `SymbolicTransform`;
- składanie:
  ```text
  T_base_tcp = multiplyTransforms(T_base_tool0, T_tool0_tcp)
  ```
- integracja z fasadą;
- numeryczna weryfikacja względem ręcznych i kwaternionowych oracle.

### Minimalne testy

```text
IdentityTcpLeavesForwardKinematicsUnchanged
AppliesTranslationOnlyTcp
AppliesRotationOnlyTcp
AppliesCombinedTcp
AppliesTcpInToolFrameNotBaseFrame
ChangingTcpInvalidatesTcpForwardKinematics
RejectsNonFiniteTcpTranslation
RejectsNonFiniteTcpRotation
BuildsKr4TcpForwardKinematics
BuildsKr640TcpForwardKinematics
```

### Gate

```text
T_base_tcp jest symboliczne
ostatni wiersz pozostaje [0 0 0 1]
wyniki numeryczne zgadzają się z niezależną referencją
zmiana TCP nie pozostawia starego wyniku
pełny zestaw testów przechodzi
```

---

## F2.3 — model domenowy równań i targetów

### Cel

Zanim powstanie `ConstraintBuilder`, potrzebny jest jawny model jego wejścia i wyjścia.

### Planowane typy

Kandydaci:

```cpp
struct Equation
{
    Expression lhs;
    Expression rhs;
};

struct IkEquationSystem
{
    std::vector<Equation> equations;
    ConstraintKind kind;
};
```

Targety:

```text
PositionTarget
PoseTarget
ToolDirectionTarget
PositionAndDirectionTarget
```

Do rozstrzygnięcia:

- czy prawa strona jest `Expression`, czy `double`;
- jak reprezentować target rotation:
  - macierz 3×3,
  - kwaternion,
  - RPY tylko na granicy API;
- czy targety mają walidować:
  - wartości skończone,
  - ortogonalność macierzy,
  - `det(R) = +1`,
  - jednostkowość kierunku;
- jak zapisywać metadane pochodzenia równania;
- czy `Equation` jest uporządkowaną parą `lhs = rhs`, czy normalizowaną postacią:
  ```text
  lhs - rhs = 0
  ```

### Dokumenty

```text
proposal-ik-equation-model-architecture.md
proposal-ik-equation-model-implementation.md
```

---

## F2.3.R — remediacja po review kodu  ✅ zamknięty 2026-08-18

### Skąd się wziął ten etap

Pełne review kodu (2026-08-17) zgłosiło 10 uwag. **Wszystkie 10 zostało
odtworzone eksperymentalnie** przed przyjęciem — nie na podstawie samej lektury
kodu. Review nie zgłosiło żadnej uwagi do FK, warstwy symbolicznej, TCP ani
modelu równań; wszystkie znaleziska o wysokiej wadze siedzą w starej warstwie
`mt::kinematics`, czyli w kodzie sprzed właściwego pipeline'u.

Kluczowe: **nic z F2.1–F2.3 nie jest cofane ani przepisywane.** Nowy kod okazał
się mocniejszy od starego loadera i to stary loader jest przedmiotem tego etapu.

### Dowody — co dokładnie odtworzono

```text
ten sam KR4, dwa uporządkowania XML:
    kr4_r600.urdf                 → accepted, chain = 6 actuated joints
    kr4_r600.urdf (2 jointy swap) → REJECTED, incomplete_kinematic_chain

URDF z cyklem root→A→B→A:
    load_urdf(...)                → nie kończy się w 8 s (exit 124)

URDF rozspójniony (dwa korzenie):
    2 jointy w pliku              → ACCEPTED, chain = 1 joint
                                    (drugie poddrzewo znika po cichu,
                                     wybór korzenia zależy od kolejności
                                     iteracji unordered_map)

<limit velocity="1" effort="1"/> bez lower/upper:
    accepted, limits = [0, 0]     → joint aktuowany zamrożony po cichu
<limit lower="banana" .../>       → accepted
<limit velocity="nan" .../>       → accepted

within_limits({NaN})              → true
build(robot{links={"base"}}, "base", "ghost") → ACCEPTED

Release (-DNDEBUG), 1e308 * 1e308:
    isConstant = true, constantValue = inf   → inwariant DAG złamany

g++ -Wall -Wextra -Werror na rdzeniu:
    dokładnie 2 błędy, nic więcej
```

### Podział na proposale

Grupowanie po warstwie, nie po kolejności uwag w review — jeden proposal dotyka
jednej warstwy.

Wszystkie dokumenty tego etapu żyją w **osobnym podfolderze**, bo nie są
kolejnym krokiem funkcjonalnym roadmapy, a naprawą znalezisk — i mają dać się
czytać jako jedna paczka:

```text
proposal/done/F2.3.R-review-remediation/     (od 2026-08-21; wcześniej proposal/F2.3.R-review-remediation/)
```

Indeks etapu:
[`done/F2.3.R-review-remediation/README.md`](done/F2.3.R-review-remediation/README.md).

| # | Proposal (w `F2.3.R-review-remediation/`) | Uwagi z review | Waga |
|---|---|---|---|
| R1 | `proposal-warning-policy.md` | 8, 9 | MEDIUM (proces) |
| R2 | `proposal-loader-topology-architecture.md` + `-implementation.md` | 1, 2, 3 + znalezisko dodatkowe | **HIGH** |
| R3 | `proposal-loader-limits-strict-parsing-architecture.md` + `-implementation.md` | 4 | MEDIUM/HIGH |
| R4 | `proposal-legacy-robot-model-boundary-architecture.md` + `-implementation.md` | 5, 6 | MEDIUM |
| R5 | `proposal-expression-factory-finite-invariant-architecture.md` + `-implementation.md` | 7 | MEDIUM |

Uwaga 10 nie dostaje własnego proposala — wchodzi jako warunek kontraktowy do
F2.4 (szczegóły niżej).

`R1` jest jednym dokumentem, nie parą — precedens: `proposal-loader-test-coverage.md`.
Pozostałe idą parami architektura + implementacja, zgodnie z resztą repo.

### Kolejność i uzasadnienie

```text
R1  →  (R2 + R4)  →  R3  →  R5  →  F2.4
                            (R5 może iść równolegle — nie dotyka loadera)
```

**R2 i R4 są jedną jednostką releasowalną**, choć osobnymi proposalami —
ustalone w review R2 (v1 → v2). Usunięcie `trace_chain()` sprawia, że
`RobotModel::chain_` jest puste dla każdego robota, a wtedy `within_limits({})`
zwraca `true` dla dowolnego. Nie wolno scalać remediacji do `main` z R2 bez R4.

**R1 jest pierwsze, nie ostatnie**, i to jedyne odstępstwo od kolejności
proponowanej w review. Powód: R1 to dwie jednoliniowe poprawki plus flagi w
CMake, a jednocześnie jest to *mechanizm ochronny* dla wszystkich kolejnych
zmian. Zrobione na końcu oznaczałoby, że R2–R5 pisze się bez tej ochrony —
a R1 zawiera dokładnie tę ochronę, która wyłapuje regresję z uwagi 9.

### R1 — polityka ostrzeżeń

Zakres:

- `-Wall -Wextra` dla całego projektu, `-Werror` **tylko w CI** (nie w buildzie
  lokalnym — nowa wersja kompilatora nie powinna blokować pracy);
- naprawa dwóch istniejących naruszeń: `IkEquationBuilder.hpp:57` (diagram ASCII
  kończący linię `\`) i `KinematicChainBuilder.cpp:59` (`Frame{baseLink}`);
- utwardzenie `mapJointType` w `UrdfModelLoader.cpp:12` — samo usunięcie
  `return JointType::Fixed` nie wystarczy, bo funkcja „spadnie z końca" przy
  wartości podrobionej. Wzorem ma być rozwiązanie już zastosowane dla
  `IkTaskKind`: wyczerpujący `switch` **plus** jawne odrzucenie wartości spoza
  enumeratorów. Dwa różne zagrożenia, żadne nie zastępuje drugiego.

Gate:

```text
-Wall -Wextra -Werror przechodzi na całym rdzeniu
dodanie enumeratora do mt::kinematics::JointType wywala build
```

### R2 — topologia loadera (najważniejsze)

Uwagi 1, 2, 3 to trzy objawy jednej przyczyny: `trace_chain()` udaje, że URDF
opisuje łańcuch liniowy, podczas gdy URDF opisuje drzewo.

Do rozstrzygnięcia w proposalu architektonicznym:

- czy `trace_chain()` **usunąć w całości** (rekomendacja review i moja) — jego
  wynik `RobotModel::chain()` nie jest używany nigdzie w `src/`, a właściwy
  wybór ścieżki robi już `KinematicChainBuilder`, który rozumie branching;
- co wchodzi w miejsce: **walidacja drzewa**, nie wybór arbitralnej ścieżki —
  każdy `joint.parent`/`joint.child` musi istnieć w `<link>`, żaden link nie może
  być dzieckiem dwóch jointów, cykl musi być odrzucony, a korzeń musi być
  **jednoznaczny** (dziś rozspójniony URDF przechodzi i po cichu gubi poddrzewo,
  a wybór korzenia zależy od kolejności iteracji `unordered_map` — czyli nawet
  nie od kolejności XML);
- konsekwencja dla `KinematicChainBuilder`: po tej gwarancji unia
  `knownLinks` z jointów (linie 119–129) ma **zniknąć**, nie zostać. Dziś ta unia
  powoduje, że `BaseLinkNotFound`/`ToolLinkNotFound` nie mogą zadziałać dla
  linku wspomnianego w jakimkolwiek joincie — kody błędów są słabsze, niż wyglądają;
- nowe kody błędów i ich kolejność sprawdzania (repo ma już ustalony wzorzec
  determinizmu z `IkEquationSystem::create`);
- czy `RobotModel::chain()` / `set_chain()` znikają razem z tym (styk z R4).

Testy minimalne:

```text
AcceptsBranchingTree
RejectsUndeclaredParentLink
RejectsUndeclaredChildLink
RejectsLinkClaimedByTwoJoints
RejectsCyclicTopology
RejectsDisconnectedForest
LoadsIdenticallyRegardlessOfJointOrderInXml
BuildsKr4RegardlessOfJointOrder
```

Ostatnie dwa są istotą tego etapu: **wynik loadera nie może zależeć od kolejności
elementów w XML.**

### R3 — ścisłe parsowanie `<limit>`

Uwaga 4. `<limit>` ma być walidowany tak samo rygorystycznie jak `xyz`/`rpy`,
które mają własny parser oparty na `std::from_chars`. Dziś jest na
`as_double()`, co daje `0.0` dla brakującego **i** dla niepoprawnego atrybutu.

Do rozstrzygnięcia:

- czy brak `lower`/`upper` to błąd, czy udokumentowany default — dziś cicho daje
  `[0, 0]`, czyli **zamraża joint aktuowany**, co jest gorsze niż odrzucenie;
- czy `velocity`/`effort` są wymagane (URDF mówi, że tak, dla revolute/prismatic);
- czy `velocity`/`effort` muszą być nieujemne, nie tylko skończone;
- co z `JointLimits::hasPositionLimits`, które istnieje w modelu domenowym, ale
  nie jest dziś przez loader ustawiane — to jest już przygotowane pole pod
  `continuous`.

Testy minimalne:

```text
RejectsMissingLowerLimit          RejectsMalformedLowerLimit
RejectsMissingUpperLimit          RejectsMalformedUpperLimit
RejectsMissingVelocity            RejectsNonFiniteVelocity
RejectsMissingEffort              RejectsNonFiniteEffort
RejectsNegativeVelocity           AcceptsZeroWidthLimitRange
```

### R4 — granica starej warstwy `mt::kinematics`

Uwagi 5 i 6.

- `within_limits` zwraca `true` dla `NaN`, bo obie relacje porównania są wtedy
  fałszywe. Poprawka jednoliniowa (`!isfinite(v) → false`).
- `set_chain()` jest publiczne, a `joints_[chain_[k]]` nieindeksowane —
  sanitizer łapie heap-buffer-overflow. **To realne UB, nie teoretyczne.**

Decyzja do podjęcia: bounds check czy zwężenie API. Skłaniam się do zwężenia
(zgodnie z review), bo `chain()` jest w `src/` nieużywane, a cała ta warstwa jest
legacy — dodawanie do niej zabezpieczeń utrwala API, które prawdopodobnie ma
zniknąć. Proposal ma rozstrzygnąć, czy `set_chain`/`chain`/`within_limits`
zostają, czy padają razem z `trace_chain()` z R2.

### R5 — inwariant skończoności w `ExpressionFactory`

Uwaga 7. `ExpressionFactory` obiecuje, że stałe są skończone (`assert` w
`constant()`), ale constant folding może samo wyprodukować `inf` z dwóch
legalnych operandów. W Release `assert` znika i `ConstantNode{inf}` powstaje.

Rekomendowany kierunek (z review, i zgadzam się): fold tylko wtedy, gdy wynik
jest skończony; w przeciwnym razie zbuduj normalny węzeł operacji i pozwól
`ExpressionEvaluator` zwrócić `NonFiniteResult` — czyli wykorzystaj istniejący
model błędów, zamiast dodawać nowy.

**Zakres szerszy niż w review:** dotyczy `add`, `subtract`, `multiply` **i**
`divide`, nie tylko `multiply`. `divide` ma już `assert(!isZero(rhs))`, ale
`1e308 / 1e-308` też daje `inf`. Poprawka musi być jednolita dla wszystkich
czterech, inaczej inwariant zostaje połowiczny.

Testy minimalne:

```text
DoesNotFoldOverflowingProduct     DoesNotFoldOverflowingSum
DoesNotFoldOverflowingDifference  DoesNotFoldOverflowingQuotient
OverflowingFoldEvaluatesToNonFiniteResult
AllConstantLeavesRemainFinite
```

### Uwaga 10 — poza tym etapem

`IkEquationSystem::create` przyjmuje system, którego równania zależą od symbolu
niebędącego na liście `unknowns` (sprawdzone: `unknowns = {q1}`, równanie na
`q999` → accepted). Zgadzam się z oceną review, że **nie jest to błąd F2.3** —
proposal F2.3 świadomie nie deklarował tego inwariantu, a jego sprawdzenie
wymaga kolektora symboli, którego nie ma.

Nie zakładam więc dla tego proposala. Zamiast tego wchodzi jako **warunek
kontraktowy do F2.4**: `ConstraintBuilder` musi być jedynym producentem
`IkEquationSystem` i sam gwarantować spójność zbioru symboli. Pełna walidacja
wróci, gdy powstanie kolektor symboli — najpóźniej przed F3.

### Definition of Done etapu F2.3.R

```text
wynik loadera nie zależy od kolejności XML
URDF z cyklem jest odrzucany, nie zawiesza procesu
URDF rozspójniony jest odrzucany, nie gubi po cichu poddrzewa
odwołanie do niezadeklarowanego <link> jest odrzucane w loaderze
<limit> walidowany tak samo ściśle jak xyz/rpy
ExpressionFactory nie potrafi wyprodukować nieskończonej stałej
brak UB w mt::kinematics osiągalnego z publicznego API
-Wall -Wextra -Werror przechodzi
pełny zestaw testów przechodzi
```

Dopiero po tym gate'cie wchodzi F2.4.

---

## F2.4 — `ConstraintBuilder`: `PositionOnly`  ✅ zaimplementowane 2026-08-20

> **Zamknięty.** Trzy równania translacji, `IkTaskKind::Position`, plus warunek
> kontraktowy odziedziczony po review (uwaga 10): każdy symbol w równaniach jest
> zadeklarowaną niewiadomą — sprawdzane, nie zakładane. Dwanaście testów.
> Dokumenty: [`done/F2.4-constraint-builder-position/`](done/F2.4-constraint-builder-position/).

### Cel

Pierwszy realny builder równań IK:

```text
T_base_tcp(q)
+ target_position
→ 3 równania
```

Równania:

```text
T(0,3)(q) = target_x
T(1,3)(q) = target_y
T(2,3)(q) = target_z
```

### Minimalne testy

```text
BuildsThreePositionEquations
UsesTcpTranslationColumn
PreservesJointSymbols
UsesTargetValuesOnRightHandSide
RejectsNonFinitePositionTarget
BuildsKr4PositionEquationSystem
BuildsKr640PositionEquationSystem
```

### Gate

Po tym etapie projekt potrafi automatycznie zbudować układ równań pozycyjnego IK, ale go nie rozwiązuje.

---

## F2.5 — `ConstraintBuilder`: `FullPose`  ✅ zaimplementowane 2026-08-20

> **Zamknięty.** Wariant A: dziewięć równań komórkowych rotacji plus trzy
> pozycji, `IkTaskKind::Pose`, dwanaście równań w kolejności row-major.
> Dwanaście testów F2.4 przeszło refaktor bez zmiany choćby jednej asercji, co
> było właściwym sprawdzianem tego refaktoru. Dokumenty:
> [`done/F2.5-constraint-builder-fullpose/`](done/F2.5-constraint-builder-fullpose/).

### Cel

Rozszerzyć target o orientację.

Minimalny zakres:

```text
3 równania pozycji
+ niezależny zestaw równań orientacji
```

Nie należy automatycznie zakładać dziewięciu niezależnych równań macierzy rotacji. Proposal musi rozstrzygnąć:

- które elementy macierzy wybrać;
- czy użyć pełnych dziewięciu równań z informacją o redundancji;
- czy przejść przez błąd orientacji;
- jak późniejszy solver ma rozpoznawać strukturę.

### Minimalne testy

```text
BuildsFullPoseEquationSystem
IncludesPositionAndOrientationConstraints
RejectsNonOrthogonalTargetRotation
RejectsReflectionTargetRotation
```

> **Zmienione 2026-08-20, w review architektury F2.5.** `AcceptsValidQuaternionTarget`
> i `ProducesEquivalentTargetsForQuaternionAndMatrix` **wypadają z F2.5** i
> tworzą osobny etap `F2.5Q` niżej. Powód: wejście kwaternionowe to zmiana
> `PoseTarget`, czyli modelu zatwierdzonego w F2.3, plus druga — produkcyjna —
> implementacja konwersji, bo kwaterniony w `tests/support` mają celowo pozostać
> niezależnym oraclem. To jest osobna praca, nie doklejka do etapu o równaniach
> orientacji. Bez tej zmiany F2.5 zostałby zamknięty mimo niespełnionego
> własnego DoD.

### Gate

```text
pełna poza → jawny IkEquationSystem
target rotation jest poprawnym obrotem (walidacja macierzy, bez konwersji)
brak ukrytej konwersji zależnej od MotionBridge
```

---

## F2.5Q — target kwaternionowy  ✅ zaimplementowane 2026-08-21

> Wydzielone z F2.5 dnia 2026-08-20, podjęte i zamknięte 2026-08-21.
> Dokumenty: [`done/F2.5Q-quaternion-target/`](done/F2.5Q-quaternion-target/).
>
> **Zamknięty.** Kwaternion wchodzi jedną funkcją wytwórczą na granicy modelu
> (`makePoseTargetFromQuaternion`), więc `IkTarget`, `IkEquationSystem` i
> `ConstraintBuilder` nie zmieniły się ani o linijkę. Norma ma własną, zmierzoną
> tolerancję `1e-8`, a przyjęty kwaternion jest normalizowany przed konwersją —
> bez tego cel na krawędzi pasma dawałby macierz odchyloną `8e-8`, czyli osiem
> razy poza próg walidacji modelu. 21 testów, gate 365/365 w obu konfiguracjach.
>
> **Jedna obietnica zawężona w review i to jest kontrakt, nie szczegół:** próg
> normy **nie jest** detektorem tego, czy dane przeszły przez `float` — 41.65%
> takich kwaternionów przechodzi, a wartości dokładnie reprezentowalne w `float`
> przechodzą zawsze. Gwarancja brzmi dokładnie: skończony i w promieniu `1e-8` od
> jedności → przyjęty i znormalizowany; w przeciwnym razie odrzucony.

### Cel

Przyjmować orientację celu również jako kwaternion, bez utraty niezależności
kontroli numerycznej FK.

### Zakres i warunek konieczny

```text
+ typ kwaternionu w modelu (albo alternatywa w PoseTarget, albo osobne wejście)
+ własna tolerancja normalizacji, niezależna od kOrientationTolerance
+ konwersja kwaternion -> RotationMatrix3 NAPISANA W PRODUKCIE OD ZERA
```

Ostatni punkt jest warunkiem, nie preferencją: `tests/support/NumericForwardKinematics`
jest drugą implementacją FK i jego wartość znika, jeśli produkt zacznie wołać
ten sam kod. Krzyżowa walidacja FK musi dalej porównywać dwie niezależne
implementacje.

### Minimalne testy

```text
AcceptsValidQuaternionTarget
ProducesEquivalentTargetsForQuaternionAndMatrix
RejectsNonNormalizedQuaternion
MatchesQuaternionRotationWithoutFormingAMatrix
AgreesWithRodriguesRotationFormula
```

> **Zmienione 2026-08-21, w review architektury F2.5Q.** Wpis
> `QuaternionConversionIsIndependentOfTestOracle` zastąpiony dwoma konkretnymi
> testami, bo niezależności nie sprawdza się jednym asertem. Rozbicie odpowiada
> dwóm różnym drogom kontroli: pierwszy test porównuje `R·v` z kanapką
> kwaternionową oracle'a (która nigdy nie buduje macierzy), drugi — z macierzą ze
> wzoru Rodriguesa, czyli drogą trygonometryczną, **nieprzechodzącą przez
> kwaterniony w ogóle**. Dopiero ten drugi wyłapuje pomyloną konwencję `(w,x,y,z)`
> / `(x,y,z,w)`, która w świecie kwaternionowym jest spójna sama ze sobą.
> Strukturalna połowa warunku — brak współdzielenia kodu z `tests/support` — nie
> jest testem, tylko faktem z budowy: to źródło należy do celu `kinemaforge_tests`,
> nie do `kinemaforge_ik`.

### Gate

```text
ten sam IkEquationSystem dla kwaternionu i dla równoważnej macierzy
konwersja produkcyjna nie dzieli kodu z tests/support
```

---

## F2.6 — constrainty kierunkowe i pozycje wymuszone  ✅ zaimplementowane 2026-08-21

> **Zamknięty, w dwóch połowach.** Pięć nazw z listy niżej okazało się **dwiema
> rodzinami równań**: równością kierunku osi narzędzia (F2.6a) i kątem do kierunku
> odniesienia (F2.6b), każda z pozycją i bez niej — sześć rodzajów zadania razem z
> `Position` i `Pose`. `AxisAlignment` to ta sama rodzina co `FixedToolDirection`,
> `PositionAndAxis` to kompozycja, a **`FreeRotationAroundToolAxis` nie jest
> ograniczeniem** — to własność równań kierunku, sprawdzana testem.
>
> 45 testów, gate 410/410 w obu konfiguracjach. Dokumenty:
> [`F2.6-directional-constraints/`](done/F2.6-directional-constraints/proposal-directional-constraints-architecture.md).

### Cel

Obsłużyć zadania istotne dla narzędzi technologicznych:

```text
FixedToolDirection
PositionAndAxis
AxisAlignment
FreeRotationAroundToolAxis
FixedAngleToSurfaceNormal
```

Przykłady:

```text
Z_tcp(q) = desired_direction
```

```text
dot(Z_tcp(q), surface_normal) = cos(required_angle)
```

Przy swobodnym obrocie wokół osi dyszy nie wolno przypadkiem narzucić pełnej orientacji.

### Kolejność

1. `FixedToolDirection`;
2. `PositionAndDirection`;
3. `FreeRotationAroundToolAxis`;
4. `FixedAngleToSurfaceNormal`.

### Gate

Każdy constraint ma:

- jednoznaczną semantykę;
- policzalną liczbę stopni swobody;
- testy odróżniające go od `FullPose`;
- brak nadmiarowych ograniczeń narzucanych przez przypadek.

---

## F2.7 — integracja z fasadą i milestone Fazy 2  ✅ zaimplementowane 2026-08-21

> **Zamknięty, i razem z nim cała Faza 2.** Kandydat z „docelowego przepływu"
> niżej **został odrzucony w review architektury**: cel jest **argumentem**
> zapytania, a nie stanem fasady, więc nie ma `buildConstraints()` ani
> `equationSystem()`. Jest jedna metoda `const`:
>
> ```cpp
> std::expected<IkEquationSystem, IkEquationBuilderError>
> buildEquationSystem(const IkTarget& target) const;
> ```
>
> Powód: graf stanu istnieje dla rzeczy **drogich i współdzielonych**, a cel jest
> jednym pytaniem o taniej odpowiedzi. TCP **zostaje** stanem, bo jest
> konfiguracją narzędzia współdzieloną przez tysiące celów.
>
> Z listy testów end-to-end niżej **wypadły dwa** — `ChangingTargetInvalidates...`
> i `ChangingTcpInvalidates...` — bo przy tym API nie mają czego sprawdzać;
> zastąpiły je `ChangingTcpChangesTheEquationSystem` (test semantyczny, z
> podstawieniem) i `ChangingChainClearsTcpAndRequiresFreshForwardKinematics`.
>
> 12 testów, gate **422/422** w obu konfiguracjach. Dokumenty:
> [`done/F2.7-facade-integration/`](done/F2.7-facade-integration/proposal-phase2-facade-integration.md).

### Docelowy przepływ

Kandydat:

```cpp
IkEquationBuilder builder;

builder.loadRobotModel(...);
builder.selectChain("base_link", "tool0");
builder.buildForwardKinematics();
builder.setTcp(...);
builder.buildTcpForwardKinematics();
builder.buildConstraints(target);

const IkEquationSystem* equations = builder.equationSystem();
```

Proposal integracyjny ma rozstrzygnąć, czy API pozostaje stanowe, czy target i TCP są argumentami operacji.

### Testy end-to-end

```text
BuildsKr4PositionIkEquationsFromUrdf
BuildsKr640PositionIkEquationsFromUrdf
BuildsFullPoseEquationsWithTcp
ChangingTargetInvalidatesEquationSystem
ChangingTcpInvalidatesEquationSystem
ChangingChainInvalidatesTcpAndEquationSystem
ReusesFacadeForDifferentRobotAndTarget
```

### Definition of Done Fazy 2

```text
URDF + TCP + target
→ poprawny symboliczny IkEquationSystem
```

Projekt umie w tym momencie **formułować IK**, ale jeszcze nie umie automatycznie wyprowadzać `q`.

### Proponowany release

```text
v0.2-ik-equations
```

---

# 3. Faza 3 — Analytic IK Solver

## 3.1 Cel Fazy 3

Doprowadzić projekt do stanu:

```text
IkEquationSystem
→ uproszczone równania
→ rozpoznana geometria robota
→ wielogałęziowe rozwiązania q
→ zweryfikowany kod C++
```

## 3.2 Realistyczny zakres

Pierwszy solver nie powinien obiecywać:

```text
dowolny URDF → pełne analityczne IK
```

Pierwszy wspierany zakres:

```text
roboty przemysłowe 6R
wybrane struktury geometryczne
KR4 / KR640 jako roboty walidacyjne
spherical wrist lub inne jawnie rozpoznane wzorce
```

Brak rozpoznanego wzorca powinien dawać jawne:

```text
UnsupportedKinematicStructure
```

a nie próbę zgadywania rozwiązania.

## 3.3 Struktura Fazy 3

Faza 3 ma **4 główne komponenty**:

1. `EquationSimplifier`,
2. `IkPatternDetector`,
3. `EquationSolver`,
4. `CodeGenerator`.

Realizacyjnie dzielimy ją na **12 etapów**.

---

## F3.1 — semantyka rewrite engine

### Cel

Zaprojektować bezpieczny mechanizm przekształcania wyrażeń.

Najważniejsza zasada odziedziczona z Fazy 1:

```text
nie wolno upraszczać kosztem zmiany dziedziny
```

Przykład:

```text
(1/q) * 0
```

nie może zostać bezwarunkowo zamienione na `0`.

### Zakres

- wynik rewritingu;
- informacja „zmieniono / nie zmieniono”;
- warunki poprawności;
- iteracja do punktu stałego;
- limit kroków;
- unikanie pętli rewrite;
- zachowanie DAG lub świadome przebudowanie;
- testowanie równoważności numerycznej z uwzględnieniem dziedziny.

---

## F3.2 — `EquationSimplifier`: kanonizacja algebraiczna

### Pierwszy bezpieczny zakres

```text
x + 0 → x
x - 0 → x
x * 1 → x
x / 1 → x
-x → Negate(x)
-(-x) → x
constant folding
normalizacja znaków
porządkowanie składników
```

Reguły warunkowe muszą znać preconditions.

### Gate

- każda reguła ma test domeny;
- brak globalnego `x * 0 → 0`;
- wynik stabilizuje się;
- simplifier nie zwiększa drzewa bez limitu.

---

## F3.3 — `EquationSimplifier`: trygonometria i równania

### Zakres

- kanoniczne `sin(-x)`, `cos(-x)`;
- rozpoznawanie wspólnych argumentów;
- kontrolowane tożsamości:
  ```text
  sin²(x) + cos²(x) = 1
  ```
- przenoszenie stron równania;
- izolowanie bezpiecznych składników;
- eliminacja identycznych składników po obu stronach;
- przygotowanie postaci wejściowej dla pattern detectora.

Nie wprowadzać od razu pełnego CAS.

---

## F3.4 — geometryczny model osi i jointów

### Cel

Przekształcić `KinematicChain` w model użyteczny do rozpoznawania struktury IK:

- linia osi jointu w odpowiednim układzie;
- punkt na osi;
- kierunek osi;
- typ jointu;
- relacje:
  - równoległość,
  - prostopadłość,
  - przecięcie,
  - współosiowość,
  - offset.

### Wymaganie

Tolerancje geometryczne muszą być jawne i uzasadnione. Nie wolno mieszać dokładnej symboliki z przybliżonym rozpoznaniem geometrii bez oznaczenia granicy.

---

## F3.5 — `IkPatternDetector`

### Pierwsze wzorce

```text
spherical wrist
trzy przecinające się osie nadgarstka
planar 2R / 3R subproblem
shoulder offset
parallel-axis chain
decoupling position / orientation
```

### Wynik

Kandydat:

```cpp
struct DetectedIkPattern
{
    PatternKind kind;
    std::vector<JointIndex> joints;
    PatternEvidence evidence;
};
```

Detektor ma zwracać dowody/metryki, nie tylko enum.

---

## F3.6 — model rozwiązań analitycznych

### Cel

Zaprojektować IR wynikowe solvera przed pisaniem algorytmów.

Musi reprezentować:

- wiele gałęzi;
- `atan2`;
- `sqrt`;
- znak `±`;
- warunki istnienia;
- osobliwości;
- wolne parametry;
- brak rozwiązania;
- nieskończenie wiele rozwiązań;
- mapowanie rozwiązania na joint;
- klasy `Shoulder / Elbow / Wrist`;
- ograniczenia jointów.

Przykładowe pojęcia:

```text
SolutionBranch
SolutionCondition
JointExpression
SingularityCase
FreeParameter
```

---

## F3.7 — primitive solvers

Budować małe, testowalne solvery:

```text
a·x + b = 0
sin(x) = c
cos(x) = c
a·sin(x) + b·cos(x) = c
planar 2R
triangle / law-of-cosines subproblem
atan2(y, x)
```

Każdy primitive solver musi zwracać wszystkie gałęzie i warunki dziedziny.

Nie zaczynać od monolitycznego solvera 6R.

---

## F3.8 — solver pozycji dla wspieranej rodziny 6R

### Cel

Na podstawie wykrytego wzorca rozwiązać część pozycyjną:

```text
q1, q2, q3
```

Pierwszy zakres powinien być jawnie ograniczony do geometrii, którą potwierdzają KR4/KR640 lub wybrana wspólna podrodzina.

### Gate

- wszystkie gałęzie elbow/shoulder;
- detekcja braku osiągalności;
- rozwiązania podstawione do FK odtwarzają target;
- brak arbitralnego wyboru jednej gałęzi.

---

## F3.9 — solver orientacji / wrist

### Cel

Po rozwiązaniu pozycji wyprowadzić:

```text
q4, q5, q6
```

dla rozpoznanego nadgarstka.

Zakres:

- rozkład macierzy orientacji;
- wielogałęziowe `atan2`;
- wrist flip;
- osobliwość przy środkowym kącie nadgarstka;
- ciągłość i równoważne reprezentacje kąta.

---

## F3.10 — osobliwości, degeneracje i limity

### Zakres

- jawne przypadki singularne;
- `sin(q) = 0`;
- rozwiązania z wolnym parametrem;
- deduplikacja kątów modulo `2π`;
- normalizacja do limitów;
- odrzucanie rozwiązań spoza limitów;
- klasyfikacja shoulder/elbow/wrist;
- stabilne zachowanie blisko singularności.

### Zasada

Nie wolno „naprawiać” przypadku singularnego arbitralną wartością bez oznaczenia tego w wyniku.

---

## F3.11 — walidacja solvera end-to-end

### Pipeline

```text
target
→ solver analityczny
→ wszystkie q
→ FK(q)
→ porównanie z targetem
```

### Zestawy testowe

- targety generowane z losowych konfiguracji w limitach;
- wartości blisko limitów;
- wszystkie rodziny konfiguracji;
- singularności;
- targety nieosiągalne;
- targety z wieloma rozwiązaniami;
- KR4;
- KR640;
- porównanie z niezależnym solverem numerycznym jako oracle pomocniczym.

### Gate

Każde zwrócone rozwiązanie przechodzi FK round-trip. Solver nie może zwrócić rozwiązania, którego nie potwierdza własne zweryfikowane FK.

---

## F3.12 — `CodeGenerator`

### Cel

Przekształcić rozwiązania analityczne w wyspecjalizowany kod:

```text
AnalyticIkSolution
→ C++
```

### Pierwszy backend

```text
C++23
```

### Generowany kod powinien zawierać

- obliczenia wspólnych podwyrażeń;
- wszystkie gałęzie;
- warunki osiągalności;
- limity jointów;
- singularności;
- jawne wyniki błędów;
- brak zależności od ogólnego solvera symbolicznego w runtime.

### Walidacja

```text
kod wygenerowany
→ kompilacja
→ targety testowe
→ FK round-trip
→ porównanie z interpreterem rozwiązania symbolicznego
```

### Definition of Done Fazy 3

Dla jawnie wspieranej rodziny robotów:

```text
URDF + target
→ rozwiązania analityczne IK
→ wszystkie gałęzie
→ limity i singularności
→ wygenerowany i zweryfikowany kod C++
```

### Proponowany release

```text
v1.0-analytic-ik
```

---

# 4. Kolejność dokumentów

## Faza 2

```text
--- zamknięte, w proposal/done/ ---

done/F2.1-F2.2-tcp-transform/
    proposal-tcp-transform-architecture.md                  ✅ zaimplementowane
    proposal-tcp-transform-implementation.md                ✅ zaimplementowane

done/F2.3-ik-equation-model/
    proposal-ik-equation-model-architecture.md              ✅ zaimplementowane
    proposal-ik-equation-model-implementation.md            ✅ zaimplementowane

done/F2.3.R-review-remediation/                             ✅ etap zamknięty 2026-08-18
    README.md                                               indeks etapu
    proposal-warning-policy.md                              R1
    proposal-loader-topology-architecture.md                R2 — implementacja bez
                                                            osobnego proposala, na polecenie
    proposal-loader-limits-strict-parsing-architecture.md   R3
    proposal-loader-limits-strict-parsing-implementation.md R3
    proposal-legacy-robot-model-boundary.md                 R4 — jeden dokument, nie para
    proposal-expression-factory-finite-invariant-architecture.md    R5
    proposal-expression-factory-finite-invariant-implementation.md  R5

done/F2.4-constraint-builder-position/
    proposal-constraint-builder-architecture.md             ✅ zaimplementowane
    proposal-constraint-builder-implementation.md           ✅ zaimplementowane

done/F2.5-constraint-builder-fullpose/
    proposal-constraint-builder-fullpose-architecture.md    ✅ zaimplementowane
    proposal-constraint-builder-fullpose-implementation.md  ✅ zaimplementowane

--- w toku, w proposal/ ---

done/F2.5Q-quaternion-target/
    proposal-quaternion-target-architecture.md              ✅ zaimplementowane
    proposal-quaternion-target-implementation.md            ✅ zaimplementowane

--- w toku, w proposal/ ---

done/F2.6-directional-constraints/
    proposal-directional-constraints-architecture.md        ✅ zaimplementowane
    proposal-directional-constraints-a-implementation.md    ✅ zaimplementowane
    proposal-directional-constraints-b-implementation.md    ✅ zaimplementowane

--- w toku, w proposal/ ---

done/F2.7-facade-integration/
    proposal-phase2-facade-integration.md                   ✅ zaimplementowane
    proposal-phase2-facade-integration-implementation.md    ✅ zaimplementowane

--- w toku, w proposal/ ---

Faza 3 — jeszcze nic nie napisane. Pierwszy dokument: F3.1, semantyka
rewrite engine'u.
```

Nazwy plików F2.4 i F2.5 różnią się od zapowiadanych tu wcześniej
(`proposal-constraint-builder-position-*`, `-full-pose-*`). Zostają takie, jak
zostały zatwierdzone: przenumerowanie dokumentów po fakcie kosztuje więcej, niż
warta jest zgodność z zapowiedzią.

## Faza 3

Każdy etap powinien zachować proces:

```text
analiza
→ proposal architektoniczny
→ review
→ proposal implementacyjny
→ review
→ implementacja
→ clean build
→ pełne testy
→ aktualizacja STATUS
```

Najbardziej ryzykowne etapy wymagające spike'a przed pełną architekturą:

```text
F3.3 trygonometria
F3.5 pattern detector
F3.8 solver pozycji 6R
F3.9 solver wrist
F3.10 singularności
```

---

# 5. Podsumowanie liczbowe

```text
Faza 2:
    2 główne komponenty
    8 etapów realizacyjnych
      = 7 planowanych pierwotnie
      + F2.5Q (target kwaternionowy), wydzielony z F2.5 w review 2026-08-20
  + 1 etap remediacyjny (F2.3.R, 5 proposali) dodany po review 2026-08-17

    zamknięte:  wszystkie — F2.1, F2.2, F2.3, F2.3.R, F2.4, F2.5, F2.5Q,
                F2.6, F2.7
    FAZA 2 ZAMKNIĘTA 2026-08-21, release v0.2-ik-equations

Faza 3:
    4 główne komponenty
    12 etapów realizacyjnych
```

Nie oznacza to, że Faza 3 jest tylko dwa razy większa. Etapy solvera są badawczo i implementacyjnie znacznie cięższe niż etapy Fazy 2.

---

# 6. Najbliższy krok

**Stan na 2026-08-21: Faza 2 zamknięta.** Wszystkie dziewięć etapów
zaimplementowanych, **422/422** testów w obu konfiguracjach (`build/` w Release i
`build-werror/` z `-Werror` oraz żywymi asercjami). Pipeline działa end-to-end
przez jedną fasadę:

```text
URDF + wybrany łańcuch + opcjonalny TCP + cel  ->  IkEquationSystem
```

Sześć rodzajów zadań, orientacja celu jako macierz albo kwaternion, wszystko pod
`IkEquationBuilder` prywatne. Release: **`v0.2-ik-equations`**.

Zdanie, które musi paść razem z tym milestone'em: **projekt umie formułować IK,
ale nie umie go rozwiązywać.** Wyprowadzenie `q` to Faza 3 i mierzone wysiłkiem
jest większe niż wszystko dotychczas razem wzięte.

Etapem bieżącym jest **F3.1** — semantyka rewrite engine'u, na której stoi każdy
późniejszy etap upraszczania i rozwiązywania. Do napisania:

```text
proposal/F3.1-rewrite-engine/proposal-rewrite-engine-semantics-architecture.md
```

Uwaga z sekcji 4, która staje się teraz aktualna: `F3.3` (trygonometria), `F3.5`
(pattern detector), `F3.8` (solver pozycji 6R), `F3.9` (solver wrist) i `F3.10`
(singularności) są badawczo najcięższe i **wymagają spike'a przed pełną
architekturą**. F3.1 do nich nie należy — jest fundamentem, który kładzie się
normalnym procesem.

Historyczny zapis „najbliższego kroku" z 2026-08-17 — R1 i kolejność
R1 → (R2 + R4) → R3 → R5 — jest w sekcji `F2.3.R` i w
`proposal/done/F2.3.R-review-remediation/README.md`.
