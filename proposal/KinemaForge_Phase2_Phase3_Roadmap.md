# KinemaForge / IkEquationBuilder — roadmapa Fazy 2 i Fazy 3

**Data:** 2026-08-03  
**Zakres:** rozwój po zakończeniu Fazy 1 (`URDF → zweryfikowane symboliczne FK`)  
**Założenie wejściowe:** `IkEquationBuilder` jest zaimplementowany, pełny clean build przechodzi, a Faza 1 jest oznaczona jako zakończona.

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

Realizacyjnie dzielimy ją na **7 etapów**.

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

## F2.4 — `ConstraintBuilder`: `PositionOnly`

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

## F2.5 — `ConstraintBuilder`: `FullPose`

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
AcceptsValidQuaternionTarget
ProducesEquivalentTargetsForQuaternionAndMatrix
```

### Gate

```text
pełna poza → jawny IkEquationSystem
target rotation jest poprawnym obrotem
brak ukrytej konwersji zależnej od MotionBridge
```

---

## F2.6 — constrainty kierunkowe i pozycje wymuszone

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

## F2.7 — integracja z fasadą i milestone Fazy 2

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
proposal-tcp-transform-architecture.md
proposal-tcp-transform-implementation.md
implementation + tests

proposal-ik-equation-model-architecture.md
proposal-ik-equation-model-implementation.md
implementation + tests

proposal-constraint-builder-position-architecture.md
proposal-constraint-builder-position-implementation.md
implementation + tests

proposal-constraint-builder-full-pose-architecture.md
proposal-constraint-builder-full-pose-implementation.md
implementation + tests

proposal-directional-constraints-architecture.md
proposal-directional-constraints-implementation.md
implementation + tests

proposal-phase2-facade-integration.md
implementation + end-to-end validation
```

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
    7 etapów realizacyjnych

Faza 3:
    4 główne komponenty
    12 etapów realizacyjnych
```

Nie oznacza to, że Faza 3 jest tylko dwa razy większa. Etapy solvera są badawczo i implementacyjnie znacznie cięższe niż etapy Fazy 2.

---

# 6. Najbliższy krok

Po domknięciu fasady Fazy 1:

```text
proposal-tcp-transform-architecture.md
```

Pierwszy etap Fazy 2 ma odpowiedzieć wyłącznie na pytanie:

> Jak reprezentujemy, walidujemy, składamy i unieważniamy stałą transformację `tool0 → TCP`, nie wprowadzając jeszcze targetów ani równań IK?
