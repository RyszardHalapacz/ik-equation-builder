# Proposal: constrainty kierunkowe — F2.6a (rodzina kierunku), implementacja

**Etap:** F2.6a · **v5 — APPROVED, ZAIMPLEMENTOWANE 2026-08-21** · część 2 z 3

> **Zaimplementowane.** Kod wszedł w brzmieniu z tego dokumentu. Gate osiągnięty,
> ale **liczba testów wyszła o jeden większa, niż zapowiadał rachunek**:
> `396/396` zamiast `395/395`, bo plik kierunkowy dostał szesnasty test —
> `RejectsUndeclaredSymbolInDirectionEquations`, wymieniony na liście testowej
> **architektury**, którego rachunek w 7.1 pominął. Test jest potrzebny: gwarancja
> symboli musiała zostać sprawdzona także na równaniach kierunku, nie tylko
> pozycji i orientacji.
>
> Jedna rzecz w testach wyszła inaczej, niż planował ten dokument, i to jest
> odnotowane w 7.1: syntetyczna transformacja miała mieć komórki rotacji
> wypełnione **symbolami** `r00..r22`. Odpadły natychmiast — i słusznie — na
> gwarancji symboli, bo łańcuch testowy deklaruje wyłącznie `q1`. Komórki są
> wypełnione **różnymi stałymi**: dają tę samą tożsamość węzłów dla `sameNode`,
> nie pożyczając przy tym cudzej gwarancji.

> **v5, po czwartym review.** Kod produkcyjny zatwierdzony i **nietknięty**;
> wszystkie trzy poprawki są w kontrakcie testowym:
>
> 1. **Blocker: funkcja DOF nie miała testu wartości.** Sprawdzany był wyłącznie
>    podrobiony `IkTaskKind`, więc implementacja z **przestawionymi** liczbami
>    (`Position -> 5`, `Pose -> 2`, ...) przechodziłaby komplet testów — przy
>    czym „policzalna liczba stopni swobody" jest jednym z gate'ów F2.6.
>    `RejectsForgedTaskKindInDofQuery` zastąpiony przez
>    `ReportsTaskSpaceConstrainedDegreesOfFreedom`, który pinuje wszystkie pięć
>    przypadków (7.2).
> 2. `UsesToolAxisColumnCells` przerobiony na tabelę po **trzech** osiach — dotąd
>    oś Y nie była sprawdzana wcale, czyli `toolAxisColumn` miał przypięte 2/3
>    kontraktu (7.1).
> 3. Dodany `RejectsInvalidDirectionTargetBeforeCanonicalization` — granica między
>    błędem danych a złamanym inwariantem: builder ma **zwrócić** `InvalidTarget`,
>    a nie dojść do `throw` w emiterze (7.1).
>
> **Rachunek gate'u wychodzi inaczej niż w review**, więc podaję go wprost: poprawka
> 2 **usuwa** `UsesXToolAxisColumnWhenAsked` (tabela go pochłania), a poprawka 3
> dodaje jeden test — netto zero. Gate zostaje **`395/395`**, nie 396.

> **v4, po trzecim review.** Domknięcie tej samej dziury, którą v3 zamknęło tylko
> w połowie:
>
> 1. **`profileOf(taskKind)` biegło przed przebiegami.** Podrobiony
>    `static_cast<IkTaskKind>(99)` zwraca `TaskEquationMismatch` **zanim**
>    ktokolwiek spojrzy na osie, więc przy jednoczesnym podrobionym `ToolAxis`
>    kontrakt `InvalidToolAxis > MixedToolAxes > TaskEquationMismatch` znowu był
>    nieprawdziwy. Profil przeniesiony **za PASS 2** (5.3) — co jest też logicznie
>    czystsze: najpierw integralność pojedynczego źródła, potem relacje między
>    źródłami, na końcu zgodność z rodzajem zadania.
> 2. Trzeci blok w teście priorytetu zamiast nowego testu — gate zostaje
>    **`395/395`** (7.2).
> 3. `<stdexcept>` dopisany do listy include'ów (0.1).

> **v3, po drugim review.** Jeden blocker, obie poprawki w warstwie modelu:
>
> 1. **Deklarowana kolejność błędów nie była deterministyczna.** `InvalidToolAxis`,
>    `MixedToolAxes` i `TaskEquationMismatch` powstawały w **jednym** przebiegu po
>    równaniach, więc zwrócony kod zależał od tego, które wadliwe równanie stoi
>    wcześniej w wektorze — mimo że komentarz przy `create()` obiecuje stały
>    priorytet. `checkEquationContent` rozbite na **trzy przebiegi** (5.3), plus
>    test priorytetu pinujący obie krawędzie łańcucha (7.2). Gate `394` → **`395`**.
> 2. Komentarze mówiące o „trzeciej alternatywie" — która od F2.6a **istnieje** —
>    przepisane na neutralne (5.6).

> **v2, po pierwszym review.** Architektura i algorytmy produkcyjne bez zmian — review
> odtworzyło modelowe fragmenty na repo i potwierdziło czysty build pod
> `-Wall -Wextra -Wswitch -Werror` oraz poprawność skalowanej kanonizacji. Pięć
> poprawek, wszystkie w testach i diagnostyce:
>
> 1. **Fixture duplikatu był opisany błędnie.** „Dziewięć komórek + duplikat" to
>    **dziesięć** równań orientacji, więc `checkEquationContent` odrzuciłoby je na
>    liczbie, zanim kontrola duplikatu w ogóle by ruszyła. Poprawny fixture ma
>    **dokładnie dziewięć równań, z czego jedno powtarza źródło** — czyli jedna
>    komórka jest siłą rzeczy nieobecna (7.2).
> 2. Dopisany `AcceptsPoseWithCompleteOrientation` — nowy kontrakt `Pose` musi mieć
>    przypięte **obie** krawędzie: osiem odrzucone, dziewięć przyjęte (7.2).
> 3. Gate `393/393` → **`394/394`** po dodaniu tego testu.
> 4. `componentIndex` przestaje mówić „position equation" — od F2.6a używa go także
>    źródło kierunku i komunikat kłamałby o rodzaju równania (5.5).
> 5. Jawna lista nowych include'ów (0.1) i odświeżone komentarze kontraktowe (5.6).

Architektura: [`proposal-directional-constraints-architecture.md`](proposal-directional-constraints-architecture.md)
— **v4, APPROVED** w review 2026-08-21. Ten dokument nie podejmuje decyzji
projektowych; wykłada kod pierwszej z dwóch rodzin i uzasadnia wyłącznie te
wybory, które w architekturze były opisane słowem, a tutaj muszą stać się
instrukcją.

## Prompt

> Napisać proposal implementacyjny F2.6a — rodzina kierunku — po zatwierdzeniu
> architektury. Po jego review dopiero F2.6b z `DirectionAngle`.

## 0. Zakres: dokładnie ta połowa, która zostawia model spójny

```text
W F2.6a                                    Zostaje na F2.6b
--------------------------------------     ----------------------------------
ToolAxis + toolAxisColumn                  DirectionAngleTarget
ToolDirectionTarget                        PositionAndDirectionAngleTarget
PositionAndToolDirectionTarget             DirectionAngleEquationSource
DirectionEquationSource                    walidacja kąta (7.2 architektury)
IkTaskKind: 2 -> 4                         IkTaskKind: 4 -> 6
przebudowa IkEquationSystem (8)            pierwsze liczone lhs
Pose domknięte do dziewięciu (6.2)         dwa przeciążenia buildera
taskSpaceConstrainedDegreesOfFreedom
dwa przeciążenia buildera + kanonizacja
```

Po tym etapie **każdy istniejący `IkTaskKind` ma komplet**: źródło równania,
walidację celu, przeciążenie buildera i testy. To jest warunek, dla którego
podział przebiega po rodzinach, a nie po warstwach.

Bilans plików:

```text
NOWE        model/ToolAxis.hpp                  ~40 linii, enum + mapowanie
            model/IkTaskDegreesOfFreedom.hpp    ~45 linii, jedna funkcja
            tests/test_directional_constraints.cpp   16 testów

ZMIENIONE   model/Equation.hpp                  +IkTaskKind x2, +źródło, +EquationKind
            model/IkTarget.hpp                  +2 alternatywy
            model/TargetValidation.{hpp,cpp}    +3 kody, +2 przeciążenia
            model/IkEquationSystem.{hpp,cpp}    +2 kody, przebudowa dwóch kontroli
            builders/ConstraintBuilder.{hpp,cpp} +2 przeciążenia, +emiter, +kanonizacja
            tests/test_ik_equation_model.cpp    +11 testów, 2 fixture'y rosną
            tests/test_target_validation.cpp    +4 testy
            tests/CMakeLists.txt                +1 wiersz

BEZ ZMIAN   CMakeLists.txt -- oba nowe pliki produkcyjne są header-only, a
            biblioteka wymienia w źródłach wyłącznie .cpp

NIETKNIĘTE  warstwa symboliczna, loader, budowniczowie transformacji, fasada,
            tests/support, Quaternion*, cały F2.5Q
```

### 0.1 Nowe include'y

Wypisane wprost, bo część z nich przyjechałaby tranzytywnie — a kod produkcyjny nie
ma prawa się na tym opierać:

```text
model/Equation.hpp            "ik_equations/model/ToolAxis.hpp"
model/IkTarget.hpp            "ik_equations/model/ToolAxis.hpp"
model/IkTaskDegreesOfFreedom.hpp  "ik_equations/model/Equation.hpp", <cstddef>, <optional>
model/ToolAxis.hpp            <cstddef>, <optional>
model/TargetValidation.cpp    <algorithm>   // std::max(initializer_list)
model/IkEquationSystem.cpp    <compare>     // operator<=> w SourceOrderKey
                              <optional>    // std::optional<SourceOrderKey>, axis
builders/ConstraintBuilder.cpp  <algorithm> // std::max(initializer_list)
                              <cmath>       // std::abs, std::hypot
                              <stdexcept>   // std::logic_error w emiterze
tests/test_directional_constraints.cpp  <cfloat> albo <limits> // DBL_MAX, denorm_min
```

`TargetValidation.cpp` ma już `<cmath>` i `<string>`; `ConstraintBuilder.cpp` ma
już `<algorithm>` i `<stdexcept>` — zostają w tabeli, żeby lista była kompletna
jako **kontrakt**, a nie jako diff. Rzecz w tym, żeby po tym etapie każdy z tych
plików deklarował wszystko, czego używa, niezależnie od tego, co przyjechało
tranzytywnie wcześniej.

---

## 1. `src/ik_equations/model/ToolAxis.hpp` — nowy

```cpp
#pragma once

#include <cstddef>
#include <optional>

namespace kinemaforge::ik {

// Which axis of the tool the task talks about.
//
// WHOSE axis: the final frame of the SymbolicTransform handed to
// ConstraintBuilder. If that transform came from tcpForwardKinematics(), this
// is an axis of the TCP; if it ended at the chain tip, it is an axis of the last
// link. The builder cannot tell them apart and does not try -- same class of
// gap as "a target does not know its frame".
//
// Only the principal axes. A tool tilted off its frame is described by the TCP
// transform (F2.1/F2.2), not by a free axis here: two places encoding tool
// geometry would eventually disagree.
enum class ToolAxis
{
    X,
    Y,
    Z
};

// The one and only conversion from an axis to a column of the rotation block.
//
// Returns nullopt for a value forged with static_cast<ToolAxis>(99): such an
// object is a valid ToolAxis with no enumerator behind it, and indexing with it
// would read out of bounds. Same defence the model already has for
// CartesianComponent.
//
// std::optional rather than std::expected on purpose: this header must not know
// any error vocabulary. The axis is validated in two layers with two different
// error types -- TargetValidationErrorCode::InvalidToolAxis for a target,
// IkEquationSystemErrorCode::InvalidToolAxis for an equation source -- and an
// expected would force this file to pick one of them.
[[nodiscard]] constexpr std::optional<std::size_t> toolAxisColumn(ToolAxis axis) noexcept
{
    switch (axis)
    {
    case ToolAxis::X: return 0;
    case ToolAxis::Y: return 1;
    case ToolAxis::Z: return 2;
    }
    return std::nullopt;
}

} // namespace kinemaforge::ik
```

Nagłówek bez `.cpp`, tak jak `Vector3.hpp` i `Quaternion.hpp` — to jeden
`switch`, a `constexpr` pozwala użyć go w kontekście stałym, gdyby kiedyś było
potrzebne. Odstępstwo od planu z architektury (`.hpp` + `.cpp`) jest świadome:
plik `.cpp` z jedną funkcją nie miałby czego ukrywać.

---

## 2. `src/ik_equations/model/Equation.hpp` — trzy dopiski

### 2.1 `IkTaskKind`

```cpp
enum class IkTaskKind
{
    Position,
    Pose,
    ToolDirection,
    PositionAndToolDirection
};
```

Dwie wartości rodziny II dojdą w F2.6b. Każda z tych czterech ma w tym etapie
komplet: profil zawartości (5.2), walidację celu (4), przeciążenie buildera (6) i
DOF (3).

### 2.2 `DirectionEquationSource`

```cpp
// One component of the tool axis direction, expressed in the base frame.
//
// NOT an OrientationEquationSource, even though for ToolAxis::Z these are
// literally cells (0,2), (1,2), (2,2). A source says WHAT is constrained, not
// which cell happened to be read: three orientation cells inside a Pose task are
// a fragment of a nine-cell constraint, while the same three cells in a
// ToolDirection task are the WHOLE constraint, with a different degree-of-freedom
// count and a different way of being solved. A solver that cannot tell them
// apart counts the degrees of freedom wrong.
struct DirectionEquationSource
{
    CartesianComponent component{};
    ToolAxis axis{ToolAxis::Z};

    friend bool operator==(const DirectionEquationSource&,
                           const DirectionEquationSource&) noexcept = default;
};

using EquationSource = std::variant<PositionEquationSource,
                                    OrientationEquationSource,
                                    DirectionEquationSource>;
```

### 2.3 `EquationKind` i wizytor

```cpp
enum class EquationKind
{
    Position,
    Orientation,
    Direction
};
```

```cpp
struct EquationKindVisitor
{
    EquationKind operator()(const PositionEquationSource&) const noexcept { ... }
    EquationKind operator()(const OrientationEquationSource&) const noexcept { ... }

    EquationKind operator()(const DirectionEquationSource&) const noexcept
    {
        return EquationKind::Direction;
    }
};
```

Ten wizytor jest powodem, dla którego F2.3 wybrało wariant zamiast pary
`{kind, optional<cell>}`: bez tego przeciążenia **kod się nie kompiluje**. Dodanie
alternatywy do `EquationSource` bez decyzji o jej rodzaju jest niemożliwe, i tak
ma zostać.

---

## 3. `src/ik_equations/model/IkTaskDegreesOfFreedom.hpp` — nowy

```cpp
#pragma once

#include "ik_equations/model/Equation.hpp"

#include <cstddef>
#include <optional>

namespace kinemaforge::ik {

// How many degrees of freedom of the TASK SPACE a kind of task removes.
//
// The long name is deliberate: this is not the rank of the robot's Jacobian and
// not the rank of the equation system. It is a property of the geometry of the
// constraint, independent of the robot and of the configuration -- which is
// exactly what makes it honest to state at all.
//
//   Position                  3   position fixed, orientation free
//   Pose                      6   nothing free
//   ToolDirection             2   a unit direction lives on a sphere: TWO
//                                 degrees of freedom, not three. The third
//                                 equation is dependent on the solution set,
//                                 because |R(q) e_t| = 1 holds identically for a
//                                 rigid transform and |d| = 1 is enforced by the
//                                 builder's canonicalisation.
//   PositionAndToolDirection  5   spin about the tool axis stays free
//
// THE EQUATION COUNT IS NOT THIS NUMBER. ToolDirection carries three equations
// and removes two; Pose carries twelve and removes six.
//
// nullopt for a value forged with static_cast<IkTaskKind>(99). Returning 0 would
// be worse than useless: it reads as a legitimate "constrains nothing".
[[nodiscard]] constexpr std::optional<std::size_t>
taskSpaceConstrainedDegreesOfFreedom(IkTaskKind kind) noexcept
{
    switch (kind)
    {
    case IkTaskKind::Position:                 return 3;
    case IkTaskKind::Pose:                     return 6;
    case IkTaskKind::ToolDirection:            return 2;
    case IkTaskKind::PositionAndToolDirection: return 5;
    }
    return std::nullopt;
}

} // namespace kinemaforge::ik
```

`Pose -> 6` jest prawdziwe **dopiero** po zaostrzeniu z 5.2: dopóki model
dopuszczał `Pose` z jednym równaniem orientacji, ta funkcja kłamałaby o systemie,
który sama biblioteka potrafi zbudować.

---

## 4. `IkTarget.hpp` i walidacja

### 4.1 Dwa nowe cele

```cpp
// A direction the tool axis must point along, in the base frame.
//
// direction is finite and NON-ZERO. It does NOT have to be a unit vector and
// this type does not promise that it is: a direction is a ray, its length
// carries no semantics, and a normal produced by a cross product has whatever
// length it has. Canonicalisation belongs to ConstraintBuilder, which is the
// only thing that reads it -- validation checks, it never rewrites.
//
// "Alignment" here means an ORIENTED direction: R(q) e_t = d points the tool
// along d. The opposite setting is a different target, -d, not the same one.
struct ToolDirectionTarget
{
    Vector3 direction;
    ToolAxis axis{ToolAxis::Z};
};

struct PositionAndToolDirectionTarget
{
    Vector3 position;    // metres
    Vector3 direction;   // finite, non-zero; length carries no semantics
    ToolAxis axis{ToolAxis::Z};
};

using IkTarget = std::variant<PositionTarget,
                              PoseTarget,
                              ToolDirectionTarget,
                              PositionAndToolDirectionTarget>;
```

### 4.2 `TargetValidation` — trzy kody i dwa przeciążenia

```cpp
enum class TargetValidationErrorCode
{
    NonFinitePosition,
    NonFiniteOrientation,
    NonOrthogonalOrientation,
    InvalidOrientationDeterminant,
    NonFiniteQuaternion,
    NonNormalizedQuaternion,
    NonFiniteDirection,
    ZeroDirection,
    InvalidToolAxis
};
```

```cpp
// Checked in this order, and the order is observable:
//   1. all three components finite  -> NonFiniteDirection
//   2. the vector is not zero       -> ZeroDirection
//   3. the axis is a real enumerator -> InvalidToolAxis
//
// NO norm tolerance. A direction is a ray: (0,0,1), (0,0,5) and (0,0,0.2) are
// the same direction, and rejecting the last two for not being unit length would
// invent a contract the caller never agreed to. Contrast with a quaternion,
// where unit length IS the caller's declaration and a norm of 1.7 proves a bug.
//
// Step 1 comes first because everything after it assumes numbers: the zero test
// takes max(|x|,|y|,|z|), and with NaN in a component every comparison inside
// max() is false, so its result depends on argument order -- and that same value
// is later used as the scale for canonicalisation.
[[nodiscard]] std::expected<void, TargetValidationError>
validate(const ToolDirectionTarget& target);

[[nodiscard]] std::expected<void, TargetValidationError>
validate(const PositionAndToolDirectionTarget& target);
```

Implementacja, w anonimowej przestrzeni nazw obok istniejących helperów:

```cpp
const char* describeVectorComponent(std::size_t index) noexcept
{
    switch (index)
    {
    case 0: return "x";
    case 1: return "y";
    default: return "z";
    }
}

// The zero test runs on the largest component in magnitude, not on a norm: a
// vector is zero exactly when max(|x|,|y|,|z|) is zero. No intermediate
// arithmetic, so neither overflow nor underflow can happen here -- and the same
// value is what ConstraintBuilder uses as its scale.
std::expected<void, TargetValidationError> checkDirection(const Vector3& direction)
{
    const double components[3] = {direction.x, direction.y, direction.z};
    for (std::size_t index = 0; index < 3; ++index)
        if (!std::isfinite(components[index]))
            return makeError(TargetValidationErrorCode::NonFiniteDirection,
                             std::string("direction ") + describeVectorComponent(index)
                                 + " is not finite");

    const double scale = std::max({std::abs(direction.x), std::abs(direction.y),
                                   std::abs(direction.z)});
    if (scale == 0.0)
        return makeError(TargetValidationErrorCode::ZeroDirection,
                         "direction is the zero vector and names no direction");

    return {};
}

std::expected<void, TargetValidationError> checkToolAxis(ToolAxis axis)
{
    if (!toolAxisColumn(axis))
        return makeError(TargetValidationErrorCode::InvalidToolAxis,
                         "tool axis is not one of X, Y, Z");
    return {};
}
```

```cpp
std::expected<void, TargetValidationError> validate(const ToolDirectionTarget& target)
{
    if (auto checked = checkDirection(target.direction); !checked)
        return checked;

    return checkToolAxis(target.axis);
}

std::expected<void, TargetValidationError>
validate(const PositionAndToolDirectionTarget& target)
{
    if (auto checked = validate(PositionTarget{target.position}); !checked)
        return checked;

    return validate(ToolDirectionTarget{target.direction, target.axis});
}
```

Pozycja jest walidowana pierwsza, tak samo jak w `validate(PoseTarget)` i w
`makePoseTargetFromQuaternion` — jedna kolejność w całym modelu, obserwowalna
testem.

`TargetVisitor` dostaje dwa przeciążenia. Jak przy `EquationKindVisitor`: bez
nich `validate(const IkTarget&)` **nie kompiluje się**, i to jest cała ochrona,
dla której ten wizytor jest jawnym zestawem przeciążeń.

---

## 5. `IkEquationSystem` — przebudowa dwóch kontroli

### 5.1 Dwa nowe kody

```cpp
enum class IkEquationSystemErrorCode
{
    NoEquations, NoUnknowns, EmptyUnknownName, DuplicateUnknownName,
    DuplicateUnknownIndex, UnorderedUnknowns,
    InvalidToolAxis,          // nowe
    MixedToolAxes,            // nowe
    TaskEquationMismatch, DuplicateEquationSource, UnorderedEquations
};
```

Kolejność sprawdzania w `create()` nie zmienia się: `checkUnknowns`,
`checkEquationContent`, `checkDuplicateSources`, `checkEquationOrder`. Nowe kody
wychodzą z `checkEquationContent` i z `sourceOrderKey`.

### 5.2 Profil zawartości zamiast dwóch reguł ad hoc

```cpp
// What a task kind is allowed to contain. Zeros matter as much as the completes:
// without them a Position task could carry a direction equation.
struct EquationProfile
{
    bool position{};              // exactly X, Y, Z
    std::size_t orientation{};    // exact count: 0 or 9
    bool direction{};             // exactly X, Y, Z of ONE tool axis
};

std::expected<EquationProfile, IkEquationSystemError> profileOf(IkTaskKind taskKind)
{
    // No default: a new IkTaskKind trips -Wswitch, which -Werror turns into a
    // build failure. Each valid case returns, so a forged value falls through to
    // the rejection below instead of quietly succeeding.
    switch (taskKind)
    {
    case IkTaskKind::Position:                 return EquationProfile{true, 0, false};
    case IkTaskKind::Pose:                     return EquationProfile{true, 9, false};
    case IkTaskKind::ToolDirection:            return EquationProfile{false, 0, true};
    case IkTaskKind::PositionAndToolDirection: return EquationProfile{true, 0, true};
    }

    return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                     "unsupported IK task kind");
}
```

`Pose -> 9` jest zaostrzeniem kontraktu F2.3, uzasadnionym w 6.2 architektury:
skoro częściowa orientacja ma od teraz własne rodzaje zadań, `Pose` znaczy pełną
pozę, a `taskSpaceConstrainedDegreesOfFreedom(Pose) == 6` przestaje być
obietnicą, którą model potrafi złamać.

**Kompletność dziewięciu komórek nie jest sprawdzana wprost** i to jest
świadome — nota 13c architektury:

```text
dziewięć równań orientacji            (liczba, tutaj)
+ poprawne OrientationEquationSource  (0..2 x 0..2, wymuszone przez create())
+ brak duplikatu źródła               (DuplicateEquationSource, bez zmian)
= komplet dziewięciu komórek
```

Gdyby liczyć „brakujące komórki" tutaj, test z duplikatem przestałby docierać do
`DuplicateEquationSource`, który jest po to, żeby diagnozować dwa równania na tę
samą komórkę.

### 5.3 Zawartość w trzech przebiegach, bo kolejność kodów jest kontraktem

v2 liczyła wszystko w jednej pętli i to jest **błąd**, wychwycony w review. Trzy
kody — `InvalidToolAxis`, `MixedToolAxes`, `TaskEquationMismatch` — powstawały w
tym samym przebiegu, więc zwyciężał ten, którego wadliwe równanie stało w wektorze
wcześniej:

```text
[0] DirectionEquationSource{ static_cast<CartesianComponent>(99), ToolAxis::Z }
[1] DirectionEquationSource{ CartesianComponent::Y, static_cast<ToolAxis>(99) }

jeden przebieg -> TaskEquationMismatch   (z componentIndex na [0])
po zamianie    -> InvalidToolAxis        (z [1], które teraz jest pierwsze)
```

Ten sam zbięr błędów, dwa różne kody, różnica wyłącznie w kolejności wejścia —
podczas gdy komentarz przy `create()` obiecuje stały priorytet diagnoz. Model już
dziś rozwiązuje to w jedyny sposób, który naprawdę działa, i mówi o tym wprost przy
`checkUnknowns`:

> *Each rule gets its own pass over the whole vector. That is deliberate: the
> priority in create() is per rule, not per element.*

`checkEquationContent` dostaje więc tę samą strukturę — jedna funkcja, **trzy
przebiegi**:

```cpp
std::expected<void, IkEquationSystemError>
checkEquationContent(IkTaskKind taskKind, const std::vector<Equation>& equations)
{
    // PASS 1 -- every tool axis is a real enumerator. First, because a forged
    // axis makes every later statement about "the" axis meaningless.
    for (const auto& equation : equations)
        if (const auto* direction = std::get_if<DirectionEquationSource>(&equation.source))
            if (!toolAxisColumn(direction->axis))
                return makeError(IkEquationSystemErrorCode::InvalidToolAxis,
                                 "a direction equation carries an invalid tool axis");

    // PASS 2 -- all direction equations agree about which axis. Completeness
    // alone would accept three equations about three DIFFERENT tool axes: X, Y
    // and Z all present, and the system internally contradictory.
    std::optional<ToolAxis> axis;
    for (const auto& equation : equations)
        if (const auto* direction = std::get_if<DirectionEquationSource>(&equation.source))
        {
            if (axis && *axis != direction->axis)
                return makeError(IkEquationSystemErrorCode::MixedToolAxes,
                                 "direction equations disagree about the tool axis");
            axis = direction->axis;
        }

    // Only now. profileOf() reports TaskEquationMismatch for a forged
    // IkTaskKind, so running it earlier would let a broken task kind outrank a
    // broken tool axis -- and the promised order would depend on which of the
    // two the caller got wrong. The sequence is also the honest one:
    // integrity of a single source, then relations between sources, then
    // agreement with the task kind.
    const auto profile = profileOf(taskKind);
    if (!profile)
        return std::unexpected(profile.error());

    // PASS 3 -- components and the task profile. Everything here reports
    // TaskEquationMismatch, so the order WITHIN this pass is not observable.
    bool position[3] = {false, false, false};
    bool direction[3] = {false, false, false};
    std::size_t orientationCount = 0;

    for (const auto& equation : equations)
    {
        const auto checked = std::visit(
            Overloaded{
                [&position](const PositionEquationSource& source)
                    -> std::expected<void, IkEquationSystemError> {
                    const auto slot = componentIndex(source.component);
                    if (!slot)
                        return std::unexpected(slot.error());
                    position[*slot] = true;
                    return {};
                },
                [&orientationCount](const OrientationEquationSource&)
                    -> std::expected<void, IkEquationSystemError> {
                    ++orientationCount;
                    return {};
                },
                [&direction](const DirectionEquationSource& source)
                    -> std::expected<void, IkEquationSystemError> {
                    // The axis is already known good: pass 1.
                    const auto slot = componentIndex(source.component);
                    if (!slot)
                        return std::unexpected(slot.error());
                    direction[*slot] = true;
                    return {};
                }},
            equation.source);

        if (!checked)
            return std::unexpected(checked.error());
    }

    // Position: complete or absent, per the profile. A task constraining two of
    // three coordinates would be a different IkTaskKind, not a truncated one.
    for (std::size_t slot = 0; slot < 3; ++slot)
        if (position[slot] != profile->position)
            return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                             profile->position
                                 ? std::string("missing position equation for ")
                                       + describe(static_cast<CartesianComponent>(slot))
                                 : std::string("this task cannot carry position equations"));

    if (orientationCount != profile->orientation)
        return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                         "this task needs exactly " + std::to_string(profile->orientation)
                             + " orientation equations, got " + std::to_string(orientationCount));

    for (std::size_t slot = 0; slot < 3; ++slot)
        if (direction[slot] != profile->direction)
            return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                             profile->direction
                                 ? std::string("missing direction equation for ")
                                       + describe(static_cast<CartesianComponent>(slot))
                                 : std::string("this task cannot carry direction equations"));

    return {};
}
```

Trzy przebiegi zamiast jednego kosztują dwa dodatkowe przejścia po wektorze,
który ma najwyżej dwanaście elementów. Kupują za to zdanie, które **jest
prawdziwe**: kolejność diagnoz nie zależy od kolejności równań.

**Położenie `profileOf` jest częścią tej samej poprawki**, i v3 miała je złe.
Profil raportuje `TaskEquationMismatch` dla podrobionego `IkTaskKind`, więc
uruchomiony na wejściu funkcji **wyprzedzałby** obie kontrole osi:

```text
IkTaskKind    static_cast<IkTaskKind>(99)
equation[0]   DirectionEquationSource{ CartesianComponent::X, static_cast<ToolAxis>(99) }

profil na wejściu -> TaskEquationMismatch   (oś nigdy nie sprawdzona)
profil po PASS 2  -> InvalidToolAxis         (zgodnie z kontraktem)
```

Kolejność „integralność źródła → relacje między źródłami → zgodność z zadaniem"
jest przy tym niezależnie sensowna: nie ma sensu pytać, czy zbiór równań pasuje do
rodzaju zadania, dopóki pojedyncze równania nie są jeszcze samo w sobie spójne.

`std::get_if` zamiast `std::visit` w przebiegach 1 i 2 jest świadome: te pętle
pytają o **jedną** alternatywę, a nie klasyfikują wszystkich, więc jawny zestaw
przeciążeń nie miałby tu czego chronić. Klasyfikacja — czyli miejsce, w którym
nowa alternatywa **musi** zatrzymać kompilację — zostaje w przebiegu 3.

Pętla `position[slot] != profile->position` załatwia obie strony tabeli naraz:
komplet tam, gdzie profil go żąda, i **zero** tam, gdzie nie.
`describe(static_cast<...>(slot))` jest bezpieczne, bo `slot` pochodzi z
literalnej pętli `0..2` — to ten sam wzorzec, którego używa dzisiejszy kod.

### 5.4 Porządek: jeden klucz

```cpp
// (rank of the source kind, index within the kind). Strictly increasing over the
// whole vector.
struct SourceOrderKey
{
    std::size_t rank{};
    std::size_t index{};

    friend auto operator<=>(const SourceOrderKey&, const SourceOrderKey&) noexcept = default;
};

// Defensive on purpose: this must NOT assume checkEquationContent already ran.
// Today create() calls them in the right order, but swapping two lines must not
// turn a validation error into an out-of-range read -- the same reason
// componentIndex exists.
std::expected<SourceOrderKey, IkEquationSystemError>
sourceOrderKey(const EquationSource& source)
{
    return std::visit(
        Overloaded{
            [](const PositionEquationSource& position)
                -> std::expected<SourceOrderKey, IkEquationSystemError> {
                const auto index = componentIndex(position.component);
                if (!index)
                    return std::unexpected(index.error());
                return SourceOrderKey{0, *index};
            },
            [](const OrientationEquationSource& orientation)
                -> std::expected<SourceOrderKey, IkEquationSystemError> {
                // row() and column() are 0..2 by construction: the type has no
                // public constructor and create() rejects anything else.
                return SourceOrderKey{1, orientation.row() * 3 + orientation.column()};
            },
            [](const DirectionEquationSource& direction)
                -> std::expected<SourceOrderKey, IkEquationSystemError> {
                const auto index = componentIndex(direction.component);
                if (!index)
                    return std::unexpected(index.error());

                if (!toolAxisColumn(direction.axis))
                    return makeError(IkEquationSystemErrorCode::InvalidToolAxis,
                                     "a direction equation carries an invalid tool axis");

                // The axis is not part of the key: MixedToolAxes already
                // guarantees one axis per system.
                return SourceOrderKey{2, *index};
            }},
        source);
}

std::expected<void, IkEquationSystemError>
checkEquationOrder(const std::vector<Equation>& equations)
{
    std::optional<SourceOrderKey> previous;

    for (std::size_t position = 0; position < equations.size(); ++position)
    {
        const auto key = sourceOrderKey(equations[position].source);
        if (!key)
            return std::unexpected(key.error());

        if (previous && !(*previous < *key))
            return makeError(IkEquationSystemErrorCode::UnorderedEquations,
                             "equation " + std::to_string(position) +
                                 " does not follow the canonical order: position (X, Y, Z), "
                                 "then orientation row-major, then direction (X, Y, Z)");

        previous = *key;
    }

    return {};
}
```

Trzy pętle i warunek `equations.size() < 3` znikają. Ten ostatni istniał wyłącznie
przez indeksowanie slotów — teraz nie ma czego indeksować, więc system trzech
równań kierunku bez pozycji przechodzi, a system z pozycją po kierunku odpada.
Dotychczasowe zachowanie jest zachowane co do joty: pozycje najpierw w X, Y, Z,
potem orientacja row-major.

### 5.5 `componentIndex` przestaje mówić o pozycji

Dzisiejszy komunikat:

```cpp
return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                 "position equation carries an invalid Cartesian component");
```

Od F2.6a ten sam helper obsługuje **też** źródło kierunku (5.3, 5.4), więc
podrobiony `CartesianComponent` w `DirectionEquationSource` zostanie poprawnie
odrzucony — ale z komunikatem mówiącym o równaniu pozycji. Kod błędu zostaje
(`TaskEquationMismatch`), zmienia się wyłącznie diagnostyka:

```cpp
return makeError(IkEquationSystemErrorCode::TaskEquationMismatch,
                 "equation source carries an invalid Cartesian component");
```

Żadne asercje tego nie dotykają — istniejące testy odwołują się do tego przypadku
przez `SCOPED_TRACE`, nie przez treść komunikatu. Moment na tę zmianę jest teraz,
bo dopiero `RejectsForgedCartesianComponentInDirectionSource` (7.2) sprawia, że
komunikat miałby okazję skłamać.

### 5.6 Komentarze kontraktowe do odświeżenia

Zmienia się **publiczny** kontrakt, więc komentarze muszą iść razem z kodem —
inaczej następny czytelnik dostanie sprzeczne źródła prawdy.

**`IkEquationSystem.hpp`, przy `equations()`** — dziś:

```cpp
// Position X, Y, Z first; then orientation cells in row-major order.
```

po F2.6a:

```cpp
// Position X, Y, Z first; then orientation cells in row-major order; then
// direction components X, Y, Z. Only the source kinds legal for this task kind
// are present, and a direction system carries exactly one tool axis.
```

**`IkEquationSystem::create()`** — lista kodów w komentarzu rośnie o dwa, w
miejscu odpowiadającym kolejności sprawdzania:

```text
NoEquations, NoUnknowns, EmptyUnknownName, DuplicateUnknownName,
DuplicateUnknownIndex, UnorderedUnknowns, InvalidToolAxis, MixedToolAxes,
TaskEquationMismatch, DuplicateEquationSource, UnorderedEquations
```

**Trzy komentarze o „trzeciej alternatywie"** — `Equation.hpp:82` i `:111` oraz
`TargetValidation.cpp:94` — mówią o czymś, co od F2.6a **istnieje**. Wszędzie
wystarczy „a new alternative" zamiast „a third alternative"; sens komentarzy się
nie zmienia, przestają tylko liczyć do trzech.

**`Equation.hpp`, przy `EquationSource`** — dziś komentarz obiecuje przyszłość,
która właśnie nadeszła:

```cpp
// ... Adding a directional constraint later forces every consumer using an
// explicit overload set to handle it, at compile time.
```

po F2.6a ma opisywać stan faktyczny — że tak się stało i że mechanizm zadziałał:

```cpp
// ... That is not hypothetical any more: F2.6a added DirectionEquationSource,
// and every explicit overload set in the model had to grow before the build
// would pass again.
```

---

## 6. `ConstraintBuilder` — dwa przeciążenia, emiter i kanonizacja

```cpp
[[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
build(const SymbolicTransform& transform, const KinematicChain& chain,
      const ToolDirectionTarget& target) const;

[[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
build(const SymbolicTransform& transform, const KinematicChain& chain,
      const PositionAndToolDirectionTarget& target) const;

private:
    // Precondition: the direction passed validation, so it is finite and
    // non-zero. Scaled on purpose -- see the comment in the .cpp.
    [[nodiscard]] static Vector3 normalizedDirection(const Vector3& direction) noexcept;

    void appendDirectionEquations(std::vector<Equation>& equations,
                                  const SymbolicTransform& transform,
                                  ToolAxis axis,
                                  const Vector3& unitDirection) const;
```

```cpp
// Scale first, then hypot, then divide. The obvious version -- hypot on the raw
// components -- does not cover the contract this class advertises:
//
//   {DBL_MAX, DBL_MAX, 0}       hypot = inf         raw x/norm -> (0, 0, 0)
//   {denorm_min, denorm_min, 0} hypot = 4.94e-324   raw x/norm -> (1, 1, 0)
//
// The first silently destroys a direction that is exact to the last bit. The
// second is worse: it returns a vector of norm sqrt(2) that PRETENDS to be a
// unit direction and would go on to poison the equations.
//
// After scaling by the largest component, s lies in [-1, 1]^3 with max |s| = 1,
// so hypot(s) lies in [1, sqrt(3)] -- measured -- and neither overflow nor
// underflow has anywhere to happen. The scale is the same quantity validation
// compared against zero, so this division is safe by construction.
//
// One limit, stated plainly: for a subnormal input the ratio v/scale loses the
// significant digits the subnormal never had. The direction is recovered to the
// precision the input actually carried, and no method gives more.
Vector3 ConstraintBuilder::normalizedDirection(const Vector3& direction) noexcept
{
    const double scale = std::max({std::abs(direction.x), std::abs(direction.y),
                                   std::abs(direction.z)});

    const double x = direction.x / scale;
    const double y = direction.y / scale;
    const double z = direction.z / scale;

    const double norm = std::hypot(x, y, z);
    return Vector3{x / norm, y / norm, z / norm};
}
```

```cpp
// The tool axis column of the rotation block IS the direction of that axis in
// the base frame: R(q) e_t is column t. So the left-hand sides are raw FK cells,
// exactly like the position ones -- no expression is built here. That changes in
// F2.6b, where the angle equation needs a dot product.
void ConstraintBuilder::appendDirectionEquations(std::vector<Equation>& equations,
                                                 const SymbolicTransform& transform,
                                                 ToolAxis axis,
                                                 const Vector3& unitDirection) const
{
    const auto column = toolAxisColumn(axis);
    if (!column)
        throw std::logic_error("tool axis must be X, Y or Z");

    const double components[3] = {unitDirection.x, unitDirection.y, unitDirection.z};

    for (std::size_t row = 0; row < 3; ++row)
        equations.push_back(Equation{transform(row, *column),
                                     factory_.constant(components[row]),
                                     DirectionEquationSource{
                                         static_cast<CartesianComponent>(row), axis}});
}
```

Dwie rzeczy warte komentarza.

**`throw` zamiast `expected`, i dlaczego to nie jest niekonsekwencja.** Oś jest
już sprawdzona przez `validate(target)` w `build()`, więc `nullopt` tutaj oznacza
błąd programisty, nie błąd danych — dokładnie ten sam układ, co
`orientationSource(row, column)` w F2.5 i `mapJointType` w loaderze.
`std::logic_error` nie dziedziczy po `runtime_error`, więc nie zostanie złapany
przez fasadę jako „zły URDF".

**`static_cast<CartesianComponent>(row)` jest tu bezpieczne**, bo `row` biegnie po
literalnej pętli `0..2` — ten sam wzorzec, który stosuje F2.5 przy komórkach
orientacji.

`build()` składa się wtedy z czterech kroków, w stałej kolejności:

```cpp
std::expected<IkEquationSystem, ConstraintBuilderError>
ConstraintBuilder::build(const SymbolicTransform& transform, const KinematicChain& chain,
                         const PositionAndToolDirectionTarget& target) const
{
    if (auto checked = validate(target); !checked)
        return std::unexpected(invalidTarget(std::move(checked.error())));

    const Vector3 unitDirection = normalizedDirection(target.direction);

    auto equations = positionEquations(transform, target.position);
    appendDirectionEquations(equations, transform, target.axis, unitDirection);

    return assemble(chain, IkTaskKind::PositionAndToolDirection, std::move(equations));
}
```

Walidacja orzeka, builder kanonizuje, emiter dostaje wektor jednostkowy —
kontrakt z 7.5 architektury, zapisany jako kolejność trzech linii.
`ToolDirectionTarget` różni się tylko brakiem `positionEquations` i rodzajem
zadania.

---

## 7. Testy

### 7.1 `tests/test_directional_constraints.cpp` (nowy) — 16

```text
BuildsThreeDirectionEquations              ToolDirection: 3 równania, kind zgodny
UsesToolAxisColumnCells                    tabela po trzech osiach: X -> kolumna 0,
                                           Y -> 1, Z -> 2; lhs przez sameNode
DirectionEquationsAreOrderedXYZ            source po kolei X, Y, Z
UsesNormalizedDirectionOnRightHandSide     (0,0,5) daje te same rhs co (0,0,1)
NormalizesMaxFiniteDirectionWithoutOverflow  {DBL_MAX, DBL_MAX, 0} -> (1/√2, 1/√2, 0)
NormalizesSubnormalDirection               {denorm_min, denorm_min, 0} -> to samo
ToolDirectionSystemHasNoPositionEquations  zero równań pozycji w systemie
BuildsSixEquationsForPositionAndDirection  3 + 3, w tej kolejności
PositionEquationsMatchPositionOnlyBuild    lhs sameNode, source ==, rhs po wartości
AntiParallelDirectionIsADifferentTarget    d i -d dają różne rhs
DirectionTargetDoesNotConstrainFullOrientation   zero równań orientacji
DirectionTargetLeavesSpinFree              dwa roboty, zmierzone osie -- niżej
SpinFreedomTestIsNotVacuous                kontrola negatywna -- niżej
RejectsInvalidDirectionTargetBeforeCanonicalization  granica błędu -- niżej
RejectsUndeclaredSymbolInDirectionEquations  gwarancja symboli także dla kierunku
                                           (z listy architektury; rachunek v5 go
                                           pominął, stąd gate 396, nie 395)
BuildsKr640PositionAndDirectionSystem      end-to-end
```

**`UsesToolAxisColumnCells` jest teraz tabelą**, a nie testem jednej osi plus
osobnym testem drugiej. Powod jest prosty: `toolAxisColumn` obiecuje trzy
przejścia, a v4 pinowała dwa — oś Y nie pojawiała się w żadnym teście tego pliku.
Tabela `{X, 0}, {Y, 1}, {Z, 2}` z `SCOPED_TRACE` na oś przypina cały kontrakt i
zastępuje `UsesXToolAxisColumnWhenAsked`, który staje się jej podzbiorem.

**`RejectsInvalidDirectionTargetBeforeCanonicalization`** pilnuje granicy, której
testy `TargetValidation` **nie** dotykają: one dowodzą, że walidacja odrzuca zły
cel, ale nie że **builder ją woła**. Dwa bloki:

```text
blok 1   ToolDirectionTarget{ {1,0,0}, static_cast<ToolAxis>(99) }
         -> ConstraintBuilderErrorCode::InvalidTarget
            + zagnieżdżone TargetValidationErrorCode::InvalidToolAxis
         NIE: std::logic_error z emitera

blok 2   PositionAndToolDirectionTarget{ {NaN,0,0}, {0,0,0}, ToolAxis::Z }
         -> NonFinitePosition, nie ZeroDirection
```

Blok 1 rozróżnia **błąd danych** od **złamanego inwariantu programu**: podrobiona
oś w celu jest błędem caller'a i ma wrócić jako `expected`, a `throw` z
`appendDirectionEquations` (6) jest zarezerwowany na sytuację nieosiągalną po
walidacji. Gdyby builder poszedł do emitera z niezwalidowanym celem, ten test
złapałby wyjątek zamiast wartości.

Blok 2 jest **testem obietnicy z 4.2**, że pozycja jest walidowana pierwsza. Cel
jest zły na dwa sposoby naraz, więc kolejność przestaje być deklaracją w
komentarzu.

**`DirectionTargetLeavesSpinFree`** jest jedynym testem tego etapu sprawdzającym
**nieobecność** ograniczenia. Kształt: zbuduj FK, weź konfigurację, potem drugą
różniącą się **wyłącznie ostatnim jointem**, podstaw obie przez
`ExpressionEvaluator` i porównaj trzy lewe strony równań kierunku. Muszą dać tę
samą wartość, mimo że orientacja się zmieniła.

To jest test, który realizuje zdanie z promptu roadmapy: „przy swobodnym obrocie
wokół osi dyszy nie wolno przypadkiem narzucić pełnej orientacji". Test liczący
równania tego nie pokazuje — pokazuje to dopiero podstawienie.

**Warunek wstępny został zmierzony przed napisaniem tego proposala i wywrócił
pierwotne założenie.** v1 zakładała „KR640, oś Z". Pomiar (obie transformacje FK
z repo, zmiana ostatniego jointa o 1 rad, najgorsza zmiana w kolumnie):

```text
                 kolumna X       kolumna Y       kolumna Z
kr640            0   <-- 0.939           0.955
kr4_r600         0.955           0.939           1.11e-16   <-- 0
```

Czyli **ostatnia oś `kr640` pokrywa się z osią X ramki końcowej, nie Z**, a
robotem, w którym ostatni joint kręci wokół Z, jest `kr4_r600`. Test napisany
według v1 — KR640 z domyślną `ToolAxis::Z` — **nie przeszedłby**, i to nie z
powodu błędu w kodzie produkcyjnym.

Test obejmuje więc obie zmierzone pary, bo obie są prawdziwe i razem pokazują, że
własność zależy od geometrii, a nie od wybranego robota:

```text
kr4_r600 + ToolAxis::Z   -> trzy lewe strony niezmienione (1.11e-16, próg 1e-12)
kr640    + ToolAxis::X   -> trzy lewe strony niezmienione (dokładnie 0)
```

**`SpinFreedomTestIsNotVacuous`** jest kontrolą negatywną i wynika wprost z tego
samego pomiaru: test niezmienności przechodzi także wtedy, gdy lewe strony w ogóle
nie zależą od ostatniego jointa — na przykład gdyby emiter wziął złą kolumnę i
trafił w coś stałego. Kontrola pokazuje, że ten sam joint na tym samym robocie
**rusza** kierunkiem, jeśli wskazać inną oś:

```text
kr640 + ToolAxis::Z      -> zmiana 0.955, czyli test niezmienności ma co obalać
```

Bez tej pary `DirectionTargetLeavesSpinFree` byłby testem, który potrafi przejść
z niewłaściwego powodu.

### 7.2 `tests/test_ik_equation_model.cpp` — 11 nowych, 2 fixture'y rosną

```text
AcceptsPoseWithCompleteOrientation               XYZ + 9 unikalnych komórek -> Pose OK
AcceptsDirectionSystemWithoutPositionEquations   ToolDirection bez pozycji przechodzi
RejectsPositionEquationInDirectionTask           ToolDirection z pozycją -> mismatch
RejectsDirectionEquationInPositionTask           Position z kierunkiem -> mismatch
RejectsMixedToolAxesInOneSystem                  X, Y, Z ale trzy różne osie
RejectsForgedToolAxisInEquationSource            static_cast<ToolAxis>(99)
RejectsForgedCartesianComponentInDirectionSource static_cast<...>(99), źródło kierunku
OrdersPositionBeforeDirection                    kierunek przed pozycją -> Unordered
RejectsPoseWithIncompleteOrientation             osiem komórek to za mało
ReportsTaskSpaceConstrainedDegreesOfFreedom       wszystkie wartości + nullopt
DiagnosesEquationContentInFixedOrder             priorytet kodów -- patrz niżej
```

**`ReportsTaskSpaceConstrainedDegreesOfFreedom`** sprawdza **wartości**, nie tylko
przypadek zdegenerowany, i to jest poprawka z v5. v4 testowała wyłącznie
`nullopt` dla podrobionego rodzaju zadania — przez co implementacja z
przestawionymi liczbami:

```cpp
case IkTaskKind::Position:                 return 5;   // ma być 3
case IkTaskKind::Pose:                     return 2;   // ma być 6
case IkTaskKind::ToolDirection:            return 6;   // ma być 2
case IkTaskKind::PositionAndToolDirection: return 3;   // ma być 5
```

przechodziłaby komplet testów tego etapu. Tymczasem „policzalna liczba stopni
swobody" jest **gate'em F2.6**, a nie szczegółem — to jedyne miejsce, w którym
projekt mówi wprost, ile ograniczenie zabiera. Test pinuje wszystkie pięć
przypadków naraz: cztery wartości z tabeli 3 i `nullopt` dla
`static_cast<IkTaskKind>(99)`.

**`DiagnosesEquationContentInFixedOrder`** pinuje obie krawędzie łańcucha z 5.3,
w dwóch blokach `SCOPED_TRACE` wewnątrz jednego testu — tak jak istniejące testy
modelu:

```text
blok 1   podrobiona oś + podrobiona składowa, oś PÓŹNIEJ w wektorze
         -> InvalidToolAxis      (nie TaskEquationMismatch)

blok 2   pomieszane osie + niekompletna pozycja
         -> MixedToolAxes        (nie TaskEquationMismatch)

blok 3   podrobiony IkTaskKind + podrobiona oś
         -> InvalidToolAxis      (nie TaskEquationMismatch z profileOf)
```

Każdy blok zawiera **dwa** błędy naraz i ustawia je tak, żeby wadliwa wersja
zwróciła ten drugi: blok 1 i 2 obalają wariant jednoprzebiegowy z v2, blok 3
obala położenie `profileOf` z v3. Bez tego rozstawienia test przechodziłby
również dla kodu, który miał opisaną dziurę — czyli nie sprawdzałby niczego.

Trzy bloki w jednym teście, nie trzy testy: to jeden kontrakt — łańcuch
priorytetów — i ma pękać jako jedna rzecz. Gate zostaje `395/395`.

**Dwa istniejące testy rosną i tylko one są w tym etapie dotykane.**
`RejectsDuplicateEquationSource` i `RejectsUnorderedEquations` budują dziś `Pose`
z dwoma równaniami orientacji; po zaostrzeniu z 5.2 złapałyby najpierw
`TaskEquationMismatch`. **Intencja obu testów zostaje bez zmian**, rośnie
rusztowanie — ale jego kształt trzeba podać dokładnie, bo v1 opisała go błędnie i
prowadziła wprost do testu padającego z niewłaściwym kodem:

```text
duplikat:   dokładnie DZIEWIĘĆ równań orientacji, z czego jedno powtarza źródło
            -> osiem różnych komórek, dziewiąta NIEOBECNA, jedna zdublowana
            content  PASS  (orientationCount == 9)
            duplicate FAIL -> DuplicateEquationSource

porządek:   dziewięć RÓŻNYCH źródeł, komplet komórek, jedna para zamieniona
            content  PASS
            duplicate PASS
            order    FAIL -> UnorderedEquations
```

Kluczowe jest pierwsze pudełko. „Dziewięć komórek **plus** duplikat" to dziesięć
równań, a `checkEquationContent` sprawdza liczbę **przed** duplikatami — taki
fixture padłby na `TaskEquationMismatch` i test przestałby sprawdzać to, po co
istnieje. Duplikat musi **zastąpić** brakującą komórkę, nie dojść do kompletu.

Fakt, że oba testy zachowują dotychczasowe **kody błędu**, jest przy tym dowodem,
że kolejność diagnoz nie uległa zmianie (nota 13c architektury).

### 7.3 `tests/test_target_validation.cpp` — 4

```text
AcceptsUnnormalizedDirection        (0,0,5) i (0,0,0.2) przechodzą
RejectsNonFiniteDirection           NaN i ±inf, każda składowa osobno
RejectsZeroDirection                {0,0,0} -> ZeroDirection
RejectsInvalidToolAxisInTarget      static_cast<ToolAxis>(99) -> InvalidToolAxis
```

`RejectsNonFiniteDirection` dostaje dodatkowo blok `SCOPED_TRACE` pinujący
**priorytet** kodów na celu zepsutym wielokrotnie:

```text
direction = {NaN, 0, 0}, axis = static_cast<ToolAxis>(99)
    -> NonFiniteDirection, nie InvalidToolAxis
direction = {0, 0, 0},   axis = static_cast<ToolAxis>(99)
    -> ZeroDirection,     nie InvalidToolAxis
```

Komentarz przy `validate` deklaruje kolejność `NonFiniteDirection ->
ZeroDirection -> InvalidToolAxis` jako obserwowalną, a testy pokrywające każdy
kod osobno tego nie dowodzą — dokładnie ta sama luka, którą review zamknęło po
stronie `IkEquationSystem`. Nie dochodzi nowy `TEST`, więc gate zostaje `395/395`.

Razem **+31 testów** (16 + 11 + 4), gate `396/396` (365 + 31).

Rachunek po v5, bo zmieniły się dwie pozycje naraz: `UsesXToolAxisColumnWhenAsked`
**znika** (pochłonięty przez tabelę w `UsesToolAxisColumnCells`), a
`RejectsInvalidDirectionTargetBeforeCanonicalization` dochodzi — netto zero, więc
plik kierunkowy ma nadal piętnaście testów. `ReportsTaskSpaceConstrainedDegreesOfFreedom`
zastępuje `RejectsForgedTaskKindInDofQuery`, więc model też zostaje na jedenastu.

---

## 8. Gate

```text
build (Release)        0 ostrzeżeń
build-werror           0 ostrzeżeń, asercje żywe
ctest                  396/396 w obu (365 + 31)  -- osiągnięte
istniejące testy       dwa fixture'y Pose rosną (7.2), zero zmian w oczekiwanych
                       kodach błędu; poza tym zero zmienionych asercji
zakres                 dwa nowe nagłówki modelu, jeden nowy plik testowy,
                       pięć plików modelu/buildera zmienionych
diff kontrolny         git diff --stat nie może pokazać warstwy symbolicznej,
                       loadera, budowniczych transformacji, fasady ani
                       tests/support
model spójny           każdy IkTaskKind ma profil, walidację, przeciążenie
                       buildera i testy -- warunek podziału po rodzinach
```

Przedostatni wiersz jest sprawdzianem architektury, nie porządku: gdyby
którykolwiek z tych plików musiał się zmienić, znaczyłoby to, że ograniczenie
kierunkowe przecieka poza warstwę modelu i buildera.

---

## 9. Ryzyka implementacyjne

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Zaostrzenie `Pose` psuje diagnozę duplikatu | **realne, złapane w review v1** | Liczba, nie kompletność (5.2); fixture duplikatu ma **dziewięć** równań z jednym powtórzonym źródłem, nie dziewięć plus duplikat (7.2) |
| Nowy kontrakt `Pose` przypięty tylko z jednej strony | **realne, złapane w review v1** | `RejectsPoseWithIncompleteOrientation` **i** `AcceptsPoseWithCompleteOrientation`: osiem odrzucone, dziewięć przyjęte (7.2) |
| Komunikat błędu mówi „position" o równaniu kierunku | realne | `componentIndex` uogólniony; kod błędu bez zmiany (5.5) |
| Kod błędu zależy od kolejności wadliwych równań | **realne, złapane w review v2** | Trzy przebiegi zamiast jednego (5.3), precedens `checkUnknowns`; `DiagnosesEquationContentInFixedOrder` ustawia błędy tak, że wersja jednoprzebiegowa **oblewa** (7.2) |
| Podrobiony `IkTaskKind` wyprzedza kontrolę osi | **realne, złapane w review v3** | `profileOf` przeniesione za PASS 2 (5.3); blok 3 testu priorytetu obala poprzednie położenie (7.2) |
| Wartości DOF przestawione między rodzajami zadań | **realne, złapane w review v4** | `ReportsTaskSpaceConstrainedDegreesOfFreedom` pinuje wszystkie cztery wartości, nie tylko `nullopt` (7.2) |
| Builder pomija walidację i wchodzi do emitera | **realne** | `RejectsInvalidDirectionTargetBeforeCanonicalization`: podrobiona oś ma wrócić jako `InvalidTarget`, nie jako `std::logic_error` (7.1) |
| `toolAxisColumn` przypięty w 2/3 | **realne, złapane w review v4** | `UsesToolAxisColumnCells` jako tabela po trzech osiach (7.1) |
| Naiwna normalizacja niszczy kierunek na granicy | **realne, policzone** | Skalowanie (6); dwa testy graniczne, `DBL_MAX` i subnormal |
| Trzy równania o trzech osiach | **realne** | `MixedToolAxes` w zliczaniu (5.3), test w modelu |
| `sourceOrderKey` zakłada, że treść już sprawdzona | **realne** | Klucz waliduje sam: `componentIndex` i `toolAxisColumn` przed użyciem (5.4) |
| Kolumna osi pomylona z komórką orientacji | realne | `UsesToolAxisColumnCells` jako tabela po trzech osiach, `lhs` przez `sameNode` — pomyłka kolumny nie przejdzie dla żadnej osi |
| `DirectionTargetLeavesSpinFree` zakłada geometrię robota | **było realne, zmierzone przed proposalem** | Założenie v1 (KR640 + oś Z) okazało się **fałszywe**: ostatnia oś KR640 pokrywa się z X, a nie Z. Test bierze dwie zmierzone pary i kontrolę negatywną (7.1) |
| Test niezmienności przechodzi z niewłaściwego powodu | **realne** | `SpinFreedomTestIsNotVacuous`: ten sam joint i robot, inna oś -> zmiana 0.955 (7.1) |
| `throw` w emiterze | niskie | Nieosiągalne po `validate(target)`; precedens `orientationSource` z F2.5 (6) |

---

## 10. Rekomendacja

Zaimplementować po zatwierdzeniu v5. Po F2.6a model ma cztery spójne rodzaje
zadań, a rodzina kierunku działa end-to-end. Następny dokument: **F2.6b** — rodzina
kąta, dwa pozostałe rodzaje zadań i pierwsze liczone `lhs` w projekcie.

Otwarte pytania: brak.
