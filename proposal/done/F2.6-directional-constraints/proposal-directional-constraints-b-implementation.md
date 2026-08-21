# Proposal: constrainty kierunkowe — F2.6b (rodzina kąta), implementacja

**Etap:** F2.6b · **v4 — APPROVED, ZAIMPLEMENTOWANE 2026-08-21** · część 3 z 3

> **Zaimplementowane.** Kod wszedł w brzmieniu z tego dokumentu, z bezpiecznikiem
> liczby iteracji w pętli wyszukującej punkt zapadnięcia (uwaga nieblokująca z
> review): 200 kroków i `ASSERT_LT` z czytelnym komunikatem, żeby nietypowy libm
> dał diagnozę zamiast zawieszenia.
>
> **Gate wyszedł `410/410`, nie `409/409`**, bo plik kierunkowy dostał dziewiąty
> test zamiast ośmiu: `RejectsDegenerateAngleTarget` — odrzucenie kąta
> zdegenerowanego **przez builder**, nie przez sam walidator. Wartość jest ta
> sama, co przy `RejectsInvalidDirectionTargetBeforeCanonicalization` z F2.6a:
> testy `TargetValidation` dowodzą, że walidacja odrzuca, ale nie że builder ją
> woła przed emisją.

> **v4, po trzecim review.** Kod produkcyjny zatwierdzony i nietknięty; dwie
> poprawki:
>
> 1. **Test degeneracji kodował `1e-9` jako próg.** Dokument mówi, że szerokość
>    pasma zapadnięcia jest zależna od platformy i **nie jest** publiczną
>    tolerancją — a test zakładał, że `std::cos(1e-9) == 1.0` wszędzie. Test
>    **wyszukuje** teraz punkt zapadnięcia zamiast go znać (7.2).
> 2. **Uzasadnienie normalizacji `reference` w 6 było za wąskie.** Twierdziło, że
>    kanonizacja jest warunkiem poprawności tylko w rodzinie II — nieprawda:
>    w rodzinie I lewa strona ma normę 1 tożsamościowo, więc `d = (0,0,5)` bez
>    kanonizacji dałoby układ nierozwiązywalny. Poprawione (6).

> **v3, po drugim review.** Dwa blockery i dwie pozostałości tekstowe:
>
> 1. **`sameSource` dostaje po F2.6b fałszywy publiczny komentarz.** Dotąd każde
>    źródło dawało się sprowadzić do **komórki macierzy**; równanie kąta nie adresuje
>    żadnej komórki, a samo źródło nie niesie ani `reference`, ani `angle`. Trzy
>    komentarze przechodzą na pojęcie **kanonicznego slotu ograniczenia** (5.5).
> 2. **Zerowa kolumna `angle` w profilu nie była przypięta.** Test pokrywał
>    wyłącznie ścieżkę szczęśliwą; strona negatywna — `Position` z równaniem kąta —
>    nie była sprawdzana wcale. `AcceptsAngleSystemWithOneEquation` zastąpiony
>    przez `EnforcesDirectionAngleEquationProfile` z czterema blokami (7.3).
> 3. Dwa warunki w komentarzu walidacji były **odwrócone** względem kodu (4.1).
> 4. Wyczyszczone: opis doboru `reference` w 7.1 i stara nazwa testu w tabeli ryzyk.
>
> Gate bez zmian: `409/409`.

> **v2, po pierwszym review.** Trzy blockery i jedna korekta testu, wszystkie przyjęte:
>
> 1. **Brakowało dwóch przeciążeń `TargetVisitor`.** `IkTarget` rośnie o dwie
>    alternatywy, więc bez nich `validate(const IkTarget&)` **się nie kompiluje** —
>    a to jest dokładnie ten mechanizm, na który ten proposal się powołuje przy
>    `EquationKindVisitor`. Dopisane (4.3).
> 2. **`sourceOrderKey` nie realizował własnej obietnicy.** Dokument pisał, że
>    walidacja osi zostaje „mimo to", a pokazany kod dla źródła kąta jej nie miał.
>    Poprawione (5.3) — ten helper ma nie zależeć od kolejności wywołań, i to był
>    cały powód jego kształtu w F2.6a.
> 3. **DOF `PositionAndDirectionAngle == 4` nie było przypięte.** Ta sama luka,
>    którą review zamknęło w F2.6a: implementacja zwracająca `6` przeszłaby gate.
>    Istniejący `ReportsTaskSpaceConstrainedDegreesOfFreedom` **rozszerzony** o dwa
>    wiersze, bez nowego testu (7.3).
> 4. **Test struktury `lhs` miał za mocny kontrakt.** Sprawdzone w kodzie:
>    `ExpressionFactory::multiply` zwija `1 * x -> x`, więc dla współczynnika `1.0`
>    żaden `MultiplyNode` nie powstaje. Zdanie „struktura nie zależy od danych"
>    było **nieprawdziwe** i zostało wycofane; test buduje oczekiwane wyrażenie
>    **tą samą fabryką** i porównuje przez `structurallyEqual` (6, 7.1).
>
> Plus dwie drobne: routing wariantu przez `validate(IkTarget{...})` w teście
> walidacji i poprawiony zapis kroku 3 w komentarzu kontraktowym (4.1).

Architektura: [`proposal-directional-constraints-architecture.md`](proposal-directional-constraints-architecture.md)
— **v4, APPROVED**. Poprzednia połowa:
[`proposal-directional-constraints-a-implementation.md`](proposal-directional-constraints-a-implementation.md)
— **zaimplementowana 2026-08-21**, gate `396/396` w obu konfiguracjach.

Ten dokument domyka etap F2.6: druga rodzina równań, dwa ostatnie rodzaje zadań i
**pierwsza w projekcie lewa strona równania, która nie jest komórką macierzy**.

## Prompt

> Napisać proposal implementacyjny F2.6b po zamknięciu F2.6a.

## 0. Zakres i stan wyjściowy

```text
JEST po F2.6a                              DOCHODZI w F2.6b
--------------------------------------     ----------------------------------
IkTaskKind: 4 wartości                     +2 (DirectionAngle, +Position)
IkTarget: 4 alternatywy                    +2
EquationSource: 3 alternatywy              +1 (DirectionAngleEquationSource)
ToolAxis + toolAxisColumn                  bez zmian
trzy przebiegi checkEquationContent        przebiegi 1-2 obejmują też źródło kąta
sourceOrderKey: rangi 0..2                 +ranga 3
DOF: 3 / 6 / 2 / 5                         +1 i +4
walidacja kierunku                         +walidacja kąta
emitery: pozycja, orientacja, kierunek     +kąt (pierwsze liczone lhs)
396 testów                                 +13
```

Po tym etapie **cała rodzina F2.6 jest zamknięta**, a Faza 2 ma do zrobienia już
tylko F2.7 — integrację z fasadą.

---

## 1. `Equation.hpp` — trzy dopiski

```cpp
enum class IkTaskKind
{
    Position,
    Pose,
    ToolDirection,
    PositionAndToolDirection,
    DirectionAngle,
    PositionAndDirectionAngle
};
```

```cpp
// The angle between the tool axis and a reference direction:
//
//     dot(R(q) e_t, n) = cos(angle)
//
// One scalar equation, so there is no index to carry -- only which tool axis it
// talks about. The name deliberately avoids "axis-angle": that is a
// representation of a rotation, and confusing the two would be expensive.
struct DirectionAngleEquationSource
{
    ToolAxis axis{ToolAxis::Z};

    friend bool operator==(const DirectionAngleEquationSource&,
                           const DirectionAngleEquationSource&) noexcept = default;
};

using EquationSource = std::variant<PositionEquationSource,
                                    OrientationEquationSource,
                                    DirectionEquationSource,
                                    DirectionAngleEquationSource>;
```

`EquationKind` dostaje `DirectionAngle`, a `EquationKindVisitor` czwarte
przeciążenie — bez niego build staje, tak jak przy F2.6a.

---

## 2. `IkTarget.hpp` — dwa ostatnie cele

```cpp
// The angle between the tool axis and a reference direction, in the base frame:
//
//     dot(R(q) e_t, n) = cos(angle)      ONE equation, ONE degree of freedom
//
// The tool axis ends up somewhere on a cone about n. What stays free is the
// position of that axis on the cone AND the spin about the axis itself.
//
// reference is finite and non-zero, and does NOT have to be a unit vector --
// same contract as a direction, for the same reason: it names a ray. A surface
// normal from a cross product is the expected source, hence any length.
//
// The name says nothing about surfaces: the model does not know, and has no way
// to know, that n came from a normal. It is an angle to a reference direction,
// and a surface normal is one source of it.
//
// angle is in radians and its interval is OPEN: 0 and pi are rejected. There the
// cone collapses to a line, the constraint removes two rotational degrees of
// freedom instead of one, and the scalar equation goes singular -- and that
// geometry already has an exact description in ToolDirectionTarget.
struct DirectionAngleTarget
{
    Vector3 reference;
    double angle{};
    ToolAxis axis{ToolAxis::Z};
};

struct PositionAndDirectionAngleTarget
{
    Vector3 position;     // metres
    Vector3 reference;    // finite, non-zero; length carries no semantics
    double angle{};       // radians, open interval
    ToolAxis axis{ToolAxis::Z};
};

using IkTarget = std::variant<PositionTarget,
                              PoseTarget,
                              ToolDirectionTarget,
                              PositionAndToolDirectionTarget,
                              DirectionAngleTarget,
                              PositionAndDirectionAngleTarget>;
```

---

## 3. `IkTaskDegreesOfFreedom.hpp` — dwa wiersze

```cpp
    case IkTaskKind::DirectionAngle:            return 1;
    case IkTaskKind::PositionAndDirectionAngle: return 4;
```

Komentarz rośnie o zdanie, które jest **warunkiem** prawdziwości tych liczb:

```cpp
//   DirectionAngle            1   the tool axis lands on a cone about n: ONE
//                                 degree of freedom removed, not two. True only
//                                 because the angle interval is open -- at 0 and
//                                 pi the cone degenerates to a line and the
//                                 constraint removes two. Those targets are
//                                 rejected, and belong to ToolDirection.
//   PositionAndDirectionAngle 4   position (3) + the cone (1); the axis stays
//                                 free on the cone and the spin stays free too
```

To jest jedyne miejsce w F2.6b, w którym walidacja kąta (4) i publiczna funkcja
DOF **muszą** się zgadzać co do słowa: gdyby końce przedziału zostały
dopuszczone, ta tabela byłaby fałszywa dla legalnego celu, a funkcja musiałaby
przyjmować cel zamiast rodzaju zadania.

---

## 4. Walidacja kąta

### 4.1 Kody i kontrakt

```cpp
    NonFiniteAngle,
    AngleOutOfRange,
    DegenerateAngle
```

```cpp
// Checked in this order, and the order is observable:
//   1. reference finite, non-zero      -> NonFiniteDirection / ZeroDirection
//   2. angle finite                    -> NonFiniteAngle
//   3. angle outside [0, pi]           -> AngleOutOfRange
//   4. angle is 0 or pi                -> DegenerateAngle   (mathematics)
//   5. |cos(angle)| == 1               -> DegenerateAngle   (arithmetic)
//   6. tool axis is an enumerator      -> InvalidToolAxis
//
// Step 3 exists because cos() is what reaches the equation: angle = 3*pi would
// be accepted and silently mean pi, handing the caller a different task than the
// one they asked for.
//
// Steps 4 and 5 are separate on purpose. Step 4 rejects the MATHEMATICAL
// endpoints and is portable -- it does not depend on anyone's libm. Step 5
// catches what has numerically collapsed onto them, and the width of that band
// is a platform detail, not a published tolerance: measured here, cos(angle) is
// exactly 1.0 for angle <= 1.05e-8 rad, but that number appears in no constant
// and in no test.
//
// The message must not say "angle 0 or pi": 1e-9 is rejected too.
[[nodiscard]] std::expected<void, TargetValidationError>
validate(const DirectionAngleTarget& target);

[[nodiscard]] std::expected<void, TargetValidationError>
validate(const PositionAndDirectionAngleTarget& target);
```

### 4.2 Implementacja

```cpp
std::expected<void, TargetValidationError> checkAngle(double angle)
{
    if (!std::isfinite(angle))
        return makeError(TargetValidationErrorCode::NonFiniteAngle,
                         "angle is not finite");

    if (angle < 0.0 || angle > std::numbers::pi_v<double>)
        return makeError(TargetValidationErrorCode::AngleOutOfRange,
                         "angle must lie in [0, pi], got " + format(angle));

    // The mathematical endpoints first, so the rejection does not depend on the
    // implementation of std::cos.
    if (angle == 0.0 || angle == std::numbers::pi_v<double>)
        return makeError(TargetValidationErrorCode::DegenerateAngle,
                         "angle constrains the tool axis to a single direction; "
                         "use ToolDirectionTarget with direction = +/- reference");

    // ...then whatever collapsed onto them numerically. This tests the very
    // value that becomes the right-hand side.
    if (std::abs(std::cos(angle)) == 1.0)
        return makeError(TargetValidationErrorCode::DegenerateAngle,
                         "angle is numerically degenerate: cos(angle) is "
                         "indistinguishable from +/-1; use ToolDirectionTarget "
                         "with direction = +/- reference");

    return {};
}
```

```cpp
std::expected<void, TargetValidationError> validate(const DirectionAngleTarget& target)
{
    if (auto checked = checkDirection(target.reference); !checked)
        return checked;

    if (auto checked = checkAngle(target.angle); !checked)
        return checked;

    return checkToolAxis(target.axis);
}

std::expected<void, TargetValidationError>
validate(const PositionAndDirectionAngleTarget& target)
{
    if (auto checked = validate(PositionTarget{target.position}); !checked)
        return checked;

    return validate(DirectionAngleTarget{target.reference, target.angle, target.axis});
}
```

`checkDirection` i `checkToolAxis` są już w pliku od F2.6a — rodzina kąta nie
dorabia własnych reguł dla wektora, bo `reference` jest **tym samym rodzajem
wielkości** co `direction`. Dochodzi `#include <numbers>`.

### 4.3 `TargetVisitor` — dwa przeciążenia, bez których build staje

v1 tego pominęła, a jest to **błąd kompilacji**, nie niedopatrzenie stylistyczne:
`IkTarget` rośnie o dwie alternatywy, a `validate(const IkTarget&)` chodzi po nim
jawnym zestawem przeciążeń właśnie po to, żeby nowa alternatywa **zatrzymała
build** do czasu decyzji.

```cpp
struct TargetVisitor
{
    // ... cztery przeciążenia z F2.3, F2.6a ...

    std::expected<void, TargetValidationError>
    operator()(const DirectionAngleTarget& target) const
    {
        return validate(target);
    }

    std::expected<void, TargetValidationError>
    operator()(const PositionAndDirectionAngleTarget& target) const
    {
        return validate(target);
    }
};
```

Ten sam mechanizm, na który dokument powołuje się przy `EquationKindVisitor` (1) —
i tym razem trzeba go było zastosować do siebie samego.

---

## 5. `IkEquationSystem` — trzy drobne rozszerzenia i jedna pułapka

### 5.1 Pułapka: przebiegi 1 i 2 muszą widzieć **oba** źródła z osią

To jest jedyne miejsce w F2.6b, gdzie łatwo o cichy błąd. Po F2.6a przebiegi
sprawdzające oś patrzą wyłącznie na `DirectionEquationSource`:

```cpp
if (const auto* direction = std::get_if<DirectionEquationSource>(&equation.source))
```

`DirectionAngleEquationSource` **też niesie `ToolAxis`**, więc bez zmiany:

```text
DirectionAngleEquationSource{ static_cast<ToolAxis>(99) }
        -> przechodzi przez PASS 1 i PASS 2 niezauważona
        -> trafia do sourceOrderKey, który ją odrzuci... ale kodem z innego
           miejsca i po sprawdzeniu zawartości, czyli poza deklarowanym
           priorytetem
```

Oba przebiegi dostają więc mały helper zamiast `get_if` po jednym typie:

```cpp
// The tool axis carried by a source, if it carries one. Two alternatives do;
// adding a third that does must be a conscious edit here.
std::optional<ToolAxis> toolAxisOf(const EquationSource& source)
{
    return std::visit(
        Overloaded{
            [](const PositionEquationSource&) { return std::optional<ToolAxis>{}; },
            [](const OrientationEquationSource&) { return std::optional<ToolAxis>{}; },
            [](const DirectionEquationSource& direction) {
                return std::optional<ToolAxis>{direction.axis};
            },
            [](const DirectionAngleEquationSource& angle) {
                return std::optional<ToolAxis>{angle.axis};
            }},
        source);
}
```

Jawny zestaw przeciążeń, nie `get_if` — bo pytanie „czy to źródło ma oś"
dotyczy **wszystkich** alternatyw, a każda nowa musi na nie odpowiedzieć
świadomie. Przebiegi 1 i 2 chodzą wtedy po `toolAxisOf` i nie wiedzą, ilu
rodzajów źródeł dotyczą.

### 5.2 Profil i zliczanie

```cpp
struct EquationProfile
{
    bool position{};
    std::size_t orientation{};
    bool direction{};
    std::size_t angle{};        // nowe: 0 albo 1
};
```

```cpp
    case IkTaskKind::DirectionAngle:            return EquationProfile{false, 0, false, 1};
    case IkTaskKind::PositionAndDirectionAngle: return EquationProfile{true,  0, false, 1};
```

Cztery wcześniejsze wiersze dostają `0` w nowej kolumnie — i to zero jest
istotne: bez niego `Position` mogłoby nieść równanie kąta.

Przebieg 3 zlicza `angleCount` i porównuje z profilem, dokładnie jak
`orientationCount`.

### 5.3 Porządek

```cpp
            [](const DirectionAngleEquationSource& angle)
                -> std::expected<SourceOrderKey, IkEquationSystemError> {
                if (!toolAxisColumn(angle.axis))
                    return makeError(IkEquationSystemErrorCode::InvalidToolAxis,
                                     "a direction-angle equation carries an invalid tool axis");

                // One equation per system, so the index is constant.
                return SourceOrderKey{3, 0};
            }},
```

Oś nie wchodzi do **klucza** z tego samego powodu, co przy kierunku:
`MixedToolAxes` już gwarantuje jedną oś na system. Ale **sprawdzana jest tu tak
samo**, i v1 tego nie robiła, choć pisała, że robi. To nie jest kosmetyka: cały
sens tego helpera w F2.6a polegał na tym, żeby **nie zależeć od kolejności
wywołań w `create()`**. Argument „`checkEquationContent` chodzi wcześniej" jest
dokładnie tym założeniem, którego ten kod ma nie robić — zamiana dwóch linii w
`create()` nie może zamieniać błędu walidacji w odczyt poza zakresem.

### 5.5 `sameSource` przestaje mówić o komórkach macierzy

To jest zmiana **publicznego kontraktu**, nie kosmetyka, i F2.6b jest etapem,
który ją wymusza. Do tej pory każde `EquationSource` dawało się sprowadzić do
konkretnej komórki FK, więc komentarz przy `sameSource` mógł mówić wprost o
komórce. Po dodaniu źródła kąta to przestaje być prawdą na dwa sposoby:

```text
lewa strona    nx*T(0,c) + ny*T(1,c) + nz*T(2,c)   -- nie jest żadną komórką
źródło         DirectionAngleEquationSource{axis}   -- nie niesie reference ani angle
```

Drugi punkt jest ważniejszy, niż wygląda: źródło **nie jest** pełną tożsamością
matematycznego ograniczenia. Dwa cele o różnych `reference` i różnych `angle`
mają to samo źródło — i tak ma być, bo źródło odpowiada na pytanie „które miejsce
w układzie zajmuje to równanie", a nie „co dokładnie ono mówi".

Algorytm zostaje bez zmiany (`lhs == rhs`); zmieniają się **trzy komentarze**:

**`Equation.hpp`, przy `sameSource`** — dziś:

```cpp
// Identity of the constrained quantity. Two equations with equal sources
// address the same matrix cell -- either redundant or contradictory.
```

po F2.6b:

```cpp
// Identity of the canonical constraint slot a given equation occupies within a
// task:
//
//   Position        one Cartesian component of the translation
//   Orientation     one cell of the rotation block
//   Direction       one Cartesian component of one tool axis direction
//   DirectionAngle  the single scalar angle slot for one tool axis
//
// Equal sources mean two equations occupy the SAME slot -- redundant at best,
// contradictory at worst. They do NOT imply equal expression trees or equal
// right-hand sides: a direction-angle source carries neither the reference
// direction nor the angle, because those describe what the constraint says, not
// which slot it fills.
```

**`IkEquationSystem.cpp`, przy `checkDuplicateSources`** — „two equations
addressing one matrix cell" staje się „two equations occupying the same canonical
constraint slot".

**`IkEquationSystem.hpp`, w liście priorytetów `create()`** — to samo zdanie w
uzasadnieniu `DuplicateEquationSource before UnorderedEquations`.

---

## 6. `ConstraintBuilder` — pierwsze liczone `lhs`

```cpp
// The first left-hand side in this project that is not a cell of the FK matrix.
//
//     lhs = n_x T(0,c) + n_y T(1,c) + n_z T(2,c)
//
// This is exactly what F2.5 prepared for when the symbol collector moved from
// the transform cells to the equations: the guarantee "every symbol in the
// equations is a declared unknown" needs no change to cover a computed side.
//
// Every term is built, including one whose coefficient is zero: dropping it
// would be constant folding by hand, and this project deliberately does NOT fold
// x * 0, because that erases domain information.
//
// The resulting SHAPE still depends on the data, and a test must not assume
// otherwise: ExpressionFactory folds 1 * x -> x, so a coefficient of exactly 1
// produces no MultiplyNode at all. That is the factory's contract, not an
// accident, and the structural test builds its expectation through the same
// factory rather than describing a shape by hand (7.1).
void ConstraintBuilder::appendDirectionAngleEquation(std::vector<Equation>& equations,
                                                     const SymbolicTransform& transform,
                                                     ToolAxis axis,
                                                     const Vector3& unitReference,
                                                     double angle) const
{
    const auto column = toolAxisColumn(axis);
    if (!column)
        throw std::logic_error("tool axis must be X, Y or Z");

    const double components[3] = {unitReference.x, unitReference.y, unitReference.z};

    Expression sum = factory_.multiply(factory_.constant(components[0]),
                                       transform(0, *column));
    for (std::size_t row = 1; row < 3; ++row)
        sum = factory_.add(sum, factory_.multiply(factory_.constant(components[row]),
                                                  transform(row, *column)));

    equations.push_back(Equation{std::move(sum), factory_.constant(std::cos(angle)),
                                 DirectionAngleEquationSource{axis}});
}
```

Przeciążenia `build()` mają ten sam kształt, co w F2.6a — walidacja, kanonizacja
`reference` przez `normalizedDirection`, emisja:

```cpp
    if (auto checked = validate(target); !checked)
        return std::unexpected(invalidTarget(std::move(checked.error())));

    std::vector<Equation> equations;              // albo positionEquations(...)
    appendDirectionAngleEquation(equations, transform, target.axis,
                                 normalizedDirection(target.reference), target.angle);

    return assemble(chain, IkTaskKind::DirectionAngle, std::move(equations));
```

**Dlaczego `reference` też jest normalizowany.** Bez tego równanie brzmiałoby
`dot(oś, n) = cos(alfa)` z lewą stroną przeskalowaną przez `|n|`, czyli opisywałoby
inny kąt niż zamówiony.

Kanonizacja jest warunkiem poprawności w **obu** rodzinach — v3 sugerowała, że
tylko tutaj, i to było błędne. Różni się wyłącznie to, co kanonizacja ratuje:

```text
rodzina I    R(q) e_t = d          lewa strona ma normę 1 TOŻSAMOŚCIOWO, więc
                                   d = (0,0,5) dałoby układ bez rozwiązania
rodzina II   dot(R(q) e_t, n)      współczynniki iloczynu skalarnego; bez
                                   kanonizacji równanie opisuje inny kąt
```

W obu przypadkach kanonizacja jest tym, co sprawia, że równanie znaczy to, co
caller zamówił — raz przez prawą stronę, raz przez współczynniki lewej.

---

## 7. Testy — 13

### 7.1 `tests/test_directional_constraints.cpp` — 9

```text
BuildsOneDirectionAngleEquation            1 równanie, IkTaskKind, DOF == 1
AngleLhsMatchesDirectionDotProduct         struktura lhs, nie tylko wartość
UsesCosineOfAngleOnRightHandSide           rhs == cos(alfa), nie alfa
NormalizesReferenceBeforeBuildingEquation  n i 5n dają tę samą lewą stronę
BuildsFourEquationsForPositionAndAngle     3 + 1, w tej kolejności
AngleOnlySystemHasNoPositionEquations
AngleEquationEvaluatesToCosineAtSolution   kontrola numeryczna -- niżej
RejectsDegenerateAngleTarget               builder odrzuca kąt 0 z zagnieżdżonym
                                           DegenerateAngle -- nie sam walidator
BuildsKr4PositionAndAngleSystem            end-to-end
```

**`AngleLhsMatchesDirectionDotProduct`** sprawdza kształt wyrażenia, bo sama
wartość przepuściłaby implementację liczącą iloczyn skalarny **numerycznie** i
wstawiającą stałą — czyli równanie, które nie zależy od `q` i nie ogranicza
niczego.

Kształt **nie jest** opisywany ręcznie, i to jest poprawka z v2. Oczekiwane
wyrażenie powstaje przez tę samą fabrykę, więc test respektuje jej normalizacje
zamiast im przeczyć:

```cpp
const auto expected = factory.add(
    factory.add(factory.multiply(factory.constant(nx), transform(0, column)),
                factory.multiply(factory.constant(ny), transform(1, column))),
    factory.multiply(factory.constant(nz), transform(2, column)));

EXPECT_TRUE(structurallyEqual(system->equations()[0].lhs, expected));
```

Powodem jest konkret sprawdzony w kodzie: `ExpressionFactory::multiply` zwija
`1 * x -> x`, więc dla `reference = (0,0,1)` trzeci składnik **nie ma**
`MultiplyNode`. Test opisujący „sumę trzech iloczynów" oblewałby na poprawnej
implementacji.

Wybrany `reference` nie ma przy tym **żadnego** współczynnika równego 1 — np.
`(2, 1, 2)`, które po kanonizacji daje `(2/3, 1/3, 2/3)` — żeby test przechodził
przez pełną postać z trzema `MultiplyNode`, a nie przez wariant zwinięty.

Zero w składowej byłoby akurat nieszkodliwe (`x * 0` **nie** jest zwijane, więc
`MultiplyNode` zostaje), ale jedynka jest — i to właśnie jej trzeba unikać.
`(0.6, 0.8, 0.0)` z v2 było więc opisane błędnie jako „bez zera", choć zadziałałoby;
`(2, 1, 2)` jest po prostu jednoznaczne.

**`AngleEquationEvaluatesToCosineAtSolution`** jest testem, który wiąże symbolikę
z geometrią: weź `kr4_r600`, podstaw konfigurację, policz oś narzędzia z FK,
weź jako `reference` wektor obrócony o znany kąt względem tej osi, zbuduj układ i
podstaw **tę samą** konfigurację do lewej strony. Musi wyjść `cos(alfa)` w
granicach `1e-12`. Bez tego testu wszystkie pozostałe sprawdzają wyłącznie
składnię równania, a nie to, czy opisuje ono zadany kąt.

### 7.2 `tests/test_target_validation.cpp` — 3

```text
AcceptsAngleInsideOpenInterval        0.3, pi/2, pi - 0.3
RejectsAngleOutsideRange              -0.1 i pi + 0.1 -> AngleOutOfRange
RejectsDegenerateAngle                0 i pi (matematyka) oraz WYSZUKANY kąt,
                                      przy którym cos zapada się do ±1 -- niżej
                                      + NonFiniteAngle dla NaN i inf
                                      + priorytet: zły reference wygrywa z kątem
                                      + InvalidToolAxis dla podrobionej osi
                                      + routing: validate(IkTarget{...}) po
                                        wariancie, nie tylko po typie
```

**Degeneracja numeryczna jest wyszukiwana, nie zakładana.** v3 wpisywała do testu
`1e-9`, czyli dokładnie tę stałą, o której ten sam dokument mówi, że **nie jest
publiczną tolerancją** i zależy od implementacji `std::cos`. Test sprawdza więc
warunek, a nie próg:

```cpp
// Find the angle at which cos() has collapsed onto 1 on THIS libm, instead of
// assuming where that happens. The contract is "cos(angle) is indistinguishable
// from +/-1", not "angle < some published number".
double angle = 1e-3;
while (angle > 0.0 && std::abs(std::cos(angle)) != 1.0)
    angle *= 0.5;

ASSERT_GT(angle, 0.0);
ASSERT_LT(angle, std::numbers::pi_v<double>);
ASSERT_EQ(std::abs(std::cos(angle)), 1.0);
// ... validate(...) -> DegenerateAngle
```

Analogicznie po stronie `pi`, z dodatkowym warunkiem `pi - delta != pi`, żeby
przypadek nie zdegenerował się do samego `pi` — czyli do kroku matematycznego,
który jest sprawdzany osobno.

Zmierzone na tym toolchainie: obie pętle zbiegają się po **17 połowieniach** do
`7.63e-9`, a `pi - delta` różni się od `pi` o siedem rzędów więcej niż `ulp(pi)`
(`4.44e-16`). Liczby są tu **obserwacją**, nie kontraktem: na innej platformie
pętla zatrzyma się gdzie indziej i test nadal będzie prawdziwy.

Ostatni wiersz pinuje coś, czego przeciążenia same nie dowodzą: że
`validate(const IkTarget&)` **kieruje** wariant do właściwego przeciążenia.
Kompilacja wymusza istnienie `TargetVisitor::operator()`, ale nie to, żeby robił
cokolwiek sensownego.

### 7.3 `tests/test_ik_equation_model.cpp` — 2 nowe + 1 rozszerzony

```text
EnforcesDirectionAngleEquationProfile  cztery bloki -- patrz niżej
RejectsForgedToolAxisInAngleSource    static_cast<ToolAxis>(99) w źródle kąta
                                      -> InvalidToolAxis, nie TaskEquationMismatch

rozszerzony (bez nowego TEST):
ReportsTaskSpaceConstrainedDegreesOfFreedom   +DirectionAngle == 1
                                              +PositionAndDirectionAngle == 4
```

**`EnforcesDirectionAngleEquationProfile`** zastępuje test, który sprawdzał
wyłącznie ścieżkę szczęśliwą. Nowa kolumna `angle` w `EquationProfile` ma **dwie**
strony, a v2 pinowała jedną — zera w czterech wcześniejszych profilach nie
sprawdzało nic:

```text
blok 1   DirectionAngle + dokładnie jedno równanie kąta   -> OK
blok 2   DirectionAngle + pozycja + kąt                    -> TaskEquationMismatch
blok 3   DirectionAngle + kierunek + kąt                    -> TaskEquationMismatch
blok 4   Position + kąt                                     -> TaskEquationMismatch
```

Blok 4 jest tym, którego brakowało najbardziej: bez niego profil `Position` z
`angle = 0` jest deklaracją w kodzie, której nikt nie sprawdza. Symetria z F2.6a
jest dosłowna — tam ten sam brak zamknęły `RejectsPositionEquationInDirectionTask`
i `RejectsDirectionEquationInPositionTask`.

**Rozszerzenie testu DOF nie jest kosmetyką.** Bez niego implementacja zwracająca
`6` dla `PositionAndDirectionAngle` przechodzi cały gate tego etapu — dokładnie ta
luka, którą review zamknęło w F2.6a dla czterech poprzednich wartości. Test
sprawdza teraz komplet sześciu plus `nullopt`, i jest jedynym miejscem, w którym
liczby z tabeli DOF są w ogóle weryfikowane.

Drugi test jest sprawdzianem pułapki z 5.1: przy przebiegach patrzących wyłącznie
na `DirectionEquationSource` przechodzi z **innym** kodem.

Razem **+14** (9 + 3 + 2), gate `410/410` (396 + 14) — osiągnięte.

---

## 8. Gate

```text
build (Release)        0 ostrzeżeń
build-werror           0 ostrzeżeń, asercje żywe
ctest                  410/410 w obu (396 + 14)  -- osiągnięte
                       baseline 396 to stan zmierzony po implementacji F2.6a,
                       zielony w obu konfiguracjach -- nie deklaracja
istniejące testy       zero zmienionych asercji -- F2.6b nic nie zaostrza,
                       tylko dokłada
zakres                 model + builder + trzy pliki testowe; zero zmian w
                       warstwie symbolicznej, loaderze, fasadzie i tests/support
model spójny           sześć rodzajów zadania, każdy z profilem, walidacją,
                       przeciążeniem buildera i testami -- etap F2.6 zamknięty
```

Wiersz drugi od dołu jest inny niż w F2.6a i to jest dobra wiadomość: tamta
połowa przebudowywała reguły modelu, ta wyłącznie z nich korzysta.

---

## 9. Ryzyka implementacyjne

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Przebiegi osi pomijają źródło kąta | **realne** | `toolAxisOf` jako jawny wizytor po wszystkich alternatywach (5.1) + `RejectsForgedToolAxisInAngleSource` |
| `reference` nieznormalizowany | **realne** | Skalowałby lewą stronę, czyli opisywał inny kąt (6); `NormalizesReferenceBeforeBuildingEquation` |
| Iloczyn skalarny policzony numerycznie | **realne** | `AngleLhsMatchesDirectionDotProduct` — równanie musi zależeć od `q`, a oczekiwanie powstaje tą samą fabryką (7.1) |
| `cos(alfa)` zamieniony na `alfa` po prawej | realne | `UsesCosineOfAngleOnRightHandSide` na kącie, dla którego obie wartości są wyraźnie różne |
| Końce przedziału dopuszczone | **realne** | Dwa kroki walidacji: matematyczny i arytmetyczny (4.2); test bierze `0` i `pi` wprost, a kąt zapadnięcia **wyszukuje** (7.2) |
| Próg degeneracji wpisany do testu jako stała | **realne, złapane w review v3** | Test szuka punktu zapadnięcia na bieżącym libm; `7.63e-9` jest obserwacją, nie kontraktem (7.2) |
| Zerowe składowe `n` znikają z równania | świadome | Wszystkie trzy iloczyny budowane zawsze; zwijanie ich byłoby ręcznym foldingiem `x * 0`, którego projekt nie robi (6) |
| Zerowa kolumna `angle` w profilu nieprzypięta | **realne, złapane w review v2** | `EnforcesDirectionAngleEquationProfile`, blok 4: `Position` z równaniem kąta musi odpaść (7.3) |
| `sameSource` obiecuje komórkę macierzy | **realne, złapane w review v2** | Trzy komentarze przechodzą na kanoniczny slot ograniczenia; algorytm bez zmian (5.5) |
| DOF `1` nieprawdziwe przy degeneracji | **realne, zamknięte konstrukcyjnie** | Przedział otwarty (4) jest warunkiem prawdziwości tabeli DOF (3) |
| Wartość DOF dla nowego rodzaju zadania nieprzypięta | **realne, złapane w review v1** | Rozszerzony `ReportsTaskSpaceConstrainedDegreesOfFreedom` — komplet sześciu wartości (7.3) |
| Brak przeciążenia `TargetVisitor` | **realne, złapane w review v1** | Dwa przeciążenia (4.3); bez nich build staje — co jest zamierzonym zachowaniem tego wizytora |
| `sourceOrderKey` ufa, że treść sprawdzona | **realne, złapane w review v1** | Walidacja osi także dla źródła kąta (5.3) |
| Test struktury `lhs` przeczy kontraktowi fabryki | **realne, złapane w review v1** | Oczekiwanie budowane tą samą fabryką + `structurallyEqual`; `reference` bez współczynników `0` i `1` (7.1) |

---

## 10. Rekomendacja

Zaimplementować po zatwierdzeniu. Po F2.6b **etap F2.6 jest zamknięty**, a
`ConstraintBuilder` obsługuje komplet sześciu rodzajów zadań: pozycję, pełną
pozę, kierunek osi narzędzia i kąt do kierunku odniesienia — dwa ostatnie z
pozycją i bez niej.

W Fazie 2 zostaje wtedy **wyłącznie F2.7**: integracja z fasadą
`IkEquationBuilder`, Definition of Done Fazy 2 i release `v0.2-ik-equations`.
F2.7 ma własny proposal, jeszcze nienapisany — to on, a nie ten dokument, jest
ostatnim proposalem tej fazy.

Otwarte pytania: brak.
