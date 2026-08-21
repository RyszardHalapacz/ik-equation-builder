# Proposal: integracja z fasadą (F2.7), implementacja

**Etap:** F2.7 · **v1 — do review** · 2026-08-21 · część 2 z 2

Architektura: [`proposal-phase2-facade-integration.md`](proposal-phase2-facade-integration.md)
— **v2, APPROVED** w review 2026-08-21.

Ostatni dokument implementacyjny Fazy 2. Po nim `IkEquationBuilder` przyjmuje
URDF, opcjonalny TCP i cel, a oddaje jawny układ równań IK.

## Prompt

> Napisać proposal implementacyjny F2.7 po zatwierdzeniu architektury.

## 0. Zakres

```text
IkEquationBuilder.hpp    +2 kody błędu, +1 pole, +1 metoda, +1 składowa
IkEquationBuilder.cpp    +buildEquationSystem, +wizytor wariantu, +2 helpery
tests/test_ik_equation_builder.cpp   +12 testów
STATUS.md, README.md, roadmapa       DoD Fazy 2 (5)

BEZ ZMIAN   ConstraintBuilder, cały model, warstwa symboliczna, loader,
            budowniczowie transformacji, tests/support
```

Ostatni wiersz jest sprawdzianem, nie deklaracją: fasada ma wyłącznie **składać**
to, co istnieje. Gdyby musiała cokolwiek z tego zmienić, znaczyłoby to, że któryś
z poprzednich etapów zostawił dziurę — i to byłaby wiadomość ważniejsza niż sam
etap.

Nowe include'y:

```text
IkEquationBuilder.hpp    "ik_equations/builders/ConstraintBuilder.hpp"
                         (IkTarget, IkEquationSystem i ConstraintBuilderError
                          przyjeżdżają przez niego, ale nagłówek deklaruje to,
                          czego używa w sygnaturach:)
                         "ik_equations/model/IkTarget.hpp"
                         "ik_equations/model/IkEquationSystem.hpp"
IkEquationBuilder.cpp    <utility>   // std::move na zagnieżdżonym błędzie
                         <variant>   // std::visit
```

---

## 1. `IkEquationBuilder.hpp`

### 1.1 Kody i pole błędu

```cpp
enum class IkEquationBuilderErrorCode
{
    RobotModelNotLoaded,
    KinematicChainNotSelected,
    UrdfLoadFailed,
    ChainBuildFailed,
    ForwardKinematicsNotBuilt,
    TcpNotSet,
    InvalidTcpTransform,
    TcpForwardKinematicsNotBuilt,   // nowe
    ConstraintBuildFailed           // nowe
};
```

Komentarz przy `IkEquationBuilderError` rośnie o zdanie symetryczne do
istniejącego:

```cpp
// Errors returned by IkEquationBuilder satisfy this invariant: chainError has a
// value if and only if code == ChainBuildFailed, and constraintError has a value
// if and only if code == ConstraintBuildFailed. [...]
struct IkEquationBuilderError
{
    IkEquationBuilderErrorCode code{};
    std::string message;
    std::optional<KinematicChainError> chainError;
    std::optional<ConstraintBuilderError> constraintError;   // nowe
};
```

Dwa pola `optional` obok siebie i **żadne nie jest wariantem** — świadomie, bo
istniejący komentarz mówi wprost, że gwarancja dotyczy tego, co ta klasa
produkuje, a nie tego, co struktura potrafi pomieścić. F2.7 nie zmienia tej
umowy, tylko dokłada do niej drugi wiersz.

### 1.2 Metoda

```cpp
    // URDF + TCP + target -> an explicit system of IK equations.
    //
    // WHICH FRAME OF THE ROBOT is constrained follows from this object's own
    // state, and this is the one place in the project where that ambiguity can
    // be closed rather than documented:
    //
    //     a TCP is set      -> the TCP frame is constrained, and
    //                          tcpForwardKinematics() is used
    //     no TCP is set     -> the chain-tip frame is constrained, and
    //                          forwardKinematics() is used
    //
    // What does NOT change either way: the target's own quantities are
    // expressed in the BASE frame, always. A PoseTarget is T_base_target, a
    // direction is a direction in the base frame, and the equations read
    // T(0,3)(q) = target.x with both sides in base coordinates. Setting a TCP
    // does not mean "re-express the target in tool coordinates" -- doing that
    // would produce a constraint wrong by the whole tool transform. It means
    // "constrain the tool frame instead of the chain tip".
    //
    // For a ToolDirectionTarget both frames appear at once and neither may be
    // confused for the other: the axis names an axis OF THE CONSTRAINED FRAME,
    // while the direction it must point along is given in the base frame.
    //
    // Having no TCP is not a missing prerequisite -- it is the other legal mode.
    // But a TCP that is SET while its transform has not been built is a missing
    // prerequisite, and reports one: falling back to the chain-tip transform
    // would return a system wrong by exactly the tool offset, which compiles,
    // runs and lies.
    //
    // Prerequisites are reported in the order they must be satisfied: chain,
    // then forward kinematics, then -- only if a TCP is set -- the TCP
    // transform.
    //
    // const, and returning by value rather than storing: a target is a question
    // asked of the model, not a part of it. The returned system is a snapshot;
    // it stays valid and unchanged no matter what happens to this object
    // afterwards, because nothing here owns it. Copying one duplicates
    // Expression handles, never trees.
    [[nodiscard]] std::expected<IkEquationSystem, IkEquationBuilderError>
    buildEquationSystem(const IkTarget& target) const;
```

Do składowych dochodzi `ConstraintBuilder constraintBuilder_;` obok czterech
istniejących budowniczych.

### 1.3 Czego **nie** dochodzi

Ani jednego węzła stanu, ani jednego akcesora, ani jednego wiersza w kontrakcie
nieświeżości wskaźników. Graf stanu z komentarza klasy zostaje dokładnie taki,
jaki jest — i to jest mierzalny skutek wariantu A, wart odnotowania w diffie.

---

## 2. `IkEquationBuilder.cpp`

### 2.1 Wybór transformacji

```cpp
// Which transform the constraints are built on, or the first missing
// prerequisite. Kept separate from the dispatch below so that "which frame do
// we constrain" is one decision in one place, testable through the errors it
// reports.
//
// Note what is NOT here: a fallback. If a TCP is set, the TCP transform is the
// only acceptable answer.
std::expected<const SymbolicTransform*, IkEquationBuilderError>
IkEquationBuilder::transformForConstraints() const
{
    if (!kinematicChain_)
        return makeError(IkEquationBuilderErrorCode::KinematicChainNotSelected,
                         "no kinematic chain selected; call selectChain first");

    if (!forwardKinematics_)
        return makeError(IkEquationBuilderErrorCode::ForwardKinematicsNotBuilt,
                         "forward kinematics not built; call buildForwardKinematics first");

    if (!tcp_)
        return &*forwardKinematics_;

    if (!tcpForwardKinematics_)
        return makeError(IkEquationBuilderErrorCode::TcpForwardKinematicsNotBuilt,
                         "a TCP is set but its transform is not built; call "
                         "buildTcpForwardKinematics first");

    return &*tcpForwardKinematics_;
}
```

Kolejność `chain -> FK -> TCP FK` jest ta sama, którą stosuje dziś
`buildTcpForwardKinematics`, i z tego samego powodu: caller ma usłyszeć **krok,
który ma zrobić następny**, a nie ostatni w sekwencji.

Trzeci `if` jest sednem etapu i wygląda niepozornie: **brak TCP zwraca
transformację końca łańcucha, a nie błąd.** Czwarty `if` jest jego lustrzanym
odbiciem: TCP jest, ale nie ma jego transformacji — i tu odwrotu nie ma.

### 2.2 Rozdział wariantu na przeciążenia buildera

```cpp
namespace {

// Explicit overload set, no generic lambda: a seventh IkTarget alternative must
// stop the build here until somebody decides how the facade builds it. The same
// mechanism that stopped it twice during F2.6.
struct ConstraintDispatch
{
    const ConstraintBuilder& builder;
    const SymbolicTransform& transform;
    const KinematicChain& chain;

    template <class Target>
    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    operator()(const Target& target) const = delete;

    // ... sześć jawnych przeciążeń, każde jedną linią:
    //     return builder.build(transform, chain, target);
};

} // namespace
```

> **Uwaga do review, 8.1:** zapis z `template ... = delete` jest jednym z dwóch
> rozważanych; drugi to sześć zwykłych przeciążeń bez szablonu. Oba dają błąd
> kompilacji przy siódmej alternatywie, ale w innym momencie i z innym
> komunikatem. Rekomendacja i uzasadnienie niżej.

### 2.3 Metoda

```cpp
std::expected<IkEquationSystem, IkEquationBuilderError>
IkEquationBuilder::buildEquationSystem(const IkTarget& target) const
{
    const auto transform = transformForConstraints();
    if (!transform)
        return std::unexpected(transform.error());

    auto system = std::visit(
        ConstraintDispatch{constraintBuilder_, **transform, *kinematicChain_}, target);

    if (!system)
        return constraintBuildFailed(std::move(system.error()));

    return std::move(*system);
}
```

Cztery kroki i żadnego stanu. `*kinematicChain_` jest bezpieczne, bo
`transformForConstraints` już sprawdziło łańcuch — i to jest jedyne miejsce, w
którym ta metoda polega na kolejności wewnątrz samej siebie.

Helper błędu, obok istniejącego `makeError`:

```cpp
// The nested error is preserved, not flattened: the caller gets
// ConstraintBuildFailed -> InvalidTarget -> NonFinitePosition, three layers
// deep, and none of them lost. Same treatment chainError has had since F2.3.
std::unexpected<IkEquationBuilderError>
constraintBuildFailed(ConstraintBuilderError error)
{
    auto message = "constraint build failed: " + error.message;
    return std::unexpected(IkEquationBuilderError{
        IkEquationBuilderErrorCode::ConstraintBuildFailed, std::move(message),
        std::nullopt, std::move(error)});
}
```

---

## 3. Testy — 12

### 3.1 Lista

```text
end-to-end
  BuildsKr4PositionIkEquationsFromUrdf       bez TCP, 3 równania, 6 niewiadomych
  BuildsKr640PositionIkEquationsFromUrdf     bez TCP
  BuildsFullPoseEquationsWithTcp             z TCP, 12 równań
  ReusesFacadeForDifferentRobotAndTarget     migawka przeżywa przeładowanie
  BuildsDirectionSystemThroughFacade         wariant IkTarget naprawdę przepuszczony

wybór transformacji
  UsesTcpTransformWhenTcpIsSet               lhs to sameNode z tcpForwardKinematics()
  UsesChainTipTransformWhenNoTcpIsSet        lhs to sameNode z forwardKinematics()
  RejectsTargetWhenTcpForwardKinematicsMissing
  ChangingTcpChangesTheEquationSystem        offset narzędzia widoczny w rhs

prerekwizyty i błędy
  ReportsFirstMissingStep                    trzy stany, trzy kody
  PreservesConstraintBuilderError            trzy warstwy + inwariant pól
                                             + nietknięte akcesory
  ChangingChainClearsTcpAndRequiresFreshForwardKinematics
```

### 3.2 Cztery testy, które robią prawdziwą robotę

**`UsesTcpTransformWhenTcpIsSet` i `UsesChainTipTransformWhenNoTcpIsSet`** są
jedynymi, które sprawdzają **którą transformację** wzięto, i robią to przez
tożsamość węzła, nie przez wartość:

```cpp
EXPECT_TRUE(sameNode(system->equations()[0].lhs, (*builder.tcpForwardKinematics())(0, 3)));
```

Porównanie wartości przepuściłoby implementację, która wzięła złą transformację w
konfiguracji, gdzie obie dają liczbowo to samo — a TCP o zerowym offsecie jest
dokładnie takim przypadkiem i łatwo go wpisać do testu przez pomyłkę.

**`RejectsTargetWhenTcpForwardKinematicsMissing`** pilnuje zakazu odwrotu.
Kształt: `selectChain`, `buildForwardKinematics`, `setTcp` — i **bez**
`buildTcpForwardKinematics`. Oczekiwanie: `TcpForwardKinematicsNotBuilt`. Test
padłby na implementacji, która „życzliwie" wzięłaby FK końca łańcucha, i to jest
jego jedyny cel.

**`ChangingTcpChangesTheEquationSystem`** pokazuje, że wybór transformacji
naprawdę coś znaczy liczbowo: ten sam cel, dwa TCP różniące się o 120 mm wzdłuż
Z, dwa układy — prawe strony identyczne (bo cel się nie zmienił), a **lewe strony
to inne węzły**, bo pochodzą z innej transformacji. Gdyby fasada ignorowała TCP,
oba układy byłyby tym samym.

**`PreservesConstraintBuilderError`** ma trzy bloki w jednym teście:

```text
blok 1   PositionTarget{NaN, 0, 0}
         -> ConstraintBuildFailed
            -> ConstraintBuilderErrorCode::InvalidTarget
               -> TargetValidationErrorCode::NonFinitePosition

blok 2   inwariant pól: constraintError.has_value() == true
                        chainError.has_value()      == false
         oraz dla błędu prerekwizytu (brak łańcucha):
                        constraintError.has_value() == false

blok 3   akcesory nietknięte po nieudanym wywołaniu -- te same wskaźniki
         co przed nim
```

Blok 3 mieści się tutaj naturalnie, bo metoda jest `const`: to samo nieudane
wywołanie, które bada zagnieżdżony błąd, służy do sprawdzenia, że nic się nie
ruszyło. Osobny test badałby to samo drugi raz.

### 3.3 `ReusesFacadeForDifferentRobotAndTarget` — test migawki

```text
1. kr640, PositionTarget            -> system A, ZACHOWANY
2. loadRobotModel(kr4_r600), selectChain, buildForwardKinematics
3. PositionTarget inny               -> system B
4. system A sprawdzany PONOWNIE      -> nienaruszony
```

Krok 4 jest powodem istnienia tego testu w tej formie. W wariancie stanowym
system A przestałby istnieć w kroku 3; tutaj żyje tak długo, jak caller go
trzyma, bo fasada nigdy go nie miała. To jedyne miejsce, w którym różnica między
migawką a akcesorem jest **widoczna z zewnątrz**.

Sprawdzane w kroku 4: liczba równań, prawe strony po wartości i `lhs` przez
`sameNode` — węzły wyrażeń żyją, bo `Expression` jest współdzielonym uchwytem, a
nie widokiem na fasadę.

---

## 4. `ChangingChainClearsTcpAndRequiresFreshForwardKinematics`

Osobno, bo to jedyny test opisujący **przejście stanu**, a nie pojedyncze
wywołanie:

```text
przygotowanie   chain + FK + TCP + TCP FK, buildEquationSystem przechodzi

selectChain(inny łańcuch)

    tcp()                   == nullptr
    tcpForwardKinematics()  == nullptr
    forwardKinematics()     == nullptr

    buildEquationSystem(t)  -> ForwardKinematicsNotBuilt

buildForwardKinematics()

    buildEquationSystem(t)  -> OK, ścieżką końca łańcucha
```

Ostatnie dwie linie są tu najważniejsze i to one były w architekturze v1 opisane
błędnie: **brak TCP nie jest brakiem prerekwizytu**. Po zmianie łańcucha fasada
wraca do trybu bez narzędzia i pracuje w nim normalnie; TCP wraca dopiero wtedy,
gdy caller jawnie o niego poprosi.

---

## 5. Dokumentacja i Definition of Done Fazy 2

```text
STATUS.md    pipeline -> facade integration ✅
             licznik 410 -> 422
             sekcja o buildEquationSystem: wybór frame'u, zakaz odwrotu,
             migawka zamiast stanu
             dwa wiersze znanych braków przeformułowane: fasada usuwa OKAZJĘ,
             nie brak
             DoD Fazy 2 + zdanie, że projekt formułuje IK, ale go nie rozwiązuje

README.md    przykład użycia kończy się układem równań, nie macierzą FK;
             "(next) an explicit system of IK equations" -> zrobione

roadmapa     F2.7 -> zaimplementowane, DoD Fazy 2, release v0.2-ik-equations;
             ZMIANA zapisana jawnie: wariant A zamiast kandydata z promptu,
             dwa testy end-to-end zastąpione (architektura, 8)
```

Pełniejszy zapis DoD, zgodnie z ustaleniem review:

```text
URDF + wybrany łańcuch + OPCJONALNY TCP + cel  ->  IkEquationSystem
```

Zdanie, które musi paść przy ogłaszaniu DoD, żeby nie obiecać za dużo: **projekt
umie formułować IK, ale nie umie go rozwiązywać.** Wyprowadzenie `q` to Faza 3.

---

## 6. Gate

```text
build (Release)        0 ostrzeżeń
build-werror           0 ostrzeżeń, asercje żywe
ctest                  422/422 w obu (410 + 12)
istniejące testy       zero zmienionych asercji -- F2.7 nic nie zaostrza
zakres                 dwa pliki fasady, jeden plik testowy, trzy dokumenty
diff kontrolny         git diff --stat nie może pokazać ConstraintBuilder,
                       modelu, warstwy symbolicznej, loadera ani tests/support
graf stanu             bez zmian: cztery akcesory, żadnego nowego węzła
```

Przedostatni wiersz jest sprawdzianem architektury: fasada **składa**, nie
poprawia. Ostatni jest sprawdzianem wariantu A.

---

## 7. Ryzyka implementacyjne

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Cichy odwrót do FK przy ustawionym TCP | **realne, główne ryzyko etapu** | Brak gałęzi fallbacku w `transformForConstraints` (2.1) + `RejectsTargetWhenTcpForwardKinematicsMissing` |
| Zła transformacja niewidoczna w teście | **realne** | Porównanie przez `sameNode`, nie przez wartość: TCP o zerowym offsecie daje te same liczby (3.2) |
| Siódma alternatywa `IkTarget` przemyka | realne | Jawny zestaw przeciążeń w `ConstraintDispatch` (2.2), otwarte pytanie 8.1 |
| Błąd buildera spłaszczony do stringa | niskie | `constraintBuildFailed` zachowuje trzy warstwy (2.3) |
| Inwariant pól błędu złamany | realne | `PreservesConstraintBuilderError`, blok 2 — obie strony, `true` i `false` |
| `*kinematicChain_` bez sprawdzenia | niskie | `transformForConstraints` sprawdza łańcuch pierwszy; zależność zapisana w komentarzu (2.3) |

---

## 8. Otwarte pytania do review

**8.1 Kształt `ConstraintDispatch`.** Wariant z `template ... = delete` daje błąd
przy **próbie użycia** brakującego przeciążenia i komunikat wskazujący na
usunięty szablon; sześć zwykłych przeciążeń daje błąd `std::visit` o niepełnym
zestawie, dłuższy i mniej czytelny, ale bez sztuczki z `= delete`.

Rekomendacja: **sześć zwykłych przeciążeń**, bez szablonu. Precedens jest w
projekcie i był stosowany trzy razy — `TargetVisitor`, `EquationKindVisitor`,
`toolAxisOf` — a spójność z nimi jest warta więcej niż nieco lepszy komunikat
kompilatora. Sztuczka, której nie ma nigdzie indziej, kosztuje więcej przy
czytaniu, niż daje przy jednym błędzie kompilacji na etap.

**8.2 Czy `transformForConstraints` ma być metodą prywatną, czy wolną funkcją w
anonimowej przestrzeni nazw?** Rekomendacja: **metoda prywatna** — czyta cztery
składowe fasady, więc jako wolna funkcja musiałaby dostać je wszystkie
argumentami i przestałaby być prostsza.

---

## 9. Rekomendacja

Zaimplementować po zatwierdzeniu. Po F2.7 **Faza 2 jest zamknięta**: URDF,
opcjonalny TCP i cel wchodzą jedną fasadą, a wychodzi z niej jawny układ równań
IK dla sześciu rodzajów zadań.

Potem: **Definition of Done Fazy 2**, release `v0.2-ik-equations` i Faza 3,
która zaczyna się od `F3.1` — semantyki rewrite engine'u.
