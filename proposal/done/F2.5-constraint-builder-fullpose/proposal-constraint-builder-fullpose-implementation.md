# Proposal: `ConstraintBuilder` — `FullPose` (F2.5), implementacja

**Etap:** F2.5 · **v3 APPROVED** · 2026-08-20 · część 2 z 2

> **v2, po review.** Trzy poprawki, wszystkie implementacyjne. (1) Dodany
> `UsesTargetRotationValuesOnRightHandSide` — bez niego implementacja stawiająca
> dziewięć zer po prawej stronie albo myląca `values[row][column]` z
> `values[column][row]` przechodziła cały zestaw (5.6). (2) Dodany
> `RejectsNonFiniteTargetRotation` — przypina kolejność „walidacja przed
> budowaniem stałych" dla nowej ścieżki (5.6). (3) `invalidTarget()` **nie
> zostaje bez zmian**: komunikat „position target rejected" był semantycznie zły
> dla błędu orientacji (3.2). Testów F2.5: **12**, gate **344/344**.
>
> **v3, po drugim review.** Dwie korekty komentarzy, które deklarowały silniejszy
> kontrakt, niż kod daje: „identical node for node" o równaniach pozycji (2, 8)
> oraz brakujący kwalifikator „rigid-FK transform" w komentarzu
> `IkEquationSystem` (4). Kod bez zmian.

Architektura: [`proposal-constraint-builder-fullpose-architecture.md`](proposal-constraint-builder-fullpose-architecture.md),
**v2 APPROVED** — wariant A, dziewięć równań komórkowych rotacji. Ten dokument
nie zmienia żadnej decyzji; podaje kod i nazywa jedną rzecz, która wyszła dopiero
przy jego pisaniu (2.3).

```text
PoseTarget{position, orientation}
        ↓  validate: skończoność, ortogonalność 1e-8, det +1
3 równania pozycji  (X, Y, Z)  +  9 równań orientacji (row-major)
        ↓  symbols(equations) ⊆ nazwy unknowns
IkEquationSystem(Pose, unknowns, 12 równań)
```

Zakres poza tym niezmieniony: **zero** nowych kodów błędu, zero zmian w modelu
danych, w warstwie symbolicznej, w loaderze i w fasadzie. Cel kwaternionowy ma
własny etap `F2.5Q`.

## 1. Refaktor, który robi miejsce dla drugiego przeciążenia

F2.4 miało jedną metodę robiącą pięć kroków. Dwa przeciążenia dzielą cztery z
nich, więc kod rozdziela się na trzy prywatne części i dwa cienkie wejścia:

```text
build(PositionTarget)   validate -> positionEquations                 -> assemble
build(PoseTarget)       validate -> positionEquations
                                 -> appendOrientationEquations        -> assemble

assemble()   unknowns -> sprawdzenie symboli -> IkEquationSystem::create
```

`assemble()` jest wspólnym kręgosłupem i to w nim mieszka gwarancja z uwagi 10.
Konsekwencja projektowa jest ważniejsza, niż wygląda: **sprawdzenie symboli
przenosi się z „trzech komórek transformacji" na „równania, które właśnie
zbudowano"**. Kontrakt brzmiał zawsze „każdy symbol **w równaniach** jest
zadeklarowaną niewiadomą" — teraz kod mówi dokładnie to samo, zamiast mówić o
komórkach, które akurat trafiają do równań.

To jest też jedyny sposób, żeby F2.6 nie musiało tego pisać trzeci raz:
constrainty kierunkowe będą miały po lewej stronie **wyrażenia policzone**
(iloczyny skalarne kolumn), a nie surowe komórki. Kolektor puszczony po
`equation.lhs` obejmie je bez żadnej zmiany.

## 2. `src/ik_equations/builders/ConstraintBuilder.hpp`

Zmiana względem F2.4: jedno przeciążenie publiczne, trzy metody prywatne, akapit
kontraktu. Reszta pliku bez zmian — poniżej tylko fragmenty, które się zmieniają.

```cpp
// (bez zmian: ConstraintBuilderErrorCode, ConstraintBuilderError)

// Turns "where the tool must be" into equations: the first component that
// formulates an IK problem rather than describing geometry.
//
//     T(0,3)(q) = target.x        position, both task kinds
//     T(1,3)(q) = target.y
//     T(2,3)(q) = target.z
//
//     T(i,j)(q) = R_target(i,j)   orientation, Pose only, all nine cells
//
// Nine cells for three degrees of freedom is deliberate. A fixed subset of
// three is not independent for every rotation -- for R = I the diagonal cells
// are stationary to first order -- and dropping the third column would move the
// argument that justifies it (both matrices are proper rotations) outside the
// system, which then stops being self-contained. Keeping all nine throws
// nothing away; choosing an independent subproblem belongs to the layer that
// knows the solution structure, in Phase 3.
//
// Consequently: THE EQUATION COUNT IS NOT THE CONSTRAINT COUNT, and
// IkEquationSystem carries no rank information. A target accepted by
// TargetValidation may also be only an approximation of SO(3) -- the tolerance
// is 1e-8 -- so nine exact equalities need not have an exact solution. Phase 3
// selects the independent subproblem and verifies the rest numerically.
//
// Equations are stored as lhs = rhs, never as lhs - rhs = 0: subtracting would
// destroy which side came from the robot and which from the task, and Equation
// says so explicitly. Normalisation belongs to the future simplifier.
//
// Two preconditions cannot be checked here, and pretending otherwise would be
// worse than saying it plainly:
//
//   * WHICH transform. (unchanged from F2.4)
//   * WHETHER the pair matches. (unchanged from F2.4)
//
// An unknown that does NOT appear in the equations is accepted, not an error.
class ConstraintBuilder
{
public:
    explicit ConstraintBuilder(ExpressionFactory factory = {});

    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    build(const SymbolicTransform& transform,
          const KinematicChain& chain,
          const PositionTarget& target) const;

    // Position AND orientation. Position equations preserve the same lhs nodes
    // and sources as the PositionTarget overload; rhs constants carry the same
    // values. Non-zero rhs constants are not required to share node identity --
    // constant() builds a fresh node per call, and only zero is shared. Pinned
    // by PositionEquationsMatchPositionOnlyBuild.
    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    build(const SymbolicTransform& transform,
          const KinematicChain& chain,
          const PoseTarget& target) const;

private:
    [[nodiscard]] std::vector<Equation>
    positionEquations(const SymbolicTransform& transform, const Vector3& position) const;

    void appendOrientationEquations(std::vector<Equation>& equations,
                                    const SymbolicTransform& transform,
                                    const RotationMatrix3& orientation) const;

    // The shared spine: unknowns, the symbol guarantee, and the model's own
    // validation. Both overloads reach the model only through here, which is
    // what makes "every symbol in the equations is a declared unknown" a
    // property of the class rather than of one function.
    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    assemble(const KinematicChain& chain,
             IkTaskKind taskKind,
             std::vector<Equation> equations) const;

    ExpressionFactory factory_;
};
```

Dochodzi `#include "ik_equations/model/Equation.hpp"` (typ `Equation` i
`IkTaskKind` w sygnaturach prywatnych) oraz `<vector>`. `Vector3` i
`RotationMatrix3` przyjeżdżają z `IkTarget.hpp`, już dołączonego.

## 3. `src/ik_equations/builders/ConstraintBuilder.cpp`

### 3.1 Część niezmieniona

`kTranslationColumn`, `collectSymbols`, `unknownsOf`, `undeclaredSymbol`,
`systemRejected` — **bez żadnej zmiany**. Kolektor jest tą samą funkcją; zmienia
się tylko to, po czym się go puszcza (1).

`invalidTarget` zmienia się w jednym znaku znaczenia — 3.2.

### 3.2 `invalidTarget()` — komunikat przestaje kłamać

> **Poprawione w v2, po review.** v1 pisała, że ten helper zostaje bez zmian.
> Nieprawda: w F2.4 był jedynym wywołującym `validate(PositionTarget)`, więc
> prefiks był trafny. Po F2.5 ten sam helper obsługuje `validate(PoseTarget)` i
> produkowałby zdania w rodzaju
> `position target rejected: orientation determinant differs from +1`.

```cpp
ConstraintBuilderError invalidTarget(TargetValidationError error)
{
    auto message = "target rejected: " + error.message;
    return ConstraintBuilderError{ConstraintBuilderErrorCode::InvalidTarget,
                                  std::move(message), std::move(error), std::nullopt};
}
```

Zmienia się **wyłącznie prefiks komunikatu**: kod błędu, zagnieżdżony
`TargetValidationError` i sygnatura zostają. Żaden istniejący test nie wymaga
edycji — `RejectsNonFinitePositionTarget` z F2.4 sprawdza `code` i zagnieżdżony
błąd, nie tekst. Nazwa helpera zostaje `invalidTarget`, bo odpowiada kodowi
`InvalidTarget`.

### 3.3 Nowe: źródło równania orientacji

```cpp
// OrientationEquationSource::create returns nullopt outside 0..2, and the loops
// below are literal 0..2 -- so a failure here is a programmer defect, not a
// domain error. Precedent: mapJointType in UrdfModelLoader.cpp, which throws
// std::logic_error for the same reason. logic_error does not derive from
// runtime_error, which is what the facade catches, so it reaches the caller
// instead of being reported as a bad URDF.
//
// Deliberately not `assert` + dereference: that is exactly the shape R4 removed
// from RobotModel, where an NDEBUG build turned a broken precondition into UB.
OrientationEquationSource orientationSource(std::size_t row, std::size_t column)
{
    const auto source = OrientationEquationSource::create(row, column);
    if (!source)
        throw std::logic_error("orientation cell indices must be 0..2");
    return *source;
}
```

### 3.4 Nowe: trzy prywatne metody

```cpp
std::vector<Equation>
ConstraintBuilder::positionEquations(const SymbolicTransform& transform,
                                     const Vector3& position) const
{
    // X, Y, Z in this order: IkEquationSystem promises position first in this
    // order and rejects UnorderedEquations.
    std::vector<Equation> equations;
    equations.reserve(3);
    equations.push_back(Equation{transform(0, kTranslationColumn),
                                 factory_.constant(position.x),
                                 PositionEquationSource{CartesianComponent::X}});
    equations.push_back(Equation{transform(1, kTranslationColumn),
                                 factory_.constant(position.y),
                                 PositionEquationSource{CartesianComponent::Y}});
    equations.push_back(Equation{transform(2, kTranslationColumn),
                                 factory_.constant(position.z),
                                 PositionEquationSource{CartesianComponent::Z}});
    return equations;
}

void ConstraintBuilder::appendOrientationEquations(std::vector<Equation>& equations,
                                                   const SymbolicTransform& transform,
                                                   const RotationMatrix3& orientation) const
{
    // Row-major, strictly increasing: the order IkEquationSystem enforces.
    equations.reserve(equations.size() + 9);
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            equations.push_back(Equation{transform(row, column),
                                         factory_.constant(orientation.values[row][column]),
                                         orientationSource(row, column)});
}

std::expected<IkEquationSystem, ConstraintBuilderError>
ConstraintBuilder::assemble(const KinematicChain& chain,
                            IkTaskKind taskKind,
                            std::vector<Equation> equations) const
{
    auto unknowns = unknownsOf(chain);

    // Walked over the equations themselves, not over the cells they came from:
    // the contract is about symbols in the equations. Both sides are walked --
    // today an rhs is always a constant, so it costs nothing, but a later stage
    // introducing a symbolic target parameter has to decide explicitly how it
    // relates to the unknowns instead of slipping past this check.
    std::unordered_set<const ExpressionNode*> visited;
    std::vector<std::string_view> symbols;
    for (const auto& equation : equations)
    {
        collectSymbols(equation.lhs, visited, symbols);
        collectSymbols(equation.rhs, visited, symbols);
    }

    for (const auto& symbol : symbols)
    {
        const auto declared = std::ranges::any_of(
            unknowns,
            [&symbol](const JointVariable& unknown) { return unknown.name == symbol; });
        if (!declared)
            return std::unexpected(undeclaredSymbol(symbol));
    }

    // Last, and after every check that reads `unknowns`: this moves it away.
    auto system = IkEquationSystem::create(taskKind, std::move(unknowns), std::move(equations));
    if (!system)
        return std::unexpected(systemRejected(std::move(system.error())));

    return std::move(*system);
}
```

### 3.5 Dwa publiczne wejścia

```cpp
std::expected<IkEquationSystem, ConstraintBuilderError>
ConstraintBuilder::build(const SymbolicTransform& transform,
                         const KinematicChain& chain,
                         const PositionTarget& target) const
{
    // First: the coordinates become constants below, and
    // ExpressionFactory::constant has a finite-value precondition.
    if (auto valid = validate(target); !valid)
        return std::unexpected(invalidTarget(valid.error()));

    return assemble(chain, IkTaskKind::Position, positionEquations(transform, target.position));
}

std::expected<IkEquationSystem, ConstraintBuilderError>
ConstraintBuilder::build(const SymbolicTransform& transform,
                         const KinematicChain& chain,
                         const PoseTarget& target) const
{
    // validate(PoseTarget) covers all four codes: non-finite position,
    // non-finite orientation, non-orthogonal orientation, wrong determinant.
    if (auto valid = validate(target); !valid)
        return std::unexpected(invalidTarget(valid.error()));

    auto equations = positionEquations(transform, target.position);
    appendOrientationEquations(equations, transform, target.orientation);

    return assemble(chain, IkTaskKind::Pose, std::move(equations));
}
```

Dochodzą include'y: `<stdexcept>` (dla `std::logic_error` z 3.3). `<optional>`
już jest przez nagłówek; `<algorithm>`, `<string_view>`, `<unordered_set>`,
`<variant>`, `<vector>` bez zmian.

### 3.6 Dlaczego `throw`, a nie `assert` — jedna rzecz warta powiedzenia

`OrientationEquationSource::create` zwraca `std::optional`, a pętle chodzą po
literalnych `0..2`. Kuszące jest `assert(source); return *source;` — i to jest
dokładnie kształt, który R4 usunęło z `RobotModel`: w buildzie z `NDEBUG`
asercja znika, a wyłuskanie pustego `optional` to UB. `std::logic_error` działa
w każdej konfiguracji i nie miesza się z modelem błędów domenowych, bo `build()`
nadal nie ma kodu na „niemożliwe" — bo to nie jest błąd domenowy.

Precedens w repo: `mapJointType` w `UrdfModelLoader.cpp` rzuca `std::logic_error`
w takiej samej sytuacji, z takim samym uzasadnieniem.

## 4. `src/ik_equations/model/IkEquationSystem.hpp` — sam komentarz

Zatwierdzone w architekturze §8 pkt 3: konsumentem kontraktu jest solver, który
czyta **system**, nie buildera. Akapit dopisany do komentarza klasy, **zero
zmian w kodzie**.

> **Doprecyzowane w v2, po review.** v1 pisała „a target accepted by
> TargetValidation…", co wiązało ogólny typ z jednym producentem — a `create()`
> jest publiczne i przyjmuje układy zbudowane inaczej. Nowa wersja mówi ogólnie
> o braku informacji o rzędzie, a `ConstraintBuilder` i `PoseTarget` wymienia
> jako **przykład**, nie jako założenie.
>
> **v3.** Doszedł kwalifikator **„from a rigid-FK transform"**, którego brakowało
> względem zatwierdzonej architektury. Bez niego zdanie o trzech stopniach
> swobody było fałszywe dla ręcznie złożonej macierzy: `build()` przyjmuje
> dowolną `SymbolicTransform` i nie sprawdza, czy blok 3×3 jest obrotem, więc
> dziewięć komórek może zależeć nawet od dziewięciu niezależnych symboli.

```cpp
// What the equation count does NOT mean: this type stores no rank or
// independence information, so equations may be redundant. For a Pose system
// built by ConstraintBuilder from a rigid-FK transform, nine orientation
// equations represent a target orientation whose geometric space has three
// rotational degrees of freedom -- the qualifier matters, because the builder
// accepts any SymbolicTransform and does not check that the 3x3 block is a
// rotation at all. And when the right-hand sides come from a PoseTarget, those
// values were accepted within TargetValidation's tolerance (1e-8), so the nine
// exact equalities need not have an exact solution even for a reachable pose.
// A consumer must therefore not read "twelve equations" as "twelve independent
// constraints": selecting an independent subproblem, and verifying the
// remaining equations numerically, belongs to the solver.
```

## 5. `tests/test_constraint_builder.cpp` — dwanaście testów

### 5.1 Pomocnicze, dochodzą do istniejących

```cpp
// Identity with the rotation block replaced, translation left at the identity's
// zeros. Cells are handed in row-major order, like everything else here.
SymbolicTransform transformWithRotation(const std::array<Expression, 9>& cells)
{
    auto transform = SymbolicTransform::identity();
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            transform(row, column) = cells[row * 3 + column];
    return transform;
}

ik::RotationMatrix3 identityRotation()
{
    return ik::RotationMatrix3{{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}}};
}

// A genuine rotation about Z, not a hand-rounded one: the values come from
// std::cos/std::sin, so the target is orthogonal to machine precision and the
// test says nothing about the tolerance question (that is the solver's, Phase 3).
ik::RotationMatrix3 rotationAboutZ(double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return ik::RotationMatrix3{{{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}}};
}
```

Dochodzą include'y: `<array>`, `<cmath>`.

### 5.2 Kształt układu

```cpp
TEST(ConstraintBuilderTest, BuildsFullPoseEquationSystem)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q2"), factory.constant(0.5));
    const ik::PoseTarget target{{1.0, 2.0, 3.0}, rotationAboutZ(0.7)};

    const auto system = builder.build(transform, chainDeclaring({"q1", "q2"}), target);

    ASSERT_TRUE(system.has_value()) << static_cast<int>(system.error().code);
    EXPECT_EQ(system->taskKind(), IkTaskKind::Pose);
    EXPECT_EQ(system->equations().size(), 12u);
}

TEST(ConstraintBuilderTest, IncludesPositionAndOrientationConstraints)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    const ik::PoseTarget target{{0.0, 0.0, 0.0}, identityRotation()};

    const auto system = builder.build(transform, chainDeclaring({"q1"}), target);
    ASSERT_TRUE(system.has_value());

    std::size_t positionCount = 0;
    std::size_t orientationCount = 0;
    for (const auto& equation : system->equations())
        (ik::kindOf(equation.source) == ik::EquationKind::Position ? positionCount
                                                                   : orientationCount) += 1;

    EXPECT_EQ(positionCount, 3u);
    EXPECT_EQ(orientationCount, 9u);
}

TEST(ConstraintBuilderTest, OrientationEquationsAreRowMajor)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    const ik::PoseTarget target{{0.0, 0.0, 0.0}, identityRotation()};

    const auto system = builder.build(transform, chainDeclaring({"q1"}), target);
    ASSERT_TRUE(system.has_value());

    for (std::size_t cell = 0; cell < 9; ++cell)
    {
        const auto& source = std::get<ik::OrientationEquationSource>(
            system->equations()[3 + cell].source);
        EXPECT_EQ(source.row(), cell / 3);
        EXPECT_EQ(source.column(), cell % 3);
    }
}
```

### 5.3 Skąd bierze się każda strona

```cpp
TEST(ConstraintBuilderTest, UsesRotationBlockCells)
{
    // Every orientation lhs is the very cell of the given transform -- the same
    // rigour as DoesNotSubtractTargetFromLhs, applied to the rotation block.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    std::array<Expression, 9> cells{};
    for (std::size_t cell = 0; cell < 9; ++cell)
        cells[cell] = factory.symbol("q1");   // nine distinct nodes, one name

    const auto transform = transformWithRotation(cells);
    const ik::PoseTarget target{{0.0, 0.0, 0.0}, identityRotation()};

    const auto system = builder.build(transform, chainDeclaring({"q1"}), target);
    ASSERT_TRUE(system.has_value());

    for (std::size_t cell = 0; cell < 9; ++cell)
        EXPECT_TRUE(sameNode(system->equations()[3 + cell].lhs, cells[cell]))
            << "orientation cell " << cell;
}

TEST(ConstraintBuilderTest, PositionEquationsMatchPositionOnlyBuild)
{
    // Same transform, same position: the three position equations must be the
    // same equations, not merely equivalent ones.
    //
    // lhs by node identity, source by ==, rhs BY VALUE: constant(1.5) builds a
    // fresh node on every call, and only zero reaches the shared node -- so a
    // sameNode expectation on rhs would pass by accident at best.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q2"), factory.constant(0.5));
    const auto chain = chainDeclaring({"q1", "q2"});
    const ik::Vector3 position{1.5, -2.5, 0.75};

    const auto positionOnly = builder.build(transform, chain, PositionTarget{position});
    const auto pose = builder.build(transform, chain,
                                    ik::PoseTarget{position, rotationAboutZ(0.7)});

    ASSERT_TRUE(positionOnly.has_value());
    ASSERT_TRUE(pose.has_value());

    for (std::size_t slot = 0; slot < 3; ++slot)
    {
        const auto& lean = positionOnly->equations()[slot];
        const auto& full = pose->equations()[slot];

        EXPECT_TRUE(sameNode(full.lhs, lean.lhs)) << "slot " << slot;
        EXPECT_TRUE(ik::sameSource(full.source, lean.source)) << "slot " << slot;
        EXPECT_DOUBLE_EQ(ik::constantValue(full.rhs), ik::constantValue(lean.rhs));
    }
}
```

### 5.4 Cele odrzucone

```cpp
TEST(ConstraintBuilderTest, RejectsNonOrthogonalTargetRotation)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    // Two columns at 45 degrees to each other: far outside the 1e-8 tolerance.
    const ik::RotationMatrix3 skewed{{{{1.0, 0.7071067811865476, 0.0},
                                       {0.0, 0.7071067811865476, 0.0},
                                       {0.0, 0.0, 1.0}}}};

    const auto system =
        builder.build(transform, chainDeclaring({"q1"}), ik::PoseTarget{{0.0, 0.0, 0.0}, skewed});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::InvalidTarget);
    ASSERT_TRUE(system.error().targetError.has_value());
    EXPECT_EQ(system.error().targetError->code,
              TargetValidationErrorCode::NonOrthogonalOrientation);
}

TEST(ConstraintBuilderTest, RejectsReflectionTargetRotation)
{
    // diag(1, 1, -1) is perfectly orthogonal, so it passes step 2 and fails on
    // the determinant. There is deliberately no "reflection" code: the same
    // check also rejects a uniform scaling, which is no reflection at all.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    const ik::RotationMatrix3 reflection{{{{1.0, 0.0, 0.0},
                                           {0.0, 1.0, 0.0},
                                           {0.0, 0.0, -1.0}}}};

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      ik::PoseTarget{{0.0, 0.0, 0.0}, reflection});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::InvalidTarget);
    ASSERT_TRUE(system.error().targetError.has_value());
    EXPECT_EQ(system.error().targetError->code,
              TargetValidationErrorCode::InvalidOrientationDeterminant);
}

TEST(ConstraintBuilderTest, RejectsUndeclaredSymbolInRotationBlock)
{
    // The symbol guarantee has to grow from 3 cells to 12. A collector still
    // walking only the translation column would pass every F2.4 test and every
    // PositionOnly build, and let q999 through here.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    std::array<Expression, 9> cells{};
    for (auto& cell : cells)
        cell = factory.constant(0.0);
    cells[4] = factory.symbol("q999");

    auto transform = transformWithRotation(cells);
    transform(0, 3) = factory.symbol("q1");

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      ik::PoseTarget{{0.0, 0.0, 0.0}, identityRotation()});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::UndeclaredSymbol);
    EXPECT_NE(system.error().message.find("q999"), std::string::npos);
}
```

### 5.5 End-to-end

```cpp
TEST(ConstraintBuilderTest, BuildsKr640FullPoseEquationSystem)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    const ConstraintBuilder builder;
    const ik::PoseTarget target{{2.0, 0.5, 1.5}, rotationAboutZ(0.7)};
    const auto system = builder.build(robot->fk, robot->chain, target);

    ASSERT_TRUE(system.has_value());
    EXPECT_EQ(system->taskKind(), IkTaskKind::Pose);
    EXPECT_EQ(system->equations().size(), 12u);
    EXPECT_EQ(system->unknowns().size(), 6u);
    EXPECT_TRUE(sameNode(system->equations()[3].lhs, robot->fk(0, 0)));
}

TEST(ConstraintBuilderTest, BuildsKr4FullPoseEquationSystem)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    const ConstraintBuilder builder;
    const ik::PoseTarget target{{0.4, 0.1, 0.6}, identityRotation()};
    const auto system = builder.build(robot->fk, robot->chain, target);

    ASSERT_TRUE(system.has_value());
    EXPECT_EQ(system->equations().size(), 12u);
    EXPECT_EQ(system->unknowns().size(), 6u);
}
```

Cele end-to-end **nie muszą być osiągalne** — ten etap formułuje układ, nie
rozwiązuje go i nie bada osiągalności. To jest ta sama decyzja co w F2.4.

### 5.6 Dwa testy dodane w v2

> **Po review.** Pierwszy zamyka lukę, przez którą przechodziłaby implementacja
> stawiająca po prawej stronie dziewięć zer; drugi przypina kolejność kroków dla
> nowej ścieżki.

```cpp
TEST(ConstraintBuilderTest, UsesTargetRotationValuesOnRightHandSide)
{
    // The heart of F2.5: R(q)(i,j) = R_target(i,j). Without this test an
    // implementation writing constant(0.0) nine times, or one indexing
    // values[column][row], passes every other test in this file.
    //
    // rotationAboutZ(0.7) is chosen for that second reason: R(0,1) = -sin and
    // R(1,0) = +sin differ in sign, so a transposed read cannot slip through.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    const ik::PoseTarget target{{0.0, 0.0, 0.0}, rotationAboutZ(0.7)};

    const auto system = builder.build(transform, chainDeclaring({"q1"}), target);
    ASSERT_TRUE(system.has_value());

    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
        {
            const auto& equation = system->equations()[3 + row * 3 + column];
            EXPECT_DOUBLE_EQ(ik::constantValue(equation.rhs),
                             target.orientation.values[row][column])
                << "cell (" << row << ", " << column << ")";
        }
}

TEST(ConstraintBuilderTest, RejectsNonFiniteTargetRotation)
{
    // The order "validate, then build constants" is load-bearing for the
    // rotation block exactly as it is for position: appendOrientationEquations
    // calls factory_.constant, whose precondition is a finite value. The other
    // two rejection tests use finite doubles, so only this one fails if the
    // steps are ever swapped -- and in build-werror, where assertions are live,
    // it fails by aborting rather than by returning an error.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));

    auto orientation = identityRotation();
    orientation.values[1][2] = std::numeric_limits<double>::infinity();

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      ik::PoseTarget{{0.0, 0.0, 0.0}, orientation});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::InvalidTarget);
    ASSERT_TRUE(system.error().targetError.has_value());
    EXPECT_EQ(system.error().targetError->code,
              TargetValidationErrorCode::NonFiniteOrientation);
}
```

`<limits>` jest już dołączone przez `RejectsNonFinitePositionTarget` z F2.4.

## 6. `STATUS.md`

```text
Pipeline progress   → ConstraintBuilder ✅ PositionOnly + FullPose (F2.4/F2.5)
Tests               332 -> 344
Done                sekcja ConstraintBuilder: akapit o dziewięciu równaniach
Known gaps          nowy wiersz: liczba równań != liczba więzów; cel przyjęty
                    z tolerancją 1e-8 może uczynić dziewięć dokładnych równości
                    algebraicznie niespełnialnymi -- Faza 3 wybiera podproblem
                    i weryfikuje resztę numerycznie
Working process     wiersz dla proposala implementacyjnego F2.5
```

## 7. Gate

```text
build (Release)        0 ostrzeżeń
build-werror           0 ostrzeżeń
ctest                  344/344 w obu (332 + 12)
istniejące testy       zero zmienionych asercji; 12 testów F2.4 przechodzi bez
                       edycji, co jest właściwym sprawdzeniem refaktoru z 1
zakres                 zmiany wyłącznie w ConstraintBuilder.{hpp,cpp},
                       test_constraint_builder.cpp, komentarzu
                       IkEquationSystem.hpp i STATUS.md
```

Wiersz drugi od dołu jest tu najważniejszy: F2.5 **przebudowuje** wnętrze
`build()` z F2.4, więc dwanaście istniejących testów jest testem regresji tego
refaktoru. Konieczność zmiany choćby jednego oznaczałaby, że przeciążenie
zmieniło zachowanie ścieżki pozycyjnej.

## 8. Ryzyka implementacyjne

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Refaktor zmienia zachowanie `PositionOnly` | **realne** | 12 testów F2.4 bez edycji + `PositionEquationsMatchPositionOnlyBuild`, który porównuje `lhs` po tożsamości węzła, `source` przez `sameSource`, a `rhs` po wartości — bo niezerowa stała to za każdym razem nowy węzeł |
| Kolektor puszczony dalej po `rhs` odrzuci przyszły symboliczny parametr celu | świadome | 3.3 — to jest cel, a nie skutek uboczny: taka zmiana ma wymusić decyzję, nie przemknąć |
| `throw` w ścieżce budowania | niskie | Nieosiągalne z literalnych pętli `0..2`; precedens `mapJointType`, uzasadnienie w 3.5 |
| Dziewięć równań mylnie liczone jako dziewięć więzów | **realne** | Kontrakt w trzech miejscach: komentarz `build()`, komentarz `IkEquationSystem`, wiersz w „Known gaps" |
| Prawa strona równań orientacji niesprawdzona | **było realne w v1** | `UsesTargetRotationValuesOnRightHandSide` (5.6) sprawdza wszystkie dziewięć wartości, na obrocie asymetrycznym, więc transpozycja też nie przejdzie |

## 9. Rozstrzygnięte w review (v2)

| # | Pytanie z v1 | Rozstrzygnięcie |
|---|---|---|
| 1 | Kolektor po `rhs` czy tylko po `lhs` | **Po obu.** Inwariant brzmi „każdy symbol w `Equation` musi być jawnie sklasyfikowany", a nie „każdy symbol po lewej". Symboliczny parametr celu ma zatrzymać build, a nie przemknąć jako coś nieopisanego |
| 2 | `std::logic_error` czy `assert` + wyłuskanie | **`logic_error`.** Precedens `mapJointType`; `assert` + wyłuskanie zamieniałoby w Release stan niemożliwy na potencjalne UB — kształt usunięty w R4 |
| 3 | `std::array<Expression, 9>` czy dziewięć argumentów | **Tablica.** Dziewięć argumentów tego samego typu to zaproszenie do przestawienia dwóch |

Otwarte pytania: brak.

## 10. Rekomendacja

Zaimplementować po zatwierdzeniu. Po F2.5 `ConstraintBuilder` obsługuje obie
rodziny celów z modelu F2.3, a Faza 2 ma do zamknięcia `F2.5Q` (target
kwaternionowy), `F2.6` (constrainty kierunkowe) i `F2.7` (fasada, DoD, release
`v0.2-ik-equations`).
