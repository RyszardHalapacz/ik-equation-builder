# Proposal: warstwa symboliczna — implementacja

## Prompt

> Po poprawieniu testu mnożenia przez macierz jednostkową proposal można zatwierdzić. [...] czyli teraz na podstawie tego dokumentu przygotujesz proposal implementacyjny

Realizacja architektury zatwierdzonej w `proposal-symbolic-layer-architecture.md`, po dwóch rundach review. Standardowy format repo: stan obecny + pełny kod zmian, do zatwierdzenia przed naniesieniem na źródła.

## Weryfikacja (v2 — po review)

### Sprostowanie do wersji poprzedniej

Pierwsza wersja tego dokumentu twierdziła: *„cały kod poniżej został skompilowany i uruchomiony [...] 44 sprawdzenia, wszystkie przechodzą"*. **To była nieprawda i review słusznie ją podważyło.**

Co faktycznie zaszło: uruchomiłem **prototyp** (ręcznie napisany `check.cpp` z 44 asercjami), a następnie napisałem do dokumentu **inny, większy zestaw plików testowych** i przypisałem mu wiarygodność prototypu. Testy z dokumentu nie były uruchomione ani razu. Różnica nie była kosmetyczna — prototyp sprawdzał `identity · identity` (same stałe, wszystko się zwija), a dokument zawierał wariant z symbolem `tx`, który **nie przechodzi**.

### Weryfikacja v2 — tym razem faktyczna

Żeby wykluczyć powtórzenie tego błędu, weryfikacja nie polega już na osobnym prototypie. Bloki kodu zostały **automatycznie wyekstrahowane z tego pliku `.md`** (parser wycina zawartość ogrodzeń ```` ```cpp ```` pod nagłówkami sekcji 1–5 i 9–11), zapisane do drzewa katalogów odwzorowującego `src/`, i zbudowane przeciw prawdziwemu GoogleTest z `build/` tego repo:

```
g++ -std=c++23 -Wall -Wextra -I<tree> -I<gtest/include> \
    tests/*.cpp ik_equations/symbolic/Expression.cpp \
    ik_equations/symbolic/ExpressionFactory.cpp libgtest.a libgtest_main.a
```

Wynik: **43/43 przechodzi**, zero ostrzeżeń — zarówno w buildzie diagnostycznym, jak i z `-DNDEBUG -O2` (asercje wyłączone; testy nie zależą od nich). Weryfikowany jest więc dosłownie ten tekst, który czytasz — nie jego przybliżenie. Liczniki potwierdzone przez `--gtest_list_tests`:

| Suite | Testów |
|---|---|
| `SymbolicExpressionTest` | 14 |
| `ExpressionFactoryTest` | 16 |
| `SymbolicMatrixTest` | 13 |
| **razem nowych** | **43** |

Zmierzone: `sizeof(Expression) == 16 B`, `sizeof(SymbolicMatrix<4,4>) == 256 B` (obecnie 40 B / 640 B).

Zweryfikowane zachowania nieoczywiste, wynikające wprost z decyzji z review:

| Zachowanie | Wynik |
|---|---|
| `constant(0.0)` współdzieli węzeł z `Expression{}` | ✓ |
| `multiply(x, constant(0))` **zostaje** węzłem `Multiply` | ✓ |
| `multiply(constant(3), constant(0))` zwija się do `Constant(0)` | ✓ |
| `identity()` — cała przekątna to jeden współdzielony węzeł | ✓ |
| stała `A · I` → dokładnie wartości `A` | ✓ |
| symboliczna `A · I` → `(a₀₀ + (a₀₁ · 0))`, **nie** `a₀₀` | ✓ |
| `4×4` transformacja: `(3,3)` = `Add(Multiply(0, tx), 1)`, **nie** `1` | ✓ |
| `sin(π)` nie jest kanonizowane do zera | ✓ |
| kolejność left-fold: `((a·e) + (b·h))` | ✓ |

## Stan obecny

### `src/ik_equations/symbolic/Expression.hpp`

```cpp
#pragma once

#include <memory>
#include <string>
#include <variant>

namespace kinemaforge::ik {

enum class ExpressionType
{
    Constant, Symbol, Add, Subtract, Multiply, Divide, Sin, Cos, Negate
};

class Expression;
using ExpressionPtr = std::shared_ptr<const Expression>;

struct ConstantExpression { double value{}; };
struct SymbolExpression   { std::string name; };
struct BinaryExpression   { ExpressionPtr left; ExpressionPtr right; };
struct UnaryExpression    { ExpressionPtr operand; };

struct AddExpression : BinaryExpression {};
struct SubtractExpression : BinaryExpression {};
struct MultiplyExpression : BinaryExpression {};
struct DivideExpression : BinaryExpression {};

struct SinExpression : UnaryExpression {};
struct CosExpression : UnaryExpression {};
struct NegateExpression : UnaryExpression {};

using ExpressionNode = std::variant<
    ConstantExpression, SymbolExpression, AddExpression, SubtractExpression,
    MultiplyExpression, DivideExpression, SinExpression, CosExpression, NegateExpression
>;

class Expression
{
public:
    // Defaults to the constant 0 — lets SymbolicMatrix default-construct
    // its cells without needing a placeholder "empty" expression state.
    Expression() : node_(ConstantExpression{0.0}) {}
    explicit Expression(ExpressionNode node);

    ExpressionType type() const;
    const ExpressionNode& node() const;

private:
    ExpressionNode node_;
};

} // namespace kinemaforge::ik
```

### `src/ik_equations/symbolic/ExpressionFactory.hpp`

```cpp
#pragma once

#include "ik_equations/symbolic/Expression.hpp"

#include <string>

namespace kinemaforge::ik {

class ExpressionFactory
{
public:
    ExpressionPtr constant(double value) const;
    ExpressionPtr symbol(const std::string& name) const;

    ExpressionPtr add(const ExpressionPtr& lhs, const ExpressionPtr& rhs) const;
    ExpressionPtr subtract(const ExpressionPtr& lhs, const ExpressionPtr& rhs) const;
    ExpressionPtr multiply(const ExpressionPtr& lhs, const ExpressionPtr& rhs) const;
    ExpressionPtr divide(const ExpressionPtr& lhs, const ExpressionPtr& rhs) const;

    ExpressionPtr sin(const ExpressionPtr& operand) const;
    ExpressionPtr cos(const ExpressionPtr& operand) const;
    ExpressionPtr negate(const ExpressionPtr& operand) const;
};

} // namespace kinemaforge::ik
```

### `src/ik_equations/symbolic/SymbolicMatrix.hpp`

```cpp
#pragma once

#include "ik_equations/symbolic/Expression.hpp"

#include <array>
#include <cstddef>

namespace kinemaforge::ik {

template <std::size_t Rows, std::size_t Columns>
class SymbolicMatrix
{
public:
    Expression& at(std::size_t row, std::size_t column);
    const Expression& at(std::size_t row, std::size_t column) const;

private:
    std::array<Expression, Rows * Columns> values_;
};

} // namespace kinemaforge::ik
```

### `src/ik_equations/symbolic/SymbolicTransform.hpp`

```cpp
#pragma once

#include "ik_equations/symbolic/SymbolicMatrix.hpp"

namespace kinemaforge::ik {

using SymbolicRotation  = SymbolicMatrix<3, 3>;
using SymbolicVector3   = SymbolicMatrix<3, 1>;
using SymbolicTransform = SymbolicMatrix<4, 4>;

} // namespace kinemaforge::ik
```

### `CMakeLists.txt` (fragment) i `tests/CMakeLists.txt` (fragment)

```cmake
add_library(kinemaforge_ik STATIC
    src/Kinematics.cpp
    src/kinematics/robot_model.cpp
    src/kinematics/robot_model_loader.cpp
    src/ik_equations/IkEquationBuilder.cpp
    src/ik_equations/UrdfModelLoader.cpp
    src/ik_equations/builders/KinematicChainBuilder.cpp
)
```

```cmake
add_executable(kinemaforge_tests
    test_kinematics.cpp
    test_urdf_model_loader.cpp
    test_kinematic_chain_builder.cpp
)
```

### Stan faktyczny (zweryfikowany linkerem)

Poza inline'owym konstruktorem domyślnym `Expression` **żadna** z powyższych deklaracji nie ma definicji: `SymbolicMatrix::at()`, wszystkie metody `ExpressionFactory`, `Expression(ExpressionNode)`, `Expression::type()`, `Expression::node()` kończą się `undefined reference`. Brak plików `.cpp` w `symbolic/`, brak testów warstwy.

## Co się zmienia

### 1. `src/ik_equations/symbolic/Expression.hpp` (nowa pełna treść)

```cpp
#pragma once

#include <memory>
#include <string>
#include <variant>

namespace kinemaforge::ik {

enum class ExpressionType
{
    Constant,
    Symbol,
    Add,
    Subtract,
    Multiply,
    Divide,
    Negate,
    Sin,
    Cos
};

// Declaration order below is load-bearing. ExpressionNode is forward
// declared so Expression can hold a shared_ptr to it while incomplete;
// the node structs then store Expression by value (complete by then);
// ExpressionNode is defined last. A `using ExpressionNode = variant<...>`
// alias cannot work here — an alias is not forward-declarable, so the
// Expression <-> ExpressionNode cycle would have nowhere to break.
struct ExpressionNode;

// Immutable value handle over a shared expression node. Copying an
// Expression copies one pointer — never the tree underneath it.
//
// The node is never null. Nothing can mutate a node once built, so the
// graph is acyclic by construction and sub-trees are safe to share.
class Expression
{
public:
    // The constant 0. Every cell of a fresh SymbolicMatrix starts here,
    // all sharing one node.
    Expression();

    ExpressionType type() const noexcept;
    const ExpressionNode& node() const noexcept;

private:
    // Private on purpose: every composite node goes through
    // ExpressionFactory, so the contracts it enforces (finite constants,
    // non-empty symbol names, no division by literal zero) hold for the
    // whole graph rather than only where callers remembered to use it.
    explicit Expression(ExpressionNode node);

    static const std::shared_ptr<const ExpressionNode>& sharedZeroNode();

    friend class ExpressionFactory;

    std::shared_ptr<const ExpressionNode> node_;
};

struct ConstantNode { double value{}; };
struct SymbolNode   { std::string name; };

struct AddNode      { Expression lhs, rhs; };
struct SubtractNode { Expression lhs, rhs; };
struct MultiplyNode { Expression lhs, rhs; };
struct DivideNode   { Expression lhs, rhs; };

struct NegateNode { Expression operand; };
struct SinNode    { Expression operand; };
struct CosNode    { Expression operand; };

struct ExpressionNode
{
    std::variant<
        ConstantNode,
        SymbolNode,
        AddNode,
        SubtractNode,
        MultiplyNode,
        DivideNode,
        NegateNode,
        SinNode,
        CosNode
    > value;
};

// --- predicates -----------------------------------------------------

bool isConstant(const Expression& expression) noexcept;
bool isZero(const Expression& expression) noexcept;
bool isOne(const Expression& expression) noexcept;

// Precondition: isConstant(expression).
double constantValue(const Expression& expression);

// --- comparison -----------------------------------------------------

// True when both handles refer to the very same shared node. O(1).
bool sameNode(const Expression& lhs, const Expression& rhs) noexcept;

// True when both trees have the same shape and the same leaf values.
//
// Structural, not algebraic and not numeric:
//   x + y            vs  y + x          -> false (equal algebraically)
//   constant(.1+.2)  vs  constant(.3)   -> false (close numerically)
//
// Short-circuits on sameNode first, so shared sub-trees cost O(1).
//
// Constants compare with exact ==. Non-finite constants are a
// precondition violation of ExpressionFactory::constant, so a correct
// program never has one here; should one reach this function anyway (an
// NDEBUG build of already-broken code), NaN compares unequal to itself
// except via the sameNode short-circuit.
bool structurallyEqual(const Expression& lhs, const Expression& rhs);

} // namespace kinemaforge::ik
```

**Uwaga o kolejności w enumie:** porządek `ExpressionType` został zmieniony względem obecnego (`Negate` przed `Sin`/`Cos`, zamiast na końcu), żeby odpowiadał kolejności alternatyw wariantu. Nie jest to jednak wymóg poprawności — `type()` używa `std::visit` z jawnym mapowaniem (§2), więc rozjechanie się obu kolejności niczego nie psuje. To wyłącznie kwestia czytelności.

### 2. `src/ik_equations/symbolic/Expression.cpp` (nowy plik)

```cpp
#include "ik_equations/symbolic/Expression.hpp"

#include <cassert>
#include <type_traits>
#include <utility>

namespace kinemaforge::ik {

namespace {

// Maps a node to its ExpressionType. Written out rather than casting
// variant::index(), so adding a node type without listing it here is a
// compile error instead of a silently wrong enum value.
struct TypeOfNode
{
    ExpressionType operator()(const ConstantNode&) const noexcept { return ExpressionType::Constant; }
    ExpressionType operator()(const SymbolNode&)   const noexcept { return ExpressionType::Symbol; }
    ExpressionType operator()(const AddNode&)      const noexcept { return ExpressionType::Add; }
    ExpressionType operator()(const SubtractNode&) const noexcept { return ExpressionType::Subtract; }
    ExpressionType operator()(const MultiplyNode&) const noexcept { return ExpressionType::Multiply; }
    ExpressionType operator()(const DivideNode&)   const noexcept { return ExpressionType::Divide; }
    ExpressionType operator()(const NegateNode&)   const noexcept { return ExpressionType::Negate; }
    ExpressionType operator()(const SinNode&)      const noexcept { return ExpressionType::Sin; }
    ExpressionType operator()(const CosNode&)      const noexcept { return ExpressionType::Cos; }
};

// Compares two nodes already known to be the same alternative.
struct StructuralComparator
{
    bool operator()(const ConstantNode& lhs, const ConstantNode& rhs) const
    {
        // Exact comparison on purpose: this is a structural predicate,
        // not a numeric one.
        return lhs.value == rhs.value;
    }

    bool operator()(const SymbolNode& lhs, const SymbolNode& rhs) const
    {
        return lhs.name == rhs.name;
    }

    template <typename Node>
        requires requires(const Node& n) { n.lhs; n.rhs; }
    bool operator()(const Node& lhs, const Node& rhs) const
    {
        return structurallyEqual(lhs.lhs, rhs.lhs) && structurallyEqual(lhs.rhs, rhs.rhs);
    }

    template <typename Node>
        requires requires(const Node& n) { n.operand; }
    bool operator()(const Node& lhs, const Node& rhs) const
    {
        return structurallyEqual(lhs.operand, rhs.operand);
    }
};

} // namespace

// std::make_shared is avoided on purpose in this file. Under GCC 13.1 on
// MinGW it emits a strong definition of std::type_info::operator==, which
// collides with the one in libstdc++.a as soon as -static-libstdc++ is on
// (this project needs that flag; see CMakeLists.txt). The plain
// shared_ptr(new T) form does not, at the cost of a second allocation for
// the control block — negligible here, since building an expression tree
// happens once per robot, not in a hot loop.
const std::shared_ptr<const ExpressionNode>& Expression::sharedZeroNode()
{
    static const std::shared_ptr<const ExpressionNode> zero{
        new ExpressionNode{ConstantNode{0.0}}};
    return zero;
}

Expression::Expression() : node_(sharedZeroNode()) {}

Expression::Expression(ExpressionNode node)
    : node_(new ExpressionNode{std::move(node)})
{
}

ExpressionType Expression::type() const noexcept
{
    // The variant is set once at construction and never reassigned, so it
    // cannot become valueless and std::visit cannot throw here.
    return std::visit(TypeOfNode{}, node_->value);
}

const ExpressionNode& Expression::node() const noexcept
{
    return *node_;
}

bool isConstant(const Expression& expression) noexcept
{
    return std::holds_alternative<ConstantNode>(expression.node().value);
}

bool isZero(const Expression& expression) noexcept
{
    const auto* constant = std::get_if<ConstantNode>(&expression.node().value);
    return constant != nullptr && constant->value == 0.0;
}

bool isOne(const Expression& expression) noexcept
{
    const auto* constant = std::get_if<ConstantNode>(&expression.node().value);
    return constant != nullptr && constant->value == 1.0;
}

double constantValue(const Expression& expression)
{
    const auto* constant = std::get_if<ConstantNode>(&expression.node().value);
    assert(constant != nullptr && "constantValue() requires a Constant expression");
    return constant->value;
}

bool sameNode(const Expression& lhs, const Expression& rhs) noexcept
{
    return &lhs.node() == &rhs.node();
}

bool structurallyEqual(const Expression& lhs, const Expression& rhs)
{
    if (sameNode(lhs, rhs))
        return true;
    if (lhs.type() != rhs.type())
        return false;

    // Single visit: the alternatives already match, so the right-hand
    // node can be fetched directly. Visiting both variants would
    // instantiate 9x9 combinations, all but nine of them dead.
    return std::visit(
        [&rhs](const auto& left) {
            using Node = std::decay_t<decltype(left)>;
            return StructuralComparator{}(left, std::get<Node>(rhs.node().value));
        },
        lhs.node().value);
}

} // namespace kinemaforge::ik
```

`sameNode` porównuje adresy wskazywanych węzłów (`&lhs.node() == &rhs.node()`), więc **nie potrzebuje deklaracji `friend`** — publiczne `node()` wystarcza.

### 3. `src/ik_equations/symbolic/ExpressionFactory.hpp` (nowa pełna treść)

```cpp
#pragma once

#include "ik_equations/symbolic/Expression.hpp"

#include <string>

namespace kinemaforge::ik {

// The only way to build a composite expression: Expression's node
// constructor is private and befriends this class.
//
// Applies two kinds of local, O(1) normalization while building:
//   * constant folding — sin(c), c1 + c2, ... collapse to a constant
//   * neutral elements — x + 0 -> x, x * 1 -> x, x / 1 -> x, ...
//
// Both look at the immediate operands only. Anything needing traversal,
// operand reordering or equivalence checking belongs to a future
// EquationSimplifier, not here.
//
// Deliberately NOT applied: x * 0 -> 0. It would erase domain
// information — (1/q) * 0 is undefined at q = 0, but the folded 0 claims
// it is defined everywhere. That matters once IK equations start
// carrying variable denominators whose roots mark singularities. Folding
// two known finite constants (c * 0 -> 0) stays, since nothing is lost.
class ExpressionFactory
{
public:
    // Precondition: value is finite. NaN and +/-Inf have no legitimate
    // use here, so passing one is a caller bug, caught by assert in
    // diagnostic builds. Folded results route back through this method,
    // so an overflow in constant folding trips the same assert instead
    // of quietly producing Inf.
    //
    // This is a precondition, not a runtime guarantee: an NDEBUG build
    // will happily store whatever it is given.
    //
    // Returns the shared zero node for 0.0.
    Expression constant(double value) const;

    Expression symbol(std::string name) const;

    Expression add(Expression lhs, Expression rhs) const;
    Expression subtract(Expression lhs, Expression rhs) const;
    Expression multiply(Expression lhs, Expression rhs) const;
    Expression divide(Expression lhs, Expression rhs) const;

    Expression negate(Expression operand) const;
    Expression sin(Expression operand) const;
    Expression cos(Expression operand) const;
};

} // namespace kinemaforge::ik
```

### 4. `src/ik_equations/symbolic/ExpressionFactory.cpp` (nowy plik)

```cpp
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <cassert>
#include <cmath>
#include <utility>

namespace kinemaforge::ik {

Expression ExpressionFactory::constant(double value) const
{
    assert(std::isfinite(value) && "constants must be finite");
    if (value == 0.0)
        return Expression{}; // shared zero node
    return Expression{ExpressionNode{ConstantNode{value}}};
}

Expression ExpressionFactory::symbol(std::string name) const
{
    assert(!name.empty() && "symbol names must not be empty");
    return Expression{ExpressionNode{SymbolNode{std::move(name)}}};
}

Expression ExpressionFactory::add(Expression lhs, Expression rhs) const
{
    if (isConstant(lhs) && isConstant(rhs))
        return constant(constantValue(lhs) + constantValue(rhs));
    if (isZero(rhs))
        return lhs;
    if (isZero(lhs))
        return rhs;
    return Expression{ExpressionNode{AddNode{std::move(lhs), std::move(rhs)}}};
}

Expression ExpressionFactory::subtract(Expression lhs, Expression rhs) const
{
    if (isConstant(lhs) && isConstant(rhs))
        return constant(constantValue(lhs) - constantValue(rhs));
    if (isZero(rhs))
        return lhs;
    return Expression{ExpressionNode{SubtractNode{std::move(lhs), std::move(rhs)}}};
}

Expression ExpressionFactory::multiply(Expression lhs, Expression rhs) const
{
    if (isConstant(lhs) && isConstant(rhs))
        return constant(constantValue(lhs) * constantValue(rhs));
    // No x * 0 -> 0 here: see the class comment.
    if (isOne(rhs))
        return lhs;
    if (isOne(lhs))
        return rhs;
    return Expression{ExpressionNode{MultiplyNode{std::move(lhs), std::move(rhs)}}};
}

Expression ExpressionFactory::divide(Expression lhs, Expression rhs) const
{
    assert(!isZero(rhs) && "division by literal zero");
    if (isConstant(lhs) && isConstant(rhs))
        return constant(constantValue(lhs) / constantValue(rhs));
    if (isOne(rhs))
        return lhs;
    return Expression{ExpressionNode{DivideNode{std::move(lhs), std::move(rhs)}}};
}

Expression ExpressionFactory::negate(Expression operand) const
{
    if (isConstant(operand))
        return constant(-constantValue(operand));
    return Expression{ExpressionNode{NegateNode{std::move(operand)}}};
}

Expression ExpressionFactory::sin(Expression operand) const
{
    if (isConstant(operand))
        return constant(std::sin(constantValue(operand)));
    return Expression{ExpressionNode{SinNode{std::move(operand)}}};
}

Expression ExpressionFactory::cos(Expression operand) const
{
    if (isConstant(operand))
        return constant(std::cos(constantValue(operand)));
    return Expression{ExpressionNode{CosNode{std::move(operand)}}};
}

} // namespace kinemaforge::ik
```

**Uwaga o `subtract(x, 0)`:** brak symetrycznej reguły `subtract(0, x) → negate(x)`, bo to **przekształcenie**, nie usunięcie elementu neutralnego — należy do simplifiera (§11.6 proposalu architektonicznego).

**Uwaga o `divide`:** `assert(!isZero(rhs))` łapie wyłącznie **literalne** zero w mianowniku. Mianownik będący wyrażeniem, które zeruje się dla pewnych wartości zmiennych, jest poza zasięgiem lokalnego sprawdzenia — i to jest właśnie ta informacja, którą chroni odrzucenie anihilatora.

### 5. `src/ik_equations/symbolic/SymbolicMatrix.hpp` (nowa pełna treść)

```cpp
#pragma once

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <array>
#include <cassert>
#include <cstddef>

namespace kinemaforge::ik {

// Fixed-size matrix of symbolic expressions. Cells default to the
// constant 0, all sharing one node, so only the very first matrix in the
// program pays an allocation for its zeros.
//
// Access is operator(), not at(): the bounds check is an assert that
// disappears in release, so borrowing at()'s checked-access connotation
// from the standard library would promise a guarantee this does not give.
//
// Deliberately not a linear algebra library: it carries only what
// composing homogeneous transforms needs.
template <std::size_t Rows, std::size_t Columns>
class SymbolicMatrix
{
    static_assert(Rows > 0, "SymbolicMatrix requires a non-zero row count");
    static_assert(Columns > 0, "SymbolicMatrix requires a non-zero column count");

public:
    static constexpr std::size_t rows = Rows;
    static constexpr std::size_t columns = Columns;

    static SymbolicMatrix zeros() { return SymbolicMatrix{}; }

    static SymbolicMatrix identity()
        requires (Rows == Columns)
    {
        SymbolicMatrix result;
        const ExpressionFactory factory;
        const Expression one = factory.constant(1.0); // built once, shared
        for (std::size_t i = 0; i < Rows; ++i)
            result(i, i) = one;
        return result;
    }

    Expression& operator()(std::size_t row, std::size_t column)
    {
        assert(row < Rows && column < Columns);
        return values_[row * Columns + column];
    }

    const Expression& operator()(std::size_t row, std::size_t column) const
    {
        assert(row < Rows && column < Columns);
        return values_[row * Columns + column];
    }

private:
    std::array<Expression, Rows * Columns> values_;
};

// C(i,j) = ((A(i,0)*B(0,j) + A(i,1)*B(1,j)) + ...), left-folded over
// ascending k, so the produced tree shape is deterministic.
//
// Takes the factory explicitly rather than defining operator*: the cost
// stays visible at the call site (one 4x4 product builds up to 112
// nodes), and a factory that later gains configuration can be passed in
// without touching callers.
//
// Note that A * identity() does NOT give back A's cells for symbolic
// input: x * 0 is deliberately kept (see ExpressionFactory), so the
// off-diagonal products survive as Multiply nodes.
template <std::size_t R, std::size_t K, std::size_t C>
SymbolicMatrix<R, C> multiply(
    const SymbolicMatrix<R, K>& lhs,
    const SymbolicMatrix<K, C>& rhs,
    const ExpressionFactory& factory)
{
    SymbolicMatrix<R, C> result;
    for (std::size_t i = 0; i < R; ++i)
    {
        for (std::size_t j = 0; j < C; ++j)
        {
            Expression sum = factory.multiply(lhs(i, 0), rhs(0, j));
            for (std::size_t k = 1; k < K; ++k)
                sum = factory.add(sum, factory.multiply(lhs(i, k), rhs(k, j)));
            result(i, j) = sum;
        }
    }
    return result;
}

} // namespace kinemaforge::ik
```

### 6. `src/ik_equations/symbolic/SymbolicTransform.hpp` — **bez zmian**

Alias pozostaje w obecnej postaci.

### 7. `CMakeLists.txt` (nowa pełna treść)

```cmake
cmake_minimum_required(VERSION 3.21)

project(KinemaForge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (MINGW)
    # Statically link the MinGW runtime so binaries don't depend on
    # libstdc++-6.dll / libgcc_s_seh-1.dll / libwinpthread-1.dll being
    # found on PATH (other MinGW installs, e.g. Anaconda, can shadow
    # these with incompatible versions and crash with 0xc0000139).
    add_link_options(-static-libgcc -static-libstdc++ -static)
endif()

include(FetchContent)

# --- pugixml (URDF parsing) ---
FetchContent_Declare(
    pugixml
    GIT_REPOSITORY https://github.com/zeux/pugixml.git
    GIT_TAG        v1.14
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(pugixml)

add_library(kinemaforge_ik STATIC
    src/Kinematics.cpp
    src/kinematics/robot_model.cpp
    src/kinematics/robot_model_loader.cpp
    src/ik_equations/IkEquationBuilder.cpp
    src/ik_equations/UrdfModelLoader.cpp
    src/ik_equations/builders/KinematicChainBuilder.cpp
    src/ik_equations/symbolic/Expression.cpp
    src/ik_equations/symbolic/ExpressionFactory.cpp
)
target_include_directories(kinemaforge_ik PUBLIC src)
target_link_libraries(kinemaforge_ik PUBLIC pugixml::pugixml)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE kinemaforge_ik)

enable_testing()
add_subdirectory(tests)
```

(dwie dodane linie: `Expression.cpp`, `ExpressionFactory.cpp`)

### 8. `tests/CMakeLists.txt` (nowa pełna treść)

```cmake
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)
# Windows: use the same dynamic CRT as the rest of the project
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(kinemaforge_tests
    test_kinematics.cpp
    test_urdf_model_loader.cpp
    test_kinematic_chain_builder.cpp
    test_symbolic_expression.cpp
    test_expression_factory.cpp
    test_symbolic_matrix.cpp
)
target_link_libraries(kinemaforge_tests PRIVATE kinemaforge_ik GTest::gtest_main)
target_compile_definitions(kinemaforge_tests PRIVATE
    KINEMAFORGE_URDF_DATA_DIR="${CMAKE_SOURCE_DIR}/data/urdf"
)

include(GoogleTest)
gtest_discover_tests(kinemaforge_tests)
```

(trzy dodane linie)

### 9. `tests/test_symbolic_expression.cpp` (nowy plik)

```cpp
#include <gtest/gtest.h>

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <variant>

using kinemaforge::ik::AddNode;
using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::ExpressionType;
using kinemaforge::ik::MultiplyNode;
using kinemaforge::ik::SinNode;
using kinemaforge::ik::SymbolNode;
using kinemaforge::ik::constantValue;
using kinemaforge::ik::isConstant;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::sameNode;
using kinemaforge::ik::structurallyEqual;

TEST(SymbolicExpressionTest, DefaultConstructedIsConstantZero)
{
    const Expression expression;
    EXPECT_EQ(expression.type(), ExpressionType::Constant);
    EXPECT_TRUE(isConstant(expression));
    EXPECT_TRUE(isZero(expression));
    EXPECT_DOUBLE_EQ(constantValue(expression), 0.0);
}

TEST(SymbolicExpressionTest, DefaultConstructedInstancesShareOneNode)
{
    EXPECT_TRUE(sameNode(Expression{}, Expression{}));
}

TEST(SymbolicExpressionTest, CreatesConstant)
{
    const ExpressionFactory factory;
    const auto expression = factory.constant(2.5);

    EXPECT_EQ(expression.type(), ExpressionType::Constant);
    EXPECT_DOUBLE_EQ(constantValue(expression), 2.5);
    EXPECT_FALSE(isZero(expression));
    EXPECT_FALSE(isOne(expression));
}

TEST(SymbolicExpressionTest, CreatesSymbol)
{
    const ExpressionFactory factory;
    const auto expression = factory.symbol("q1");

    EXPECT_EQ(expression.type(), ExpressionType::Symbol);
    EXPECT_FALSE(isConstant(expression));
    EXPECT_EQ(std::get<SymbolNode>(expression.node().value).name, "q1");
}

TEST(SymbolicExpressionTest, CopyingSharesNodeInsteadOfCloning)
{
    const ExpressionFactory factory;
    const auto original = factory.symbol("q1");
    const auto copy = original;

    EXPECT_TRUE(sameNode(original, copy));
}

TEST(SymbolicExpressionTest, CopiedExpressionOutlivesSource)
{
    const ExpressionFactory factory;

    const auto outer = [&factory] {
        const auto inner = factory.symbol("q9");
        return factory.sin(inner); // inner goes out of scope here
    }();

    ASSERT_EQ(outer.type(), ExpressionType::Sin);
    const auto& operand = std::get<SinNode>(outer.node().value).operand;
    EXPECT_EQ(std::get<SymbolNode>(operand.node().value).name, "q9");
}

TEST(SymbolicExpressionTest, BinaryNodeExposesBothOperands)
{
    const ExpressionFactory factory;
    const auto lhs = factory.symbol("x");
    const auto rhs = factory.symbol("y");
    const auto sum = factory.add(lhs, rhs);

    ASSERT_EQ(sum.type(), ExpressionType::Add);
    const auto& node = std::get<AddNode>(sum.node().value);
    EXPECT_TRUE(sameNode(node.lhs, lhs));
    EXPECT_TRUE(sameNode(node.rhs, rhs));
}

TEST(SymbolicExpressionTest, UnaryNodeExposesOperand)
{
    const ExpressionFactory factory;
    const auto operand = factory.symbol("q1");
    const auto sine = factory.sin(operand);

    ASSERT_EQ(sine.type(), ExpressionType::Sin);
    EXPECT_TRUE(sameNode(std::get<SinNode>(sine.node().value).operand, operand));
}

TEST(SymbolicExpressionTest, RepeatedOperandIsSharedNotDuplicated)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");
    const auto product = factory.multiply(x, x);

    ASSERT_EQ(product.type(), ExpressionType::Multiply);
    const auto& node = std::get<MultiplyNode>(product.node().value);
    EXPECT_TRUE(sameNode(node.lhs, node.rhs));
}

TEST(SymbolicExpressionTest, StructurallyEqualMatchesIdenticalTrees)
{
    const ExpressionFactory factory;
    const auto left = factory.add(factory.symbol("x"), factory.symbol("y"));
    const auto right = factory.add(factory.symbol("x"), factory.symbol("y"));

    EXPECT_TRUE(structurallyEqual(left, right));
    EXPECT_FALSE(sameNode(left, right)); // independently built
}

TEST(SymbolicExpressionTest, StructurallyEqualRejectsDifferentShape)
{
    const ExpressionFactory factory;
    const auto sum = factory.add(factory.symbol("x"), factory.symbol("y"));
    const auto product = factory.multiply(factory.symbol("x"), factory.symbol("y"));

    EXPECT_FALSE(structurallyEqual(sum, product));
}

TEST(SymbolicExpressionTest, StructurallyEqualIsNotAlgebraic)
{
    const ExpressionFactory factory;
    const auto left = factory.add(factory.symbol("x"), factory.symbol("y"));
    const auto swapped = factory.add(factory.symbol("y"), factory.symbol("x"));

    // Equal as mathematics, different as trees. This is the documented
    // limit of the predicate, not a defect.
    EXPECT_FALSE(structurallyEqual(left, swapped));
}

TEST(SymbolicExpressionTest, StructurallyEqualIsNotNumeric)
{
    const ExpressionFactory factory;
    EXPECT_FALSE(structurallyEqual(factory.constant(0.1 + 0.2), factory.constant(0.3)));
}

TEST(SymbolicExpressionTest, SameNodeIsStrongerThanStructuralEquality)
{
    const ExpressionFactory factory;
    const auto left = factory.constant(1.0);
    const auto right = factory.constant(1.0);

    EXPECT_TRUE(structurallyEqual(left, right));
    EXPECT_FALSE(sameNode(left, right)); // no interning for non-zero constants
}
```

### 10. `tests/test_expression_factory.cpp` (nowy plik)

```cpp
#include <gtest/gtest.h>

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <cmath>
#include <numbers>

using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::ExpressionType;
using kinemaforge::ik::constantValue;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::sameNode;
using kinemaforge::ik::structurallyEqual;

TEST(ExpressionFactoryTest, BuildsBinaryOperations)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");
    const auto y = factory.symbol("y");

    EXPECT_EQ(factory.add(x, y).type(), ExpressionType::Add);
    EXPECT_EQ(factory.subtract(x, y).type(), ExpressionType::Subtract);
    EXPECT_EQ(factory.multiply(x, y).type(), ExpressionType::Multiply);
    EXPECT_EQ(factory.divide(x, y).type(), ExpressionType::Divide);
}

TEST(ExpressionFactoryTest, BuildsUnaryOperations)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    EXPECT_EQ(factory.negate(x).type(), ExpressionType::Negate);
    EXPECT_EQ(factory.sin(x).type(), ExpressionType::Sin);
    EXPECT_EQ(factory.cos(x).type(), ExpressionType::Cos);
}

TEST(ExpressionFactoryTest, FoldsConstantArithmetic)
{
    const ExpressionFactory factory;

    EXPECT_DOUBLE_EQ(constantValue(factory.add(factory.constant(2.0), factory.constant(3.0))), 5.0);
    EXPECT_DOUBLE_EQ(constantValue(factory.subtract(factory.constant(5.0), factory.constant(3.0))), 2.0);
    EXPECT_DOUBLE_EQ(constantValue(factory.multiply(factory.constant(3.0), factory.constant(4.0))), 12.0);
    EXPECT_DOUBLE_EQ(constantValue(factory.divide(factory.constant(6.0), factory.constant(3.0))), 2.0);
}

TEST(ExpressionFactoryTest, FoldsNegateOfConstant)
{
    const ExpressionFactory factory;
    EXPECT_DOUBLE_EQ(constantValue(factory.negate(factory.constant(2.0))), -2.0);
}

TEST(ExpressionFactoryTest, FoldsConstantTrig)
{
    const ExpressionFactory factory;

    EXPECT_TRUE(isZero(factory.sin(factory.constant(0.0))));
    EXPECT_TRUE(isOne(factory.cos(factory.constant(0.0))));
}

TEST(ExpressionFactoryTest, DoesNotSnapTinyConstantsToZero)
{
    const ExpressionFactory factory;
    const auto tiny = factory.constant(1e-300);

    // No tolerance anywhere in the factory: a tiny constant stays itself.
    // This is the canonicalization contract stated without involving libm
    // at all.
    EXPECT_FALSE(isZero(tiny));
    EXPECT_DOUBLE_EQ(constantValue(tiny), 1e-300);
}

TEST(ExpressionFactoryTest, TrigFoldingMatchesStdSinExactly)
{
    const ExpressionFactory factory;
    const auto sinePi = factory.sin(factory.constant(std::numbers::pi));

    // Folding must hand back exactly what std::sin produced, with no
    // canonicalization of near-zero results — a tolerance would become
    // part of the symbolic semantics, which is a separate, deferred
    // decision.
    //
    // The reference goes through volatile so it is computed by the same
    // runtime libm call the factory made. Writing std::sin(std::numbers::pi)
    // directly would be constant-folded by the compiler (correctly rounded
    // via MPFR) and disagree with libm in the last few bits: 1.2246468e-16
    // versus 1.2246064e-16 on this toolchain, at -O0 and -O2 alike.
    //
    // No specific residual is pinned, so this stays valid on any libm.
    volatile double argument = std::numbers::pi;
    const double expected = std::sin(argument);

    ASSERT_EQ(sinePi.type(), ExpressionType::Constant);
    EXPECT_DOUBLE_EQ(constantValue(sinePi), expected);
}

TEST(ExpressionFactoryTest, DropsAdditiveIdentity)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");
    const auto zero = factory.constant(0.0);

    EXPECT_TRUE(sameNode(factory.add(x, zero), x));
    EXPECT_TRUE(sameNode(factory.add(zero, x), x));
}

TEST(ExpressionFactoryTest, DropsSubtractiveIdentity)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    EXPECT_TRUE(sameNode(factory.subtract(x, factory.constant(0.0)), x));
}

TEST(ExpressionFactoryTest, DropsMultiplicativeIdentity)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");
    const auto one = factory.constant(1.0);

    EXPECT_TRUE(sameNode(factory.multiply(x, one), x));
    EXPECT_TRUE(sameNode(factory.multiply(one, x), x));
}

TEST(ExpressionFactoryTest, DropsDivisionByOne)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    EXPECT_TRUE(sameNode(factory.divide(x, factory.constant(1.0)), x));
}

TEST(ExpressionFactoryTest, KeepsSymbolicMultiplicationByZero)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    // x * 0 -> 0 is deliberately absent: it would erase the domain of x.
    // (1/q) * 0 is undefined at q = 0, but a folded 0 would claim
    // otherwise.
    const auto product = factory.multiply(x, factory.constant(0.0));
    EXPECT_EQ(product.type(), ExpressionType::Multiply);
}

TEST(ExpressionFactoryTest, FoldsConstantMultipliedByZero)
{
    const ExpressionFactory factory;

    // Both operands known and finite: nothing about a domain is lost.
    const auto product = factory.multiply(factory.constant(3.0), factory.constant(0.0));
    EXPECT_TRUE(isZero(product));
}

TEST(ExpressionFactoryTest, ConstantZeroSharesTheDefaultNode)
{
    const ExpressionFactory factory;
    EXPECT_TRUE(sameNode(factory.constant(0.0), Expression{}));
}

TEST(ExpressionFactoryTest, DoesNotReorderOperands)
{
    const ExpressionFactory factory;
    const auto forward = factory.subtract(factory.symbol("a"), factory.symbol("b"));
    const auto reversed = factory.subtract(factory.symbol("b"), factory.symbol("a"));

    EXPECT_FALSE(structurallyEqual(forward, reversed));
}

TEST(ExpressionFactoryTest, DoesNotSimplifyBeyondScope)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    // x - x -> 0 needs operand equivalence checking, which belongs to a
    // future EquationSimplifier.
    EXPECT_EQ(factory.subtract(x, x).type(), ExpressionType::Subtract);
}
```

### 11. `tests/test_symbolic_matrix.cpp` (nowy plik)

```cpp
#include <gtest/gtest.h>

#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicMatrix.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <variant>

using kinemaforge::ik::AddNode;
using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::ExpressionType;
using kinemaforge::ik::MultiplyNode;
using kinemaforge::ik::SymbolicMatrix;
using kinemaforge::ik::SymbolicTransform;
using kinemaforge::ik::constantValue;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::multiply;
using kinemaforge::ik::sameNode;

TEST(SymbolicMatrixTest, DefaultMatrixCellsAreZero)
{
    const SymbolicMatrix<4, 4> matrix;

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
            EXPECT_TRUE(isZero(matrix(row, column)));
}

TEST(SymbolicMatrixTest, DefaultMatrixCellsShareOneNode)
{
    const SymbolicMatrix<4, 4> matrix;
    EXPECT_TRUE(sameNode(matrix(0, 0), matrix(2, 3)));
}

TEST(SymbolicMatrixTest, SubscriptReadsAndWritesCell)
{
    const ExpressionFactory factory;
    SymbolicMatrix<4, 4> matrix;
    const auto value = factory.symbol("s");

    matrix(1, 2) = value;
    EXPECT_TRUE(sameNode(matrix(1, 2), value));
}

TEST(SymbolicMatrixTest, SubscriptIsRowMajor)
{
    const ExpressionFactory factory;
    SymbolicMatrix<4, 4> matrix;

    matrix(0, 1) = factory.symbol("s");
    EXPECT_TRUE(isZero(matrix(1, 0)));
}

TEST(SymbolicMatrixTest, ZerosMatchesDefaultConstruction)
{
    const auto matrix = SymbolicMatrix<3, 3>::zeros();

    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            EXPECT_TRUE(isZero(matrix(row, column)));
}

TEST(SymbolicMatrixTest, IdentityHasOnesOnDiagonal)
{
    const auto matrix = SymbolicMatrix<4, 4>::identity();

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            if (row == column)
                EXPECT_TRUE(isOne(matrix(row, column)));
            else
                EXPECT_TRUE(isZero(matrix(row, column)));
        }
}

TEST(SymbolicMatrixTest, IdentityDiagonalSharesOneNode)
{
    const auto matrix = SymbolicMatrix<4, 4>::identity();
    EXPECT_TRUE(sameNode(matrix(0, 0), matrix(3, 3)));
}

TEST(SymbolicMatrixTest, MultiplySmallMatrices)
{
    const ExpressionFactory factory;
    SymbolicMatrix<2, 2> lhs;
    SymbolicMatrix<2, 2> rhs;

    lhs(0, 0) = factory.symbol("a"); lhs(0, 1) = factory.symbol("b");
    lhs(1, 0) = factory.symbol("c"); lhs(1, 1) = factory.symbol("d");
    rhs(0, 0) = factory.symbol("e"); rhs(0, 1) = factory.symbol("f");
    rhs(1, 0) = factory.symbol("g"); rhs(1, 1) = factory.symbol("h");

    const auto product = multiply(lhs, rhs, factory);

    // Expect (a*e) + (b*g): left-folded, left operand from lhs.
    ASSERT_EQ(product(0, 0).type(), ExpressionType::Add);
    const auto& sum = std::get<AddNode>(product(0, 0).node().value);

    ASSERT_EQ(sum.lhs.type(), ExpressionType::Multiply);
    const auto& first = std::get<MultiplyNode>(sum.lhs.node().value);
    EXPECT_TRUE(sameNode(first.lhs, lhs(0, 0)));
    EXPECT_TRUE(sameNode(first.rhs, rhs(0, 0)));

    ASSERT_EQ(sum.rhs.type(), ExpressionType::Multiply);
    const auto& second = std::get<MultiplyNode>(sum.rhs.node().value);
    EXPECT_TRUE(sameNode(second.lhs, lhs(0, 1)));
    EXPECT_TRUE(sameNode(second.rhs, rhs(1, 0)));
}

TEST(SymbolicMatrixTest, MultiplyProducesLeftFoldedSum)
{
    const ExpressionFactory factory;
    SymbolicMatrix<1, 3> lhs;
    SymbolicMatrix<3, 1> rhs;

    lhs(0, 0) = factory.symbol("a"); lhs(0, 1) = factory.symbol("b"); lhs(0, 2) = factory.symbol("c");
    rhs(0, 0) = factory.symbol("d"); rhs(1, 0) = factory.symbol("e"); rhs(2, 0) = factory.symbol("f");

    const auto product = multiply(lhs, rhs, factory);

    // Expect ((a*d + b*e) + c*f): the outer Add holds the third term.
    ASSERT_EQ(product(0, 0).type(), ExpressionType::Add);
    const auto& outer = std::get<AddNode>(product(0, 0).node().value);
    EXPECT_EQ(outer.lhs.type(), ExpressionType::Add);
    EXPECT_EQ(outer.rhs.type(), ExpressionType::Multiply);
}

TEST(SymbolicMatrixTest, MultiplyConstantMatrixByIdentityReturnsSameValues)
{
    const ExpressionFactory factory;
    SymbolicMatrix<2, 2> constants;
    constants(0, 0) = factory.constant(1.0); constants(0, 1) = factory.constant(2.0);
    constants(1, 0) = factory.constant(3.0); constants(1, 1) = factory.constant(4.0);

    const auto product = multiply(constants, SymbolicMatrix<2, 2>::identity(), factory);

    // Constant folding collapses each c*0 to 0, after which x+0 -> x
    // fires, so the original values come back.
    EXPECT_DOUBLE_EQ(constantValue(product(0, 0)), 1.0);
    EXPECT_DOUBLE_EQ(constantValue(product(0, 1)), 2.0);
    EXPECT_DOUBLE_EQ(constantValue(product(1, 0)), 3.0);
    EXPECT_DOUBLE_EQ(constantValue(product(1, 1)), 4.0);
}

TEST(SymbolicMatrixTest, MultiplySymbolicMatrixByIdentityKeepsZeroProducts)
{
    const ExpressionFactory factory;
    SymbolicMatrix<2, 2> symbols;
    symbols(0, 0) = factory.symbol("a00"); symbols(0, 1) = factory.symbol("a01");
    symbols(1, 0) = factory.symbol("a10"); symbols(1, 1) = factory.symbol("a11");

    const auto product = multiply(symbols, SymbolicMatrix<2, 2>::identity(), factory);

    // With x * 0 -> 0 deliberately absent, A * I is (a00 + (a01 * 0)),
    // not a00. Recovering A needs a simplifier that tracks domains.
    ASSERT_EQ(product(0, 0).type(), ExpressionType::Add);
    EXPECT_FALSE(sameNode(product(0, 0), symbols(0, 0)));

    const auto& sum = std::get<AddNode>(product(0, 0).node().value);
    EXPECT_TRUE(sameNode(sum.lhs, symbols(0, 0)));
    ASSERT_EQ(sum.rhs.type(), ExpressionType::Multiply);
    const auto& zeroProduct = std::get<MultiplyNode>(sum.rhs.node().value);
    EXPECT_TRUE(sameNode(zeroProduct.lhs, symbols(0, 1)));
    EXPECT_TRUE(isZero(zeroProduct.rhs));
}

TEST(SymbolicMatrixTest, MultiplyOfConstantMatricesFoldsToConstants)
{
    const ExpressionFactory factory;
    SymbolicMatrix<2, 2> lhs;
    SymbolicMatrix<2, 2> rhs;

    lhs(0, 0) = factory.constant(1.0); lhs(0, 1) = factory.constant(2.0);
    lhs(1, 0) = factory.constant(3.0); lhs(1, 1) = factory.constant(4.0);
    rhs(0, 0) = factory.constant(5.0); rhs(0, 1) = factory.constant(6.0);
    rhs(1, 0) = factory.constant(7.0); rhs(1, 1) = factory.constant(8.0);

    const auto product = multiply(lhs, rhs, factory);

    EXPECT_DOUBLE_EQ(constantValue(product(0, 0)), 1.0 * 5.0 + 2.0 * 7.0);
    EXPECT_DOUBLE_EQ(constantValue(product(1, 1)), 3.0 * 6.0 + 4.0 * 8.0);
}

TEST(SymbolicMatrixTest, MultiplyFourByFourTransforms)
{
    const ExpressionFactory factory;
    const auto left = SymbolicTransform::identity();
    auto right = SymbolicTransform::identity();
    const auto tx = factory.symbol("tx");
    right(0, 3) = tx;

    const auto product = multiply(left, right, factory);

    EXPECT_EQ(SymbolicTransform::rows, 4u);
    EXPECT_EQ(SymbolicTransform::columns, 4u);

    // Cells whose terms are all constant fold away completely.
    EXPECT_TRUE(isOne(product(0, 0)));
    EXPECT_TRUE(isZero(product(1, 0)));

    // The symbol reaches (0,3) unchanged: 1*tx folds by the identity rule
    // and the remaining 0*0 terms fold to constant 0, which x+0 -> x drops.
    EXPECT_TRUE(sameNode(product(0, 3), tx));

    // (3,3) is where the missing annihilator shows: the 0*tx term cannot
    // fold (tx is symbolic), so the cell is Add(Multiply(0, tx), 1) rather
    // than the constant 1 a full simplifier would produce.
    ASSERT_EQ(product(3, 3).type(), ExpressionType::Add);
    const auto& sum = std::get<AddNode>(product(3, 3).node().value);

    ASSERT_EQ(sum.lhs.type(), ExpressionType::Multiply);
    const auto& zeroProduct = std::get<MultiplyNode>(sum.lhs.node().value);
    EXPECT_TRUE(isZero(zeroProduct.lhs));
    EXPECT_TRUE(sameNode(zeroProduct.rhs, tx));

    EXPECT_TRUE(isOne(sum.rhs));
}
```

## Uwagi implementacyjne

**`type()` używa `std::visit`, nie rzutowania `variant::index()` (decyzja z review).** Pierwsza wersja rzutowała indeks wariantu wprost na enum i opisywała powstały ukryty kontrakt („kolejność `ExpressionType` == kolejność alternatyw") jako akceptowalny. Nie jest: po dodaniu w przyszłości np. `Atan2Node` w innym miejscu listy kod dalej by się **kompilował**, zwracając ciche, błędne wartości enuma. `TypeOfNode` z dziewięcioma przeciążeniami zamienia to na błąd kompilacji — dodanie typu węzła bez wpisu tutaj nie przejdzie. Kolejność enuma przestaje być częścią kontraktu.

`type()` pozostaje `noexcept`: `std::visit` rzuca tylko dla wariantu `valueless_by_exception`, a ten wariant jest ustawiany raz przy konstrukcji i nigdy nie podmieniany, więc taki stan jest nieosiągalny.

**`structurallyEqual` używa pojedynczego `std::visit`.** Wersja z podwójnym `visit` instancjonowałaby 81 kombinacji (9×9), z których 72 są martwe, bo zgodność typów jest już sprawdzona wcześniej. Wersja pojedyncza pobiera prawy węzeł przez `std::get<Node>`.

**`StructuralComparator` używa `requires` na kształcie węzła**, nie na liście typów: jedno przeciążenie obsługuje wszystkie węzły dwuargumentowe (`n.lhs`, `n.rhs`), drugie wszystkie jednoargumentowe (`n.operand`). Dodanie węzła o tym samym kształcie nie wymaga tu żadnej zmiany.

**Kontrakty asertowane to preconditions, nie gwarancje runtime (doprecyzowanie z review).** Poprzednia wersja pisała, że `constant()` „odrzuca `NaN`/`Inf`" i że „wartości niefinitywne nie mogą trafić do grafu". To prawda **wyłącznie** w buildach z aktywnymi asercjami. Z `-DNDEBUG` zarówno `factory.constant(inf)`, jak i `factory.divide(constant(1.0), constant(0.0))` wstawią `Inf` do grafu bez żadnego sygnału.

Właściwe sformułowanie: wartości niefinitywne, puste nazwy symboli i literalne dzielenie przez zero są **naruszeniem precondition**. Są wykrywane przez `assert` w buildach diagnostycznych; poprawny kod nie może ich przekazywać. Nie jest to inwariant utrzymywany w Release.

Zostaję przy `assert` zamiast rzucania wyjątku lub `std::expected`, bo jest to warstwa **wewnętrzna**, nie parser danych użytkownika — jedynymi jej klientami będą `JointTransformBuilder` i `ForwardKinematicsBuilder`, czyli kod tego projektu. Gdyby inwariant miał obowiązywać zawsze, potrzebne byłoby realne sprawdzenie (`if (!std::isfinite(value)) throw std::domain_error{...}`), co jednak wprowadza koszt na każde utworzenie stałej — a stałe powstają w tej warstwie dziesiątkami tysięcy.

Jedna konsekwencja warta odnotowania: komentarz przy `structurallyEqual` również został przeformułowany. Jeśli `NaN` mimo wszystko trafi do grafu (czyli w programie, który już złamał kontrakt), porównanie jest niespójne — ten sam węzeł wyjdzie równy przez skrót `sameNode`, a dwa osobne węzły `NaN` wyjdą różne. Nie próbuję tego naprawiać; odnotowuję jako zachowanie kodu już niepoprawnego.

**Testy nie mogą sprawdzić, że konstruktor z węzła jest niedostępny** — prywatny konstruktor czyni taki kod **niekompilowalnym**, a nie zawodzącym w czasie wykonania. Inwariant jest egzekwowany przez kompilator, więc test byłby niewykonalny z założenia.

**Brak testu błędnego indeksu.** Sprawdzenie zakresu to `assert`, więc test wymagałby `EXPECT_DEATH` — mechanizmu nieużywanego dziś w tym repo i kapryśnego na MinGW. Zatwierdzone w §19 proposalu architektonicznego.

**`Kinematics.h`/`example_forward` bez zmian.** Poza zakresem.

## Zgodność z zatwierdzoną architekturą

| Decyzja | Gdzie w kodzie |
|---|---|
| `Expression` = `shared_ptr<const ExpressionNode>`, 16 B | `Expression.hpp`, pole `node_` |
| `ExpressionNode` jako **struktura**, nie alias (cykl deklaracji) | `Expression.hpp`, `struct ExpressionNode;` + definicja na końcu |
| Płaskie węzły `*Node`, bez baz `Binary`/`Unary` | `Expression.hpp` |
| Brak stanu nieprawidłowego, konstruktor domyślny = stała 0 | `Expression::Expression()` |
| Współdzielony węzeł zera | `Expression::sharedZeroNode()`, `constant(0.0)` |
| **Prywatny** konstruktor z węzła + `friend ExpressionFactory` | `Expression.hpp` |
| Zwijanie stałych (kategoria A) | każda metoda `ExpressionFactory` |
| Elementy neutralne (kategoria B) | `add`, `subtract`, `multiply`, `divide` |
| **Brak** anihilatora `x·0 → 0` | `multiply()` — komentarz w miejscu, gdzie reguły by nie było |
| `constant()` wymaga skończoności jako **precondition** (`assert`), także dla wyników zwijania | `assert(std::isfinite(...))` |
| `structurallyEqual` startuje od `sameNode` | `Expression.cpp` |
| `sameNode` bez `friend` | `&lhs.node() == &rhs.node()` |
| `operator()` zamiast `at()` | `SymbolicMatrix.hpp` |
| `static_assert` na zerowych wymiarach | `SymbolicMatrix.hpp` |
| `identity()` ze współdzieloną jedynką, `requires (Rows == Columns)` | `SymbolicMatrix.hpp` |
| `multiply(lhs, rhs, factory)` jako wolna funkcja, left-fold | `SymbolicMatrix.hpp` |
| Brak `std::expected`; `assert` dla naruszeń kontraktu | całość |
| `SymbolicTransform` zostaje aliasem | plik bez zmian |
| `Expression.cpp` + `ExpressionFactory.cpp`, matryca header-only | §7 CMake |

## Jak to zweryfikuję po zatwierdzeniu

```powershell
cmake -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

Oczekiwany wynik: 27 dotychczasowych + 14 (`SymbolicExpressionTest`) + 16 (`ExpressionFactoryTest`) + 13 (`SymbolicMatrixTest`) = **70 zielonych**.

*(Poprzednia wersja podawała 71 na podstawie zawyżonych liczników — 16/14 zamiast 15/13. Po podziale testu trygonometrycznego na dwa faktyczna liczba nowych testów to 43, potwierdzona przez `--gtest_list_tests` na zbudowanym binarium.)*

## Wdrożone — i jedna rzecz, której weryfikacja nie złapała

Zatwierdzone i naniesione na źródła. Wynik: **70/70 testów** (27 dotychczasowych + 43 nowe), build czysty.

**Ale pierwszy build w repo nie przeszedł**, mimo że weryfikacja w katalogu roboczym pokazywała 43/43 w obu trybach:

```
multiple definition of `std::type_info::operator==(std::type_info const&) const'
  libstdc++.a(tinfo.o)          — first defined here
  libkinemaforge_ik.a(Expression.cpp.obj)
```

**Dlaczego weryfikacja tego nie wykryła:** budowałem prosto przez `g++`, bez flag, które `CMakeLists.txt` dodaje dla MinGW (`-static-libgcc -static-libstdc++ -static`). Środowisko weryfikacji różniło się od prawdziwego buildu dokładnie tą flagą, która ma tu znaczenie. To ten sam rodzaj błędu co poprzednio (weryfikacja czegoś innego niż to, co się deklaruje) — tylko na poziomie flag kompilacji, nie treści plików.

**Diagnoza** (bisekcja na minimalnych plikach):

| Konstrukcja | Emituje kolidujący symbol? |
|---|---|
| `std::visit`, `std::get`, `std::get_if`, `holds_alternative`, `index()` | nie |
| `std::shared_ptr<T>(new T(...))` | nie |
| **`std::make_shared<T>(...)`** | **tak, jako definicja silna (`T`)** |

| Flagi linkera | Kolizja? |
|---|---|
| brak | nie |
| **`-static-libstdc++`** | **tak** |

Czyli: błąd GCC 13.1/MinGW, w którym `make_shared` emituje `std::type_info::operator==` z zewnętrznym linkowaniem zamiast comdat. Ujawnia się dopiero przy statycznym libstdc++, a tej flagi usunąć nie można — `CMakeLists.txt` dokumentuje jej powód (przesłanianie DLL przez Anacondę, crash `0xc0000139`).

**Rozważone naprawy:**

| Wariant | Ocena |
|---|---|
| `shared_ptr(new T)` zamiast `make_shared` (**wybrane**) | Zmiana w dwóch linijkach jednego pliku, samodokumentująca się komentarzem. Koszt: druga alokacja na blok kontrolny — bez znaczenia, bo drzewo powstaje raz na robota, nie w pętli. |
| `-Wl,--allow-multiple-definition` | Działa, ale to flaga **projektowa**: na stałe osłabia wykrywanie kolizji ODR w całym projekcie, żeby obejść jeden błąd w jednym pliku. Odrzucone. |
| usunięcie `-static-libstdc++` | Niedopuszczalne — przywraca udokumentowany crash. |

Kod powyżej (§2) zawiera już tę poprawkę. Weryfikacja z katalogu roboczego (43/43 w Debug i Release) pozostaje ważna dla samej logiki — nie obejmowała tylko linkowania statycznego.
