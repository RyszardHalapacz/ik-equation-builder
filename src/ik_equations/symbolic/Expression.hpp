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
