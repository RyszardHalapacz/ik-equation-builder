#pragma once

#include "ik_equations/symbolic/Expression.hpp"

#include <optional>
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
// Folding happens only when the result is finite. This factory is closed
// over the finite-constant contract: given operands that satisfy it, no
// operation here produces a constant that violates it. An overflow
// declines to fold and builds the operation node instead, which then
// reports NonFiniteResult when evaluated. That is a property of the
// factory's own transformations, not a runtime guard -- constant() still
// trusts its caller.
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
    // diagnostic builds.
    //
    // Constant folding never routes a non-finite result here: an
    // overflowing fold declines and builds the operation node instead, so
    // this precondition can only be violated by a caller.
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

private:
    // Constant folding, but only when the arithmetic stayed finite.
    //
    // Overflow in a fold is not a caller error: 1e308 and 1e308 are both
    // legitimate constants and multiply() is a legitimate operation, yet
    // their product is not representable. Folding it would put a
    // non-finite leaf in the tree, which the class comment promises this
    // factory never does -- and which the next fold would propagate
    // (inf - inf is NaN).
    //
    // Declining to fold loses nothing: the operation node is built
    // instead, and ExpressionEvaluator reports NonFiniteResult when it is
    // evaluated. The subtree keeps saying what it is rather than
    // pretending to be a number.
    //
    // Underflow is not this function's business: 1e-308 * 1e-308 is 0.0,
    // which is finite, a legitimate constant, and folded like any other.
    //
    // Only the four binary operations route through here. negate, sin and
    // cos cannot produce a non-finite result from a finite operand.
    [[nodiscard]] std::optional<Expression> foldFinite(double value) const;
};

} // namespace kinemaforge::ik
