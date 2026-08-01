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
