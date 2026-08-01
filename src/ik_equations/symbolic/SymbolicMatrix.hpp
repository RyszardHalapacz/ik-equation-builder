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
