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
