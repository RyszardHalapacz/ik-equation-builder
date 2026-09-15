#pragma once

#include "ik_equations/model/IkEquationSystem.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <expected>

namespace kinemaforge::ik {

class EquationSimplifier
{
public:
    explicit EquationSimplifier(ExpressionFactory factory);

    [[nodiscard]] std::expected<IkEquationSystem, IkEquationSystemError>
    normalize(const IkEquationSystem& system) const;

    [[nodiscard]] std::expected<IkEquationSystem, IkEquationSystemError>
    simplify(const IkEquationSystem& system) const;

private:
    ExpressionFactory factory_;
};

} // namespace kinemaforge::ik
