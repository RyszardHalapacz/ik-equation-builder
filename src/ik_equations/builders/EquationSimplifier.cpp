#include "ik_equations/builders/EquationSimplifier.hpp"

#include "ik_equations/symbolic/ExpressionCanonicalizer.hpp"
#include "ik_equations/symbolic/TrigonometricSimplifier.hpp"

#include <span>
#include <utility>
#include <vector>

namespace kinemaforge::ik {

EquationSimplifier::EquationSimplifier(ExpressionFactory factory)
    : factory_(std::move(factory))
{
}

std::expected<IkEquationSystem, IkEquationSystemError>
EquationSimplifier::normalize(const IkEquationSystem& system) const
{
    CanonicalizationSession session{factory_};

    std::vector<Equation> equations;
    equations.reserve(system.equations().size());

    for (const Equation& equation : system.equations())
    {
        Equation normalized = equation;
        normalized.lhs = session.canonicalize(equation.lhs);
        normalized.rhs = session.canonicalize(equation.rhs);
        equations.push_back(std::move(normalized));
    }

    const std::span<const JointVariable> unknowns = system.unknowns();
    return IkEquationSystem::create(system.taskKind(),
                                    std::vector<JointVariable>(unknowns.begin(), unknowns.end()),
                                    std::move(equations));
}

std::expected<IkEquationSystem, IkEquationSystemError>
EquationSimplifier::simplify(const IkEquationSystem& system) const
{
    TrigonometricSimplificationSession session{factory_};

    std::vector<Equation> equations;
    equations.reserve(system.equations().size());

    for (const Equation& equation : system.equations())
    {
        Equation simplified = equation;
        simplified.lhs = session.simplify(equation.lhs);
        simplified.rhs = session.simplify(equation.rhs);
        equations.push_back(std::move(simplified));
    }

    const std::span<const JointVariable> unknowns = system.unknowns();
    return IkEquationSystem::create(system.taskKind(),
                                    std::vector<JointVariable>(unknowns.begin(), unknowns.end()),
                                    std::move(equations));
}

} // namespace kinemaforge::ik
