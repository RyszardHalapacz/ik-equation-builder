#include "ik_equations/builders/ConstraintBuilder.hpp"

#include "ik_equations/model/JointVariable.hpp"
#include "ik_equations/symbolic/Expression.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace kinemaforge::ik {
namespace {

// Column 3 of a homogeneous transform, rows 0..2 -- the translation.
constexpr std::size_t kTranslationColumn = 3;

// Symbol names a subtree depends on, in first-encounter order.
//
// Order matters: with several undeclared symbols, the reported one must not
// depend on hash iteration. That is the same trap R2 removed from the loader,
// where the root link was chosen by unordered_map iteration order. Here the
// walk follows equation order, each side depth-first left to right, so the
// message is reproducible.
//
// The visited set is by node identity, because an FK DAG shares sub-trees
// heavily -- 281 unique nodes against 21 882 counted with multiplicity for
// kr640. Without it the walk is exponential in practice.
//
// Every string_view here points at a SymbolNode::name inside the caller's
// transform, which outlives build(); nothing keeps one past return.
void collectSymbols(const Expression& expression,
                    std::unordered_set<const ExpressionNode*>& visited,
                    std::vector<std::string_view>& symbols)
{
    if (!visited.insert(&expression.node()).second)
        return;

    std::visit(
        [&](const auto& node) {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, SymbolNode>)
            {
                if (std::ranges::find(symbols, node.name) == symbols.end())
                    symbols.emplace_back(node.name);
            }
            else if constexpr (std::is_same_v<Node, AddNode>
                               || std::is_same_v<Node, SubtractNode>
                               || std::is_same_v<Node, MultiplyNode>
                               || std::is_same_v<Node, DivideNode>)
            {
                collectSymbols(node.lhs, visited, symbols);
                collectSymbols(node.rhs, visited, symbols);
            }
            else if constexpr (std::is_same_v<Node, NegateNode>
                               || std::is_same_v<Node, SinNode>
                               || std::is_same_v<Node, CosNode>)
            {
                collectSymbols(node.operand, visited, symbols);
            }
            else
            {
                // Constant is the only other leaf. A node type added later must
                // decide explicitly whether it has children: falling into
                // "nothing to do" would silently stop the walk above its
                // subtree, and this walk is what the symbol guarantee rests on.
                static_assert(std::is_same_v<Node, ConstantNode>,
                              "Unhandled ExpressionNode type");
            }
        },
        expression.node().value);
}

std::vector<JointVariable> unknownsOf(const KinematicChain& chain)
{
    std::vector<JointVariable> unknowns;
    for (const auto& joint : chain.joints)
        if (joint.variable)
            unknowns.push_back(*joint.variable);
    return unknowns;
}

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

ConstraintBuilderError invalidTarget(TargetValidationError error)
{
    auto message = "target rejected: " + error.message;
    return ConstraintBuilderError{ConstraintBuilderErrorCode::InvalidTarget,
                                  std::move(message), std::move(error), std::nullopt};
}

ConstraintBuilderError undeclaredSymbol(std::string_view name)
{
    return ConstraintBuilderError{
        ConstraintBuilderErrorCode::UndeclaredSymbol,
        "equation depends on '" + std::string{name}
            + "', which is not an unknown of the given chain",
        std::nullopt, std::nullopt};
}

ConstraintBuilderError systemRejected(IkEquationSystemError error)
{
    auto message = "equation system rejected: " + error.message;
    return ConstraintBuilderError{ConstraintBuilderErrorCode::SystemRejected,
                                  std::move(message), std::nullopt, std::move(error)};
}

} // namespace

ConstraintBuilder::ConstraintBuilder(ExpressionFactory factory)
    : factory_(std::move(factory))
{
}

std::vector<Equation>
ConstraintBuilder::positionEquations(const SymbolicTransform& transform,
                                     const Vector3& position) const
{
    // X, Y, Z in this order: IkEquationSystem promises position first in this
    // order and rejects UnorderedEquations, so a producer that emits them any
    // other way builds a system its own validation refuses.
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

// Scale first, then hypot, then divide. The obvious version -- hypot on the raw
// components -- does not cover the contract this class advertises, that any
// finite non-zero vector names a direction:
//
//   {DBL_MAX, DBL_MAX, 0}        hypot = inf         raw x/norm -> (0, 0, 0)
//   {denorm_min, denorm_min, 0}  hypot = 4.94e-324   raw x/norm -> (1, 1, 0)
//
// The first silently destroys a direction that is exact to the last bit. The
// second is worse: it returns a vector of norm sqrt(2) that PRETENDS to be a
// unit direction and would go on to poison the equations.
//
// After scaling by the largest component, s lies in [-1, 1]^3 with max |s| = 1,
// so hypot(s) lies in [1, sqrt(3)] -- measured -- and neither overflow nor
// underflow has anywhere to happen. The scale is the same quantity validation
// compared against zero, so this division is safe by construction.
//
// One limit, stated plainly: for a subnormal input the ratio v/scale loses the
// significant digits the subnormal never had. The direction is recovered to the
// precision the input actually carried, and no method gives more.
Vector3 ConstraintBuilder::normalizedDirection(const Vector3& direction) noexcept
{
    const double scale = std::max({std::abs(direction.x), std::abs(direction.y),
                                   std::abs(direction.z)});

    const double x = direction.x / scale;
    const double y = direction.y / scale;
    const double z = direction.z / scale;

    const double norm = std::hypot(x, y, z);
    return Vector3{x / norm, y / norm, z / norm};
}

// The tool axis column of the rotation block IS the direction of that axis in
// the base frame: R(q) e_t is column t. So the left-hand sides are raw FK cells,
// exactly like the position ones -- no expression is built here.
void ConstraintBuilder::appendDirectionEquations(std::vector<Equation>& equations,
                                                 const SymbolicTransform& transform,
                                                 ToolAxis axis,
                                                 const Vector3& unitDirection) const
{
    // Unreachable after validate(target): a target carrying an axis outside the
    // enumerators is rejected before this is called, so reaching it means a
    // broken invariant, not bad data. Same shape as orientationSource() above.
    const auto column = toolAxisColumn(axis);
    if (!column)
        throw std::logic_error("tool axis must be X, Y or Z");

    const double components[3] = {unitDirection.x, unitDirection.y, unitDirection.z};

    for (std::size_t row = 0; row < 3; ++row)
        equations.push_back(Equation{transform(row, *column),
                                     factory_.constant(components[row]),
                                     DirectionEquationSource{
                                         static_cast<CartesianComponent>(row), axis}});
}

// The first left-hand side in this project that is not a cell of the FK matrix:
//
//     lhs = n_x T(0,c) + n_y T(1,c) + n_z T(2,c)
//
// This is exactly what F2.5 prepared for when the symbol collector moved from
// the transform cells to the equations: the guarantee "every symbol in the
// equations is a declared unknown" needs no change to cover a computed side.
//
// Every term is built, including one whose coefficient is zero -- dropping it
// would be constant folding by hand, and this project deliberately does NOT fold
// x * 0, because that erases domain information. The resulting SHAPE still
// depends on the data, though: the factory folds 1 * x -> x, so a coefficient of
// exactly one produces no MultiplyNode at all.
void ConstraintBuilder::appendDirectionAngleEquation(std::vector<Equation>& equations,
                                                     const SymbolicTransform& transform,
                                                     ToolAxis axis,
                                                     const Vector3& unitReference,
                                                     double angle) const
{
    // Unreachable after validate(target); same shape as the emitters above.
    const auto column = toolAxisColumn(axis);
    if (!column)
        throw std::logic_error("tool axis must be X, Y or Z");

    const double components[3] = {unitReference.x, unitReference.y, unitReference.z};

    Expression sum = factory_.multiply(factory_.constant(components[0]),
                                       transform(0, *column));
    for (std::size_t row = 1; row < 3; ++row)
        sum = factory_.add(sum, factory_.multiply(factory_.constant(components[row]),
                                                  transform(row, *column)));

    equations.push_back(Equation{std::move(sum), factory_.constant(std::cos(angle)),
                                 DirectionAngleEquationSource{axis}});
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

std::expected<IkEquationSystem, ConstraintBuilderError>
ConstraintBuilder::build(const SymbolicTransform& transform,
                         const KinematicChain& chain,
                         const PositionTarget& target) const
{
    // First, and the order is load-bearing: the target's coordinates become
    // constants below, and ExpressionFactory::constant has a finite-value
    // precondition. Validating here makes that precondition hold by
    // construction rather than by hope.
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
    // Same load-bearing order as above -- the rotation cells become constants
    // too, so they must be known finite before appendOrientationEquations runs.
    if (auto valid = validate(target); !valid)
        return std::unexpected(invalidTarget(valid.error()));

    auto equations = positionEquations(transform, target.position);
    appendOrientationEquations(equations, transform, target.orientation);

    return assemble(chain, IkTaskKind::Pose, std::move(equations));
}

std::expected<IkEquationSystem, ConstraintBuilderError>
ConstraintBuilder::build(const SymbolicTransform& transform,
                         const KinematicChain& chain,
                         const ToolDirectionTarget& target) const
{
    // Validation first, canonicalisation second, emission third -- and the order
    // is the contract: validate() only ever states an opinion, the unit vector
    // is made here, and the emitter is handed something it can trust.
    if (auto checked = validate(target); !checked)
        return std::unexpected(invalidTarget(std::move(checked.error())));

    std::vector<Equation> equations;
    appendDirectionEquations(equations, transform, target.axis,
                             normalizedDirection(target.direction));

    return assemble(chain, IkTaskKind::ToolDirection, std::move(equations));
}

std::expected<IkEquationSystem, ConstraintBuilderError>
ConstraintBuilder::build(const SymbolicTransform& transform,
                         const KinematicChain& chain,
                         const PositionAndToolDirectionTarget& target) const
{
    if (auto checked = validate(target); !checked)
        return std::unexpected(invalidTarget(std::move(checked.error())));

    auto equations = positionEquations(transform, target.position);
    appendDirectionEquations(equations, transform, target.axis,
                             normalizedDirection(target.direction));

    return assemble(chain, IkTaskKind::PositionAndToolDirection, std::move(equations));
}

std::expected<IkEquationSystem, ConstraintBuilderError>
ConstraintBuilder::build(const SymbolicTransform& transform,
                         const KinematicChain& chain,
                         const DirectionAngleTarget& target) const
{
    if (auto checked = validate(target); !checked)
        return std::unexpected(invalidTarget(std::move(checked.error())));

    // The reference is canonicalised for the same reason the direction is, one
    // side of the equation further along: without it the coefficients carry |n|
    // and the equation describes a different angle than the one asked for.
    std::vector<Equation> equations;
    appendDirectionAngleEquation(equations, transform, target.axis,
                                 normalizedDirection(target.reference), target.angle);

    return assemble(chain, IkTaskKind::DirectionAngle, std::move(equations));
}

std::expected<IkEquationSystem, ConstraintBuilderError>
ConstraintBuilder::build(const SymbolicTransform& transform,
                         const KinematicChain& chain,
                         const PositionAndDirectionAngleTarget& target) const
{
    if (auto checked = validate(target); !checked)
        return std::unexpected(invalidTarget(std::move(checked.error())));

    auto equations = positionEquations(transform, target.position);
    appendDirectionAngleEquation(equations, transform, target.axis,
                                 normalizedDirection(target.reference), target.angle);

    return assemble(chain, IkTaskKind::PositionAndDirectionAngle, std::move(equations));
}

} // namespace kinemaforge::ik
