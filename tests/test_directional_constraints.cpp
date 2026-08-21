#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ConstraintBuilder.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/model/IkTaskDegreesOfFreedom.hpp"
#include "ik_equations/model/ToolAxis.hpp"
#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ik = kinemaforge::ik;

using ik::CartesianComponent;
using ik::ConstraintBuilder;
using ik::ConstraintBuilderErrorCode;
using ik::DirectionAngleTarget;
using ik::DirectionEquationSource;
using ik::Expression;
using ik::ExpressionFactory;
using ik::IkTaskKind;
using ik::PositionAndDirectionAngleTarget;
using ik::PositionAndToolDirectionTarget;
using ik::PositionEquationSource;
using ik::PositionTarget;
using ik::SymbolicTransform;
using ik::TargetValidationErrorCode;
using ik::ToolAxis;
using ik::ToolDirectionTarget;
using ik::Vector3;
using ik::sameNode;
using ik::sameSource;
using ik::structurallyEqual;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr std::size_t kTranslationColumn = 3;
constexpr double kTolerance = 1e-12;

// A chain that declares the given unknowns and nothing else -- the builder only
// ever asks it what its unknowns are.
ik::KinematicChain chainDeclaring(std::initializer_list<const char*> names)
{
    ik::KinematicChain chain;
    chain.baseLink = "base_link";
    chain.toolLink = "tool0";

    std::size_t index = 0;
    for (const char* name : names)
    {
        ik::KinematicJoint joint;
        joint.index = index;
        joint.name = std::string{"joint_"} + name;
        joint.type = ik::JointType::Revolute;
        joint.variable = ik::JointVariable{name, index + 1};

        chain.joints.push_back(std::move(joint));
        ++index;
    }
    return chain;
}

// Identity with every rotation cell replaced by a DISTINCT node, so a
// mis-indexed column cannot hide behind a shared one.
//
// Constants, not symbols: the builder guarantees that every symbol in the
// equations is a declared unknown, so nine invented symbol names would be
// rejected for a reason that has nothing to do with what these tests check.
// Distinct constants give the same node identity for sameNode() without
// borrowing that guarantee. The translation column keeps q1, which the chain
// does declare, so the position path stays symbolic.
SymbolicTransform transformWithDistinctRotationCells(const ExpressionFactory& factory)
{
    auto transform = SymbolicTransform::identity();
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            transform(row, column) =
                factory.constant(1.0 + static_cast<double>(row * 3 + column));

    for (std::size_t row = 0; row < 3; ++row)
        transform(row, kTranslationColumn) = factory.symbol("q1");

    return transform;
}

struct LoadedRobot
{
    ik::KinematicChain chain;
    SymbolicTransform fk;
};

// A copy of the helper in test_constraint_builder.cpp, deliberately: two
// translation units, no shared test header outside support/, and neither file
// gets to change the other by accident.
std::optional<LoadedRobot> loadRobot(const char* fileName)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(std::filesystem::path{KINEMAFORGE_URDF_DATA_DIR} / fileName);

    ik::KinematicChainBuilder chainBuilder;
    auto chain = chainBuilder.build(robot, "base_link", "tool0");
    if (!chain)
    {
        ADD_FAILURE() << "chain build failed for " << fileName;
        return std::nullopt;
    }

    const ik::JointTransformBuilder transformBuilder;
    const ik::ForwardKinematicsBuilder fkBuilder;
    auto fk = fkBuilder.build(*chain, transformBuilder);

    return LoadedRobot{std::move(*chain), std::move(fk)};
}

} // namespace

// --- shape ----------------------------------------------------------

TEST(DirectionalConstraintsTest, BuildsThreeDirectionEquations)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto system = builder.build(transformWithDistinctRotationCells(factory),
                                      chainDeclaring({"q1"}),
                                      ToolDirectionTarget{Vector3{0.0, 0.0, 1.0}});

    ASSERT_TRUE(system.has_value()) << system.error().message;
    EXPECT_EQ(system->taskKind(), IkTaskKind::ToolDirection);
    EXPECT_EQ(system->equations().size(), 3u);
    EXPECT_EQ(ik::taskSpaceConstrainedDegreesOfFreedom(system->taskKind()), 2u);
}

TEST(DirectionalConstraintsTest, UsesToolAxisColumnCells)
{
    // The whole contract of toolAxisColumn, not two thirds of it: each axis has
    // its own column, and the left-hand sides must be those very cells.
    struct Case
    {
        ToolAxis axis;
        std::size_t column;
    };

    const ExpressionFactory factory;
    const ConstraintBuilder builder;
    const auto transform = transformWithDistinctRotationCells(factory);

    for (const Case& testCase : {Case{ToolAxis::X, 0}, Case{ToolAxis::Y, 1},
                                 Case{ToolAxis::Z, 2}})
    {
        SCOPED_TRACE(testing::Message() << "column " << testCase.column);

        const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                          ToolDirectionTarget{Vector3{0.0, 0.0, 1.0},
                                                              testCase.axis});
        ASSERT_TRUE(system.has_value()) << system.error().message;
        ASSERT_EQ(system->equations().size(), 3u);

        for (std::size_t row = 0; row < 3; ++row)
            EXPECT_TRUE(sameNode(system->equations()[row].lhs, transform(row, testCase.column)))
                << "row " << row;
    }
}

TEST(DirectionalConstraintsTest, DirectionEquationsAreOrderedXYZ)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto system = builder.build(transformWithDistinctRotationCells(factory),
                                      chainDeclaring({"q1"}),
                                      ToolDirectionTarget{Vector3{0.0, 0.0, 1.0}});
    ASSERT_TRUE(system.has_value());

    const CartesianComponent expected[3] = {CartesianComponent::X, CartesianComponent::Y,
                                            CartesianComponent::Z};
    for (std::size_t row = 0; row < 3; ++row)
        EXPECT_TRUE(sameSource(system->equations()[row].source,
                               DirectionEquationSource{expected[row], ToolAxis::Z}))
            << "row " << row;
}

TEST(DirectionalConstraintsTest, UsesNormalizedDirectionOnRightHandSide)
{
    // The target keeps a ray; the equations get a unit vector. (0,0,5) and
    // (0,0,1) are the same direction and must produce the same right-hand sides.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;
    const auto transform = transformWithDistinctRotationCells(factory);

    const auto scaled = builder.build(transform, chainDeclaring({"q1"}),
                                      ToolDirectionTarget{Vector3{0.0, 0.0, 5.0}});
    const auto unit = builder.build(transform, chainDeclaring({"q1"}),
                                    ToolDirectionTarget{Vector3{0.0, 0.0, 1.0}});

    ASSERT_TRUE(scaled.has_value());
    ASSERT_TRUE(unit.has_value());

    for (std::size_t row = 0; row < 3; ++row)
        EXPECT_DOUBLE_EQ(ik::constantValue(scaled->equations()[row].rhs),
                         ik::constantValue(unit->equations()[row].rhs))
            << "row " << row;
}

TEST(DirectionalConstraintsTest, NormalizesMaxFiniteDirectionWithoutOverflow)
{
    // The boundary of the contract, not just a large number: the true length of
    // this vector exceeds DBL_MAX, so hypot on the raw components returns inf
    // and the naive division would yield (0, 0, 0).
    const ExpressionFactory factory;
    const ConstraintBuilder builder;
    const double max = std::numeric_limits<double>::max();

    const auto system = builder.build(transformWithDistinctRotationCells(factory),
                                      chainDeclaring({"q1"}),
                                      ToolDirectionTarget{Vector3{max, max, 0.0}});

    ASSERT_TRUE(system.has_value()) << system.error().message;

    const double half = 1.0 / std::sqrt(2.0);
    EXPECT_NEAR(ik::constantValue(system->equations()[0].rhs), half, kTolerance);
    EXPECT_NEAR(ik::constantValue(system->equations()[1].rhs), half, kTolerance);
    EXPECT_TRUE(ik::isZero(system->equations()[2].rhs));
}

TEST(DirectionalConstraintsTest, NormalizesSubnormalDirection)
{
    // The other end, and the worse failure: hypot underflows, and the naive
    // division returns (1, 1, 0) -- a vector of norm sqrt(2) pretending to be a
    // unit direction.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;
    const double tiny = std::numeric_limits<double>::denorm_min();

    const auto system = builder.build(transformWithDistinctRotationCells(factory),
                                      chainDeclaring({"q1"}),
                                      ToolDirectionTarget{Vector3{tiny, tiny, 0.0}});

    ASSERT_TRUE(system.has_value()) << system.error().message;

    const double half = 1.0 / std::sqrt(2.0);
    EXPECT_NEAR(ik::constantValue(system->equations()[0].rhs), half, kTolerance);
    EXPECT_NEAR(ik::constantValue(system->equations()[1].rhs), half, kTolerance);
}

TEST(DirectionalConstraintsTest, AntiParallelDirectionIsADifferentTarget)
{
    // "Alignment" means an oriented direction. -d is a different task, not the
    // same one seen from the other side.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;
    const auto transform = transformWithDistinctRotationCells(factory);

    const auto forward = builder.build(transform, chainDeclaring({"q1"}),
                                       ToolDirectionTarget{Vector3{0.0, 0.0, 1.0}});
    const auto backward = builder.build(transform, chainDeclaring({"q1"}),
                                        ToolDirectionTarget{Vector3{0.0, 0.0, -1.0}});

    ASSERT_TRUE(forward.has_value());
    ASSERT_TRUE(backward.has_value());

    EXPECT_DOUBLE_EQ(ik::constantValue(forward->equations()[2].rhs), 1.0);
    EXPECT_DOUBLE_EQ(ik::constantValue(backward->equations()[2].rhs), -1.0);
}

TEST(DirectionalConstraintsTest, ToolDirectionSystemHasNoPositionEquations)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto system = builder.build(transformWithDistinctRotationCells(factory),
                                      chainDeclaring({"q1"}),
                                      ToolDirectionTarget{Vector3{0.0, 0.0, 1.0}});
    ASSERT_TRUE(system.has_value());

    for (const auto& equation : system->equations())
        EXPECT_EQ(ik::kindOf(equation.source), ik::EquationKind::Direction);
}

TEST(DirectionalConstraintsTest, DirectionTargetDoesNotConstrainFullOrientation)
{
    // The point of the whole family: three equations, not twelve, on the very
    // same transform.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;
    const auto transform = transformWithDistinctRotationCells(factory);

    const auto direction = builder.build(transform, chainDeclaring({"q1"}),
                                         ToolDirectionTarget{Vector3{0.0, 0.0, 1.0}});
    ASSERT_TRUE(direction.has_value());
    EXPECT_EQ(direction->equations().size(), 3u);

    for (const auto& equation : direction->equations())
        EXPECT_NE(ik::kindOf(equation.source), ik::EquationKind::Orientation);
}

// --- composition ----------------------------------------------------

TEST(DirectionalConstraintsTest, BuildsSixEquationsForPositionAndDirection)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto system = builder.build(
        transformWithDistinctRotationCells(factory), chainDeclaring({"q1"}),
        PositionAndToolDirectionTarget{Vector3{1.0, 2.0, 3.0}, Vector3{0.0, 0.0, 1.0}});

    ASSERT_TRUE(system.has_value()) << system.error().message;
    EXPECT_EQ(system->taskKind(), IkTaskKind::PositionAndToolDirection);
    ASSERT_EQ(system->equations().size(), 6u);
    EXPECT_EQ(ik::taskSpaceConstrainedDegreesOfFreedom(system->taskKind()), 5u);

    for (std::size_t index = 0; index < 3; ++index)
        EXPECT_EQ(ik::kindOf(system->equations()[index].source), ik::EquationKind::Position);
    for (std::size_t index = 3; index < 6; ++index)
        EXPECT_EQ(ik::kindOf(system->equations()[index].source), ik::EquationKind::Direction);
}

TEST(DirectionalConstraintsTest, PositionEquationsMatchPositionOnlyBuild)
{
    // Same transform, same position: the three position equations must be the
    // same equations, not merely equivalent ones.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;
    const auto transform = transformWithDistinctRotationCells(factory);
    const Vector3 position{1.0, 2.0, 3.0};

    const auto positionOnly = builder.build(transform, chainDeclaring({"q1"}),
                                            PositionTarget{position});
    const auto composed = builder.build(
        transform, chainDeclaring({"q1"}),
        PositionAndToolDirectionTarget{position, Vector3{0.0, 0.0, 1.0}});

    ASSERT_TRUE(positionOnly.has_value());
    ASSERT_TRUE(composed.has_value());

    for (std::size_t index = 0; index < 3; ++index)
    {
        SCOPED_TRACE(testing::Message() << "equation " << index);
        EXPECT_TRUE(sameNode(composed->equations()[index].lhs,
                             positionOnly->equations()[index].lhs));
        EXPECT_TRUE(sameSource(composed->equations()[index].source,
                               positionOnly->equations()[index].source));
        EXPECT_DOUBLE_EQ(ik::constantValue(composed->equations()[index].rhs),
                         ik::constantValue(positionOnly->equations()[index].rhs));
    }
}

// --- rejections -----------------------------------------------------

TEST(DirectionalConstraintsTest, RejectsInvalidDirectionTargetBeforeCanonicalization)
{
    // The boundary between bad DATA and a broken INVARIANT. A forged axis in a
    // target is the caller's mistake and must come back as a value; the throw in
    // appendDirectionEquations is reserved for a state unreachable after
    // validation. If the builder skipped validate(), this would escape as an
    // exception instead.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;
    const auto transform = transformWithDistinctRotationCells(factory);

    {
        SCOPED_TRACE("forged tool axis");
        const auto system = builder.build(
            transform, chainDeclaring({"q1"}),
            ToolDirectionTarget{Vector3{1.0, 0.0, 0.0}, static_cast<ToolAxis>(99)});

        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::InvalidTarget);
        ASSERT_TRUE(system.error().targetError.has_value());
        EXPECT_EQ(system.error().targetError->code, TargetValidationErrorCode::InvalidToolAxis);
    }
    {
        // Position is validated first, and that is observable.
        SCOPED_TRACE("non-finite position and zero direction");
        const auto system = builder.build(
            transform, chainDeclaring({"q1"}),
            PositionAndToolDirectionTarget{Vector3{kNaN, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}});

        ASSERT_FALSE(system.has_value());
        ASSERT_TRUE(system.error().targetError.has_value());
        EXPECT_EQ(system.error().targetError->code, TargetValidationErrorCode::NonFinitePosition);
    }
}

TEST(DirectionalConstraintsTest, RejectsUndeclaredSymbolInDirectionEquations)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    auto transform = SymbolicTransform::identity();
    transform(0, 2) = factory.symbol("q999");

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      ToolDirectionTarget{Vector3{0.0, 0.0, 1.0}});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::UndeclaredSymbol);
    EXPECT_NE(system.error().message.find("q999"), std::string::npos);
}

// --- free rotation, and proof the check is not vacuous ---------------

namespace {

// The three left-hand sides of a direction system, evaluated at a configuration
// that differs from the base one only in the LAST joint.
std::optional<std::array<double, 3>> directionAtLastJoint(const LoadedRobot& robot,
                                                          ToolAxis axis, double lastValue)
{
    const ConstraintBuilder builder;
    const auto system = builder.build(robot.fk, robot.chain,
                                      ToolDirectionTarget{Vector3{0.0, 0.0, 1.0}, axis});
    if (!system)
    {
        ADD_FAILURE() << system.error().message;
        return std::nullopt;
    }

    const double base[6] = {0.3, -0.7, 1.1, 0.2, -0.5, 0.9};

    ik::SymbolValues values;
    std::size_t index = 0;
    for (const auto& joint : robot.chain.joints)
        if (joint.variable)
        {
            values[joint.variable->name] = (index == 5) ? lastValue : base[index];
            ++index;
        }

    ik::ExpressionEvaluator evaluator{values};
    std::array<double, 3> out{};
    for (std::size_t row = 0; row < 3; ++row)
    {
        const auto value = evaluator.evaluate(system->equations()[row].lhs);
        if (!value)
        {
            ADD_FAILURE() << "evaluation failed";
            return std::nullopt;
        }
        out[row] = *value;
    }
    return out;
}

} // namespace

TEST(DirectionalConstraintsTest, DirectionTargetLeavesSpinFree)
{
    // Measured, not assumed: kr4_r600's last joint turns about the tool frame's
    // Z axis, kr640's about its X. The pairing below follows the measurement --
    // an earlier draft assumed KR640 + Z and would have failed for reasons that
    // had nothing to do with the code under test.
    struct Case
    {
        const char* file;
        ToolAxis axis;
    };

    for (const Case& testCase : {Case{"kr4_r600.urdf", ToolAxis::Z},
                                 Case{"kr640.urdf", ToolAxis::X}})
    {
        SCOPED_TRACE(testing::Message() << testCase.file);

        const auto robot = loadRobot(testCase.file);
        ASSERT_TRUE(robot.has_value());

        const auto before = directionAtLastJoint(*robot, testCase.axis, 0.9);
        const auto after = directionAtLastJoint(*robot, testCase.axis, 1.9);
        ASSERT_TRUE(before.has_value());
        ASSERT_TRUE(after.has_value());

        // Turning the last joint does not move the tool axis, so the constraint
        // says nothing about that joint: the spin really is free.
        for (std::size_t row = 0; row < 3; ++row)
            EXPECT_NEAR((*before)[row], (*after)[row], kTolerance) << "row " << row;
    }
}

TEST(DirectionalConstraintsTest, SpinFreedomTestIsNotVacuous)
{
    // A test of invariance passes just as happily when the left-hand sides never
    // depended on the joint at all -- if the emitter picked a constant column,
    // say. The same joint on the same robot must move a DIFFERENT axis.
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    const auto before = directionAtLastJoint(*robot, ToolAxis::Z, 0.9);
    const auto after = directionAtLastJoint(*robot, ToolAxis::Z, 1.9);
    ASSERT_TRUE(before.has_value());
    ASSERT_TRUE(after.has_value());

    double worst = 0.0;
    for (std::size_t row = 0; row < 3; ++row)
        worst = std::max(worst, std::abs((*before)[row] - (*after)[row]));

    EXPECT_GT(worst, 0.1) << "the last joint must move the Z axis of kr640";
}

// --- end to end -----------------------------------------------------

TEST(DirectionalConstraintsTest, BuildsKr640PositionAndDirectionSystem)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    const ConstraintBuilder builder;
    const auto system = builder.build(
        robot->fk, robot->chain,
        PositionAndToolDirectionTarget{Vector3{2.0, 0.5, 1.5}, Vector3{0.0, 0.0, -1.0}});

    ASSERT_TRUE(system.has_value()) << system.error().message;
    EXPECT_EQ(system->taskKind(), IkTaskKind::PositionAndToolDirection);
    EXPECT_EQ(system->equations().size(), 6u);
    EXPECT_EQ(system->unknowns().size(), 6u);
    EXPECT_TRUE(sameNode(system->equations()[3].lhs, robot->fk(0, 2)));
}


// --- the angle family -----------------------------------------------

namespace {

// A reference with no coefficient equal to 1 after canonicalisation: (2,1,2)
// normalises to (2/3, 1/3, 2/3). That matters for the structural test, because
// ExpressionFactory folds 1 * x -> x, and a coefficient of one would leave no
// MultiplyNode to compare against.
constexpr Vector3 kSkewReference{2.0, 1.0, 2.0};

Vector3 canonicalSkewReference()
{
    const double norm = 3.0;   // |(2,1,2)| exactly
    return Vector3{kSkewReference.x / norm, kSkewReference.y / norm,
                   kSkewReference.z / norm};
}

} // namespace

TEST(DirectionalConstraintsTest, BuildsOneDirectionAngleEquation)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto system = builder.build(transformWithDistinctRotationCells(factory),
                                      chainDeclaring({"q1"}),
                                      DirectionAngleTarget{kSkewReference, 0.7});

    ASSERT_TRUE(system.has_value()) << system.error().message;
    EXPECT_EQ(system->taskKind(), IkTaskKind::DirectionAngle);
    EXPECT_EQ(system->equations().size(), 1u);
    EXPECT_EQ(ik::taskSpaceConstrainedDegreesOfFreedom(system->taskKind()), 1u);
    EXPECT_EQ(ik::kindOf(system->equations()[0].source), ik::EquationKind::DirectionAngle);
}

TEST(DirectionalConstraintsTest, AngleLhsMatchesDirectionDotProduct)
{
    // Shape, not just value: an implementation computing the dot product
    // NUMERICALLY and emitting a constant would satisfy every value-based check
    // while producing an equation that does not depend on q at all.
    //
    // The expectation is built through the same factory rather than described by
    // hand, so the test respects its folding rules instead of contradicting them.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;
    const auto transform = transformWithDistinctRotationCells(factory);
    const Vector3 unit = canonicalSkewReference();

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      DirectionAngleTarget{kSkewReference, 0.7});
    ASSERT_TRUE(system.has_value()) << system.error().message;

    const auto expected = factory.add(
        factory.add(factory.multiply(factory.constant(unit.x), transform(0, 2)),
                    factory.multiply(factory.constant(unit.y), transform(1, 2))),
        factory.multiply(factory.constant(unit.z), transform(2, 2)));

    EXPECT_TRUE(structurallyEqual(system->equations()[0].lhs, expected));
}

TEST(DirectionalConstraintsTest, UsesCosineOfAngleOnRightHandSide)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;
    const double angle = 0.7;   // cos(0.7) = 0.7648..., clearly not 0.7

    const auto system = builder.build(transformWithDistinctRotationCells(factory),
                                      chainDeclaring({"q1"}),
                                      DirectionAngleTarget{kSkewReference, angle});
    ASSERT_TRUE(system.has_value());

    EXPECT_DOUBLE_EQ(ik::constantValue(system->equations()[0].rhs), std::cos(angle));
    EXPECT_NE(ik::constantValue(system->equations()[0].rhs), angle);
}

TEST(DirectionalConstraintsTest, NormalizesReferenceBeforeBuildingEquation)
{
    // Without canonicalisation the coefficients would carry |n|, and the equation
    // would describe a different angle than the one asked for.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;
    const auto transform = transformWithDistinctRotationCells(factory);

    const auto scaled = builder.build(transform, chainDeclaring({"q1"}),
                                      DirectionAngleTarget{Vector3{10.0, 5.0, 10.0}, 0.7});
    const auto unit = builder.build(transform, chainDeclaring({"q1"}),
                                    DirectionAngleTarget{kSkewReference, 0.7});

    ASSERT_TRUE(scaled.has_value());
    ASSERT_TRUE(unit.has_value());
    EXPECT_TRUE(structurallyEqual(scaled->equations()[0].lhs, unit->equations()[0].lhs));
}

TEST(DirectionalConstraintsTest, BuildsFourEquationsForPositionAndAngle)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto system = builder.build(
        transformWithDistinctRotationCells(factory), chainDeclaring({"q1"}),
        PositionAndDirectionAngleTarget{Vector3{1.0, 2.0, 3.0}, kSkewReference, 0.7});

    ASSERT_TRUE(system.has_value()) << system.error().message;
    EXPECT_EQ(system->taskKind(), IkTaskKind::PositionAndDirectionAngle);
    ASSERT_EQ(system->equations().size(), 4u);
    EXPECT_EQ(ik::taskSpaceConstrainedDegreesOfFreedom(system->taskKind()), 4u);

    for (std::size_t index = 0; index < 3; ++index)
        EXPECT_EQ(ik::kindOf(system->equations()[index].source), ik::EquationKind::Position);
    EXPECT_EQ(ik::kindOf(system->equations()[3].source), ik::EquationKind::DirectionAngle);
}

TEST(DirectionalConstraintsTest, AngleOnlySystemHasNoPositionEquations)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto system = builder.build(transformWithDistinctRotationCells(factory),
                                      chainDeclaring({"q1"}),
                                      DirectionAngleTarget{kSkewReference, 0.7});
    ASSERT_TRUE(system.has_value());

    for (const auto& equation : system->equations())
        EXPECT_NE(ik::kindOf(equation.source), ik::EquationKind::Position);
}

TEST(DirectionalConstraintsTest, RejectsDegenerateAngleTarget)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto system = builder.build(transformWithDistinctRotationCells(factory),
                                      chainDeclaring({"q1"}),
                                      DirectionAngleTarget{kSkewReference, 0.0});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::InvalidTarget);
    ASSERT_TRUE(system.error().targetError.has_value());
    EXPECT_EQ(system.error().targetError->code, TargetValidationErrorCode::DegenerateAngle);
}

TEST(DirectionalConstraintsTest, AngleEquationEvaluatesToCosineAtSolution)
{
    // Ties the symbolic side to the geometry: take a real FK, read the tool axis
    // at a configuration, build a reference that sits at a known angle from it,
    // and substitute the SAME configuration back into the equation. Every other
    // test in this family checks the syntax of the equation; this one checks that
    // it describes the angle it claims to.
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    const double configuration[6] = {0.3, -0.7, 1.1, 0.2, -0.5, 0.9};

    ik::SymbolValues values;
    std::size_t index = 0;
    for (const auto& joint : robot->chain.joints)
        if (joint.variable)
        {
            values[joint.variable->name] = configuration[index];
            ++index;
        }

    // The tool axis (column Z) at that configuration.
    ik::ExpressionEvaluator axisEvaluator{values};
    double axis[3];
    for (std::size_t row = 0; row < 3; ++row)
    {
        const auto cell = axisEvaluator.evaluate(robot->fk(row, 2));
        ASSERT_TRUE(cell.has_value());
        axis[row] = *cell;
    }

    // A reference at a known angle from it: rotate the axis towards an arbitrary
    // independent vector by Gram-Schmidt, then combine.
    const double angle = 0.6;
    double other[3] = {axis[1] - axis[2], axis[2] - axis[0], axis[0] - axis[1]};
    double dot = 0.0;
    for (std::size_t row = 0; row < 3; ++row)
        dot += other[row] * axis[row];
    for (std::size_t row = 0; row < 3; ++row)
        other[row] -= dot * axis[row];

    double norm = 0.0;
    for (std::size_t row = 0; row < 3; ++row)
        norm += other[row] * other[row];
    norm = std::sqrt(norm);
    ASSERT_GT(norm, 1e-6) << "degenerate helper vector";

    const Vector3 reference{std::cos(angle) * axis[0] + std::sin(angle) * other[0] / norm,
                            std::cos(angle) * axis[1] + std::sin(angle) * other[1] / norm,
                            std::cos(angle) * axis[2] + std::sin(angle) * other[2] / norm};

    const ConstraintBuilder builder;
    const auto system = builder.build(robot->fk, robot->chain,
                                      DirectionAngleTarget{reference, angle});
    ASSERT_TRUE(system.has_value()) << system.error().message;

    ik::ExpressionEvaluator evaluator{values};
    const auto lhs = evaluator.evaluate(system->equations()[0].lhs);
    ASSERT_TRUE(lhs.has_value());

    EXPECT_NEAR(*lhs, std::cos(angle), kTolerance);
    EXPECT_NEAR(*lhs, ik::constantValue(system->equations()[0].rhs), kTolerance);
}

TEST(DirectionalConstraintsTest, BuildsKr4PositionAndAngleSystem)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    const ConstraintBuilder builder;
    const auto system = builder.build(
        robot->fk, robot->chain,
        PositionAndDirectionAngleTarget{Vector3{0.4, 0.1, 0.6}, Vector3{0.0, 0.0, -1.0}, 0.5});

    ASSERT_TRUE(system.has_value()) << system.error().message;
    EXPECT_EQ(system->taskKind(), IkTaskKind::PositionAndDirectionAngle);
    EXPECT_EQ(system->equations().size(), 4u);
    EXPECT_EQ(system->unknowns().size(), 6u);
}
