#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ConstraintBuilder.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace ik = kinemaforge::ik;

using ik::CartesianComponent;
using ik::ConstraintBuilder;
using ik::ConstraintBuilderErrorCode;
using ik::Expression;
using ik::ExpressionFactory;
using ik::IkTaskKind;
using ik::PositionEquationSource;
using ik::PositionTarget;
using ik::PoseTarget;
using ik::SymbolicTransform;
using ik::TargetValidationErrorCode;
using ik::sameNode;

namespace {

constexpr std::size_t kTranslationColumn = 3;

// A chain that declares the given unknowns and nothing else. The joints carry
// no geometry: every test here feeds the builder a transform built by hand or
// by the real pipeline, so the chain is only ever asked what its unknowns are.
//
// Numbering matches KinematicChainBuilder: "q1" has index 1.
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

// Identity with the translation column replaced by the three given cells.
SymbolicTransform transformWithTranslation(Expression x, Expression y, Expression z)
{
    auto transform = SymbolicTransform::identity();
    transform(0, kTranslationColumn) = std::move(x);
    transform(1, kTranslationColumn) = std::move(y);
    transform(2, kTranslationColumn) = std::move(z);
    return transform;
}

// Identity with the rotation block replaced, translation left at the identity's
// zeros. Cells are handed in row-major order, like everything else here.
SymbolicTransform transformWithRotation(const std::array<Expression, 9>& cells)
{
    auto transform = SymbolicTransform::identity();
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            transform(row, column) = cells[row * 3 + column];
    return transform;
}

ik::RotationMatrix3 identityRotation()
{
    return ik::RotationMatrix3{{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}}};
}

// A genuine rotation about Z, not a hand-rounded one: the values come from
// std::cos/std::sin, so the target is orthogonal to machine precision and these
// tests say nothing about the tolerance question -- that one is the solver's.
ik::RotationMatrix3 rotationAboutZ(double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return ik::RotationMatrix3{{{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}}};
}

struct LoadedRobot
{
    ik::KinematicChain chain;
    SymbolicTransform fk;
};

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

// --- shape of the result --------------------------------------------

TEST(ConstraintBuilderTest, BuildsThreePositionEquations)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q2"), factory.constant(0.5));
    const auto chain = chainDeclaring({"q1", "q2"});

    const auto system = builder.build(transform, chain, PositionTarget{{1.0, 2.0, 3.0}});

    ASSERT_TRUE(system.has_value()) << static_cast<int>(system.error().code);
    EXPECT_EQ(system->taskKind(), IkTaskKind::Position);
    EXPECT_EQ(system->equations().size(), 3u);
}

TEST(ConstraintBuilderTest, EquationsAreOrderedXYZ)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      PositionTarget{{1.0, 2.0, 3.0}});

    ASSERT_TRUE(system.has_value());
    const auto& equations = system->equations();
    EXPECT_EQ(std::get<PositionEquationSource>(equations[0].source).component,
              CartesianComponent::X);
    EXPECT_EQ(std::get<PositionEquationSource>(equations[1].source).component,
              CartesianComponent::Y);
    EXPECT_EQ(std::get<PositionEquationSource>(equations[2].source).component,
              CartesianComponent::Z);
}

// --- where each side comes from -------------------------------------

TEST(ConstraintBuilderTest, UsesTcpTranslationColumn)
{
    // The builder reads cells (0..2, 3) of the transform it is given -- which
    // is the TCP transform whenever the caller passes one. It cannot tell.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto x = factory.symbol("q1");
    const auto y = factory.symbol("q2");
    const auto z = factory.constant(0.25);
    const auto transform = transformWithTranslation(x, y, z);

    const auto system = builder.build(transform, chainDeclaring({"q1", "q2"}),
                                      PositionTarget{{0.0, 0.0, 0.0}});

    ASSERT_TRUE(system.has_value());
    EXPECT_TRUE(sameNode(system->equations()[0].lhs, x));
    EXPECT_TRUE(sameNode(system->equations()[1].lhs, y));
    EXPECT_TRUE(sameNode(system->equations()[2].lhs, z));
}

TEST(ConstraintBuilderTest, DoesNotSubtractTargetFromLhs)
{
    // sameNode, not structural equality: the left-hand side must be the very
    // cell of the transform, not a rebuilt or rearranged expression. Equation
    // stores lhs = rhs as given; forming lhs - rhs would erase which side came
    // from the robot and which from the task.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto x = factory.symbol("q1");
    const auto transform = transformWithTranslation(x, x, x);

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      PositionTarget{{7.0, 8.0, 9.0}});

    ASSERT_TRUE(system.has_value());
    for (const auto& equation : system->equations())
    {
        EXPECT_TRUE(sameNode(equation.lhs, x));
        EXPECT_EQ(equation.rhs.type(), ik::ExpressionType::Constant);
    }
}

TEST(ConstraintBuilderTest, UsesTargetValuesOnRightHandSide)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      PositionTarget{{1.5, -2.5, 0.0}});

    ASSERT_TRUE(system.has_value());
    const auto& equations = system->equations();
    EXPECT_DOUBLE_EQ(ik::constantValue(equations[0].rhs), 1.5);
    EXPECT_DOUBLE_EQ(ik::constantValue(equations[1].rhs), -2.5);
    EXPECT_TRUE(ik::isZero(equations[2].rhs)); // 0.0 reaches the shared zero node
}

TEST(ConstraintBuilderTest, PreservesJointSymbols)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    const ConstraintBuilder builder;
    const auto system = builder.build(robot->fk, robot->chain, PositionTarget{{1.0, 0.0, 1.0}});

    ASSERT_TRUE(system.has_value());
    ASSERT_EQ(system->unknowns().size(), 6u);
    for (std::size_t position = 0; position < 6; ++position)
    {
        EXPECT_EQ(system->unknowns()[position].name, "q" + std::to_string(position + 1));
        EXPECT_EQ(system->unknowns()[position].index, position + 1);
    }
}

// --- the symbol invariant, both directions --------------------------

TEST(ConstraintBuilderTest, RejectsUndeclaredSymbol)
{
    // The shape of review issue 10: an equation depending on a symbol the
    // system never declares. Nothing downstream would catch it --
    // IkEquationSystem::create cannot collect symbols.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q999"), factory.symbol("q2"));

    const auto system = builder.build(transform, chainDeclaring({"q1", "q2"}),
                                      PositionTarget{{0.0, 0.0, 0.0}});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::UndeclaredSymbol);
    EXPECT_NE(system.error().message.find("q999"), std::string::npos);
}

TEST(ConstraintBuilderTest, AcceptsUnknownAbsentFromEquations)
{
    // The inclusion holds one way only. A declared unknown that does not appear
    // is normal: a wrist roll with no tool offset does not move the TCP
    // position. The system is then positionally underdetermined, which is a
    // correct answer for PositionOnly, not a fault.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.constant(1.0));

    const auto system = builder.build(transform, chainDeclaring({"q1", "q2"}),
                                      PositionTarget{{0.0, 0.0, 1.0}});

    ASSERT_TRUE(system.has_value());
    EXPECT_EQ(system->unknowns().size(), 2u);
}

// --- refusals -------------------------------------------------------

TEST(ConstraintBuilderTest, RejectsNonFinitePositionTarget)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    const PositionTarget target{{0.0, std::numeric_limits<double>::infinity(), 0.0}};

    const auto system = builder.build(transform, chainDeclaring({"q1"}), target);

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::InvalidTarget);
    ASSERT_TRUE(system.error().targetError.has_value());
    EXPECT_EQ(system.error().targetError->code, TargetValidationErrorCode::NonFinitePosition);
    EXPECT_FALSE(system.error().systemError.has_value());
}

TEST(ConstraintBuilderTest, PreservesSystemRejection)
{
    // A chain with no joints declares no unknowns, and an identity transform
    // has no symbols in its translation column -- so the symbol check passes
    // vacuously and the refusal comes from the layer below, unchanged.
    const ConstraintBuilder builder;
    const auto transform = SymbolicTransform::identity();
    const ik::KinematicChain chain;

    const auto system = builder.build(transform, chain, PositionTarget{{0.0, 0.0, 0.0}});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::SystemRejected);
    EXPECT_FALSE(system.error().targetError.has_value());

    ASSERT_TRUE(system.error().systemError.has_value());
    EXPECT_EQ(system.error().systemError->code, ik::IkEquationSystemErrorCode::NoUnknowns);
}

// --- end to end -----------------------------------------------------

TEST(ConstraintBuilderTest, BuildsKr640PositionEquationSystem)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    const ConstraintBuilder builder;
    const auto system = builder.build(robot->fk, robot->chain, PositionTarget{{2.0, 0.5, 1.5}});

    ASSERT_TRUE(system.has_value());
    EXPECT_EQ(system->taskKind(), IkTaskKind::Position);
    EXPECT_EQ(system->equations().size(), 3u);
    EXPECT_EQ(system->unknowns().size(), 6u);
    EXPECT_TRUE(sameNode(system->equations()[0].lhs, robot->fk(0, kTranslationColumn)));
}

TEST(ConstraintBuilderTest, BuildsKr4PositionEquationSystem)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    const ConstraintBuilder builder;
    const auto system = builder.build(robot->fk, robot->chain, PositionTarget{{0.4, 0.1, 0.6}});

    ASSERT_TRUE(system.has_value());
    EXPECT_EQ(system->equations().size(), 3u);
    EXPECT_EQ(system->unknowns().size(), 6u);
}

// --- FullPose: shape of the system ----------------------------------

TEST(ConstraintBuilderTest, BuildsFullPoseEquationSystem)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q2"), factory.constant(0.5));
    const PoseTarget target{{1.0, 2.0, 3.0}, rotationAboutZ(0.7)};

    const auto system = builder.build(transform, chainDeclaring({"q1", "q2"}), target);

    ASSERT_TRUE(system.has_value()) << static_cast<int>(system.error().code);
    EXPECT_EQ(system->taskKind(), IkTaskKind::Pose);
    EXPECT_EQ(system->equations().size(), 12u);
}

TEST(ConstraintBuilderTest, IncludesPositionAndOrientationConstraints)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    const PoseTarget target{{0.0, 0.0, 0.0}, identityRotation()};

    const auto system = builder.build(transform, chainDeclaring({"q1"}), target);
    ASSERT_TRUE(system.has_value());

    std::size_t positionCount = 0;
    std::size_t orientationCount = 0;
    for (const auto& equation : system->equations())
    {
        if (ik::kindOf(equation.source) == ik::EquationKind::Position)
            ++positionCount;
        else
            ++orientationCount;
    }

    EXPECT_EQ(positionCount, 3u);
    EXPECT_EQ(orientationCount, 9u);
}

TEST(ConstraintBuilderTest, OrientationEquationsAreRowMajor)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    const PoseTarget target{{0.0, 0.0, 0.0}, identityRotation()};

    const auto system = builder.build(transform, chainDeclaring({"q1"}), target);
    ASSERT_TRUE(system.has_value());

    for (std::size_t cell = 0; cell < 9; ++cell)
    {
        const auto& source =
            std::get<ik::OrientationEquationSource>(system->equations()[3 + cell].source);
        EXPECT_EQ(source.row(), cell / 3);
        EXPECT_EQ(source.column(), cell % 3);
    }
}

// --- FullPose: where each side comes from ---------------------------

TEST(ConstraintBuilderTest, UsesRotationBlockCells)
{
    // Every orientation lhs is the very cell of the given transform -- the same
    // rigour as DoesNotSubtractTargetFromLhs, applied to the rotation block.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    std::array<Expression, 9> cells{};
    for (auto& cell : cells)
        cell = factory.symbol("q1"); // nine distinct nodes, one name

    const auto transform = transformWithRotation(cells);
    const PoseTarget target{{0.0, 0.0, 0.0}, identityRotation()};

    const auto system = builder.build(transform, chainDeclaring({"q1"}), target);
    ASSERT_TRUE(system.has_value());

    for (std::size_t cell = 0; cell < 9; ++cell)
        EXPECT_TRUE(sameNode(system->equations()[3 + cell].lhs, cells[cell]))
            << "orientation cell " << cell;
}

TEST(ConstraintBuilderTest, UsesTargetRotationValuesOnRightHandSide)
{
    // The heart of F2.5: R(q)(i,j) = R_target(i,j). Without this test an
    // implementation writing constant(0.0) nine times, or one indexing
    // values[column][row], passes every other test in this file.
    //
    // rotationAboutZ(0.7) is chosen for that second reason: R(0,1) = -sin and
    // R(1,0) = +sin differ in sign, so a transposed read cannot slip through.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    const PoseTarget target{{0.0, 0.0, 0.0}, rotationAboutZ(0.7)};

    const auto system = builder.build(transform, chainDeclaring({"q1"}), target);
    ASSERT_TRUE(system.has_value());

    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
        {
            const auto& equation = system->equations()[3 + row * 3 + column];
            EXPECT_DOUBLE_EQ(ik::constantValue(equation.rhs),
                             target.orientation.values[row][column])
                << "cell (" << row << ", " << column << ")";
        }
}

TEST(ConstraintBuilderTest, PositionEquationsMatchPositionOnlyBuild)
{
    // Same transform, same position: the three position equations must be the
    // same equations, not merely equivalent ones.
    //
    // lhs by node identity, source by sameSource, rhs BY VALUE: constant(1.5)
    // builds a fresh node on every call, and only zero reaches the shared node,
    // so a sameNode expectation on rhs would pass by accident at best.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q2"), factory.constant(0.5));
    const auto chain = chainDeclaring({"q1", "q2"});
    const ik::Vector3 position{1.5, -2.5, 0.75};

    const auto positionOnly = builder.build(transform, chain, PositionTarget{position});
    const auto pose = builder.build(transform, chain, PoseTarget{position, rotationAboutZ(0.7)});

    ASSERT_TRUE(positionOnly.has_value());
    ASSERT_TRUE(pose.has_value());

    for (std::size_t slot = 0; slot < 3; ++slot)
    {
        const auto& lean = positionOnly->equations()[slot];
        const auto& full = pose->equations()[slot];

        EXPECT_TRUE(sameNode(full.lhs, lean.lhs)) << "slot " << slot;
        EXPECT_TRUE(ik::sameSource(full.source, lean.source)) << "slot " << slot;
        EXPECT_DOUBLE_EQ(ik::constantValue(full.rhs), ik::constantValue(lean.rhs));
    }
}

// --- FullPose: refusals ---------------------------------------------

TEST(ConstraintBuilderTest, RejectsNonOrthogonalTargetRotation)
{
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    // Two columns at 45 degrees to each other: far outside the 1e-8 tolerance.
    const ik::RotationMatrix3 skewed{{{{1.0, 0.7071067811865476, 0.0},
                                       {0.0, 0.7071067811865476, 0.0},
                                       {0.0, 0.0, 1.0}}}};

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      PoseTarget{{0.0, 0.0, 0.0}, skewed});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::InvalidTarget);
    ASSERT_TRUE(system.error().targetError.has_value());
    EXPECT_EQ(system.error().targetError->code,
              TargetValidationErrorCode::NonOrthogonalOrientation);
}

TEST(ConstraintBuilderTest, RejectsReflectionTargetRotation)
{
    // diag(1, 1, -1) is perfectly orthogonal, so it passes step 2 and fails on
    // the determinant. There is deliberately no "reflection" code: the same
    // check also rejects a uniform scaling, which is no reflection at all.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));
    const ik::RotationMatrix3 reflection{
        {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}}}};

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      PoseTarget{{0.0, 0.0, 0.0}, reflection});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::InvalidTarget);
    ASSERT_TRUE(system.error().targetError.has_value());
    EXPECT_EQ(system.error().targetError->code,
              TargetValidationErrorCode::InvalidOrientationDeterminant);
}

TEST(ConstraintBuilderTest, RejectsNonFiniteTargetRotation)
{
    // The order "validate, then build constants" is load-bearing for the
    // rotation block exactly as it is for position: appendOrientationEquations
    // calls factory_.constant, whose precondition is a finite value. The other
    // two rejection tests use finite doubles, so only this one fails if the
    // steps are ever swapped -- and in build-werror, where assertions are live,
    // it fails by aborting rather than by returning an error.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    const auto transform = transformWithTranslation(
        factory.symbol("q1"), factory.symbol("q1"), factory.symbol("q1"));

    auto orientation = identityRotation();
    orientation.values[1][2] = std::numeric_limits<double>::infinity();

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      PoseTarget{{0.0, 0.0, 0.0}, orientation});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::InvalidTarget);
    ASSERT_TRUE(system.error().targetError.has_value());
    EXPECT_EQ(system.error().targetError->code,
              TargetValidationErrorCode::NonFiniteOrientation);
}

TEST(ConstraintBuilderTest, RejectsUndeclaredSymbolInRotationBlock)
{
    // The symbol guarantee has to grow from 3 cells to 12. A collector still
    // walking only the translation column would pass every F2.4 test and every
    // PositionOnly build, and let q999 through here.
    const ExpressionFactory factory;
    const ConstraintBuilder builder;

    std::array<Expression, 9> cells{};
    for (auto& cell : cells)
        cell = factory.constant(0.0);
    cells[4] = factory.symbol("q999");

    auto transform = transformWithRotation(cells);
    transform(0, kTranslationColumn) = factory.symbol("q1");

    const auto system = builder.build(transform, chainDeclaring({"q1"}),
                                      PoseTarget{{0.0, 0.0, 0.0}, identityRotation()});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, ConstraintBuilderErrorCode::UndeclaredSymbol);
    EXPECT_NE(system.error().message.find("q999"), std::string::npos);
}

// --- FullPose: end to end -------------------------------------------

TEST(ConstraintBuilderTest, BuildsKr640FullPoseEquationSystem)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    const ConstraintBuilder builder;
    const PoseTarget target{{2.0, 0.5, 1.5}, rotationAboutZ(0.7)};
    const auto system = builder.build(robot->fk, robot->chain, target);

    ASSERT_TRUE(system.has_value());
    EXPECT_EQ(system->taskKind(), IkTaskKind::Pose);
    EXPECT_EQ(system->equations().size(), 12u);
    EXPECT_EQ(system->unknowns().size(), 6u);
    EXPECT_TRUE(sameNode(system->equations()[3].lhs, robot->fk(0, 0)));
}

TEST(ConstraintBuilderTest, BuildsKr4FullPoseEquationSystem)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    const ConstraintBuilder builder;
    const PoseTarget target{{0.4, 0.1, 0.6}, identityRotation()};
    const auto system = builder.build(robot->fk, robot->chain, target);

    ASSERT_TRUE(system.has_value());
    EXPECT_EQ(system->equations().size(), 12u);
    EXPECT_EQ(system->unknowns().size(), 6u);
}
