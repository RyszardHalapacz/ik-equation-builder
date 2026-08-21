#include <gtest/gtest.h>

#include "ik_equations/IkEquationBuilder.hpp"

#include "ik_equations/symbolic/ExpressionEvaluator.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>

using kinemaforge::ik::ConstraintBuilderErrorCode;
using kinemaforge::ik::Expression;
using kinemaforge::ik::FixedRigidTransform;
using kinemaforge::ik::IkEquationBuilder;
using kinemaforge::ik::IkEquationBuilderErrorCode;
using kinemaforge::ik::IkTaskKind;
using kinemaforge::ik::PoseTarget;
using kinemaforge::ik::PositionTarget;
using kinemaforge::ik::TargetValidationErrorCode;
using kinemaforge::ik::ToolDirectionTarget;
using kinemaforge::ik::Vector3;
using kinemaforge::ik::sameNode;
using kinemaforge::ik::KinematicChainError;
using kinemaforge::ik::SymbolicTransform;
using kinemaforge::ik::SymbolNode;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::structurallyEqual;

namespace {

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

bool containsSymbol(const Expression& expression, std::string_view name)
{
    return std::visit(
        [name](const auto& node) -> bool {
            using Node = std::decay_t<decltype(node)>;
            if constexpr (std::is_same_v<Node, SymbolNode>)
                return node.name == name;
            else if constexpr (requires { node.lhs; node.rhs; })
                return containsSymbol(node.lhs, name) || containsSymbol(node.rhs, name);
            else if constexpr (requires { node.operand; })
                return containsSymbol(node.operand, name);
            else
                return false;
        },
        expression.node().value);
}

bool transformContainsSymbol(const SymbolicTransform& transform, std::string_view name)
{
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
            if (containsSymbol(transform(row, column), name)) return true;
    return false;
}

void expectHomogeneousLastRow(const SymbolicTransform& transform)
{
    EXPECT_TRUE(isZero(transform(3, 0)));
    EXPECT_TRUE(isZero(transform(3, 1)));
    EXPECT_TRUE(isZero(transform(3, 2)));
    EXPECT_TRUE(isOne(transform(3, 3)));
}

// Drives the whole pipeline the way README shows it.
void expectBuildsThroughFacade(IkEquationBuilder& builder, const char* fileName)
{
    ASSERT_TRUE(builder.loadRobotModel(urdfPath(fileName)).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    const auto* chain = builder.kinematicChain();
    ASSERT_NE(chain, nullptr);
    EXPECT_EQ(chain->joints.size(), 7u);

    const auto* fk = builder.forwardKinematics();
    ASSERT_NE(fk, nullptr);

    for (const auto& joint : chain->joints)
        if (joint.variable)
        {
            SCOPED_TRACE(joint.variable->name);
            EXPECT_TRUE(transformContainsSymbol(*fk, joint.variable->name));
        }

    expectHomogeneousLastRow(*fk);
}

} // namespace

// --- happy path -----------------------------------------------------

TEST(IkEquationBuilderTest, LoadsRobotModel)
{
    IkEquationBuilder builder;

    EXPECT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());

    // Loading a model alone produces nothing downstream.
    EXPECT_EQ(builder.kinematicChain(), nullptr);
    EXPECT_EQ(builder.forwardKinematics(), nullptr);
}

TEST(IkEquationBuilderTest, SelectsChainAfterLoadingModel)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());

    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());

    const auto* chain = builder.kinematicChain();
    ASSERT_NE(chain, nullptr);
    EXPECT_EQ(chain->joints.size(), 7u);
    EXPECT_EQ(builder.forwardKinematics(), nullptr);
}

TEST(IkEquationBuilderTest, BuildsForwardKinematicsEndToEnd)
{
    // Exactly the sequence README documents as the public entry point.
    IkEquationBuilder builder;

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    ASSERT_NE(builder.forwardKinematics(), nullptr);
}

TEST(IkEquationBuilderTest, BuildsKr4ThroughFacade)
{
    IkEquationBuilder builder;
    expectBuildsThroughFacade(builder, "kr4_r600.urdf");
}

TEST(IkEquationBuilderTest, BuildsKr640ThroughFacade)
{
    IkEquationBuilder builder;
    expectBuildsThroughFacade(builder, "kr640.urdf");
}

// --- calls out of order ---------------------------------------------

TEST(IkEquationBuilderTest, RejectsChainSelectionBeforeLoadingModel)
{
    IkEquationBuilder builder;

    const auto result = builder.selectChain("base_link", "tool0");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::RobotModelNotLoaded);
    EXPECT_FALSE(result.error().chainError.has_value());
}

TEST(IkEquationBuilderTest, RejectsForwardKinematicsBeforeSelectingChain)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());

    const auto result = builder.buildForwardKinematics();

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::KinematicChainNotSelected);
    // The other side of the chainError invariant.
    EXPECT_FALSE(result.error().chainError.has_value());
}

TEST(IkEquationBuilderTest, RejectsChainAccessBeforeSelection)
{
    // An accessor reports absence with nullptr, not with an error code -- it
    // performs no operation that could fail in more than one way.
    IkEquationBuilder builder;
    EXPECT_EQ(builder.kinematicChain(), nullptr);

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    EXPECT_EQ(builder.kinematicChain(), nullptr);
}

TEST(IkEquationBuilderTest, RejectsForwardKinematicsAccessBeforeBuild)
{
    IkEquationBuilder builder;
    EXPECT_EQ(builder.forwardKinematics(), nullptr);

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    EXPECT_EQ(builder.forwardKinematics(), nullptr);
}

// --- invalidation ---------------------------------------------------

TEST(IkEquationBuilderTest, LoadingNewRobotClearsChain)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_NE(builder.kinematicChain(), nullptr);

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr4_r600.urdf")).has_value());

    // A chain names links of one specific robot.
    EXPECT_EQ(builder.kinematicChain(), nullptr);
}

TEST(IkEquationBuilderTest, LoadingNewRobotClearsForwardKinematics)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());
    ASSERT_NE(builder.forwardKinematics(), nullptr);

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr4_r600.urdf")).has_value());

    EXPECT_EQ(builder.forwardKinematics(), nullptr);
}

TEST(IkEquationBuilderTest, SelectingNewChainClearsForwardKinematics)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr4_r600.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());
    ASSERT_NE(builder.forwardKinematics(), nullptr);

    // base_link -> base is a different, valid chain of the same robot.
    ASSERT_TRUE(builder.selectChain("base_link", "base").has_value());

    EXPECT_EQ(builder.forwardKinematics(), nullptr);
    ASSERT_NE(builder.kinematicChain(), nullptr);
    EXPECT_EQ(builder.kinematicChain()->joints.size(), 1u);
}

// --- state preserved on failure -------------------------------------

TEST(IkEquationBuilderTest, FailedLoadPreservesPreviousState)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    // Captured before the failing call: asserting only "still not null" would
    // not prove the promise, which is that the object is *untouched*. Pointer
    // identity also pins that a failed operation does not invalidate pointers
    // handed out earlier.
    const auto* chainBefore = builder.kinematicChain();
    const auto* fkBefore = builder.forwardKinematics();
    ASSERT_NE(chainBefore, nullptr);
    ASSERT_NE(fkBefore, nullptr);

    const auto result = builder.loadRobotModel(urdfPath("does_not_exist.urdf"));

    ASSERT_FALSE(result.has_value());
    // Clearing before loading would leave a failed call worse off than no call.
    EXPECT_EQ(builder.kinematicChain(), chainBefore);
    EXPECT_EQ(builder.forwardKinematics(), fkBefore);
}

TEST(IkEquationBuilderTest, FailedChainSelectionPreservesPreviousState)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr4_r600.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    const auto* chainBefore = builder.kinematicChain();
    const auto* fkBefore = builder.forwardKinematics();
    ASSERT_NE(chainBefore, nullptr);
    ASSERT_NE(fkBefore, nullptr);

    const auto result = builder.selectChain("no_such_link", "tool0");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(builder.kinematicChain(), chainBefore);
    EXPECT_EQ(builder.forwardKinematics(), fkBefore);
    EXPECT_EQ(builder.kinematicChain()->joints.size(), 7u);
}

TEST(IkEquationBuilderTest, PropagatesChainBuilderError)
{
    // flange and tool0 both exist in kr4_r600.urdf but hang off link_6 as
    // siblings, so there is no downward path between them. The typed code must
    // survive the facade rather than be flattened into a string.
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr4_r600.urdf")).has_value());

    const auto result = builder.selectChain("flange", "tool0");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::ChainBuildFailed);
    ASSERT_TRUE(result.error().chainError.has_value());
    EXPECT_EQ(*result.error().chainError, KinematicChainError::NoPathFound);
}

TEST(IkEquationBuilderTest, ReportsUrdfLoadFailure)
{
    // The message is all that survives a loader failure -- UrdfModelLoader
    // consumes its structured LoadError and throws a string. So the message
    // being non-empty is the whole diagnostic, and needs pinning.
    IkEquationBuilder builder;

    const auto result = builder.loadRobotModel(urdfPath("does_not_exist.urdf"));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::UrdfLoadFailed);
    EXPECT_FALSE(result.error().message.empty());
    EXPECT_FALSE(result.error().chainError.has_value());
}

// --- reuse ----------------------------------------------------------

TEST(IkEquationBuilderTest, ReusesFacadeForSecondRobot)
{
    // The invalidation tests above check one rule each; this checks that their
    // composition leaves a coherent object rather than a mixture of two robots.
    // It is also the most likely real usage.
    IkEquationBuilder builder;

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr4_r600.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());
    ASSERT_NE(builder.forwardKinematics(), nullptr);

    // Copied out: the pointer itself is invalidated by the next successful
    // call, exactly as the accessor contract says.
    const SymbolicTransform kr4Fk = *builder.forwardKinematics();

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    const auto* kr640Fk = builder.forwardKinematics();
    ASSERT_NE(kr640Fk, nullptr);

    bool differs = false;
    for (std::size_t row = 0; row < 4 && !differs; ++row)
        for (std::size_t column = 0; column < 4 && !differs; ++column)
            differs = !structurallyEqual(kr4Fk(row, column), (*kr640Fk)(row, column));

    EXPECT_TRUE(differs) << "two different robots produced the same transform";
}


// --- F2.7: URDF + TCP + target -> IkEquationSystem -------------------

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr std::size_t kTranslationColumn = 3;

// Everything up to a symbolic FK of the chain tip. No TCP: that is the other
// legal mode, and several tests below depend on it staying that way.
void prepareChainTip(IkEquationBuilder& builder, const char* fileName)
{
    ASSERT_TRUE(builder.loadRobotModel(urdfPath(fileName)).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());
}

kinemaforge::ik::SymbolValues zeroConfiguration(const IkEquationBuilder& builder)
{
    kinemaforge::ik::SymbolValues values;
    const auto* chain = builder.kinematicChain();
    if (chain == nullptr)
    {
        ADD_FAILURE() << "no chain";
        return values;
    }

    double angle = 0.2;
    for (const auto& joint : chain->joints)
        if (joint.variable)
        {
            values[joint.variable->name] = angle;
            angle += 0.15;
        }
    return values;
}

} // namespace

TEST(IkEquationBuilderTest, BuildsKr640PositionIkEquationsFromUrdf)
{
    IkEquationBuilder builder;
    prepareChainTip(builder, "kr640.urdf");

    const auto system = builder.buildEquationSystem(PositionTarget{Vector3{2.0, 0.5, 1.5}});

    ASSERT_TRUE(system.has_value()) << system.error().message;
    EXPECT_EQ(system->taskKind(), IkTaskKind::Position);
    EXPECT_EQ(system->equations().size(), 3u);
    EXPECT_EQ(system->unknowns().size(), 6u);
}

TEST(IkEquationBuilderTest, BuildsKr4PositionIkEquationsFromUrdf)
{
    IkEquationBuilder builder;
    prepareChainTip(builder, "kr4_r600.urdf");

    const auto system = builder.buildEquationSystem(PositionTarget{Vector3{0.4, 0.1, 0.6}});

    ASSERT_TRUE(system.has_value()) << system.error().message;
    EXPECT_EQ(system->equations().size(), 3u);
    EXPECT_EQ(system->unknowns().size(), 6u);
}

TEST(IkEquationBuilderTest, BuildsFullPoseEquationsWithTcp)
{
    IkEquationBuilder builder;
    prepareChainTip(builder, "kr640.urdf");
    ASSERT_TRUE(builder.setTcp(FixedRigidTransform{{0.0, 0.0, 0.120}, {0.0, 0.0, 0.0}})
                    .has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const PoseTarget target{Vector3{2.0, 0.5, 1.5},
                            kinemaforge::ik::RotationMatrix3{{{{1.0, 0.0, 0.0},
                                                               {0.0, 1.0, 0.0},
                                                               {0.0, 0.0, 1.0}}}}};

    const auto system = builder.buildEquationSystem(target);

    ASSERT_TRUE(system.has_value()) << system.error().message;
    EXPECT_EQ(system->taskKind(), IkTaskKind::Pose);
    EXPECT_EQ(system->equations().size(), 12u);
}

TEST(IkEquationBuilderTest, BuildsDirectionSystemThroughFacade)
{
    // One of the F2.6 task kinds, so the IkTarget variant is genuinely routed
    // rather than only the two the other tests happen to use.
    IkEquationBuilder builder;
    prepareChainTip(builder, "kr640.urdf");

    const auto system =
        builder.buildEquationSystem(ToolDirectionTarget{Vector3{0.0, 0.0, -1.0}});

    ASSERT_TRUE(system.has_value()) << system.error().message;
    EXPECT_EQ(system->taskKind(), IkTaskKind::ToolDirection);
    EXPECT_EQ(system->equations().size(), 3u);
}

TEST(IkEquationBuilderTest, ReusesFacadeForDifferentRobotAndTarget)
{
    // The snapshot contract, and the only test where the difference between a
    // returned value and an accessor is visible from outside: the first system
    // is kept across a full reload of the facade and must be untouched.
    IkEquationBuilder builder;
    prepareChainTip(builder, "kr640.urdf");

    const auto first = builder.buildEquationSystem(PositionTarget{Vector3{2.0, 0.5, 1.5}});
    ASSERT_TRUE(first.has_value());

    const auto* firstLhs = &first->equations()[0].lhs;
    const double firstRhs = kinemaforge::ik::constantValue(first->equations()[0].rhs);

    prepareChainTip(builder, "kr4_r600.urdf");
    const auto second = builder.buildEquationSystem(PositionTarget{Vector3{0.4, 0.1, 0.6}});
    ASSERT_TRUE(second.has_value());

    // The facade has been reloaded twice over; the first system does not care.
    EXPECT_EQ(first->equations().size(), 3u);
    EXPECT_DOUBLE_EQ(kinemaforge::ik::constantValue(first->equations()[0].rhs), firstRhs);
    EXPECT_EQ(&first->equations()[0].lhs, firstLhs);
    EXPECT_FALSE(sameNode(first->equations()[0].lhs, second->equations()[0].lhs));
}

// --- which transform ------------------------------------------------

TEST(IkEquationBuilderTest, UsesChainTipTransformWhenNoTcpIsSet)
{
    IkEquationBuilder builder;
    prepareChainTip(builder, "kr640.urdf");

    const auto system = builder.buildEquationSystem(PositionTarget{Vector3{2.0, 0.5, 1.5}});
    ASSERT_TRUE(system.has_value());

    const auto* fk = builder.forwardKinematics();
    ASSERT_NE(fk, nullptr);

    // By node identity, not by value: the question is WHICH transform was used.
    for (std::size_t row = 0; row < 3; ++row)
        EXPECT_TRUE(sameNode(system->equations()[row].lhs, (*fk)(row, kTranslationColumn)));
}

TEST(IkEquationBuilderTest, UsesTcpTransformWhenTcpIsSet)
{
    IkEquationBuilder builder;
    prepareChainTip(builder, "kr640.urdf");
    ASSERT_TRUE(builder.setTcp(FixedRigidTransform{{0.0, 0.0, 0.120}, {0.0, 0.0, 0.0}})
                    .has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto system = builder.buildEquationSystem(PositionTarget{Vector3{2.0, 0.5, 1.5}});
    ASSERT_TRUE(system.has_value());

    const auto* tcpFk = builder.tcpForwardKinematics();
    ASSERT_NE(tcpFk, nullptr);

    for (std::size_t row = 0; row < 3; ++row)
        EXPECT_TRUE(sameNode(system->equations()[row].lhs, (*tcpFk)(row, kTranslationColumn)));
}

TEST(IkEquationBuilderTest, RejectsTargetWhenTcpForwardKinematicsMissing)
{
    // The no-fallback rule. A helpful implementation would reach for the
    // chain-tip transform here and return a system wrong by the tool offset.
    IkEquationBuilder builder;
    prepareChainTip(builder, "kr640.urdf");
    ASSERT_TRUE(builder.setTcp(FixedRigidTransform{{0.0, 0.0, 0.120}, {0.0, 0.0, 0.0}})
                    .has_value());
    // deliberately no buildTcpForwardKinematics()

    const auto system = builder.buildEquationSystem(PositionTarget{Vector3{2.0, 0.5, 1.5}});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, IkEquationBuilderErrorCode::TcpForwardKinematicsNotBuilt);
}

TEST(IkEquationBuilderTest, ChangingTcpChangesTheEquationSystem)
{
    // Semantic, not structural. Comparing node identity would pass for the wrong
    // reason: buildTcpForwardKinematics builds a fresh tree every call, so two
    // IDENTICAL TCPs already give different nodes -- and so would an
    // implementation that ignored the TCP value entirely.
    //
    // Substituting the same configuration into both systems asks the question
    // that matters: did the MEANING change? Two TCPs 120 mm apart along the
    // tool's own Z must put the constrained point 120 mm apart, whatever the
    // orientation of the tool is at that configuration -- a rotation moves the
    // offset but preserves its length.
    IkEquationBuilder builder;
    prepareChainTip(builder, "kr640.urdf");
    const auto values = zeroConfiguration(builder);

    auto positionAt = [&](double offset) {
        EXPECT_TRUE(builder.setTcp(FixedRigidTransform{{0.0, 0.0, offset}, {0.0, 0.0, 0.0}})
                        .has_value());
        EXPECT_TRUE(builder.buildTcpForwardKinematics().has_value());

        const auto system = builder.buildEquationSystem(PositionTarget{Vector3{2.0, 0.5, 1.5}});
        EXPECT_TRUE(system.has_value());

        kinemaforge::ik::ExpressionEvaluator evaluator{values};
        std::array<double, 3> point{};
        for (std::size_t row = 0; row < 3; ++row)
        {
            const auto value = evaluator.evaluate(system->equations()[row].lhs);
            EXPECT_TRUE(value.has_value());
            point[row] = value.value_or(0.0);
        }

        std::array<double, 3> rhs{};
        for (std::size_t row = 0; row < 3; ++row)
            rhs[row] = kinemaforge::ik::constantValue(system->equations()[row].rhs);

        return std::pair{point, rhs};
    };

    const auto [pointA, rhsA] = positionAt(0.100);
    const auto [pointB, rhsB] = positionAt(0.220);

    const double distance = std::hypot(pointB[0] - pointA[0], pointB[1] - pointA[1],
                                       pointB[2] - pointA[2]);
    EXPECT_NEAR(distance, 0.120, 1e-12);

    // The target did not change, so the right-hand sides must be identical.
    for (std::size_t row = 0; row < 3; ++row)
        EXPECT_DOUBLE_EQ(rhsA[row], rhsB[row]);
}

// --- prerequisites and errors ---------------------------------------

TEST(IkEquationBuilderTest, ReportsFirstMissingStep)
{
    const PositionTarget target{Vector3{1.0, 0.0, 1.0}};

    {
        SCOPED_TRACE("no chain selected");
        IkEquationBuilder builder;
        const auto system = builder.buildEquationSystem(target);
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationBuilderErrorCode::KinematicChainNotSelected);
    }
    {
        SCOPED_TRACE("chain selected, no forward kinematics");
        IkEquationBuilder builder;
        ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
        ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());

        const auto system = builder.buildEquationSystem(target);
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationBuilderErrorCode::ForwardKinematicsNotBuilt);
    }
    {
        SCOPED_TRACE("TCP set, its transform not built");
        IkEquationBuilder builder;
        prepareChainTip(builder, "kr640.urdf");
        ASSERT_TRUE(builder.setTcp(FixedRigidTransform{{0.0, 0.0, 0.120}, {0.0, 0.0, 0.0}})
                        .has_value());

        const auto system = builder.buildEquationSystem(target);
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code,
                  IkEquationBuilderErrorCode::TcpForwardKinematicsNotBuilt);
    }
}

TEST(IkEquationBuilderTest, PreservesConstraintBuilderError)
{
    IkEquationBuilder builder;
    prepareChainTip(builder, "kr640.urdf");

    const auto* chainBefore = builder.kinematicChain();
    const auto* fkBefore = builder.forwardKinematics();

    const auto system = builder.buildEquationSystem(PositionTarget{Vector3{kNaN, 0.0, 0.0}});

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, IkEquationBuilderErrorCode::ConstraintBuildFailed);

    // Three layers, none of them flattened into a string.
    ASSERT_TRUE(system.error().constraintError.has_value());
    EXPECT_EQ(system.error().constraintError->code, ConstraintBuilderErrorCode::InvalidTarget);
    ASSERT_TRUE(system.error().constraintError->targetError.has_value());
    EXPECT_EQ(system.error().constraintError->targetError->code,
              TargetValidationErrorCode::NonFinitePosition);

    // The invariant, from both sides.
    EXPECT_FALSE(system.error().chainError.has_value());

    IkEquationBuilder empty;
    const auto prerequisite = empty.buildEquationSystem(PositionTarget{Vector3{}});
    ASSERT_FALSE(prerequisite.has_value());
    EXPECT_FALSE(prerequisite.error().constraintError.has_value());

    // The query is const, and nothing moved.
    EXPECT_EQ(builder.kinematicChain(), chainBefore);
    EXPECT_EQ(builder.forwardKinematics(), fkBefore);
    EXPECT_EQ(builder.tcp(), nullptr);
}

TEST(IkEquationBuilderTest, ChangingChainClearsTcpAndRequiresFreshForwardKinematics)
{
    IkEquationBuilder builder;
    prepareChainTip(builder, "kr640.urdf");
    ASSERT_TRUE(builder.setTcp(FixedRigidTransform{{0.0, 0.0, 0.120}, {0.0, 0.0, 0.0}})
                    .has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const PositionTarget target{Vector3{2.0, 0.5, 1.5}};
    ASSERT_TRUE(builder.buildEquationSystem(target).has_value());

    ASSERT_TRUE(builder.selectChain("base_link", "link_6").has_value());

    EXPECT_EQ(builder.tcp(), nullptr);
    EXPECT_EQ(builder.tcpForwardKinematics(), nullptr);
    EXPECT_EQ(builder.forwardKinematics(), nullptr);

    {
        const auto system = builder.buildEquationSystem(target);
        ASSERT_FALSE(system.has_value());
        // Not TcpForwardKinematicsNotBuilt: there is no TCP any more.
        EXPECT_EQ(system.error().code, IkEquationBuilderErrorCode::ForwardKinematicsNotBuilt);
    }

    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    // Having no TCP is not a missing prerequisite -- it is the other legal mode.
    const auto system = builder.buildEquationSystem(target);
    ASSERT_TRUE(system.has_value()) << system.error().message;
    EXPECT_EQ(system->equations().size(), 3u);
}
