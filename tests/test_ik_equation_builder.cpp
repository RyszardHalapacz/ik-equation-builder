#include <gtest/gtest.h>

#include "ik_equations/IkEquationBuilder.hpp"

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>

using kinemaforge::ik::Expression;
using kinemaforge::ik::IkEquationBuilder;
using kinemaforge::ik::IkEquationBuilderErrorCode;
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
