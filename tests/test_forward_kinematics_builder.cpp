#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <numbers>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

using kinemaforge::ik::Expression;
using kinemaforge::ik::ForwardKinematicsBuilder;
using kinemaforge::ik::JointTransformBuilder;
using kinemaforge::ik::JointType;
using kinemaforge::ik::JointVariable;
using kinemaforge::ik::KinematicChain;
using kinemaforge::ik::KinematicJoint;
using kinemaforge::ik::SymbolicTransform;
using kinemaforge::ik::SymbolNode;
using kinemaforge::ik::Vector3;
using kinemaforge::ik::constantValue;
using kinemaforge::ik::isConstant;
using kinemaforge::ik::isIdentityTransform;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::structurallyEqual;

namespace {

constexpr double kPi = std::numbers::pi;

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

// Recursive walk. A composite node does not imply a symbol: Add(Constant,
// Constant) is composite and carries none.
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

bool containsAnySymbol(const Expression& expression)
{
    return std::visit(
        [](const auto& node) -> bool {
            using Node = std::decay_t<decltype(node)>;
            if constexpr (std::is_same_v<Node, SymbolNode>)
                return true;
            else if constexpr (requires { node.lhs; node.rhs; })
                return containsAnySymbol(node.lhs) || containsAnySymbol(node.rhs);
            else if constexpr (requires { node.operand; })
                return containsAnySymbol(node.operand);
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

bool transformContainsAnySymbol(const SymbolicTransform& transform)
{
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
            if (containsAnySymbol(transform(row, column))) return true;
    return false;
}

KinematicJoint makeJoint(JointType type, Vector3 axis, Vector3 translation, Vector3 rpy,
                         std::string variableName = "q1")
{
    KinematicJoint joint;
    joint.name = "j";
    joint.type = type;
    joint.axis = axis;
    joint.origin.translation = translation;
    joint.origin.rpy = rpy;
    if (type != JointType::Fixed)
        joint.variable = JointVariable{std::move(variableName), 1};
    return joint;
}

KinematicChain makeChain(std::vector<KinematicJoint> joints)
{
    KinematicChain chain;
    chain.baseLink = "base";
    chain.toolLink = "tool";
    chain.joints = std::move(joints);
    return chain;
}

void expectHomogeneousLastRow(const SymbolicTransform& transform)
{
    EXPECT_TRUE(isZero(transform(3, 0)));
    EXPECT_TRUE(isZero(transform(3, 1)));
    EXPECT_TRUE(isZero(transform(3, 2)));
    EXPECT_TRUE(isOne(transform(3, 3)));
}

// constantValue asserts on a non-constant, which aborts the process instead
// of reporting a failure. Check first.
void expectConstantNear(const Expression& expression, double expected)
{
    ASSERT_TRUE(isConstant(expression)) << "cell did not fold to a constant";
    EXPECT_NEAR(constantValue(expression), expected, 1e-12);
}

} // namespace

// --- contracts ------------------------------------------------------

TEST(ForwardKinematicsBuilderTest, EmptyChainReturnsIdentity)
{
    // baseLink == toolLink is success for KinematicChainBuilder, so the empty
    // product must be the identity, not an error.
    const ForwardKinematicsBuilder builder;
    const JointTransformBuilder transformBuilder;

    const auto transform = builder.build(makeChain({}), transformBuilder);

    EXPECT_TRUE(isIdentityTransform(transform));
}

TEST(ForwardKinematicsBuilderTest, BuildsSingleJointForwardKinematics)
{
    const ForwardKinematicsBuilder builder;
    const JointTransformBuilder transformBuilder;
    const auto chain = makeChain({makeJoint(JointType::Revolute, {0.0, 0.0, 1.0},
                                            {0.0, 0.0, 0.75}, {})});

    const auto transform = builder.build(chain, transformBuilder);

    EXPECT_TRUE(transformContainsSymbol(transform, "q1"));
    expectConstantNear(transform(2, 3), 0.75);
    expectHomogeneousLastRow(transform);
}

TEST(ForwardKinematicsBuilderTest, SingleJointResultMatchesJointTransformBuilder)
{
    // Pins the identity-accumulator fast path: without it the result would be
    // mathematically equal but structurally different, and this fails.
    const ForwardKinematicsBuilder builder;
    const JointTransformBuilder transformBuilder;
    const auto joint = makeJoint(JointType::Revolute, {0.0, 0.0, 1.0}, {0.1, 0.2, 0.3},
                                 {0.0, 0.0, kPi / 2.0});

    const auto viaChain = builder.build(makeChain({joint}), transformBuilder);
    const auto direct = transformBuilder.build(joint);

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            EXPECT_TRUE(structurallyEqual(viaChain(row, column), direct(row, column)));
        }
}

TEST(ForwardKinematicsBuilderTest, PreservesHomogeneousLastRow)
{
    // The row a full 4x4 product would silently destroy. Non-zero origins on
    // every joint, and a tilted axis, so no branch is trivially skipped.
    const double scale = 1.0 / std::sqrt(14.0);
    const Vector3 tilted{1.0 * scale, 2.0 * scale, 3.0 * scale};

    const ForwardKinematicsBuilder builder;
    const JointTransformBuilder transformBuilder;
    const auto chain = makeChain({
        makeJoint(JointType::Revolute, {0.0, 0.0, 1.0}, {0.1, 0.2, 0.3},
                  {kPi / 2.0, 0.0, 0.0}, "q1"),
        makeJoint(JointType::Fixed, {1.0, 0.0, 0.0}, {0.4, 0.0, 0.5}, {0.0, kPi / 2.0, 0.0}),
        makeJoint(JointType::Prismatic, tilted, {0.0, 0.6, 0.0}, {0.0, 0.0, kPi / 2.0}, "q2"),
        makeJoint(JointType::Revolute, tilted, {0.7, 0.0, 0.0}, {0.0, 0.0, 0.0}, "q3"),
    });

    expectHomogeneousLastRow(builder.build(chain, transformBuilder));
}

// --- order ----------------------------------------------------------

TEST(ForwardKinematicsBuilderTest, PreservesJointOrder)
{
    // Two pure translations would NOT work here: Trans(p1)*Trans(p2) equals
    // Trans(p2)*Trans(p1), so any order would pass. One joint must rotate.
    //
    //   Rz(pi/2) then TransX(1)  ->  displacement along +Y
    //   TransX(1) then Rz(pi/2)  ->  displacement along +X
    const ForwardKinematicsBuilder builder;
    const JointTransformBuilder transformBuilder;

    const auto rotate = makeJoint(JointType::Fixed, {1.0, 0.0, 0.0}, {}, {0.0, 0.0, kPi / 2.0});
    const auto slide = makeJoint(JointType::Fixed, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {});

    const auto rotateThenSlide = builder.build(makeChain({rotate, slide}), transformBuilder);
    const auto slideThenRotate = builder.build(makeChain({slide, rotate}), transformBuilder);

    {
        SCOPED_TRACE("rotate then slide");
        expectConstantNear(rotateThenSlide(0, 3), 0.0);
        expectConstantNear(rotateThenSlide(1, 3), 1.0);
    }
    {
        SCOPED_TRACE("slide then rotate");
        expectConstantNear(slideThenRotate(0, 3), 1.0);
        expectConstantNear(slideThenRotate(1, 3), 0.0);
    }
}

TEST(ForwardKinematicsBuilderTest, DistinguishesNonCommutingJointOrder)
{
    // Rz(q1) and Rx(q2) do not commute. Cell (1,0) is the discriminator:
    //
    //   Rz(q1)*Rx(q2) -> depends on q1 only
    //   Rx(q2)*Rz(q1) -> depends on q1 and q2
    //
    // A test asserting only "q1 and q2 both appear somewhere" would pass for
    // either order, and for any permutation of the joints.
    const ForwardKinematicsBuilder builder;
    const JointTransformBuilder transformBuilder;

    const auto aboutZ = makeJoint(JointType::Revolute, {0.0, 0.0, 1.0}, {}, {}, "q1");
    const auto aboutX = makeJoint(JointType::Revolute, {1.0, 0.0, 0.0}, {}, {}, "q2");

    const auto zThenX = builder.build(makeChain({aboutZ, aboutX}), transformBuilder);
    const auto xThenZ = builder.build(makeChain({aboutX, aboutZ}), transformBuilder);

    EXPECT_TRUE(containsSymbol(zThenX(1, 0), "q1"));
    EXPECT_FALSE(containsSymbol(zThenX(1, 0), "q2"));

    EXPECT_TRUE(containsSymbol(xThenZ(1, 0), "q1"));
    EXPECT_TRUE(containsSymbol(xThenZ(1, 0), "q2"));

    EXPECT_FALSE(structurallyEqual(zThenX(1, 0), xThenZ(1, 0)));
}

// --- content --------------------------------------------------------

TEST(ForwardKinematicsBuilderTest, IncludesFixedJoints)
{
    // A fixed joint carries a real offset; dropping it from the product would
    // leave the tool 290 mm short.
    const ForwardKinematicsBuilder builder;
    const JointTransformBuilder transformBuilder;

    const auto actuated = makeJoint(JointType::Revolute, {0.0, 0.0, 1.0}, {}, {}, "q1");
    const auto offset = makeJoint(JointType::Fixed, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.29}, {});

    const auto withOffset = builder.build(makeChain({actuated, offset}), transformBuilder);
    const auto withoutOffset = builder.build(makeChain({actuated}), transformBuilder);

    expectConstantNear(withOffset(2, 3), 0.29);
    EXPECT_TRUE(isZero(withoutOffset(2, 3)));
}

TEST(ForwardKinematicsBuilderTest, UsesAllJointVariables)
{
    const ForwardKinematicsBuilder builder;
    const JointTransformBuilder transformBuilder;
    const auto chain = makeChain({
        makeJoint(JointType::Revolute, {0.0, 0.0, 1.0}, {0.0, 0.0, 0.75}, {}, "q1"),
        makeJoint(JointType::Revolute, {0.0, 1.0, 0.0}, {0.35, 0.0, 0.0}, {}, "q2"),
        makeJoint(JointType::Prismatic, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.15}, {}, "q3"),
    });

    const auto transform = builder.build(chain, transformBuilder);

    EXPECT_TRUE(transformContainsSymbol(transform, "q1"));
    EXPECT_TRUE(transformContainsSymbol(transform, "q2"));
    EXPECT_TRUE(transformContainsSymbol(transform, "q3"));
}

TEST(ForwardKinematicsBuilderTest, DoesNotIntroduceSymbolsForFixedOnlyChain)
{
    const ForwardKinematicsBuilder builder;
    const JointTransformBuilder transformBuilder;
    const auto chain = makeChain({
        makeJoint(JointType::Fixed, {1.0, 0.0, 0.0}, {0.1, 0.2, 0.3}, {0.0, 0.0, kPi / 2.0}),
        makeJoint(JointType::Fixed, {1.0, 0.0, 0.0}, {0.4, 0.5, 0.6}, {kPi / 2.0, 0.0, 0.0}),
    });

    EXPECT_FALSE(transformContainsAnySymbol(builder.build(chain, transformBuilder)));
}

// --- real robots ----------------------------------------------------

TEST(ForwardKinematicsBuilderTest, BuildsIdentityForKr4BaseToBaseChain)
{
    // base_link -> base is one fixed joint with a zero origin, so the real
    // URDF pipeline produces an exactly identity joint transform. The direct
    // SymbolicTransform tests separately pin the rhs-identity fast path —
    // here the accumulator is still identity, so it is the lhs branch that
    // fires.
    kinemaforge::ik::UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr4_r600.urdf"));

    kinemaforge::ik::KinematicChainBuilder chainBuilder;
    const auto chain = chainBuilder.build(robot, "base_link", "base");
    ASSERT_TRUE(chain.has_value());
    ASSERT_EQ(chain->joints.size(), 1u);

    const ForwardKinematicsBuilder builder;
    const JointTransformBuilder transformBuilder;

    EXPECT_TRUE(isIdentityTransform(builder.build(*chain, transformBuilder)));
}

TEST(ForwardKinematicsBuilderTest, BuildsKr4SymbolicForwardKinematics)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr4_r600.urdf"));

    kinemaforge::ik::KinematicChainBuilder chainBuilder;
    const auto chain = chainBuilder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(chain.has_value());
    ASSERT_EQ(chain->joints.size(), 7u);

    const ForwardKinematicsBuilder builder;
    const JointTransformBuilder transformBuilder;
    const auto transform = builder.build(*chain, transformBuilder);

    for (const auto& joint : chain->joints)
        if (joint.variable)
        {
            SCOPED_TRACE(joint.variable->name);
            EXPECT_TRUE(transformContainsSymbol(transform, joint.variable->name));
        }

    expectHomogeneousLastRow(transform);
}

TEST(ForwardKinematicsBuilderTest, BuildsKr640SymbolicForwardKinematics)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    kinemaforge::ik::KinematicChainBuilder chainBuilder;
    const auto chain = chainBuilder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(chain.has_value());
    ASSERT_EQ(chain->joints.size(), 7u);

    const ForwardKinematicsBuilder builder;
    const JointTransformBuilder transformBuilder;
    const auto transform = builder.build(*chain, transformBuilder);

    for (const auto& joint : chain->joints)
        if (joint.variable)
        {
            SCOPED_TRACE(joint.variable->name);
            EXPECT_TRUE(transformContainsSymbol(transform, joint.variable->name));
        }

    expectHomogeneousLastRow(transform);
}
