#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
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

using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::ExpressionType;
using kinemaforge::ik::JointTransformBuilder;
using kinemaforge::ik::JointType;
using kinemaforge::ik::JointVariable;
using kinemaforge::ik::KinematicJoint;
using kinemaforge::ik::SymbolicTransform;
using kinemaforge::ik::SymbolNode;
using kinemaforge::ik::Vector3;
using kinemaforge::ik::constantValue;
using kinemaforge::ik::isConstant;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::structurallyEqual;

namespace {

constexpr double kPi = std::numbers::pi;

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

// Recursive walk looking for a named symbol. Kept in the test file: the
// only consumer today is this suite, and a public API would want to
// return every symbol rather than answer about one.
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

// Same walk as containsSymbol, without caring which symbol. Checking only
// whether a cell is a composite node would be wrong: Add(Constant,
// Constant) is composite and carries no symbol at all.
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

KinematicJoint fixedJoint(Vector3 translation = {}, Vector3 rpy = {})
{
    return makeJoint(JointType::Fixed, Vector3{1.0, 0.0, 0.0}, translation, rpy);
}

KinematicJoint revoluteJoint(Vector3 axis, Vector3 translation = {}, Vector3 rpy = {},
                             std::string variableName = "q1")
{
    return makeJoint(JointType::Revolute, axis, translation, rpy, std::move(variableName));
}

void expectHomogeneousLastRow(const SymbolicTransform& transform)
{
    EXPECT_TRUE(isZero(transform(3, 0)));
    EXPECT_TRUE(isZero(transform(3, 1)));
    EXPECT_TRUE(isZero(transform(3, 2)));
    EXPECT_TRUE(isOne(transform(3, 3)));
}

// Rotation blocks come from folded constants, so they compare numerically.
void expectRotationBlock(const SymbolicTransform& transform, const double (&expected)[3][3])
{
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            // constantValue asserts on a non-constant, which would abort the
            // process instead of reporting a failure. Check first.
            ASSERT_TRUE(isConstant(transform(row, column)))
                << "rotation cell did not fold to a constant";
            EXPECT_NEAR(constantValue(transform(row, column)), expected[row][column], 1e-12);
        }
}

} // namespace

// --- fixed joints and origin ----------------------------------------

TEST(JointTransformBuilderTest, BuildsIdentityForFixedJointWithoutOrigin)
{
    const JointTransformBuilder builder;
    const auto transform = builder.build(fixedJoint());

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            if (row == column)
                EXPECT_TRUE(isOne(transform(row, column)));
            else
                EXPECT_TRUE(isZero(transform(row, column)));
        }
}

TEST(JointTransformBuilderTest, BuildsTranslationFromFixedJointOrigin)
{
    const JointTransformBuilder builder;
    const auto transform = builder.build(fixedJoint({1.0, 2.0, 3.0}));

    EXPECT_DOUBLE_EQ(constantValue(transform(0, 3)), 1.0);
    EXPECT_DOUBLE_EQ(constantValue(transform(1, 3)), 2.0);
    EXPECT_DOUBLE_EQ(constantValue(transform(2, 3)), 3.0);

    const double identity[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    expectRotationBlock(transform, identity);
    expectHomogeneousLastRow(transform);
}

TEST(JointTransformBuilderTest, BuildsRotationFromFixedJointRpy)
{
    const JointTransformBuilder builder;
    const auto transform = builder.build(fixedJoint({}, {0.0, 0.0, kPi / 2.0}));

    const double expected[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};   // Rz(pi/2)
    expectRotationBlock(transform, expected);
}

TEST(JointTransformBuilderTest, MapsRollPitchYawToCorrectAxes)
{
    const JointTransformBuilder builder;
    const double half = kPi / 2.0;

    const double aroundX[3][3] = {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}};
    const double aroundY[3][3] = {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
    const double aroundZ[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};

    {
        SCOPED_TRACE("roll");
        expectRotationBlock(builder.build(fixedJoint({}, {half, 0.0, 0.0})), aroundX);
    }
    {
        SCOPED_TRACE("pitch");
        expectRotationBlock(builder.build(fixedJoint({}, {0.0, half, 0.0})), aroundY);
    }
    {
        SCOPED_TRACE("yaw");
        expectRotationBlock(builder.build(fixedJoint({}, {0.0, 0.0, half})), aroundZ);
    }
}

TEST(JointTransformBuilderTest, ComposesRpyInFixedAxisOrder)
{
    // rpy = (pi/2, 0, -pi/2), taken from joint_4 of kr4_r600.urdf. Chosen
    // because the two conventions disagree here: a single non-zero
    // component, or two rotations by pi, would give the same matrix either
    // way and prove nothing.
    const JointTransformBuilder builder;
    const auto transform = builder.build(fixedJoint({}, {kPi / 2.0, 0.0, -kPi / 2.0}));

    // Rz(-pi/2) * Ry(0) * Rx(pi/2). The reversed order would give
    // {{0,1,0},{0,0,-1},{-1,0,0}}.
    const double expected[3][3] = {{0, 0, -1}, {-1, 0, 0}, {0, 1, 0}};
    expectRotationBlock(transform, expected);
}

TEST(JointTransformBuilderTest, CombinesTranslationAndRotationInCorrectOrder)
{
    // Translation * Rotation keeps the translation column verbatim.
    // Rotation * Translation would rotate it to [0, 1, 0].
    const JointTransformBuilder builder;
    const auto transform = builder.build(fixedJoint({1.0, 0.0, 0.0}, {0.0, 0.0, kPi / 2.0}));

    EXPECT_NEAR(constantValue(transform(0, 3)), 1.0, 1e-12);
    EXPECT_NEAR(constantValue(transform(1, 3)), 0.0, 1e-12);
    EXPECT_NEAR(constantValue(transform(2, 3)), 0.0, 1e-12);
}

TEST(JointTransformBuilderTest, CombinesOriginAndMotionInCorrectOrder)
{
    // T_origin * T_motion leaves the translation column constant: the joint
    // turns about a point 350 mm away. T_motion * T_origin would make that
    // column depend on q, which is geometrically wrong.
    const JointTransformBuilder builder;
    const auto transform = builder.build(revoluteJoint({0.0, 1.0, 0.0}, {0.35, 0.0, 0.0}));

    EXPECT_DOUBLE_EQ(constantValue(transform(0, 3)), 0.35);
    EXPECT_TRUE(isZero(transform(1, 3)));
    EXPECT_TRUE(isZero(transform(2, 3)));
}

TEST(JointTransformBuilderTest, ComposesOriginAndMotionRotationsInCorrectOrder)
{
    // The translation-column test above uses rpy = 0, so it cannot tell
    // R_origin * R_motion from R_motion * R_origin — with identity origin
    // both give the same rotation block. Here origin turns about Z and the
    // joint about X, which do not commute:
    //
    //   Rz(pi/2) * Rx(q) = [[0, -c,  s], [1, 0, 0], [0, s, c]]
    //   Rx(q) * Rz(pi/2) = [[0, -1,  0], [c, 0, -s], [s, 0, c]]
    //
    // Cell (1,0) is the discriminator: constant 1 in the correct order, a
    // cosine in the reversed one. Cell (0,1) is the mirror of that.
    const JointTransformBuilder builder;
    const auto transform =
        builder.build(revoluteJoint({1.0, 0.0, 0.0}, {}, {0.0, 0.0, kPi / 2.0}));

    EXPECT_TRUE(isOne(transform(1, 0)));
    EXPECT_FALSE(isConstant(transform(0, 1)));
    EXPECT_TRUE(containsSymbol(transform(0, 1), "q1"));
    expectHomogeneousLastRow(transform);
}

// --- revolute: principal and arbitrary axes -------------------------

TEST(JointTransformBuilderTest, BuildsRevoluteJointAroundXAxis)
{
    const JointTransformBuilder builder;
    const auto transform = builder.build(revoluteJoint({1.0, 0.0, 0.0}));

    EXPECT_TRUE(isOne(transform(0, 0)));
    EXPECT_EQ(transform(1, 1).type(), ExpressionType::Cos);
    EXPECT_EQ(transform(2, 2).type(), ExpressionType::Cos);
    EXPECT_EQ(transform(2, 1).type(), ExpressionType::Sin);
    EXPECT_EQ(transform(1, 2).type(), ExpressionType::Negate);
    expectHomogeneousLastRow(transform);
}

TEST(JointTransformBuilderTest, BuildsRevoluteJointAroundYAxis)
{
    // Ry carries the opposite sign pattern to Rx and Rz: sin sits above the
    // diagonal, minus-sin below. Copying either neighbour breaks here.
    const JointTransformBuilder builder;
    const auto transform = builder.build(revoluteJoint({0.0, 1.0, 0.0}));

    EXPECT_TRUE(isOne(transform(1, 1)));
    EXPECT_EQ(transform(0, 2).type(), ExpressionType::Sin);
    EXPECT_EQ(transform(2, 0).type(), ExpressionType::Negate);
}

TEST(JointTransformBuilderTest, BuildsRevoluteJointAroundZAxis)
{
    const JointTransformBuilder builder;
    const auto transform = builder.build(revoluteJoint({0.0, 0.0, 1.0}));

    EXPECT_EQ(transform(0, 0).type(), ExpressionType::Cos);
    EXPECT_EQ(transform(1, 1).type(), ExpressionType::Cos);
    EXPECT_EQ(transform(1, 0).type(), ExpressionType::Sin);
    EXPECT_EQ(transform(0, 1).type(), ExpressionType::Negate);
    EXPECT_TRUE(isOne(transform(2, 2)));
}

TEST(JointTransformBuilderTest, BuildsRevoluteJointAroundNegativeZAxis)
{
    // R(-Z, q) = R(Z, -q): the sines swap places relative to +Z.
    const JointTransformBuilder builder;
    const auto transform = builder.build(revoluteJoint({0.0, 0.0, -1.0}));

    EXPECT_EQ(transform(0, 1).type(), ExpressionType::Sin);
    EXPECT_EQ(transform(1, 0).type(), ExpressionType::Negate);
    EXPECT_EQ(transform(0, 0).type(), ExpressionType::Cos);
}

TEST(JointTransformBuilderTest, AxisAlignedFastPathBuildsCanonicalZRotation)
{
    const ExpressionFactory factory;
    const JointTransformBuilder builder{factory};
    const auto transform = builder.build(revoluteJoint({0.0, 0.0, 1.0}));

    const Expression variable = factory.symbol("q1");
    EXPECT_TRUE(structurallyEqual(transform(0, 0), factory.cos(variable)));
    EXPECT_TRUE(structurallyEqual(transform(1, 0), factory.sin(variable)));
    EXPECT_TRUE(structurallyEqual(transform(0, 1), factory.negate(factory.sin(variable))));
}

TEST(JointTransformBuilderTest, BuildsRevoluteJointAroundArbitraryAxis)
{
    // Every component non-zero: with [1,1,0] the s*z terms vanish and the
    // test would pass even for an implementation that dropped them.
    const double scale = 1.0 / std::sqrt(14.0);
    const Vector3 axis{1.0 * scale, 2.0 * scale, 3.0 * scale};

    const ExpressionFactory factory;
    const JointTransformBuilder builder{factory};
    const auto transform = builder.build(revoluteJoint(axis));

    // Rebuild the expected cells from the formula. Checking only the node
    // types and that the two differ would also pass for an implementation
    // using the wrong component — say s*x instead of s*z.
    const Expression variable = factory.symbol("q1");
    const Expression cosine = factory.cos(variable);
    const Expression sine = factory.sin(variable);
    const Expression versine = factory.subtract(factory.constant(1.0), cosine);

    const Expression x = factory.constant(axis.x);
    const Expression y = factory.constant(axis.y);
    const Expression z = factory.constant(axis.z);
    const Expression tx = factory.multiply(versine, x);
    const Expression sz = factory.multiply(sine, z);

    // t*x*y - s*z   and   t*x*y + s*z
    EXPECT_TRUE(structurallyEqual(
        transform(0, 1), factory.subtract(factory.multiply(tx, y), sz)));
    EXPECT_TRUE(structurallyEqual(
        transform(1, 0), factory.add(factory.multiply(tx, y), sz)));

    // t*x^2 + c
    EXPECT_TRUE(structurallyEqual(
        transform(0, 0), factory.add(factory.multiply(tx, x), cosine)));

    expectHomogeneousLastRow(transform);
}

// --- continuous and prismatic ---------------------------------------

TEST(JointTransformBuilderTest, BuildsContinuousJointLikeRevolute)
{
    const JointTransformBuilder builder;
    const Vector3 axis{0.0, 0.0, 1.0};

    const auto revolute = builder.build(makeJoint(JointType::Revolute, axis, {}, {}));
    const auto continuous = builder.build(makeJoint(JointType::Continuous, axis, {}, {}));

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            EXPECT_TRUE(structurallyEqual(revolute(row, column), continuous(row, column)));
        }
}

TEST(JointTransformBuilderTest, BuildsPrismaticJointTranslationAlongAxis)
{
    // Two non-zero components: with [0,0,1] the test would pass even for an
    // implementation that hard-coded q into one cell.
    const double scale = 1.0 / std::sqrt(5.0);
    const Vector3 axis{1.0 * scale, 2.0 * scale, 0.0};

    const ExpressionFactory factory;
    const JointTransformBuilder builder{factory};
    const auto transform = builder.build(makeJoint(JointType::Prismatic, axis, {}, {}));

    // Compare against the exact expected products: asserting only "is a
    // Multiply containing q1" would also pass if both cells got the same
    // coefficient, or if the two were swapped.
    const Expression variable = factory.symbol("q1");
    EXPECT_TRUE(structurallyEqual(
        transform(0, 3), factory.multiply(factory.constant(axis.x), variable)));
    EXPECT_TRUE(structurallyEqual(
        transform(1, 3), factory.multiply(factory.constant(axis.y), variable)));

    // The zero component builds no multiplication at all.
    EXPECT_TRUE(isZero(transform(2, 3)));

    expectHomogeneousLastRow(transform);
}

TEST(JointTransformBuilderTest, PrismaticDoesNotRotate)
{
    const double scale = 1.0 / std::sqrt(5.0);
    const Vector3 axis{1.0 * scale, 2.0 * scale, 0.0};

    const JointTransformBuilder builder;
    const auto transform = builder.build(makeJoint(JointType::Prismatic, axis, {}, {}));

    const double identity[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    expectRotationBlock(transform, identity);
}

TEST(JointTransformBuilderTest, RotatesPrismaticDisplacementByOriginRotation)
{
    // p = p_origin + R_origin * (axis * q). With origin turning about Z by
    // pi/2 and the slide along X, the displacement must land in Y. Dropping
    // the rotation entirely would leave it in X, and every other prismatic
    // test here uses an identity origin, so none of them would notice.
    const ExpressionFactory factory;
    const JointTransformBuilder builder{factory};
    const auto transform = builder.build(
        makeJoint(JointType::Prismatic, {1.0, 0.0, 0.0}, {}, {0.0, 0.0, kPi / 2.0}));

    const Expression variable = factory.symbol("q1");
    EXPECT_TRUE(structurallyEqual(transform(1, 3), variable));
    EXPECT_FALSE(structurallyEqual(transform(0, 3), variable));
    expectHomogeneousLastRow(transform);
}

// --- variable, invariants, real robots ------------------------------

TEST(JointTransformBuilderTest, UsesJointVariableName)
{
    const JointTransformBuilder builder;
    const auto transform =
        builder.build(revoluteJoint({0.0, 0.0, 1.0}, {}, {}, "theta_custom"));

    EXPECT_TRUE(containsSymbol(transform(0, 0), "theta_custom"));
    EXPECT_FALSE(containsSymbol(transform(0, 0), "q1"));
}

TEST(JointTransformBuilderTest, PreservesHomogeneousLastRow)
{
    // Every joint type, each with a non-zero origin, since the last row is
    // exactly what a full 4x4 product would quietly destroy.
    const JointTransformBuilder builder;
    const Vector3 translation{0.1, 0.2, 0.3};
    const Vector3 rpy{kPi / 2.0, 0.0, -kPi / 2.0};
    const double scale = 1.0 / std::sqrt(14.0);
    const Vector3 tilted{1.0 * scale, 2.0 * scale, 3.0 * scale};

    {
        SCOPED_TRACE("fixed");
        expectHomogeneousLastRow(builder.build(fixedJoint(translation, rpy)));
    }
    {
        SCOPED_TRACE("revolute");
        expectHomogeneousLastRow(
            builder.build(makeJoint(JointType::Revolute, tilted, translation, rpy)));
    }
    {
        SCOPED_TRACE("continuous");
        expectHomogeneousLastRow(
            builder.build(makeJoint(JointType::Continuous, tilted, translation, rpy)));
    }
    {
        SCOPED_TRACE("prismatic");
        expectHomogeneousLastRow(
            builder.build(makeJoint(JointType::Prismatic, tilted, translation, rpy)));
    }
}

TEST(JointTransformBuilderTest, IgnoresAxisForFixedJoint)
{
    // A fixed joint's axis takes no part in the transform, so a degenerate
    // one must not be rejected — the loader lets it through for exactly
    // this reason.
    KinematicJoint joint = fixedJoint({0.0, 0.0, 1.0});
    joint.axis = Vector3{0.0, 0.0, 0.0};

    const JointTransformBuilder builder;
    const auto transform = builder.build(joint);

    EXPECT_DOUBLE_EQ(constantValue(transform(2, 3)), 1.0);
    expectHomogeneousLastRow(transform);
}

TEST(JointTransformBuilderTest, BuildsAllKr640ChainJoints)
{
    // One pass over real data, to catch drift between the hand-built joints
    // above and what the pipeline actually produces.
    kinemaforge::ik::UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    kinemaforge::ik::KinematicChainBuilder chainBuilder;
    const auto chain = chainBuilder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(chain.has_value());
    ASSERT_EQ(chain->joints.size(), 7u);

    const JointTransformBuilder builder;
    for (const auto& joint : chain->joints)
    {
        SCOPED_TRACE(joint.name);
        const auto transform = builder.build(joint);
        expectHomogeneousLastRow(transform);

        if (joint.variable)
        {
            bool found = false;
            for (std::size_t row = 0; row < 3 && !found; ++row)
                for (std::size_t column = 0; column < 3 && !found; ++column)
                    found = containsSymbol(transform(row, column), joint.variable->name);
            EXPECT_TRUE(found) << "variable missing from the rotation block";
        }
        else
        {
            EXPECT_FALSE(transformContainsAnySymbol(transform))
                << "a fixed joint must not introduce a symbol";
        }
    }
}
