#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <numbers>
#include <optional>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <variant>

using kinemaforge::ik::EvaluationErrorCode;
using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionEvaluator;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::ExpressionNode;
using kinemaforge::ik::ExpressionType;
using kinemaforge::ik::SymbolValues;
using kinemaforge::ik::constantValue;
using kinemaforge::ik::isConstant;
using kinemaforge::ik::isOne;
using kinemaforge::ik::isZero;
using kinemaforge::ik::sameNode;
using kinemaforge::ik::structurallyEqual;

namespace ik = kinemaforge::ik;

namespace {

// Large and tiny finite magnitudes chosen so the tested operations overflow
// or underflow while both operands remain legitimate constants.
constexpr double kHuge = 1e308;
constexpr double kTiny = 1e-308;

std::optional<ik::SymbolicTransform> forwardKinematics(const char* fileName)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName);

    ik::KinematicChainBuilder chainBuilder;
    const auto chain = chainBuilder.build(robot, "base_link", "tool0");
    if (!chain)
    {
        ADD_FAILURE() << "chain build failed for " << fileName;
        return std::nullopt;
    }

    const ik::JointTransformBuilder transformBuilder;
    const ik::ForwardKinematicsBuilder fkBuilder;
    return fkBuilder.build(*chain, transformBuilder);
}

// Walks each unique node once. An FK transform shares sub-trees heavily --
// 281 unique nodes against 21 882 counted with multiplicity for kr640 -- so a
// walk without the visited set would revisit the same subexpression thousands
// of times for no additional coverage.
//
// Counts the constant leaves it checked, so the caller can tell "every constant
// was finite" apart from "the walk never reached a constant".
void expectConstantLeavesFinite(const Expression& expression,
                                std::unordered_set<const ExpressionNode*>& visited,
                                std::size_t& constantCount)
{
    if (!visited.insert(&expression.node()).second)
        return;

    std::visit(
        [&](const auto& node) {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, ik::ConstantNode>)
            {
                ++constantCount;
                EXPECT_TRUE(std::isfinite(node.value))
                    << "non-finite constant leaf: " << node.value;
            }
            else if constexpr (std::is_same_v<Node, ik::AddNode>
                               || std::is_same_v<Node, ik::SubtractNode>
                               || std::is_same_v<Node, ik::MultiplyNode>
                               || std::is_same_v<Node, ik::DivideNode>)
            {
                expectConstantLeavesFinite(node.lhs, visited, constantCount);
                expectConstantLeavesFinite(node.rhs, visited, constantCount);
            }
            else if constexpr (std::is_same_v<Node, ik::NegateNode>
                               || std::is_same_v<Node, ik::SinNode>
                               || std::is_same_v<Node, ik::CosNode>)
            {
                expectConstantLeavesFinite(node.operand, visited, constantCount);
            }
            else
            {
                // Symbol is the only leaf with nothing to check. A node type
                // added later must state explicitly whether it has children:
                // without this, it would silently fall into "nothing to do" and
                // the walk would stop above its subtree while still passing.
                static_assert(std::is_same_v<Node, ik::SymbolNode>,
                              "Unhandled ExpressionNode type");
            }
        },
        expression.node().value);
}

} // namespace

TEST(ExpressionFactoryTest, BuildsBinaryOperations)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");
    const auto y = factory.symbol("y");

    EXPECT_EQ(factory.add(x, y).type(), ExpressionType::Add);
    EXPECT_EQ(factory.subtract(x, y).type(), ExpressionType::Subtract);
    EXPECT_EQ(factory.multiply(x, y).type(), ExpressionType::Multiply);
    EXPECT_EQ(factory.divide(x, y).type(), ExpressionType::Divide);
}

TEST(ExpressionFactoryTest, BuildsUnaryOperations)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    EXPECT_EQ(factory.negate(x).type(), ExpressionType::Negate);
    EXPECT_EQ(factory.sin(x).type(), ExpressionType::Sin);
    EXPECT_EQ(factory.cos(x).type(), ExpressionType::Cos);
}

TEST(ExpressionFactoryTest, FoldsConstantArithmetic)
{
    const ExpressionFactory factory;

    EXPECT_DOUBLE_EQ(constantValue(factory.add(factory.constant(2.0), factory.constant(3.0))), 5.0);
    EXPECT_DOUBLE_EQ(constantValue(factory.subtract(factory.constant(5.0), factory.constant(3.0))), 2.0);
    EXPECT_DOUBLE_EQ(constantValue(factory.multiply(factory.constant(3.0), factory.constant(4.0))), 12.0);
    EXPECT_DOUBLE_EQ(constantValue(factory.divide(factory.constant(6.0), factory.constant(3.0))), 2.0);
}

TEST(ExpressionFactoryTest, FoldsNegateOfConstant)
{
    const ExpressionFactory factory;
    EXPECT_DOUBLE_EQ(constantValue(factory.negate(factory.constant(2.0))), -2.0);
}

TEST(ExpressionFactoryTest, FoldsConstantTrig)
{
    const ExpressionFactory factory;

    EXPECT_TRUE(isZero(factory.sin(factory.constant(0.0))));
    EXPECT_TRUE(isOne(factory.cos(factory.constant(0.0))));
}

TEST(ExpressionFactoryTest, DoesNotSnapTinyConstantsToZero)
{
    const ExpressionFactory factory;
    const auto tiny = factory.constant(1e-300);

    // No tolerance anywhere in the factory: a tiny constant stays itself.
    // This is the canonicalization contract stated without involving libm
    // at all.
    EXPECT_FALSE(isZero(tiny));
    EXPECT_DOUBLE_EQ(constantValue(tiny), 1e-300);
}

TEST(ExpressionFactoryTest, TrigFoldingMatchesStdSinExactly)
{
    const ExpressionFactory factory;
    const auto sinePi = factory.sin(factory.constant(std::numbers::pi));

    // Folding must hand back exactly what std::sin produced, with no
    // canonicalization of near-zero results — a tolerance would become
    // part of the symbolic semantics, which is a separate, deferred
    // decision.
    //
    // The reference goes through volatile so it is computed by the same
    // runtime libm call the factory made. Writing std::sin(std::numbers::pi)
    // directly would be constant-folded by the compiler (correctly rounded
    // via MPFR) and disagree with libm in the last few bits: 1.2246468e-16
    // versus 1.2246064e-16 on this toolchain, at -O0 and -O2 alike.
    //
    // No specific residual is pinned, so this stays valid on any libm.
    volatile double argument = std::numbers::pi;
    const double expected = std::sin(argument);

    ASSERT_EQ(sinePi.type(), ExpressionType::Constant);
    EXPECT_DOUBLE_EQ(constantValue(sinePi), expected);
}

TEST(ExpressionFactoryTest, DropsAdditiveIdentity)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");
    const auto zero = factory.constant(0.0);

    EXPECT_TRUE(sameNode(factory.add(x, zero), x));
    EXPECT_TRUE(sameNode(factory.add(zero, x), x));
}

TEST(ExpressionFactoryTest, DropsSubtractiveIdentity)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    EXPECT_TRUE(sameNode(factory.subtract(x, factory.constant(0.0)), x));
}

TEST(ExpressionFactoryTest, DropsMultiplicativeIdentity)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");
    const auto one = factory.constant(1.0);

    EXPECT_TRUE(sameNode(factory.multiply(x, one), x));
    EXPECT_TRUE(sameNode(factory.multiply(one, x), x));
}

TEST(ExpressionFactoryTest, DropsDivisionByOne)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    EXPECT_TRUE(sameNode(factory.divide(x, factory.constant(1.0)), x));
}

TEST(ExpressionFactoryTest, KeepsSymbolicMultiplicationByZero)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    // x * 0 -> 0 is deliberately absent: it would erase the domain of x.
    // (1/q) * 0 is undefined at q = 0, but a folded 0 would claim
    // otherwise.
    const auto product = factory.multiply(x, factory.constant(0.0));
    EXPECT_EQ(product.type(), ExpressionType::Multiply);
}

TEST(ExpressionFactoryTest, FoldsConstantMultipliedByZero)
{
    const ExpressionFactory factory;

    // Both operands known and finite: nothing about a domain is lost.
    const auto product = factory.multiply(factory.constant(3.0), factory.constant(0.0));
    EXPECT_TRUE(isZero(product));
}

TEST(ExpressionFactoryTest, ConstantZeroSharesTheDefaultNode)
{
    const ExpressionFactory factory;
    EXPECT_TRUE(sameNode(factory.constant(0.0), Expression{}));
}

TEST(ExpressionFactoryTest, DoesNotReorderOperands)
{
    const ExpressionFactory factory;
    const auto forward = factory.subtract(factory.symbol("a"), factory.symbol("b"));
    const auto reversed = factory.subtract(factory.symbol("b"), factory.symbol("a"));

    EXPECT_FALSE(structurallyEqual(forward, reversed));
}

TEST(ExpressionFactoryTest, DoesNotSimplifyBeyondScope)
{
    const ExpressionFactory factory;
    const auto x = factory.symbol("x");

    // x - x -> 0 needs operand equivalence checking, which belongs to a
    // future EquationSimplifier.
    EXPECT_EQ(factory.subtract(x, x).type(), ExpressionType::Subtract);
}

// --- finite-constant closure ----------------------------------------

TEST(ExpressionFactoryTest, DoesNotFoldOverflowingSum)
{
    const ExpressionFactory factory;
    const auto sum = factory.add(factory.constant(kHuge), factory.constant(kHuge));

    EXPECT_EQ(sum.type(), ExpressionType::Add);
    EXPECT_FALSE(isConstant(sum));
}

TEST(ExpressionFactoryTest, DoesNotFoldOverflowingDifference)
{
    const ExpressionFactory factory;
    const auto difference =
        factory.subtract(factory.constant(-kHuge), factory.constant(kHuge));

    EXPECT_EQ(difference.type(), ExpressionType::Subtract);
    EXPECT_FALSE(isConstant(difference));
}

TEST(ExpressionFactoryTest, DoesNotFoldOverflowingProduct)
{
    const ExpressionFactory factory;
    const auto product = factory.multiply(factory.constant(kHuge), factory.constant(kHuge));

    EXPECT_EQ(product.type(), ExpressionType::Multiply);
    EXPECT_FALSE(isConstant(product));
}

TEST(ExpressionFactoryTest, DoesNotFoldOverflowingQuotient)
{
    const ExpressionFactory factory;
    const auto quotient = factory.divide(factory.constant(kHuge), factory.constant(kTiny));

    EXPECT_EQ(quotient.type(), ExpressionType::Divide);
    EXPECT_FALSE(isConstant(quotient));
}

TEST(ExpressionFactoryTest, OverflowingFoldEvaluatesToNonFiniteResult)
{
    // The other half of declining to fold: the surviving node is not a dead
    // end, it is an expression whose value happens not to be representable,
    // and the existing error model already says so.
    const ExpressionFactory factory;
    const auto product = factory.multiply(factory.constant(kHuge), factory.constant(kHuge));

    ExpressionEvaluator evaluator{SymbolValues{}};
    const auto result = evaluator.evaluate(product);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::NonFiniteResult);
}

TEST(ExpressionFactoryTest, StillFoldsWhenResultIsFinite)
{
    // Explicit regression pin for R5, read together with the four DoesNotFold
    // tests above: declining an overflow must not cost ordinary folding.
    // FoldsConstantArithmetic covers the same values from the other side.
    const ExpressionFactory factory;
    const auto sum = factory.add(factory.constant(2.0), factory.constant(3.0));

    ASSERT_EQ(sum.type(), ExpressionType::Constant);
    EXPECT_DOUBLE_EQ(constantValue(sum), 5.0);
}

TEST(ExpressionFactoryTest, FoldsUnderflowingProductToZero)
{
    // Underflow is not the same as non-finite: 1e-308 * 1e-308 is exactly 0.0,
    // which is finite, foldable, and reaches the shared zero node like any
    // other zero. isfinite is the boundary, not "the result is surprising".
    const ExpressionFactory factory;
    const auto product = factory.multiply(factory.constant(kTiny), factory.constant(kTiny));

    EXPECT_TRUE(isConstant(product));
    EXPECT_TRUE(isZero(product));
    EXPECT_TRUE(sameNode(product, Expression{}));
}

TEST(ExpressionFactoryTest, AllConstantLeavesRemainFinite)
{
    // Integration smoke test, not a proof: it says that every constant leaf in
    // the FK DAGs the current pipeline builds for these two robots is finite.
    // A future operation that folds without checking would slip past this
    // unless FK uses it and overflows on these fixtures -- the direct
    // DoesNotFold* tests and the single fold site in foldFinite are what
    // actually guard the contract.
    for (const char* fileName : {"kr640.urdf", "kr4_r600.urdf"})
    {
        const auto fk = forwardKinematics(fileName);
        ASSERT_TRUE(fk.has_value()) << fileName;

        std::unordered_set<const ExpressionNode*> visited;
        std::size_t constantCount = 0;
        for (std::size_t row = 0; row < ik::SymbolicTransform::rows; ++row)
            for (std::size_t column = 0; column < ik::SymbolicTransform::columns; ++column)
                expectConstantLeavesFinite((*fk)(row, column), visited, constantCount);

        EXPECT_GT(constantCount, 0u) << fileName;
    }
}
