#include <gtest/gtest.h>

#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <utility>

using kinemaforge::ik::EvaluationErrorCode;
using kinemaforge::ik::EvaluationStatistics;
using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionEvaluator;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::SymbolValues;
using kinemaforge::ik::sameNode;
using kinemaforge::ik::structurallyEqual;

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Most node-coverage tests need at least one symbolic operand: a purely
// constant expression is folded by the factory at build time, so the test
// would be exercising constant folding rather than evaluation.
Expression symbolic(const ExpressionFactory& factory, const char* name)
{
    return factory.symbol(name);
}

} // namespace

// --- node coverage --------------------------------------------------

TEST(ExpressionEvaluatorTest, EvaluatesConstant)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{}};

    const auto result = evaluator.evaluate(factory.constant(2.5));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 2.5);
}

TEST(ExpressionEvaluatorTest, EvaluatesSymbol)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 1.5}}};

    const auto result = evaluator.evaluate(symbolic(factory, "q1"));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 1.5);
}

TEST(ExpressionEvaluatorTest, EvaluatesAdd)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 1.0}}};

    const auto result =
        evaluator.evaluate(factory.add(symbolic(factory, "q1"), factory.constant(2.0)));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 3.0);
}

TEST(ExpressionEvaluatorTest, EvaluatesSubtract)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 1.0}}};

    const auto result =
        evaluator.evaluate(factory.subtract(symbolic(factory, "q1"), factory.constant(2.0)));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, -1.0);
}

TEST(ExpressionEvaluatorTest, EvaluatesMultiply)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 2.0}}};

    const auto result =
        evaluator.evaluate(factory.multiply(symbolic(factory, "q1"), factory.constant(3.0)));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 6.0);
}

TEST(ExpressionEvaluatorTest, EvaluatesDivide)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 2.0}}};

    const auto result =
        evaluator.evaluate(factory.divide(symbolic(factory, "q1"), factory.constant(4.0)));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 0.5);
}

TEST(ExpressionEvaluatorTest, EvaluatesNegate)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 2.0}}};

    const auto result = evaluator.evaluate(factory.negate(symbolic(factory, "q1")));

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, -2.0);
}

TEST(ExpressionEvaluatorTest, EvaluatesSin)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", kPi / 2.0}}};

    const auto result = evaluator.evaluate(factory.sin(symbolic(factory, "q1")));

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 1.0, 1e-15);
}

TEST(ExpressionEvaluatorTest, EvaluatesCos)
{
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.0}}};

    const auto result = evaluator.evaluate(factory.cos(symbolic(factory, "q1")));

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 1.0, 1e-15);
}

TEST(ExpressionEvaluatorTest, EvaluatesNestedExpression)
{
    // sin(q1)^2 + cos(q1)^2 == 1 for any q1 -- a nested expression whose
    // expected value does not depend on getting the arithmetic of a
    // particular constant right.
    const ExpressionFactory factory;
    const Expression q1 = symbolic(factory, "q1");
    const Expression sine = factory.sin(q1);
    const Expression cosine = factory.cos(q1);

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.7}}};

    const auto result = evaluator.evaluate(
        factory.add(factory.multiply(sine, sine), factory.multiply(cosine, cosine)));

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 1.0, 1e-15);
}

// --- errors and domain ----------------------------------------------

TEST(ExpressionEvaluatorTest, ReportsMissingSymbol)
{
    // Never defaults to zero: a silent zero would produce a plausible-looking
    // but wrong robot pose.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 1.0}}};

    const auto result =
        evaluator.evaluate(factory.add(symbolic(factory, "q1"), symbolic(factory, "q3")));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::MissingSymbol);
    EXPECT_EQ(result.error().symbolName, "q3");
}

TEST(ExpressionEvaluatorTest, RejectsNonFiniteSymbolValue)
{
    // All three non-finite values, not just NaN: the contract is isfinite,
    // and a regression to isnan would pass a NaN-only test.
    const ExpressionFactory factory;

    const double nonFinite[] = {kNaN, kInfinity, -kInfinity};

    for (const double value : nonFinite)
    {
        SCOPED_TRACE(testing::Message() << "binding = " << value);

        ExpressionEvaluator evaluator{SymbolValues{{"q1", value}}};
        const auto result = evaluator.evaluate(factory.sin(symbolic(factory, "q1")));

        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, EvaluationErrorCode::NonFiniteSymbolValue);
        EXPECT_EQ(result.error().symbolName, "q1");
    }
}

TEST(ExpressionEvaluatorTest, ReportsLeftOperandErrorFirst)
{
    // The visiting order is a publicly documented contract, so it needs a
    // test: without one the implementation could visit right-to-left and the
    // reported error would silently change.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{}};

    const auto result = evaluator.evaluate(
        factory.add(symbolic(factory, "missing_left"), symbolic(factory, "missing_right")));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::MissingSymbol);
    EXPECT_EQ(result.error().symbolName, "missing_left");
}

TEST(ExpressionEvaluatorTest, ReportsDivisionByZero)
{
    // The denominator has to be a symbol: ExpressionFactory::divide asserts
    // on a literal zero denominator, so that expression cannot be built.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.0}}};

    const auto result =
        evaluator.evaluate(factory.divide(factory.constant(1.0), symbolic(factory, "q1")));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::DivisionByZero);
}

TEST(ExpressionEvaluatorTest, ReportsDivisionByNegativeZero)
{
    // Negative zero can only enter through a binding: constant(-0.0) collapses
    // to the shared positive zero node, because -0.0 == 0.0.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", -0.0}}};

    const auto result =
        evaluator.evaluate(factory.divide(factory.constant(1.0), symbolic(factory, "q1")));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::DivisionByZero);
}

TEST(ExpressionEvaluatorTest, PreservesDomainWhenZeroIsRightOperand)
{
    // (1/q1) * 0 at q1 = 0 must report DivisionByZero, not yield 0. This is
    // the end-to-end check that dropping x * 0 -> 0 from the factory buys
    // something real.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.0}}};

    const Expression reciprocal =
        factory.divide(factory.constant(1.0), symbolic(factory, "q1"));
    const auto result =
        evaluator.evaluate(factory.multiply(reciprocal, factory.constant(0.0)));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::DivisionByZero);
}

TEST(ExpressionEvaluatorTest, PreservesDomainWhenZeroIsLeftOperand)
{
    // The mirror image. Not hypothetical: SymbolicMatrix::multiply folds
    // products left to right, so Constant(0) really does appear as the left
    // operand inside FK cells.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.0}}};

    const Expression reciprocal =
        factory.divide(factory.constant(1.0), symbolic(factory, "q1"));
    const auto result =
        evaluator.evaluate(factory.multiply(factory.constant(0.0), reciprocal));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::DivisionByZero);
}

TEST(ExpressionEvaluatorTest, ReportsNonFiniteResult)
{
    // Two symbols, not two constants: constant folding routes back through
    // ExpressionFactory::constant, whose isfinite assert would abort the
    // process while building 1e308 * 1e308.
    const ExpressionFactory factory;
    ExpressionEvaluator evaluator{SymbolValues{{"q1", 1e308}, {"q2", 1e308}}};

    const auto result = evaluator.evaluate(
        factory.multiply(symbolic(factory, "q1"), symbolic(factory, "q2")));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, EvaluationErrorCode::NonFiniteResult);
}

// --- session and memoization ----------------------------------------

TEST(ExpressionEvaluatorTest, MemoizesSharedSubexpression)
{
    // add(x, x) over the SAME handle: Add, Cos and Symbol are each computed
    // once, and the second occurrence of x is a hit.
    const ExpressionFactory factory;
    const Expression x = factory.cos(symbolic(factory, "q1"));

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.5}}};
    ASSERT_TRUE(evaluator.evaluate(factory.add(x, x)).has_value());

    const EvaluationStatistics statistics = evaluator.statistics();
    EXPECT_EQ(statistics.cacheMisses, 3u);
    EXPECT_EQ(statistics.cacheHits, 1u);
}

TEST(ExpressionEvaluatorTest, DoesNotMergeStructurallyEqualDistinctNodes)
{
    // Memoization is by node identity, not by structural equality. Two
    // separately built but structurally identical trees stay separate
    // entries -- this is not hash-consing.
    const ExpressionFactory factory;
    const Expression lhs = factory.cos(symbolic(factory, "q1"));
    const Expression rhs = factory.cos(symbolic(factory, "q1"));

    ASSERT_FALSE(sameNode(lhs, rhs));
    ASSERT_TRUE(structurallyEqual(lhs, rhs));

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.5}}};
    ASSERT_TRUE(evaluator.evaluate(factory.add(lhs, rhs)).has_value());

    // Add + two Cos + two Symbol, all distinct nodes.
    const EvaluationStatistics statistics = evaluator.statistics();
    EXPECT_EQ(statistics.cacheMisses, 5u);
    EXPECT_EQ(statistics.cacheHits, 0u);
}

TEST(ExpressionEvaluatorTest, MemoizesSharedLeafAcrossDistinctParents)
{
    // Constant and Symbol nodes are cached too, not only composite ones. Here
    // one Symbol leaf is reached through two different parents, so the hit
    // can only come from caching the leaf.
    const ExpressionFactory factory;
    const Expression q1 = symbolic(factory, "q1");

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.5}}};
    ASSERT_TRUE(evaluator.evaluate(factory.add(factory.sin(q1), factory.cos(q1))).has_value());

    // Add, Sin, Symbol, Cos are misses; the second visit to Symbol is a hit.
    const EvaluationStatistics statistics = evaluator.statistics();
    EXPECT_EQ(statistics.cacheMisses, 4u);
    EXPECT_EQ(statistics.cacheHits, 1u);
}

TEST(ExpressionEvaluatorTest, SharesMemoizationAcrossMultipleRootExpressions)
{
    // The reason the evaluator is a session rather than a free function: the
    // sixteen cells of an FK matrix are separate roots over one shared DAG.
    const ExpressionFactory factory;
    const Expression shared = factory.cos(symbolic(factory, "q1"));

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 0.5}}};

    ASSERT_TRUE(evaluator.evaluate(factory.add(shared, factory.constant(1.0))).has_value());
    const EvaluationStatistics afterFirst = evaluator.statistics();
    EXPECT_EQ(afterFirst.cacheHits, 0u);

    ASSERT_TRUE(
        evaluator.evaluate(factory.multiply(shared, factory.constant(2.0))).has_value());
    const EvaluationStatistics afterSecond = evaluator.statistics();

    EXPECT_EQ(afterSecond.cacheHits, 1u);
    EXPECT_GT(afterSecond.cacheMisses, afterFirst.cacheMisses);
}

TEST(ExpressionEvaluatorTest, DifferentEvaluatorsUseDifferentSymbolValues)
{
    // The same expression handle under two sessions. If a cache ever leaked
    // between instances, this is what would catch it.
    const ExpressionFactory factory;
    const Expression expression =
        factory.multiply(symbolic(factory, "q1"), factory.constant(2.0));

    ExpressionEvaluator first{SymbolValues{{"q1", 1.0}}};
    ExpressionEvaluator second{SymbolValues{{"q1", 3.0}}};

    const auto firstResult = first.evaluate(expression);
    const auto secondResult = second.evaluate(expression);

    ASSERT_TRUE(firstResult.has_value());
    ASSERT_TRUE(secondResult.has_value());
    EXPECT_DOUBLE_EQ(*firstResult, 2.0);
    EXPECT_DOUBLE_EQ(*secondResult, 6.0);
}

TEST(ExpressionEvaluatorTest, IgnoresUnusedExtraBindings)
{
    const ExpressionFactory factory;
    const Expression expression = factory.add(symbolic(factory, "q1"), factory.constant(1.0));

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 2.0}, {"unused", 99.0}}};

    const auto result = evaluator.evaluate(expression);

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 3.0);
}

TEST(ExpressionEvaluatorTest, AcceptsNonFiniteBindingForUnusedSymbol)
{
    // Bindings are validated on read, not eagerly. An unused NaN cannot
    // affect any result, so "an unused binding does not change the outcome"
    // holds without exception.
    const ExpressionFactory factory;
    const Expression expression = factory.add(symbolic(factory, "q1"), factory.constant(1.0));

    ExpressionEvaluator evaluator{SymbolValues{{"q1", 2.0}, {"unused", kNaN}}};

    const auto result = evaluator.evaluate(expression);

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 3.0);
}

// --- cache ownership ------------------------------------------------

TEST(ExpressionEvaluatorTest, KeepsEvaluatedNodesAliveAcrossTemporaries)
{
    // Every expression here is a temporary that dies at the end of its full
    // expression. Because the cache is keyed by Expression rather than by a
    // raw node pointer, each entry keeps its own node alive, so an address
    // cannot be recycled for a different node while it is still a key.
    //
    // This test can only fail when the allocator actually reuses an address,
    // which is implementation behaviour -- so a green run is not a proof.
    // The guarantee comes from the key type; this is a regression net.
    const ExpressionFactory factory;

    SymbolValues values;
    for (int i = 0; i < 64; ++i)
        values.emplace("q" + std::to_string(i), static_cast<double>(i));

    ExpressionEvaluator evaluator{std::move(values)};

    for (int i = 0; i < 64; ++i)
    {
        SCOPED_TRACE(testing::Message() << "symbol q" << i);
        const auto result = evaluator.evaluate(factory.symbol("q" + std::to_string(i)));
        ASSERT_TRUE(result.has_value());
        EXPECT_DOUBLE_EQ(*result, static_cast<double>(i));
    }
}
