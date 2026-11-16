#include <gtest/gtest.h>

#include "ik_equations/IkEquationBuilder.hpp"
#include "ik_equations/builders/EquationSimplifier.hpp"
#include "ik_equations/model/Equation.hpp"
#include "ik_equations/model/IkEquationSystem.hpp"
#include "ik_equations/model/IkTarget.hpp"
#include "ik_equations/symbolic/ExpressionCanonicalizer.hpp"
#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <random>
#include <span>
#include <string>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace ik = kinemaforge::ik;
using ik::Expression;
using ik::ExpressionFactory;

namespace {

const ExpressionFactory kFactory;

Expression sym(const char* name) { return kFactory.symbol(name); }
Expression con(double value) { return kFactory.constant(value); }
Expression norm(const Expression& expression) { return ik::canonicalize(expression, kFactory); }

int depth(const Expression& expression)
{
    return std::visit(
        [](const auto& node) -> int {
            using Node = std::decay_t<decltype(node)>;
            if constexpr (std::is_same_v<Node, ik::ConstantNode> ||
                          std::is_same_v<Node, ik::SymbolNode>)
            {
                return 0;
            }
            else if constexpr (requires { node.lhs; node.rhs; })
            {
                return 1 + std::max(depth(node.lhs), depth(node.rhs));
            }
            else
            {
                return 1 + depth(node.operand);
            }
        },
        expression.node().value);
}

bool containsType(const Expression& expression, ik::ExpressionType type)
{
    if (expression.type() == type)
        return true;

    return std::visit(
        [type](const auto& node) -> bool {
            using Node = std::decay_t<decltype(node)>;
            if constexpr (std::is_same_v<Node, ik::ConstantNode> ||
                          std::is_same_v<Node, ik::SymbolNode>)
            {
                return false;
            }
            else if constexpr (requires { node.lhs; node.rhs; })
            {
                return containsType(node.lhs, type) || containsType(node.rhs, type);
            }
            else
            {
                return containsType(node.operand, type);
            }
        },
        expression.node().value);
}

std::size_t countUniqueNodes(const std::span<const ik::Equation> equations)
{
    std::unordered_set<const ik::ExpressionNode*> nodes;
    const auto visit = [&nodes](const auto& self, const Expression& expression) -> void {
        if (!nodes.insert(&expression.node()).second)
            return;
        std::visit(
            [&self](const auto& node) {
                using Node = std::decay_t<decltype(node)>;
                if constexpr (std::is_same_v<Node, ik::ConstantNode> ||
                              std::is_same_v<Node, ik::SymbolNode>)
                {
                }
                else if constexpr (requires { node.lhs; node.rhs; })
                {
                    self(self, node.lhs);
                    self(self, node.rhs);
                }
                else
                {
                    self(self, node.operand);
                }
            },
            expression.node().value);
    };

    for (const ik::Equation& equation : equations)
    {
        visit(visit, equation.lhs);
        visit(visit, equation.rhs);
    }
    return nodes.size();
}

std::expected<ik::IkEquationSystem, ik::IkEquationSystemError> syntheticPositionSystem()
{
    std::vector<ik::Equation> equations;
    equations.push_back(ik::Equation{kFactory.add(sym("b"), sym("a")), con(1.0),
                                     ik::PositionEquationSource{ik::CartesianComponent::X}});
    equations.push_back(ik::Equation{kFactory.add(sym("c"), sym("b")), con(2.0),
                                     ik::PositionEquationSource{ik::CartesianComponent::Y}});
    equations.push_back(ik::Equation{kFactory.add(sym("a"), sym("c")), con(3.0),
                                     ik::PositionEquationSource{ik::CartesianComponent::Z}});
    return ik::IkEquationSystem::create(ik::IkTaskKind::Position,
                                        {ik::JointVariable{"q1", 1}},
                                        std::move(equations));
}

ik::PoseTarget identityPose()
{
    return ik::PoseTarget{{0.1, -0.2, 0.3},
                          ik::RotationMatrix3{{{{1.0, 0.0, 0.0},
                                                {0.0, 1.0, 0.0},
                                                {0.0, 0.0, 1.0}}}}};
}

} // namespace

// --- structural order (4) -------------------------------------------------

TEST(ExpressionCanonicalizerTest, IsTotalAndAntisymmetricOnMixedKinds)
{
    const Expression a = sym("a");
    const std::array<Expression, 9> expressions{{
        con(2.0), a, kFactory.sin(a), kFactory.cos(a), kFactory.negate(a),
        kFactory.multiply(a, sym("b")), kFactory.divide(a, sym("b")),
        kFactory.add(a, sym("b")), kFactory.subtract(a, sym("b"))}};

    for (std::size_t i = 0; i < expressions.size(); ++i)
        for (std::size_t j = 0; j < expressions.size(); ++j)
        {
            const int ij = ik::compareExpressions(expressions[i], expressions[j]);
            const int ji = ik::compareExpressions(expressions[j], expressions[i]);
            EXPECT_EQ((ij > 0) - (ij < 0), -((ji > 0) - (ji < 0)));
        }
}

TEST(ExpressionCanonicalizerTest, AgreesWithStructuralEquality)
{
    const Expression a1 = kFactory.add(sym("a"), sym("b"));
    const Expression a2 = kFactory.add(sym("a"), sym("b"));
    const Expression b = kFactory.add(sym("b"), sym("a"));
    EXPECT_EQ(ik::compareExpressions(a1, a2) == 0, ik::structurallyEqual(a1, a2));
    EXPECT_EQ(ik::compareExpressions(a1, b) == 0, ik::structurallyEqual(a1, b));
}

TEST(ExpressionCanonicalizerTest, OrdersByNameNotByHash)
{
    EXPECT_LT(ik::compareExpressions(sym("a"), sym("z")), 0);
}

TEST(ExpressionCanonicalizerTest, DoesNotDependOnConstructionOrder)
{
    EXPECT_TRUE(ik::structurallyEqual(norm(kFactory.add(sym("a"), sym("b"))),
                                      norm(kFactory.add(sym("b"), sym("a")))));
}

// --- AC normalisation (22) ------------------------------------------------

TEST(ExpressionCanonicalizerTest, FlattensNestedAssociativeChains)
{
    const Expression lhs = kFactory.add(sym("a"), kFactory.add(sym("b"), sym("c")));
    const Expression rhs = kFactory.add(kFactory.add(sym("c"), sym("a")), sym("b"));
    EXPECT_TRUE(ik::structurallyEqual(norm(lhs), norm(rhs)));
}

TEST(ExpressionCanonicalizerTest, ReducesExactConstantsAndDropsNeutral)
{
    const Expression normalized = norm(kFactory.add(kFactory.add(con(3.0), sym("x")), con(4.0)));
    ik::ExpressionEvaluator evaluator{{{"x", 2.0}}};
    const auto value = evaluator.evaluate(normalized);
    ASSERT_TRUE(value.has_value());
    EXPECT_DOUBLE_EQ(*value, 9.0);
}

TEST(ExpressionCanonicalizerTest, DoesNotJoinConstantsInexactly)
{
    const Expression input = kFactory.add(con(1e16), kFactory.add(con(1.0), sym("x")));
    const Expression normalized = norm(input);
    ik::ExpressionEvaluator before{{{"x", 3.0}}};
    ik::ExpressionEvaluator after{{{"x", 3.0}}};
    ASSERT_TRUE(before.evaluate(input).has_value());
    ASSERT_TRUE(after.evaluate(normalized).has_value());
    EXPECT_DOUBLE_EQ(*before.evaluate(input), *after.evaluate(normalized));
}

TEST(ExpressionCanonicalizerTest, ReducesConstantsToAFixedPoint)
{
    const double p = std::ldexp(1.0, 52);
    const Expression input = kFactory.add(con(p), kFactory.add(con(p + 1.0),
                                                              kFactory.add(con(p + 3.0), sym("x"))));
    const Expression once = norm(input);
    const Expression twice = norm(once);
    EXPECT_TRUE(ik::sameNode(once, twice));
}

TEST(ExpressionCanonicalizerTest, EliminatesSubtractionWhenCanonicalFormIsRepresentable)
{
    const Expression normalized = norm(kFactory.subtract(sym("a"), kFactory.subtract(sym("b"), sym("c"))));
    EXPECT_FALSE(containsType(normalized, ik::ExpressionType::Subtract));
}

TEST(ExpressionCanonicalizerTest, PreservesOriginalSubtreeAtRepresentabilityBarrier)
{
    const double m = std::numeric_limits<double>::max();
    const Expression overflow = kFactory.add(con(m), con(m));
    const Expression denominator = kFactory.subtract(overflow, overflow);
    const Expression ratio = kFactory.divide(sym("x"), denominator);
    const Expression normalized = norm(ratio);
    const auto& node = std::get<ik::DivideNode>(normalized.node().value);
    EXPECT_TRUE(ik::structurallyEqual(node.rhs, denominator));
    EXPECT_TRUE(containsType(node.rhs, ik::ExpressionType::Subtract));
}

TEST(ExpressionCanonicalizerTest, NormalizesExplicitNegatedSum)
{
    const Expression a = sym("a");
    const Expression b = sym("b");
    EXPECT_TRUE(ik::structurallyEqual(norm(kFactory.negate(kFactory.add(a, b))),
                                      norm(kFactory.add(kFactory.negate(a), kFactory.negate(b)))));
}

TEST(ExpressionCanonicalizerTest, FlattensOperatorRevealedByChildNormalization)
{
    const Expression input = kFactory.add(sym("a"),
        kFactory.negate(kFactory.negate(kFactory.add(sym("b"), sym("c")))));
    EXPECT_TRUE(ik::structurallyEqual(norm(input),
                                      norm(kFactory.add(sym("a"), kFactory.add(sym("b"), sym("c"))))));
}

TEST(ExpressionCanonicalizerTest, NormalizesSignByParity)
{
    const Expression a = sym("a"), b = sym("b");
    EXPECT_TRUE(ik::structurallyEqual(norm(kFactory.negate(kFactory.multiply(a, b))),
                                      norm(kFactory.multiply(kFactory.negate(a), b))));
    EXPECT_TRUE(ik::structurallyEqual(norm(kFactory.negate(kFactory.multiply(a, b))),
                                      norm(kFactory.multiply(a, kFactory.negate(b)))));
}

TEST(ExpressionCanonicalizerTest, NormalizesNegativeConstantSign)
{
    EXPECT_TRUE(ik::structurallyEqual(norm(kFactory.multiply(con(-2.0), sym("x"))),
                                      norm(kFactory.multiply(con(2.0), kFactory.negate(sym("x"))))));
}

TEST(ExpressionCanonicalizerTest, KeepsNegativeZeroOutOfParity)
{
    EXPECT_TRUE(ik::structurallyEqual(norm(kFactory.multiply(con(-0.0), sym("x"))),
                                      norm(kFactory.multiply(con(0.0), sym("x")))));
}

TEST(ExpressionCanonicalizerTest, NormalizesMinusOneTimesSum)
{
    const Expression normalized = norm(kFactory.multiply(con(-1.0),
                                                           kFactory.add(sym("a"), sym("b"))));
    EXPECT_FALSE(normalized.type() == ik::ExpressionType::Negate &&
                 std::get<ik::NegateNode>(normalized.node().value).operand.type() ==
                     ik::ExpressionType::Add);
}

TEST(ExpressionCanonicalizerTest, NormalizesSignThroughDivision)
{
    const Expression a = sym("a"), b = sym("b");
    EXPECT_TRUE(ik::structurallyEqual(norm(kFactory.negate(kFactory.divide(a, b))),
                                      norm(kFactory.divide(kFactory.negate(a), b))));
    EXPECT_TRUE(ik::structurallyEqual(norm(kFactory.negate(kFactory.divide(a, b))),
                                      norm(kFactory.divide(a, kFactory.negate(b)))));
}

TEST(ExpressionCanonicalizerTest, IsIdempotent)
{
    std::array<Expression, 5> inputs{{
        kFactory.add(sym("c"), kFactory.add(sym("a"), sym("b"))),
        kFactory.multiply(con(-2.0), kFactory.multiply(sym("b"), sym("a"))),
        kFactory.subtract(sym("a"), kFactory.subtract(sym("b"), sym("c"))),
        kFactory.negate(kFactory.add(sym("a"), sym("b"))),
        kFactory.divide(kFactory.negate(sym("a")), kFactory.negate(sym("b")))}};

    for (const Expression& input : inputs)
    {
        ik::CanonicalizationSession session{kFactory};
        const Expression once = session.canonicalize(input);
        const Expression twice = session.canonicalize(once);
        EXPECT_TRUE(ik::sameNode(once, twice));
    }
}

TEST(ExpressionCanonicalizerTest, RebuildsInBalancedShape)
{
    Expression input = sym("h");
    for (const char* name : {"g", "f", "e", "d", "c", "b", "a"})
        input = kFactory.add(input, sym(name));
    EXPECT_LE(depth(norm(input)), 3);
}

TEST(ExpressionCanonicalizerTest, NormalizesNestedSubtractionWithPolarity)
{
    const Expression lhs = kFactory.subtract(sym("a"), kFactory.subtract(sym("b"), sym("c")));
    const Expression rhs = kFactory.add(kFactory.subtract(sym("a"), sym("b")), sym("c"));
    EXPECT_TRUE(ik::structurallyEqual(norm(lhs), norm(rhs)));
}

TEST(ExpressionCanonicalizerTest, KeepsUnjoinableConstantsApartInRebuild)
{
    const Expression input = kFactory.add(con(1e16), kFactory.add(con(1.0), sym("x")));
    const Expression normalized = norm(input);
    ASSERT_EQ(normalized.type(), ik::ExpressionType::Add);
    const auto& root = std::get<ik::AddNode>(normalized.node().value);
    EXPECT_FALSE(ik::isConstant(root.lhs) && ik::isConstant(root.rhs));
}

TEST(ExpressionCanonicalizerTest, PreservesZeroDenominatorDefinedness)
{
    const Expression denominator = kFactory.subtract(sym("q"), sym("q"));
    const Expression input = kFactory.divide(sym("x"), denominator);
    const Expression normalized = norm(input);
    const auto& node = std::get<ik::DivideNode>(normalized.node().value);
    EXPECT_FALSE(ik::isZero(node.rhs));
    ik::ExpressionEvaluator evaluator{{{"q", 3.0}, {"x", 1.0}}};
    const auto before = evaluator.evaluate(input);
    const auto after = evaluator.evaluate(normalized);
    ASSERT_FALSE(before);
    ASSERT_FALSE(after);
    EXPECT_EQ(before.error().code, ik::EvaluationErrorCode::DivisionByZero);
    EXPECT_EQ(after.error().code, before.error().code);
}

TEST(ExpressionCanonicalizerTest, PreservesDeclinedOverflowingConstantSum)
{
    const Expression input = kFactory.add(con(1e308), con(1e308));
    EXPECT_TRUE(ik::sameNode(norm(input), input));
}

TEST(ExpressionCanonicalizerTest, KeepsOriginalDenominatorWhenConstantRegroupingProducesZero)
{
    const Expression overflowing = kFactory.add(con(1e308), con(1e308));
    const Expression denominator = kFactory.subtract(overflowing, overflowing);
    ASSERT_TRUE(ik::isZero(norm(denominator)));
    const Expression normalized = norm(kFactory.divide(sym("x"), denominator));
    ASSERT_EQ(normalized.type(), ik::ExpressionType::Divide);
    EXPECT_TRUE(ik::sameNode(std::get<ik::DivideNode>(normalized.node().value).rhs, denominator));
}

TEST(ExpressionCanonicalizerTest, PreservesDeclinedOverflowingConstantProduct)
{
    const Expression input = kFactory.multiply(con(1e308), con(1e308));
    EXPECT_TRUE(ik::sameNode(norm(input), input));
}

TEST(ExpressionCanonicalizerTest, NormalizesSignAwayFromZeroProduct)
{
    const Expression a = kFactory.multiply(kFactory.multiply(con(-1.0), con(0.0)), sym("x"));
    const Expression b = kFactory.multiply(con(-1.0), kFactory.multiply(con(0.0), sym("x")));
    const Expression explicitNegation = kFactory.negate(kFactory.multiply(con(0.0), sym("x")));
    EXPECT_TRUE(ik::structurallyEqual(norm(a), norm(b)));
    EXPECT_TRUE(ik::structurallyEqual(norm(a), norm(explicitNegation)));
}

TEST(ExpressionCanonicalizerTest, NormalizesSignAwayFromZeroNumerator)
{
    const Expression a = kFactory.divide(con(0.0), sym("q"));
    const Expression b = kFactory.negate(a);
    EXPECT_TRUE(ik::structurallyEqual(norm(a), norm(b)));
}

TEST(ExpressionCanonicalizerTest, NegatedZeroProductRetainsPartialFactorDomain)
{
    const Expression partial = kFactory.divide(con(1.0), sym("q"));
    const Expression product = kFactory.multiply(con(0.0), partial);
    const Expression normalized = norm(kFactory.negate(product));
    EXPECT_TRUE(ik::structurallyEqual(normalized, norm(product)));
    EXPECT_TRUE(ik::sameNode(normalized, norm(normalized)));
    ik::ExpressionEvaluator atZero{{{"q", 0.0}}};
    const auto undefined = atZero.evaluate(normalized);
    ASSERT_FALSE(undefined);
    EXPECT_EQ(undefined.error().code, ik::EvaluationErrorCode::DivisionByZero);
    ik::ExpressionEvaluator awayFromZero{{{"q", 2.0}}};
    const auto defined = awayFromZero.evaluate(normalized);
    ASSERT_TRUE(defined);
    EXPECT_EQ(*defined, 0.0);
}

// --- session and identity (2) --------------------------------------------

TEST(ExpressionCanonicalizerTest, KeepsNodeIdentityInAFreshSession)
{
    const Expression alreadyNormal = norm(kFactory.add(sym("b"), sym("a")));
    ik::CanonicalizationSession fresh{kFactory};
    EXPECT_TRUE(ik::sameNode(alreadyNormal, fresh.canonicalize(alreadyNormal)));
}

TEST(ExpressionCanonicalizerTest, PreservesSharingAcrossRoots)
{
    const Expression shared = kFactory.add(sym("b"), sym("a"));
    const Expression first = kFactory.sin(shared);
    const Expression second = kFactory.cos(shared);
    ik::CanonicalizationSession session{kFactory};
    const Expression normalizedFirst = session.canonicalize(first);
    const Expression normalizedSecond = session.canonicalize(second);
    const Expression firstOperand = std::get<ik::SinNode>(normalizedFirst.node().value).operand;
    const Expression secondOperand = std::get<ik::CosNode>(normalizedSecond.node().value).operand;
    EXPECT_TRUE(ik::sameNode(firstOperand, secondOperand));
}

// --- domain (1) -----------------------------------------------------------

TEST(ExpressionCanonicalizerTest, PreservesDefinednessAcrossCanonicalization)
{
    const Expression input = kFactory.divide(con(1.0), kFactory.add(sym("q"), sym("r")));
    const Expression normalized = norm(input);
    ik::ExpressionEvaluator before{{{"q", 1.0}, {"r", -1.0}}};
    ik::ExpressionEvaluator after{{{"q", 1.0}, {"r", -1.0}}};
    const auto beforeValue = before.evaluate(input);
    const auto afterValue = after.evaluate(normalized);
    ASSERT_FALSE(beforeValue.has_value());
    ASSERT_FALSE(afterValue.has_value());
    EXPECT_EQ(beforeValue.error().code, ik::EvaluationErrorCode::DivisionByZero);
    EXPECT_EQ(afterValue.error().code, ik::EvaluationErrorCode::DivisionByZero);
}

// --- exactness predicates (3) --------------------------------------------

TEST(ExpressionCanonicalizerTest, AgreesWithExactArithmeticOnBoundaryCases)
{
    struct Case { double lhs; double rhs; bool exact; };
    const std::array<Case, 10> products{{
        {1e200, 3.0, true}, {3.0, 4.0, true}, {0.5, 0.25, true},
        {std::nextafter(std::ldexp(1.0, -511), 1.0),
         std::nextafter(std::ldexp(1.0, -511), 1.0), false},
        {std::numeric_limits<double>::denorm_min(), 0.5, false},
        {std::ldexp(1.0, -1000), std::ldexp(1.0, -70), true},
        {std::nextafter(1.0, 2.0), 3.0, false},
        {std::numeric_limits<double>::max(), 1.0, true},
        {std::numeric_limits<double>::max(), 2.0, false},
        {4503599627370497.0, 2.0, true}}};
    for (const Case& product : products)
        EXPECT_EQ(ik::detail::multipliesExactly(product.lhs, product.rhs), product.exact);

    const std::array<Case, 5> sums{{
        {3.0, 4.0, true}, {1e16, 1.0, false},
        {std::ldexp(1.0, 52), std::ldexp(1.0, 52) + 1.0, false},
        {std::ldexp(1.0, 52) + 1.0, std::ldexp(1.0, 52) + 3.0, true},
        {std::numeric_limits<double>::denorm_min(),
         std::numeric_limits<double>::denorm_min(), true}}};
    for (const Case& sum : sums)
        EXPECT_EQ(ik::detail::addsExactly(sum.lhs, sum.rhs), sum.exact);
}

TEST(ExpressionCanonicalizerTest, RejectsProductWhoseRoundingErrorUnderflows)
{
    const double v = std::nextafter(std::ldexp(1.0, -511), 1.0);
    EXPECT_FALSE(ik::detail::multipliesExactly(v, v));
}

TEST(ExpressionCanonicalizerTest, AcceptsExactProductOutsideTheNormalRange)
{
    EXPECT_TRUE(ik::detail::multipliesExactly(std::ldexp(1.0, -1000),
                                               std::ldexp(1.0, -70)));
}

// --- EquationSimplifier (4) ----------------------------------------------

TEST(EquationSimplifierTest, PreservesEverythingButTheExpressions)
{
    auto system = syntheticPositionSystem();
    ASSERT_TRUE(system.has_value());
    ik::EquationSimplifier simplifier{kFactory};
    auto normalized = simplifier.normalize(*system);
    ASSERT_TRUE(normalized.has_value());
    EXPECT_EQ(normalized->taskKind(), system->taskKind());
    ASSERT_EQ(normalized->unknowns().size(), system->unknowns().size());
    ASSERT_EQ(normalized->equations().size(), system->equations().size());
    for (std::size_t i = 0; i < normalized->equations().size(); ++i)
        EXPECT_TRUE(ik::sameSource(normalized->equations()[i].source,
                                   system->equations()[i].source));
}

TEST(EquationSimplifierTest, SharesOneSessionAcrossTheSystem)
{
    const Expression shared = kFactory.add(sym("b"), sym("a"));
    std::vector<ik::Equation> equations{
        {kFactory.sin(shared), con(1.0), ik::PositionEquationSource{ik::CartesianComponent::X}},
        {kFactory.cos(shared), con(2.0), ik::PositionEquationSource{ik::CartesianComponent::Y}},
        {shared, con(3.0), ik::PositionEquationSource{ik::CartesianComponent::Z}}};
    auto system = ik::IkEquationSystem::create(ik::IkTaskKind::Position,
                                                {ik::JointVariable{"q1", 1}},
                                                std::move(equations));
    ASSERT_TRUE(system.has_value());
    ik::EquationSimplifier simplifier{kFactory};
    auto normalized = simplifier.normalize(*system);
    ASSERT_TRUE(normalized.has_value());
    const auto unique = countUniqueNodes(normalized->equations());
    EXPECT_LT(unique, 12u);
}

TEST(EquationSimplifierTest, AgreesNumericallyWithinToleranceOnFk)
{
    const std::array<const char*, 2> robots{{"kr4_r600.urdf", "kr640.urdf"}};
    std::mt19937_64 rng{0xC0FFEEu};
    std::uniform_real_distribution<double> angle{-1.0, 1.0};

    for (const char* robot : robots)
    {
        ik::IkEquationBuilder builder;
        const auto path = std::filesystem::path{KINEMAFORGE_URDF_DATA_DIR} / robot;
        ASSERT_TRUE(builder.loadRobotModel(path).has_value());
        ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
        ASSERT_TRUE(builder.buildForwardKinematics().has_value());
        auto system = builder.buildEquationSystem(identityPose());
        ASSERT_TRUE(system.has_value());

        ik::EquationSimplifier simplifier{kFactory};
        auto normalized = simplifier.normalize(*system);
        ASSERT_TRUE(normalized.has_value());

        for (int sample = 0; sample < 50; ++sample)
        {
            ik::SymbolValues values;
            for (const ik::JointVariable& unknown : system->unknowns())
                values.emplace(unknown.name, angle(rng));

            for (std::size_t i = 0; i < system->equations().size(); ++i)
            {
                ik::ExpressionEvaluator before{values};
                ik::ExpressionEvaluator after{values};
                const auto a = before.evaluate(system->equations()[i].lhs);
                const auto b = after.evaluate(normalized->equations()[i].lhs);
                ASSERT_EQ(a.has_value(), b.has_value());
                if (a && b)
                {
                    EXPECT_NEAR(*a, *b, 1e-12 * std::max({1.0, std::fabs(*a), std::fabs(*b)}));
                }
            }
        }
    }
}

TEST(EquationSimplifierTest, JoinsExactlyFoldableConstantsThroughTheChain)
{
    const Expression x = sym("x");
    const Expression a = kFactory.multiply(kFactory.multiply(con(1e200), con(3.0)), x);
    const Expression b = kFactory.multiply(con(1e200), kFactory.multiply(con(3.0), x));
    EXPECT_TRUE(ik::structurallyEqual(norm(a), norm(b)));
}

// ---- F3.R2: what the canonical form does NOT promise ------------------------
//
// These two tests pin documented behaviour. They are not regressions waiting to
// be fixed; changing either outcome is a change of contract.

TEST(ExpressionCanonicalizerTest, DoesNotRecoverConstantsAlreadyRoundedByTheFactory)
{
    // ExpressionFactory folds two ADJACENT constants at once, with rounding:
    // 1.91 + 0.1 becomes a single double. The canonicaliser merges constants
    // only when the sum is exact, and it cannot split a constant it was handed.
    // So the canonical form is a function of the expression as built.
    const Expression x = sym("x");
    const Expression separate = kFactory.add(kFactory.add(x, con(1.91)), con(0.1));
    const Expression folded = kFactory.add(x, kFactory.add(con(1.91), con(0.1)));

    EXPECT_FALSE(ik::structurallyEqual(norm(separate), norm(folded)));

    ik::ExpressionEvaluator evaluator{ik::SymbolValues{{"x", 0.25}}};
    const auto lhs = evaluator.evaluate(norm(separate));
    const auto rhs = evaluator.evaluate(norm(folded));
    ASSERT_TRUE(lhs.has_value());
    ASSERT_TRUE(rhs.has_value());
    EXPECT_NEAR(*lhs, *rhs, 1e-15);
}

TEST(ExpressionCanonicalizerTest, DomainGuaranteeIsStructuralNotBitwise)
{
    // What IS guaranteed: the divisor survives canonicalisation as a divisor,
    // even though 1 - (sin^2 a + cos^2 a) is algebraically zero.
    //
    // What is NOT guaranteed: the same ExpressionEvaluator outcome in double.
    // Canonicalisation reorders the sum, and whether it rounds to exactly zero
    // depends on that order and on the platform's sin/cos -- so a
    // DivisionByZero before can be a huge finite number after, and vice versa.
    // Singularity detection must not be built on evaluating a canonical form.
    const Expression a = sym("a");
    const Expression s = kFactory.sin(a);
    const Expression c = kFactory.cos(a);
    const Expression denominator = kFactory.subtract(
        con(1.0), kFactory.add(kFactory.multiply(s, s), kFactory.multiply(c, c)));
    const Expression quotient = kFactory.divide(sym("b"), denominator);

    const Expression canonical = norm(quotient);
    ASSERT_EQ(canonical.type(), ik::ExpressionType::Divide);
    const auto& node = std::get<ik::DivideNode>(canonical.node().value);
    EXPECT_FALSE(ik::isConstant(node.rhs));
    EXPECT_FALSE(ik::structurallyEqual(node.rhs, denominator)); // reordered
}
