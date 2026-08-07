#include <gtest/gtest.h>

#include "ik_equations/model/Equation.hpp"
#include "ik_equations/model/IkEquationSystem.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using kinemaforge::ik::CartesianComponent;
using kinemaforge::ik::Equation;
using kinemaforge::ik::EquationKind;
using kinemaforge::ik::EquationSource;
using kinemaforge::ik::Expression;
using kinemaforge::ik::ExpressionFactory;
using kinemaforge::ik::IkEquationSystem;
using kinemaforge::ik::IkEquationSystemErrorCode;
using kinemaforge::ik::IkTaskKind;
using kinemaforge::ik::JointVariable;
using kinemaforge::ik::OrientationEquationSource;
using kinemaforge::ik::PositionEquationSource;
using kinemaforge::ik::kindOf;
using kinemaforge::ik::sameNode;

namespace {

Equation positionEquation(const ExpressionFactory& factory, CartesianComponent component,
                          const char* symbol = "q1")
{
    return Equation{factory.symbol(symbol), factory.constant(1.0),
                    PositionEquationSource{component}};
}

Equation orientationEquation(const ExpressionFactory& factory, std::size_t row,
                             std::size_t column)
{
    const auto source = OrientationEquationSource::create(row, column);
    // Throws rather than EXPECT + dereference: a broken factory would
    // otherwise turn every test using this helper into UB instead of a
    // failure.
    if (!source)
        throw std::logic_error("valid orientation cell rejected by create()");
    return Equation{factory.symbol("q1"), factory.constant(0.0), *source};
}

std::vector<Equation> positionEquations(const ExpressionFactory& factory)
{
    return {positionEquation(factory, CartesianComponent::X),
            positionEquation(factory, CartesianComponent::Y),
            positionEquation(factory, CartesianComponent::Z)};
}

std::vector<JointVariable> twoUnknowns()
{
    return {JointVariable{"q1", 1}, JointVariable{"q2", 2}};
}

} // namespace

// --- Expression ownership -------------------------------------------

TEST(IkEquationModelTest, StoresEquationSidesByValue)
{
    // The equation must outlive the handles it was built from -- a system is a
    // snapshot, not a view into the builder's stack.
    const ExpressionFactory factory;

    Equation equation = [&factory] {
        const Expression lhs = factory.symbol("q1");
        const Expression rhs = factory.constant(2.5);
        return Equation{lhs, rhs, PositionEquationSource{CartesianComponent::X}};
    }();

    EXPECT_EQ(equation.lhs.type(), kinemaforge::ik::ExpressionType::Symbol);
    EXPECT_TRUE(kinemaforge::ik::isConstant(equation.rhs));
    EXPECT_DOUBLE_EQ(kinemaforge::ik::constantValue(equation.rhs), 2.5);
}

TEST(IkEquationModelTest, CopiedEquationSharesImmutableExpressionNodes)
{
    // By value is cheap: Expression is a handle over shared_ptr<const Node>,
    // so copying an equation copies two pointers, never two trees.
    const ExpressionFactory factory;
    const Equation original = positionEquation(factory, CartesianComponent::X);

    const Equation copy = original;

    EXPECT_TRUE(sameNode(original.lhs, copy.lhs));
    EXPECT_TRUE(sameNode(original.rhs, copy.rhs));
}

TEST(IkEquationModelTest, CopiesIkEquationSystemWithoutDeepCopyingExpressions)
{
    const ExpressionFactory factory;
    const auto original =
        IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(), positionEquations(factory));
    ASSERT_TRUE(original.has_value());

    const IkEquationSystem copy = *original;

    ASSERT_EQ(copy.equations().size(), original->equations().size());
    for (std::size_t index = 0; index < copy.equations().size(); ++index)
    {
        SCOPED_TRACE(testing::Message() << "equation " << index);
        EXPECT_TRUE(sameNode(original->equations()[index].lhs, copy.equations()[index].lhs));
        EXPECT_TRUE(sameNode(original->equations()[index].rhs, copy.equations()[index].rhs));
    }
}

// --- IkEquationSystem invariants ------------------------------------

TEST(IkEquationModelTest, CreatesValidSystem)
{
    const ExpressionFactory factory;

    const auto system =
        IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(), positionEquations(factory));

    ASSERT_TRUE(system.has_value());
    EXPECT_EQ(system->taskKind(), IkTaskKind::Position);
    EXPECT_EQ(system->unknowns().size(), 2u);
    EXPECT_EQ(system->equations().size(), 3u);
    EXPECT_EQ(kindOf(system->equations()[0].source), EquationKind::Position);
}

TEST(IkEquationModelTest, RejectsSystemWithoutEquations)
{
    {
        SCOPED_TRACE("equations only");
        const auto system = IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(), {});
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::NoEquations);
    }
    {
        // Both broken at once: NoEquations wins, per the documented order.
        // Without this the priority is a comment, not a contract.
        SCOPED_TRACE("equations and unknowns both empty");
        const auto system = IkEquationSystem::create(IkTaskKind::Position, {}, {});
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::NoEquations);
    }
}

TEST(IkEquationModelTest, RejectsSystemWithoutUnknowns)
{
    const ExpressionFactory factory;

    const auto system = IkEquationSystem::create(IkTaskKind::Position, {}, positionEquations(factory));

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::NoUnknowns);
}

TEST(IkEquationModelTest, RejectsUnknownWithEmptyName)
{
    const ExpressionFactory factory;
    std::vector<JointVariable> unknowns{JointVariable{"q1", 1}, JointVariable{"", 2}};

    const auto system = IkEquationSystem::create(IkTaskKind::Position, std::move(unknowns),
                                                 positionEquations(factory));

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::EmptyUnknownName);
}

TEST(IkEquationModelTest, RejectsDuplicateUnknownNames)
{
    const ExpressionFactory factory;

    {
        // Distinct indices, same name. For the symbolic layer "q1" is ONE
        // node, so the solver would get two supposedly different unknowns
        // pointing at it.
        SCOPED_TRACE("same name, distinct indices");
        std::vector<JointVariable> unknowns{JointVariable{"q1", 1}, JointVariable{"q1", 2}};
        const auto system = IkEquationSystem::create(IkTaskKind::Position, std::move(unknowns),
                                                     positionEquations(factory));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::DuplicateUnknownName);
    }
    {
        // Breaks three rules at once: duplicate name, duplicate index, and
        // non-increasing order. The name must win.
        SCOPED_TRACE("same name and same index");
        std::vector<JointVariable> unknowns{JointVariable{"q1", 1}, JointVariable{"q1", 1}};
        const auto system = IkEquationSystem::create(IkTaskKind::Position, std::move(unknowns),
                                                     positionEquations(factory));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::DuplicateUnknownName);
    }
}

TEST(IkEquationModelTest, RejectsDuplicateUnknownIndices)
{
    // Distinct names, same index. Must be reported as a duplicate rather than
    // as bad ordering -- equal indices also break the strict ordering rule, so
    // the check order is what makes this diagnosis useful.
    const ExpressionFactory factory;
    std::vector<JointVariable> unknowns{JointVariable{"q1", 1}, JointVariable{"q2", 1}};

    const auto system = IkEquationSystem::create(IkTaskKind::Position, std::move(unknowns),
                                                 positionEquations(factory));

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::DuplicateUnknownIndex);
}

TEST(IkEquationModelTest, RejectsUnorderedUnknowns)
{
    const ExpressionFactory factory;
    std::vector<JointVariable> unknowns{JointVariable{"q2", 2}, JointVariable{"q1", 1}};

    const auto system = IkEquationSystem::create(IkTaskKind::Position, std::move(unknowns),
                                                 positionEquations(factory));

    ASSERT_FALSE(system.has_value());
    EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::UnorderedUnknowns);
}

TEST(IkEquationModelTest, RejectsTaskEquationKindMismatch)
{
    const ExpressionFactory factory;

    {
        SCOPED_TRACE("orientation equation in a position task");
        auto equations = positionEquations(factory);
        equations.push_back(orientationEquation(factory, 0, 0));

        const auto system = IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::TaskEquationMismatch);
    }
    {
        SCOPED_TRACE("pose task without orientation equations");
        const auto system = IkEquationSystem::create(IkTaskKind::Pose, twoUnknowns(),
                                                     positionEquations(factory));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::TaskEquationMismatch);
    }
    {
        SCOPED_TRACE("pose task missing a position component");
        std::vector<Equation> equations{positionEquation(factory, CartesianComponent::X),
                                        positionEquation(factory, CartesianComponent::Y),
                                        orientationEquation(factory, 0, 0)};

        const auto system = IkEquationSystem::create(IkTaskKind::Pose, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::TaskEquationMismatch);
    }
    {
        // A forged enumerator: a valid object of the enum type with no
        // enumerator behind it. Without componentIndex this would index
        // present[99].
        SCOPED_TRACE("invalid Cartesian component");
        auto equations = positionEquations(factory);
        equations[0].source = PositionEquationSource{static_cast<CartesianComponent>(99)};

        const auto system = IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::TaskEquationMismatch);
    }
    {
        // The same for the task kind: -Wswitch guards new enumerators at
        // compile time, this guards forged ones at run time.
        SCOPED_TRACE("invalid IK task kind");
        const auto system = IkEquationSystem::create(static_cast<IkTaskKind>(99), twoUnknowns(),
                                                     positionEquations(factory));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::TaskEquationMismatch);
    }
}

TEST(IkEquationModelTest, RejectsDuplicateEquationSource)
{
    // Two equations addressing one cell: redundant at best, contradictory at
    // worst. Reported as its own fault rather than falling out of the strict
    // ordering rule -- the document and the code must agree on duplicates.
    const ExpressionFactory factory;

    {
        SCOPED_TRACE("duplicate position component");
        std::vector<Equation> equations{positionEquation(factory, CartesianComponent::X),
                                        positionEquation(factory, CartesianComponent::Y),
                                        positionEquation(factory, CartesianComponent::Z),
                                        positionEquation(factory, CartesianComponent::Y)};

        const auto system = IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::DuplicateEquationSource);
    }
    {
        SCOPED_TRACE("duplicate orientation cell");
        auto equations = positionEquations(factory);
        equations.push_back(orientationEquation(factory, 1, 1));
        equations.push_back(orientationEquation(factory, 1, 1));

        const auto system = IkEquationSystem::create(IkTaskKind::Pose, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::DuplicateEquationSource);
    }
}

TEST(IkEquationModelTest, RejectsUnorderedEquations)
{
    const ExpressionFactory factory;

    {
        SCOPED_TRACE("position components out of order");
        std::vector<Equation> equations{positionEquation(factory, CartesianComponent::Z),
                                        positionEquation(factory, CartesianComponent::X),
                                        positionEquation(factory, CartesianComponent::Y)};

        const auto system = IkEquationSystem::create(IkTaskKind::Position, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::UnorderedEquations);
    }
    {
        SCOPED_TRACE("orientation cells not row-major");
        auto equations = positionEquations(factory);
        equations.push_back(orientationEquation(factory, 1, 0));
        equations.push_back(orientationEquation(factory, 0, 1));

        const auto system = IkEquationSystem::create(IkTaskKind::Pose, twoUnknowns(),
                                                     std::move(equations));
        ASSERT_FALSE(system.has_value());
        EXPECT_EQ(system.error().code, IkEquationSystemErrorCode::UnorderedEquations);
    }
}

// --- EquationSource -------------------------------------------------

TEST(IkEquationModelTest, CreatesOrientationSourceForValidCell)
{
    const auto corner = OrientationEquationSource::create(0, 0);
    ASSERT_TRUE(corner.has_value());
    EXPECT_EQ(corner->row(), 0u);
    EXPECT_EQ(corner->column(), 0u);

    const auto opposite = OrientationEquationSource::create(2, 2);
    ASSERT_TRUE(opposite.has_value());
    EXPECT_EQ(opposite->row(), 2u);
    EXPECT_EQ(opposite->column(), 2u);

    // The other half of kindOf: only the position side was covered above.
    EXPECT_EQ(kindOf(EquationSource{*corner}), EquationKind::Orientation);
}

TEST(IkEquationModelTest, RejectsOutOfRangeOrientationSource)
{
    // create is the ONLY way to build one: the constructor is private, so
    // there is no OrientationEquationSource{7, 9} to reject at all.
    EXPECT_FALSE(OrientationEquationSource::create(3, 0).has_value());
    EXPECT_FALSE(OrientationEquationSource::create(0, 3).has_value());
    EXPECT_FALSE(OrientationEquationSource::create(7, 9).has_value());
}
