#include "ik_equations/symbolic/Expression.hpp"

#include <cassert>
#include <type_traits>
#include <utility>

namespace kinemaforge::ik {

namespace {

// Maps a node to its ExpressionType. Written out rather than casting
// variant::index(), so adding a node type without listing it here is a
// compile error instead of a silently wrong enum value.
struct TypeOfNode
{
    ExpressionType operator()(const ConstantNode&) const noexcept { return ExpressionType::Constant; }
    ExpressionType operator()(const SymbolNode&)   const noexcept { return ExpressionType::Symbol; }
    ExpressionType operator()(const AddNode&)      const noexcept { return ExpressionType::Add; }
    ExpressionType operator()(const SubtractNode&) const noexcept { return ExpressionType::Subtract; }
    ExpressionType operator()(const MultiplyNode&) const noexcept { return ExpressionType::Multiply; }
    ExpressionType operator()(const DivideNode&)   const noexcept { return ExpressionType::Divide; }
    ExpressionType operator()(const NegateNode&)   const noexcept { return ExpressionType::Negate; }
    ExpressionType operator()(const SinNode&)      const noexcept { return ExpressionType::Sin; }
    ExpressionType operator()(const CosNode&)      const noexcept { return ExpressionType::Cos; }
};

// Compares two nodes already known to be the same alternative.
struct StructuralComparator
{
    bool operator()(const ConstantNode& lhs, const ConstantNode& rhs) const
    {
        // Exact comparison on purpose: this is a structural predicate,
        // not a numeric one.
        return lhs.value == rhs.value;
    }

    bool operator()(const SymbolNode& lhs, const SymbolNode& rhs) const
    {
        return lhs.name == rhs.name;
    }

    template <typename Node>
        requires requires(const Node& n) { n.lhs; n.rhs; }
    bool operator()(const Node& lhs, const Node& rhs) const
    {
        return structurallyEqual(lhs.lhs, rhs.lhs) && structurallyEqual(lhs.rhs, rhs.rhs);
    }

    template <typename Node>
        requires requires(const Node& n) { n.operand; }
    bool operator()(const Node& lhs, const Node& rhs) const
    {
        return structurallyEqual(lhs.operand, rhs.operand);
    }
};

} // namespace

// std::make_shared is avoided on purpose in this file. Under GCC 13.1 on
// MinGW it emits a strong definition of std::type_info::operator==, which
// collides with the one in libstdc++.a as soon as -static-libstdc++ is on
// (this project needs that flag; see CMakeLists.txt). The plain
// shared_ptr(new T) form does not, at the cost of a second allocation for
// the control block — negligible here, since building an expression tree
// happens once per robot, not in a hot loop.
const std::shared_ptr<const ExpressionNode>& Expression::sharedZeroNode()
{
    static const std::shared_ptr<const ExpressionNode> zero{
        new ExpressionNode{ConstantNode{0.0}}};
    return zero;
}

Expression::Expression() : node_(sharedZeroNode()) {}

Expression::Expression(ExpressionNode node)
    : node_(new ExpressionNode{std::move(node)})
{
}

ExpressionType Expression::type() const noexcept
{
    // The variant is set once at construction and never reassigned, so it
    // cannot become valueless and std::visit cannot throw here.
    return std::visit(TypeOfNode{}, node_->value);
}

const ExpressionNode& Expression::node() const noexcept
{
    return *node_;
}

bool isConstant(const Expression& expression) noexcept
{
    return std::holds_alternative<ConstantNode>(expression.node().value);
}

bool isZero(const Expression& expression) noexcept
{
    const auto* constant = std::get_if<ConstantNode>(&expression.node().value);
    return constant != nullptr && constant->value == 0.0;
}

bool isOne(const Expression& expression) noexcept
{
    const auto* constant = std::get_if<ConstantNode>(&expression.node().value);
    return constant != nullptr && constant->value == 1.0;
}

double constantValue(const Expression& expression)
{
    const auto* constant = std::get_if<ConstantNode>(&expression.node().value);
    assert(constant != nullptr && "constantValue() requires a Constant expression");
    return constant->value;
}

bool sameNode(const Expression& lhs, const Expression& rhs) noexcept
{
    return &lhs.node() == &rhs.node();
}

bool structurallyEqual(const Expression& lhs, const Expression& rhs)
{
    if (sameNode(lhs, rhs))
        return true;
    if (lhs.type() != rhs.type())
        return false;

    // Single visit: the alternatives already match, so the right-hand
    // node can be fetched directly. Visiting both variants would
    // instantiate 9x9 combinations, all but nine of them dead.
    return std::visit(
        [&rhs](const auto& left) {
            using Node = std::decay_t<decltype(left)>;
            return StructuralComparator{}(left, std::get<Node>(rhs.node().value));
        },
        lhs.node().value);
}

} // namespace kinemaforge::ik
