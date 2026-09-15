#pragma once

#include "ik_equations/symbolic/Expression.hpp"

#include <cstddef>
#include <functional>
#include <unordered_map>

namespace kinemaforge::ik::detail {

// Hashing and equality by NODE IDENTITY, with the key owning its node: an entry
// keeps its node alive, so a cache filled while rewriting a temporary cannot be
// answered for a different node that later reuses the address.
struct ExpressionIdentityHash
{
    std::size_t operator()(const Expression& expression) const noexcept
    {
        return std::hash<const ExpressionNode*>{}(&expression.node());
    }
};

struct ExpressionIdentityEqual
{
    bool operator()(const Expression& lhs, const Expression& rhs) const noexcept
    {
        return sameNode(lhs, rhs);
    }
};

template <typename Value>
using ExpressionMap =
    std::unordered_map<Expression, Value, ExpressionIdentityHash, ExpressionIdentityEqual>;

} // namespace kinemaforge::ik::detail
