#pragma once

#include "ik_equations/symbolic/Expression.hpp"

#include <cstddef>
#include <optional>
#include <string>

namespace kinemaforge::ik {

class ExpressionFactory;

// A rewrite rule: given an expression, either produce a replacement or decline.
//
// nullopt means "does not apply", NOT "applies and changes nothing". A rule
// that returned its input unchanged would spin the fixed-point loop, so the
// engine treats an unchanged result as a defect.
//
// THREE CONTRACTS, all owed by the rule's author and none checkable by the
// engine on its own:
//
//   (E)         the replacement is algebraically equal to the input ON THE
//               INPUT'S DOMAIN. The engine cannot verify this and does not try.
//
//   measure     the replacement has strictly fewer nodes counted WITH
//               MULTIPLICITY. This one is asserted in diagnostic builds.
//
//   determinism apply() must be deterministic with respect to the MEANING of
//               its arguments: for STRUCTURALLY EQUIVALENT inputs it makes the
//               same decision and produces STRUCTURALLY EQUIVALENT replacements.
//               External mutable state must not affect either.
//
// On the third, two halves, and both matter.
//
// "Structurally equivalent replacement", not "the same node": this factory does
// no hash-consing, so building multiply(constant(2), q) twice yields two nodes
// that are structurallyEqual and not sameNode. A rule is pure when its output
// means the same thing every time, not when it hands back the same allocation
// -- requiring the latter would demand something ExpressionFactory never
// promised.
//
// "Structurally equivalent INPUTS", not "the same input node": Add(x, x) with
// both operands the same node and Add(x1, x2) with two structurally equal but
// separately built operands mean the same thing, so a rule must answer the same
// way for both. Node identity may be used as a FAST PATH -- sameNode(lhs, rhs)
// short-circuits structurallyEqual -- but never as the decision:
//
//     if (sameNode(lhs, rhs)) simplify(); else decline();   // FORBIDDEN
//
// A rule shaped like that makes the normal form depend on how the DAG happened
// to be allocated earlier, which is a difference no caller controls and no test
// reproduces reliably. F3.2 wants canonical forms, not forms that remember
// their allocation history.
//
// Diagnostic side effects are allowed as long as they cannot reach the result.
// Counting how often a rule was consulted is fine, and the engine's own tests
// rely on it; declining the first time and applying the second is not.
//
// Determinism is not decoration. The session memoises results across roots, so
// a rule consulted once may never be consulted again for that node; a rule
// whose answer depends on when it was asked makes the memo hold a claim the
// rules would no longer agree with.
struct RewriteRule
{
    // An owning std::string, not a string_view: the session outlives the call
    // that built its rule array, so a view would point into storage the session
    // does not control. A dozen rules copied once at construction cost nothing.
    std::string name;

    std::optional<Expression> (*apply)(const Expression&, const ExpressionFactory&);
};

// A defensive operational budget, NOT a bound derived from the size of the
// input DAG. Termination rests on each rule's own decreasing measure; this
// limit exists as an independent net for the case where that contract is
// broken. Sanity check only: the largest forward-kinematics DAG in this project
// measures 153 703 nodes with multiplicity (kr4_r600), so a million leaves
// ample headroom over anything the pipeline builds today.
inline constexpr std::size_t kDefaultRewriteStepLimit = 1'000'000;

// The result of one rewrite() call. Both flags answer DIFFERENT questions and
// are deliberately independent, because a session-wide memo makes them diverge:
//
//   changed   the output differs from the input OF THIS CALL
//   steps     new rule applications accepted DURING THIS CALL; a memo hit is
//             not a step
//
// A rule can fire while the output stays identical (the representability guard
// drops the replacement at the parent): changed = false, steps = 1. And the
// output can differ while no rule fires here (the memo already held the
// result): changed = true, steps = 0.
//
// Consumers driving repeated passes must test `changed`, never `steps > 0`.
//
// reachedLimit is NOT an error, which is why this is not an expected<>.
//
// The returned Expression is structurally valid and respects the engine's
// representability invariants. Assuming every accepted rule satisfies its
// algebraic-equivalence contract (E), the result is equivalent to the input and
// has the same domain. The engine itself enforces only the LOCAL syntactic
// domain-guard condition; GLOBAL domain preservation also relies on (E).
//
// That last sentence is not hedging. A rule breaking (E) can preserve the
// syntactic guards and still widen the domain:
//
//     a / sin(q)     with a broken rule  sin(q) -> 1
//     guards {} -> {}, measure 2 -> 1    => the engine accepts it
//     domain q != k*pi  becomes  every q
//
// The induction that carries the local condition to the whole tree runs
// d == d' => zeroSet(d) == zeroSet(d'), and that premise IS (E).
//
// The expression may simply not have reached its canonical form.
struct RewriteResult
{
    Expression expression;
    bool changed{};
    std::size_t steps{};
    bool reachedLimit{};
};

} // namespace kinemaforge::ik
