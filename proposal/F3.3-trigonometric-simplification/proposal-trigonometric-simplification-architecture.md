# Proposal: F3.3 — trigonometric simplification without widening the domain

**Stage:** F3.3 · **architecture v1 — approved for implementation in this work session** · 2026-09-13

## 1. Problem

F3.2 gives a deterministic AC-normal form for `Add`/`Multiply`, subtraction and sign.
The next stage must simplify trigonometric structure without turning the symbolic layer
into a general CAS and without breaking the domain discipline established in Phase 1.

The roadmap lists `sin(-x)`, `cos(-x)`, `sin²+cos²`, common arguments and equation-side
manipulation. A spike against the current contracts changes one important assumption:
`sin²(u)+cos²(u)->1` is **not globally domain-safe** when `u` is partial. For
`u=1/q`, the left side is undefined at `q=0`, while literal `1` is defined there.
Therefore the identity may fire only when `u` has no division guards.

## 2. Scope

F3.3 adds a short-lived `TrigonometricSimplificationSession` operating on one shared DAG.
It assumes/produces F3.2 AC-normal form and implements only:

1. `cos(-x) -> cos(x)`;
2. `sin(-x) -> -sin(x)`;
3. `sin(x)*sin(x) + cos(x)*cos(x) -> 1` inside an AC sum, including when the pair is
   surrounded by unrelated terms;
4. repeated application to a fixed point;
5. one session across every side of every equation through `EquationSimplifier::simplify`.

`cos(-x)` is also exposed as the first production `RewriteRule` for the F3.1 engine: it
strictly decreases the node-count measure. `sin(-x)` has equal node count before/after,
so forcing it through `RewriteEngine` would violate F3.1's rule contract; it is a
normalisation step in the trig session instead.

## 3. Domain contract

`sin` and `cos` are total on finite real inputs, so moving an existing sign across them
preserves every divisor guard in the operand.

The Pythagorean rule fires only if `divisorsOf(argument).empty()`. This is deliberately
stronger than algebraic validity: it preserves the project's symbolic definedness exactly.
No guard object is introduced in F3.3.

## 4. Representability contract

A Pythagorean pair may be replaced by `1` only if the resulting AC sum can be represented
without forcing `ExpressionFactory` to perform a new inexact all-constant fold. If the
pair is the whole sum, the result is simply `1`. If at least one non-constant summand
remains, F3.2 can attach constant residuals outside that symbolic material safely.
If only constants remain and exact reconstruction is impossible, the original sum is kept.

This is the F3.2 notion of a representability barrier applied to a new rewrite.

## 5. Equation policy

F3.3 **does not move terms across `lhs = rhs` and does not cancel equal terms across the
sides**. The older roadmap predates the final `Equation` contract, which intentionally
preserves which side came from robot geometry and which came from the task. Cross-side
algebra belongs in the solver IR (F3.6), where domain conditions can be represented
explicitly. Doing it inside `Equation` would silently destroy provenance and cancellation
of a partial term could widen the domain (`1/q+a = 1/q+b`).

`EquationSimplifier::normalize` remains the F3.2 operation. A new `simplify` method performs
AC normalisation plus this trig stage while copying all non-expression fields unchanged.

## 6. Termination and idempotence

The session is one-way:

- `Sin(Negate(x))` becomes `Negate(Sin(x))` and never matches again;
- `Cos(Negate(x))` loses one node;
- each Pythagorean firing removes at least six nodes.

After every local change the result is AC-normalised. The session memo stores fixed points
by node identity. A structurally unchanged result returns the original node.

## 7. Non-goals

No distributivity, no angle-addition formulas, no `sin(a±b)` expansion, no collection of
like terms, no inverse trig, no cross-equation elimination, no solver-specific isolation.

## 8. Gate

- F3.2 regression suite remains green;
- direct tests for sign normalisation and Pythagorean reduction;
- explicit `1/(q)` argument test proving the identity is rejected when it would widen the domain;
- fixed-point / fresh-session node identity;
- numeric equivalence over random finite substitutions;
- full warning-clean compile in both diagnostic and `NDEBUG` configurations where dependencies permit.
