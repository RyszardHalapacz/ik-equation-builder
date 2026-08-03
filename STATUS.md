# Project Status

**Last updated:** 2026-08-03
**Branch:** `main`
**Build:** clean · **Tests:** 161/161 passing

A snapshot of what actually exists in this repo today, and what is still
declaration-only. The pipeline this project is working toward is described in
[`README.md`](README.md); this file tracks how far along it is.

## How far along, honestly

Two numbers, because they differ a lot depending on where the finish line is:

| Scope | Progress |
|---|---|
| **Phase 1** — URDF → symbolic forward kinematics | **~85%** |
| **Full vision** — including IK solver and code generation | **~40%** by component count |

The second number is misleading and worth stating plainly: **what is done is the
easy part.** Parsing XML, walking a tree and building an expression engine are
solved, well-understood problems. What remains includes symbolically solving IK
equations — research-grade work, plausibly larger on its own than everything
built so far. Measured by effort rather than component count, the honest figure
is closer to **25%**.

Phase 1 is at ~85% rather than done: the symbolic FK equation now falls out of a
URDF end to end, but **nothing yet checks that it is numerically right** (see
`ExpressionEvaluator` under known gaps), and the facade is still undefined.

The upside: the foundation is solid. 161 tests, every architectural decision
written down and reviewed, and a long list of traps caught *before* they reached
the code — expression-domain loss from `x * 0 → 0`, trig-folding noise, a
`make_shared` symbol clash under `-static-libstdc++`, `std::hypot` losing
subnormal axes, a parser that silently rewrote malformed vectors,
`I · R_motion` quietly producing `cos(q) + 0 · sin(q)`, and a full 4×4 product
turning the homogeneous last row into 31 738 nodes.

## Pipeline progress

```
URDF
  → UrdfModelLoader          ✅ implemented + tested + validating
  → RobotDescription         ✅ complete, canonical geometry guaranteed
  → KinematicChainBuilder    ✅ implemented + tested
  → KinematicChain           ✅ complete
  → Symbolic layer           ✅ implemented + tested
  → JointTransformBuilder    ✅ implemented + tested
  → ForwardKinematicsBuilder ✅ implemented + tested
  → Symbolic FK              ✅ reachable end to end, structure verified only
  → ExpressionEvaluator      🟡 nothing yet — blocks numeric validation — next
  → ConstraintBuilder        ⬜ analysed only, no proposal yet
  → EquationSimplifier       ⬜ nothing
  → EquationSolver           ⬜ nothing (the hard one)
  → IkPatternDetector        ⬜ nothing
  → CodeGenerator            ⬜ nothing
```

## Done

### `UrdfModelLoader` — parsing and validation

Turns a URDF file into a `RobotDescription` and **guarantees canonical
geometry**:

- `origin.translation` and `origin.rpy` hold only finite values
- an actuated joint's `axis` is finite, non-zero and numerically normalized
- a fixed joint's `axis` carries no meaning

The parser requires exactly three whitespace-separated finite numbers and
rejects malformed tokens, wrong counts, trailing garbage, `nan`/`inf` and
overflow. A missing attribute means "use the default"; an attribute that is
present but empty is an error. Axes are normalized by scaling to the largest
component before `std::hypot` — plain `hypot` loses direction for subnormal
input, and naive `sqrt(x²+y²+z²)` fails on both overflow and underflow.

Errors carry context (`LoadErrorCode` plus joint name, attribute and raw
value, both truncated), and the message is assembled at the `ik` boundary.

Parsing stays strictly structural: `origin.rpy` and `axis` are never composed
or rotated. That contract is what `MapsKr4JointOriginRotation` and
`MapsKr4JointAxis` pin down.

**Tests:** 56.

### `KinematicChainBuilder` — topology

Resolves the single path between a requested base link and tool link, and
returns it as an ordered `KinematicChain`. Preserves topology only.

- Returns `std::expected<KinematicChain, KinematicChainError>` (no exceptions).
- Iterative DFS over an explicit stack; no recursion.
- Fixed joints stay in the chain — they carry real offsets.
- Only actuated joints get a `q1..qn` variable.
- `baseLink == toolLink` returns an empty chain, not an error.
- A `childLink` claimed by two joints is rejected as `InvalidRobotDescription`.

**Tests:** 17.

### Symbolic layer — expression engine

`Expression` is a 16-byte handle over `shared_ptr<const ExpressionNode>`;
copying never clones the tree. The node constructor is private with
`ExpressionFactory` as `friend`, so the factory's contracts hold for the whole
graph.

Constant folding and neutral elements (`x + 0 → x`, `x * 1 → x`) are applied
during construction. **`x * 0 → 0` deliberately is not**: it would erase domain
information — `(1/q) * 0` is undefined at `q = 0`, but a folded `0` claims
otherwise. Measured cost: FK trees grow from 87 to 375 nodes, accepted so the
future simplifier can remove those terms correctly rather than blindly.

`SymbolicMatrix` provides `operator()`, `zeros()`, `identity()` and a free
`multiply(lhs, rhs, factory)`.

**Tests:** 43.

### `JointTransformBuilder` — one joint → one symbolic transform

`T_parent_child(q) = T_origin · T_motion(q)`, with `T_origin = Translation ·
R_rpy` and `R_rpy = Rz(yaw) · Ry(pitch) · Rx(roll)` — URDF's fixed-axis
convention. Handles `Fixed`, `Revolute`, `Continuous` and `Prismatic`;
`Continuous` is geometrically identical to `Revolute`, since the difference is a
limit and limits are not this component's concern.

The public surface is one constructor and one method. Everything else lives in
an anonymous namespace in the `.cpp`.

Preconditions are **asserted, not re-validated** — the loader already guarantees
finite origins and a normalized axis, and `NDEBUG` is not defined in the default
build, so the assertions are live:

- a fixed joint has no variable; an actuated one has exactly one
- an actuated joint's axis is unit length
- the symbol name comes from `joint.variable->name`, never invented here

Four construction decisions, each of which exists because the symbolic layer
deliberately has no `x * 0 → 0` rule:

- **Signed principal axes take a fast path** to a canonical `Rx`/`Ry`/`Rz`
  (33 → 7 nodes). The sign is folded into the sine — `R(-a, q) = R(a, -q)` and
  cos is even — so no cell carries `Negate(Negate(Sin))`. The match is an exact
  comparison against `±1.0`: a slightly tilted axis is a different axis.
- **An identity `R_origin` skips the product entirely.** `I · R_motion` would
  otherwise yield `Add(Cos(q), Multiply(Constant(0), Sin(q)))` instead of
  `Cos(q)` — and all 7 joints of `kr640.urdf` have `rpy="0 0 0"`. The same
  shortcut covers `I · vector` for prismatic joints.
- **Blocks are assembled, never multiplied as full 4×4.** Starting from
  `identity()` keeps the last row exactly `[0 0 0 1]`; a 4×4 product would turn
  it into `0*R00 + 0*R10 + 0*R20` — zero mathematically, but no longer a
  `Constant(0)` that `isZero()` recognises.
- **A zero axis component builds no multiplication** for prismatic
  displacement. This is the one place applying the rejected annihilator
  locally, and it is sound for a reason the general rule lacks: the other
  operand is a bare joint variable, total on the reals.

Both fast paths are choices of *construction* based on constants known while
building — not symbolic rewrites. Both branches denote the same function.

**Tests:** 22.

### `multiplyTransforms` — homogeneous composition in the symbolic layer

Lives in `SymbolicTransform.hpp/.cpp`, not in the FK builder: it is an operation
on canonical homogeneous transforms, and the constraint builder and solver will
need it too.

```
R = R_lhs · R_rhs          p = p_lhs + R_lhs · p_rhs          last row assembled
```

**Precondition, asserted not validated:** both operands have an exactly
canonical last row (`hasCanonicalHomogeneousLastRow`). The contract is closed
under composition — every producer in the project guarantees it, including this
function, so its result is again a valid input.

Multiplying two full 4×4 matrices instead would destroy that. Measured on
`kr640.urdf`: cell `(3,0)` becomes an `Add` tree of **31 738 nodes** that is
zero mathematically but no longer a `Constant(0)`, and `(3,3)` the same for the
constant 1.

**Five explicit fast paths, reproducing six algebraic identities** —
`I·T = T`, `T·I = T`, `I·R = R`, `R·I = R`, `R·0 = 0`, `I·p = p`. These are not
tuned to the robots in `data/urdf`; they restore identities the symbolic layer
loses for want of `x · 0 → 0`. Without them, `T · I` differs from `T` in 8 of 16
cells and an identity right rotation corrupts 6 of the 9 rotation cells. There
is deliberately **no** fast path for a zero `p_lhs`: the factory already folds
`0 + x → x` as well as `x + 0 → x`.

**Tests:** 10, in `test_symbolic_transform.cpp` — the algebra is tested directly,
not only through the builder.

### `ForwardKinematicsBuilder` — chain → one symbolic transform

Left fold from the identity, accumulator on the left:

```
T_base_tool(q) = T_1(q1) · T_2(q2) · ... · T_n(qn)
```

The empty chain (`baseLink == toolLink`, which the chain builder reports as
success) yields the identity — the empty product — with no special case. A
single-joint chain returns `JointTransformBuilder`'s output **structurally
unchanged**, which is what pins the identity-accumulator fast path.

No error model: no `KinematicChain` can make this computation fail.

Measured tree size, verified against the shipped code rather than a prototype:

| Robot | nodes (with multiplicity) | unique (DAG) | depth |
|---|---:|---:|---:|
| `kr640.urdf` | 21 882 | 281 | 22 |
| `kr4_r600.urdf` | 153 703 | 516 | 24 |

A naive 4×4 accumulation gives 466 848 nodes for `kr640` — **21×** more, with a
broken last row.

**Tests:** 12.

### Supporting layers

- `mt::kinematics` (`robot_model`, `robot_model_loader`) — implemented.
- `mt::DiagnosticBag` — implemented; populated by `load_urdf` on success, and
  discarded at the `UrdfModelLoader` boundary.
- Model structs — complete aggregates, no behaviour.

## Not done

### What Phase 1 still needs

Three things, not one. Symbolic FK exists, but the phase is not finished by the
facade alone:

1. **`ExpressionEvaluator`** — substituting `q` and evaluating the tree. Nothing
   in the project can do this, so the FK equation's *values* have never been
   checked (see known gaps).
2. **End-to-end numeric validation of FK for KR4 and KR640** — the payoff of
   the evaluator, and the first real proof that the RPY convention, the axis
   handling and the `p_a + R_a·p_b` composition are jointly correct.
3. **`IkEquationBuilder`** — the facade below.

### `IkEquationBuilder` (facade) — constructor only

`loadRobotModel()`, `selectChain()`, `buildForwardKinematics()`,
`kinematicChain()` and `forwardKinematics()` are declared and **undefined**. It
links because `main.cpp` only default-constructs the object.

An open design question sits here: `UrdfModelLoader::load` throws while
`KinematicChainBuilder::build` returns `std::expected`, so the facade will have
to reconcile the two styles.

### Later stages (not started, by design)

`ConstraintBuilder`, `EquationSimplifier`, `EquationSolver`, `IkPatternDetector`,
`CodeGenerator` — no code. `ConstraintBuilder` has an architecture analysis
(`proposal/analysis-ik-pipeline-constraint-builder.md`) but no proposal yet.

## Known gaps and deferred work

| Gap | Where | Notes |
|---|---|---|
| `continuous` joints unsupported | `robot_model_loader.cpp` | `mt::kinematics::JointType` has no `continuous` value, so `type="continuous"` rejects the whole file. `KinematicChainBuilder` and `JointTransformBuilder` both handle `JointType::Continuous` already — that code is covered only by hand-built joints and is unreachable end to end. Fixing it also touches limit semantics (URDF requires no `lower`/`upper` for continuous), so it needs its own proposal. Pinned by `ReportsContinuousAsUnsupportedUntilImplemented`. |
| No `ExpressionEvaluator` | — | **The single largest gap.** Blocks the strongest possible tests: substituting `q` and comparing against a numeric matrix. The whole FK equation is therefore verified for *structure only* — symbol presence, joint order, canonical last row. A consistent sign error in the RPY convention or in `p_a + R_a·p_b` would pass every test in the repo today. Both of `JointTransformBuilder`'s fast paths — axis-aligned vs. general Rodrigues, and skipping an identity `R_origin` — are equivalences that **cannot be tested** without it: `structurallyEqual` must return false, since the trees differ by design. Today they rest on argument alone. **When it is written it must memoize on node identity**: a composed FK transform measures 281 unique nodes but 21 882 counted with multiplicity for `kr640`, and 516 vs 153 703 for `kr4_r600` — a non-memoizing recursive walk pays three orders of magnitude. Depth stays at 22–24, so recursion itself is safe. The same applies to any future printer, simplifier or code generator. |
| Trig folding leaves numeric noise | symbolic layer | `sin(π) = 1.22e-16`, `cos(π/2) = 6.12e-17`. For `kr4_r600.urdf` this blocks even `x + 0 → x`, since those cells hold near-zero rather than zero. Whether to canonicalize is deliberately deferred — a tolerance would become part of the symbolic semantics. |
| `A · I ≠ A` for symbolic matrices | symbolic layer | Direct consequence of dropping `x * 0 → 0`. Recovering `A` needs a simplifier that tracks domains. `multiplyTransforms` restores the identity **for whole blocks only** — inside a genuine rotation product, per-cell terms like `cos(q)·0` still survive. |
| Factory ownership is value-semantic | both builders | `JointTransformBuilder` and `ForwardKinematicsBuilder` each take `ExpressionFactory` **by value**, so passing the same object to both still produces two copies. Harmless today — the class has no members — but if `ExpressionFactory` ever gains state (a hash-consing cache, a node counter, tolerance configuration), passing one object will **not** share it. Sharing would require redesigning ownership: a shared state object, a reference, or a `shared_ptr`. Do not assume the current constructors already enable it. |
| Diagnostics discarded on success | `UrdfModelLoader.cpp` | `load_urdf` returns a `DiagnosticBag` with info/warning entries; the facade drops it. On failure the bag cannot be returned at all, since `std::expected` carries only the error. |
| `Kinematics.h` / `example_forward` | `src/` | Placeholder from the initial scaffold, unrelated to the real pipeline. Only kept alive by one test. |

## Build and test

```powershell
cmake -B build -G Ninja      # first time, or after CMakeLists.txt changes
cmake --build build
ctest --test-dir build --output-on-failure
```

See [`COMMANDS.md`](COMMANDS.md) for shortcuts and the MinGW/PATH note.

## Working process

Non-trivial changes go through a written proposal in [`proposal/`](proposal/)
before any code is written: the prompt, the current state, and the full code of
the change, reviewed and approved first. A proposal is a document — the code
lives inside the `.md`, not on disk, until it is approved.

| Document | Subject | State |
|---|---|---|
| `proposal-loader-test-coverage.md` | KR4/KR640 loader test expansion | implemented |
| `proposal-kinematic-chain-builder-architecture.md` | Chain builder design | approved |
| `proposal-kinematic-chain-builder-implementation.md` | Chain builder code | implemented |
| `proposal-symbolic-layer-architecture.md` | Expression tree, factory, matrix | approved |
| `proposal-symbolic-layer-implementation.md` | Symbolic layer code | implemented |
| `proposal-urdf-geometry-validation-architecture.md` | Loader validation design | approved |
| `proposal-urdf-geometry-validation-implementation.md` | Loader validation code | implemented |
| `proposal-joint-transform-builder-architecture.md` | Joint transform design | approved |
| `proposal-joint-transform-builder-implementation.md` | Joint transform code | implemented |
| `proposal-forward-kinematics-builder-architecture.md` | FK composition design | approved (v2) |
| `proposal-forward-kinematics-builder-implementation.md` | FK composition code | implemented |
| `analysis-ik-pipeline-constraint-builder.md` | Where task-space constraints belong | open |

## Next step

`ExpressionEvaluator`, **with memoization on node identity mandatory** (the
numbers are in known gaps). It is chosen on test value rather than pipeline
order: it is what turns "these two trees denote the same rotation" from an
argument into an assertion — for the axis-aligned fast path, for the identity
shortcuts, and above all for the FK equation as a whole.

Immediately after it: end-to-end numeric validation of FK for KR4 and KR640
against known poses. Only then does Phase 1's central claim — "this repo derives
a *correct* symbolic forward-kinematics equation from a URDF" — stop resting on
argument. The facade comes after that.
