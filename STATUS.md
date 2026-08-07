# Project Status

**Last updated:** 2026-08-04
**Branch:** `main`
**Build:** clean · **Tests:** 273/273 passing

A snapshot of what actually exists in this repo today, and what is still
declaration-only. The pipeline this project is working toward is described in
[`README.md`](README.md); this file tracks how far along it is.

## How far along, honestly

Two numbers, because they differ a lot depending on where the finish line is:

| Scope | Progress |
|---|---|
| **Phase 1** — URDF → symbolic forward kinematics | **done** |
| **Full vision** — including IK solver and code generation | **~50%** by component count |

The second number is misleading and worth stating plainly: **what is done is the
easy part.** Parsing XML, walking a tree and building an expression engine are
solved, well-understood problems. What remains includes symbolically solving IK
equations — research-grade work, plausibly larger on its own than everything
built so far. Measured by effort rather than component count, the honest figure
is closer to **25%**.

**Phase 1 is done.** A symbolic FK equation falls out of a URDF end to end
through one public API, and its **values** agree with an independently written
quaternion implementation to within a few ULP across 18 joint configurations on
two real robots. Two scope limits stay attached to that claim and do not
disappear with the milestone — see known gaps: the cross-check starts at
`KinematicChain`, not at the file, and the RPY convention rests on a belief
shared by both implementations rather than on an external source.

The upside: the foundation is solid. 273 tests, every architectural decision
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
  → ExpressionEvaluator      ✅ implemented + tested
  → numeric FK validation    ✅ cross-checked against a quaternion reference
  → IkEquationBuilder        ✅ implemented + tested — Phase 1 complete
  ── Phase 2 ──────────────────────────────────────────────
  → TCP transform            ✅ implemented + tested (F2.1/F2.2)
  → IK equation model        ✅ implemented + tested (F2.3)
  → ConstraintBuilder        ⬜ analysed only, no proposal yet — next (F2.4)
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

### `ExpressionEvaluator` — expression DAG → double

```
Expression + symbol values  →  std::expected<double, EvaluationError>
```

**A session, not a free function.** One instance holds one substitution of
`q1..qn`; all sixteen FK cells are evaluated with the same instance so they
share the cache. Binding the values to the object's lifetime makes reusing a
cache filled for *different* values impossible rather than merely discouraged.
Copying is deleted — a session is not a value.

**Memoization is mandatory, keyed by node identity.** The cache key is
`Expression`, not `const ExpressionNode*`: `evaluate` takes a reference that may
bind to a temporary, and a raw pointer key would leave a dangling address that
the allocator can hand to the next node — the cache would then answer for the
wrong node, silently. Hash and equality still use identity (`&node()`,
`sameNode`), never `structurallyEqual`, so lookups stay O(1) and separately
built but structurally identical trees stay separate entries.

Verified on real FK trees, not just unit examples:

| Robot | unique nodes | `cacheMisses` | `cacheHits` | total lookups |
|---|---:|---:|---:|---:|
| `kr640.urdf` | 281 | **281** | 251 | 532 |
| `kr4_r600.urdf` | 516 | **516** | 396 | 912 |

`cacheMisses` equals the unique-node count exactly for both. 532 lookups against
the 21 882 visits a non-memoized walk performs — a 41× reduction, and 168× for
`kr4_r600`.

**Four error codes**, all via `std::expected`: `MissingSymbol` (never defaults
to zero — a silent zero would produce a plausible-looking but wrong pose),
`NonFiniteSymbolValue`, `DivisionByZero` (exact `== 0.0`, no tolerance, which
catches `-0.0` for free), `NonFiniteResult`. Bindings are validated **on read**,
so an unused `NaN` binding is not an error.

**No short-circuiting on values.** Both operands of every binary node are
evaluated; a zero factor does not skip the other side. That is what makes
`(1/q) * 0` report `DivisionByZero` at `q = 0` instead of quietly yielding 0 —
the end-to-end payoff of the symbolic layer's refusal to fold `x * 0 → 0`.
Tested from both sides, since `SymbolicMatrix::multiply` really does place
`Constant(0)` on the left inside FK cells. Evaluation does stop at the first
error, and operands are visited left to right, so the reported error is
deterministic.

**Tests:** 26.

### Numeric FK validation — the symbolic matrix checked against another one

The payoff of everything above. A quaternion-based forward kinematics lives in
`tests/support/NumericForwardKinematics.{hpp,cpp}` — **test scaffolding, not
part of `kinemaforge_ik`**: its only value is being a second implementation,
and that value would evaporate the moment anything in the product called it.

Independence is by representation. Rotations are carried as quaternions and
composed by quaternion multiplication; vectors are rotated by
`v + 2w(u×v) + 2u×(u×v)` **without ever forming a matrix**. Production builds
Rodrigues matrices and multiplies 3×3 blocks. A 3×3 matrix appears in the
reference exactly once, when the result is handed to a test.

The configuration is shared as a **positional** `JointConfiguration`; the
reference consumes it by joint order, the symbolic side by
`joint.variable->name`. Two different routes from one number to one joint — so
a duplicated variable name is caught rather than agreed upon. (`emplace` does
not overwrite on a duplicate, so that case throws explicitly.)

Coverage: `q = 0`, six single-joint poses, a mixed pose and a near-limit pose,
for both robots — **18 matrix comparisons, 288 cells**. Plus synthetic cases
for branches no real robot reaches: an arbitrary Rodrigues axis, a **negative**
principal axis, and a prismatic joint with a rotated origin.

Measured worst-case error against a `1e-12` absolute + relative tolerance:

| | rotation | translation |
|---|---:|---:|
| KR4 (worst: mixed) | **5.55e-16** | 2.22e-16 |
| KR640 (worst: mixed) | 2.22e-16 | 2.22e-16 |
| hand-computed oracles | 6.12e-17 | 9.80e-17 |

Roughly **three orders of magnitude of headroom**. A real geometric error would
show up at `1e-3` or larger, so the bound is conservative without being blind.

**Tests:** 17.

### `IkEquationBuilder` — the public entry point

Wraps the whole pipeline; everything underneath stays private. No new
mathematics — it packages what the components already do.

State is held in `std::optional`, which is what makes the four stages
distinguishable. Plain members would not: a default-constructed
`SymbolicTransform` is a matrix of **zeros**, not even the identity, so
returning one after a failed sequence would be a silent, geometrically
meaningless answer rather than an obvious absence.

Each successful step invalidates what it obsoletes — a new robot clears the
chain and the transform, a new chain clears the transform. A chain names links
of one specific robot; a transform carries symbols of one specific chain.

**Failure leaves the object untouched.** Results are built into locals and
committed only once they exist; clearing first would leave a failed call worse
off than no call at all. Pinned by comparing pointer identity across a failed
operation, which also proves that failures do not invalidate pointers handed
out earlier.

Errors are unified into `std::expected` with four codes. The loader throws, so
the facade catches **exactly `std::runtime_error`** — catching `std::exception`
would swallow `std::bad_alloc` and report it as a URDF problem. The chain
builder's typed `KinematicChainError` is preserved rather than flattened into
a string; `chainError` holds a value if and only if the code is
`ChainBuildFailed`.

Accessors return `const T*`, `nullptr` until the step has run. `std::expected`
would carry no information here: absence has exactly one cause. Returned
pointers are non-owning and invalidated by the next successful state change —
stated in the class comment, because it is a real hazard.

**Tests:** 17.

### Constant TCP transform — first piece of Phase 2

```
T_base_tcp(q) = T_base_chain_tip(q) · T_chain_tip_tcp
```

`FixedRigidTransform{translation, rpy}` is a **general** constant frame offset,
not a TCP-specific type — the mathematics has nothing to do with tools, and the
workpiece frames Phase 2 will need are the same shape. Its contract is
`T_parent_child`, metres and radians, `Rz·Ry·Rx`; for a TCP, `parent` is the tip
of the currently selected chain, so **`tool0` is never hard-coded**.

Translation + RPY rather than a quaternion for one reason: **RPY is total.**
Any six finite numbers describe a valid rigid transform, so validation is
`isfinite` six times with no threshold. A quaternion would need a unit-norm
check with a tolerance, and a tolerance becomes part of the semantics.

**Composition reuses `multiplyTransforms`** — no new mathematics. An identity
TCP therefore costs **exactly zero nodes**, because that function already
returns `lhs` unchanged for an identity right operand; the test asserts
`sameNode` on all sixteen cells, not `structurallyEqual`, since a tree rebuilt
in the same shape would satisfy the weaker one.

**State is a graph, not a chain:**

```
        KinematicChain
         /          \
ForwardKinematics   TCP
         \          /
     TcpForwardKinematics
```

The TCP is a *sibling* of the transform, so it may be set before or after
`buildForwardKinematics`, and rebuilding the transform keeps it. Changing the
chain clears it — the same three numbers would name a different physical point
once the tip changes, and a plausible wrong answer is worse than a loud
absence.

Errors extend the facade's existing enum: `ForwardKinematicsNotBuilt`,
`TcpNotSet`, `InvalidTcpTransform`. Missing prerequisites are reported in
dependency order — chain, then transform, then TCP — so the caller is told the
next step to take rather than the last one.

Verified against the quaternion reference across **the same nine configurations
as Phase 1**, both robots — 18 matrix comparisons. Worst error **7.49e-16**
rotation, **2.22e-16** translation, against a `1e-12` bound; Phase 1's worst was
`5.55e-16`, so the extra composition costs a few ULP.

**Tests:** 21.

### IK equation model — what an equation and a target *are*

Types only: no builder, no solver, no algebra. Written before
`ConstraintBuilder` so that model is settled once, rather than invented
implicitly one caller at a time.

`Equation` stores **`lhs` and `rhs` exactly as given** and never forms
`lhs - rhs` implicitly. Subtracting would destroy which side came from the
robot and which from the task, and that boundary cannot be recovered without
algebraic decomposition — which this project does not have. Normalisation is
the future simplifier's job.

`EquationSource` is a `variant<PositionEquationSource,
OrientationEquationSource>`, not a `{kind, optional<cell>}` struct: a position
equation has no way to name a rotation cell, and a cell cannot exist without a
kind that gives it meaning. Every consumer dispatches through an **explicit
overload set** — `kindOf`, the content check, the target dispatcher — so a
third alternative stops the build instead of being silently classified. A
`holds_alternative` test would have called it `Orientation`, which is exactly
the failure a variant exists to prevent.

`OrientationEquationSource` is closed with a private constructor and a
`create(row, column) -> optional`: both indices must be in `0..2`, and public
fields plus a factory would leave `OrientationEquationSource{7, 9}` legal,
making the factory a suggestion rather than a gate.

`IkEquationSystem` is closed too, and `create` enforces **nine invariants** in
a fixed order, so each code has a deterministic meaning when several are broken
at once:

```
NoEquations · NoUnknowns · EmptyUnknownName · DuplicateUnknownName
DuplicateUnknownIndex · UnorderedUnknowns · TaskEquationMismatch
DuplicateEquationSource · UnorderedEquations
```

Two of those orderings are forced rather than arbitrary. `DuplicateUnknownIndex`
comes before `UnorderedUnknowns` because two equal indices already break a
strictly increasing sequence — the reverse order would report a duplicate as
bad ordering: true, but useless. `DuplicateEquationSource` comes before
`UnorderedEquations` for the same reason, and because two equations addressing
one matrix cell are either redundant or contradictory.

The system **carries its own unknowns**. Recovering them from the equation
trees is not an option: there is no public symbol collector, DAG traversal order
is not joint order, sorting by name gives `q1, q10, q2`, and a symbolic target
parameter would be indistinguishable from a joint variable.

**`enum class` is not a set-of-values type.** `static_cast<IkTaskKind>(99)`
yields a valid object with no enumerator behind it, so the exhaustive `switch`
(which `-Wswitch` uses to catch *new* enumerators at compile time) is paired
with an explicit run-time rejection for *forged* ones. `CartesianComponent`
gets the same treatment through `componentIndex`, because
`present[static_cast<std::size_t>(component)]` would otherwise write out of
bounds. The two protections cover different things and neither replaces the
other.

`IkTarget = variant<PositionTarget, PoseTarget>`, again closed — a struct of
optionals would admit a target that constrains nothing. `PoseTarget` means
`T_base_target`; the direction is part of the contract.

**Validation rejects, never repairs.** Re-orthonormalising an input rotation
would change the target the caller asked for and hide their mistake. Checked in
an observable order: finite → orthogonality → determinant.

`kOrientationTolerance = 1e-8`, and it is deliberately **not** the `1e-12` used
for FK comparisons: that bound measures error accumulated inside our own
computation, this one accepts data that arrived from elsewhere. Measured on
500 000 random rotations rounded to nine decimals:

| | worst deviation | headroom at `1e-8` |
|---|---:|---:|
| `max abs(RᵀR − I)` | 1.68e-9 | 5.95× |
| `max abs(det − 1)` | 1.85e-9 | 5.40× |

An independent run during review reached `1.94e-9` for the determinant, so the
margin is at least 5× either way. Data that has passed through `float` deviates
by ~1e-6 and is rejected on purpose.

The third error code is `InvalidOrientationDeterminant`, **not**
`ImproperRotation`: the "therefore a reflection" reading holds only for exactly
orthogonal matrices. `diag(1+4e-9, 1+4e-9, 1+4e-9)` deviates from orthogonality
by 8e-9 (accepted) and from unit determinant by 1.2e-8 (rejected) with a
*positive* determinant, and is no reflection at all. Its message is formatted
with `max_digits10` — `std::to_string` would print `1.000000` for the very
value that caused the rejection.

**Tests:** 31 (15 model, 16 validation), including forged enumerators for both
enums and threshold cases at `0.9×` and `1.1×` the tolerance.

### Supporting layers

- `mt::kinematics` (`robot_model`, `robot_model_loader`) — implemented.
- `mt::DiagnosticBag` — implemented; populated by `load_urdf` on success, and
  discarded at the `UrdfModelLoader` boundary.
- Model structs — complete aggregates, no behaviour.

## Not done

### Later stages (not started, by design)

`ConstraintBuilder`, `EquationSimplifier`, `EquationSolver`, `IkPatternDetector`,
`CodeGenerator` — no code. `ConstraintBuilder` has an architecture analysis
(`proposal/analysis-ik-pipeline-constraint-builder.md`) but no proposal yet.

## Known gaps and deferred work

| Gap | Where | Notes |
|---|---|---|
| `continuous` joints unsupported | `robot_model_loader.cpp` | `mt::kinematics::JointType` has no `continuous` value, so `type="continuous"` rejects the whole file. `KinematicChainBuilder` and `JointTransformBuilder` both handle `JointType::Continuous` already — that code is covered only by hand-built joints and is unreachable end to end. Fixing it also touches limit semantics (URDF requires no `lower`/`upper` for continuous), so it needs its own proposal. Pinned by `ReportsContinuousAsUnsupportedUntilImplemented`. |
| Validation starts at `KinematicChain`, not at the URDF file | numeric FK validation | The quaternion reference reads the **same** `KinematicChain` the symbolic side does. If `UrdfModelLoader` mis-parses an axis or `KinematicChainBuilder` gets the joint order wrong, both sides see the identical mistake and agree. So the cross-check covers the symbolic pipeline **from `KinematicChain` upward** — `JointTransformBuilder`, `multiplyTransforms`, `ForwardKinematicsBuilder`, `ExpressionEvaluator` — and says nothing about URDF parsing. Parsing has its own 56 tests, but they are structural, not end-to-end against a known pose. |
| RPY convention rests on a shared belief, not an external source | numeric FK validation | Quaternions protect against an *implementation* slip — a wrong sign, a swapped axis component — because the same formula written two ways does not break identically. They do **not** protect against a shared misreading of the URDF spec: the reference was written from the same understanding of `rpy` as production, so a convention error would be confirmed by both. The two hand-computed KR640 oracles guard the zero pose and one quarter turn, but for non-trivial `rpy` angles nothing external has been consulted. Closing this properly means comparing against KDL / `tf2` / Pinocchio, or against published KR4 poses. Deliberately deferred — it would add a dependency to the test project. |
| `FixedRigidTransform` direction is unenforced | `model/FixedRigidTransform.hpp` | The type means `T_parent_child`, but nothing stops a caller from filling it with `T_child_parent` and no compiler will notice — the fields are six anonymous doubles either way. The contract lives in a comment; `AppliesTcpTranslationInToolFrame` is what would actually catch a reversal. A distinct type per direction would enforce it, at the cost of a conversion layer nobody needs yet. |
| A target does not know its frame | `model/IkTarget.hpp` | `PositionTarget` and `PoseTarget` are expressed "in the base frame of whatever transform the constraint builder is given", and nothing in the type says whether that transform ends at the TCP or at the chain tip. Pairing a TCP target with `forwardKinematics()` instead of `tcpForwardKinematics()` compiles, runs, and yields a system that is wrong by exactly the tool offset. The model deliberately does not know the frame — it has no access to the chain — so this cannot be closed here; a frame-tagged transform type would close it, at the cost of a conversion layer nobody needs yet. Same class of gap as the `FixedRigidTransform` direction row above. |
| Orientation data that passed through `float` is rejected | `model/TargetValidation.cpp` | `kOrientationTolerance = 1e-8` accepts `double` and text of reasonable precision, and nothing else. A rotation matrix that has been through `float` at any point deviates by ~1e-6 and fails `NonOrthogonalOrientation`. That is a decision, not an oversight: a threshold loose enough to admit `float` would also admit a genuinely non-orthogonal target and make the equation system quietly unsatisfiable. The right fix is an explicit `orthonormalize` on the caller's side — which this project does not provide, so a caller with `float` data currently has to write it. |
| Evaluator cache grows monotonically | `ExpressionEvaluator` | Successfully evaluated nodes stay cached — and alive, since the key is `Expression` — until the evaluator is destroyed. There is no `clearCache()`. Correct for the intended use (one substitution, one related DAG: 281–516 nodes), wrong if someone treats one instance as a long-lived global interpreter. Stated in the class comment. |
| Recursion depth scaling unmeasured | `ExpressionEvaluator`, and any recursive walk | Measured FK depth is 22 (`kr640`) and 24 (`kr4_r600`) for 7-joint chains — safe for recursion by a wide margin. How depth grows with chain length was **never measured**. Before allowing very long chains, or after a simplifier starts deepening trees, measure it before assuming recursion still fits. |
| Trig folding leaves numeric noise | symbolic layer | `sin(π) = 1.22e-16`, `cos(π/2) = 6.12e-17`. For `kr4_r600.urdf` this blocks even `x + 0 → x`, since those cells hold near-zero rather than zero. Whether to canonicalize is deliberately deferred — a tolerance would become part of the symbolic semantics. |
| `A · I ≠ A` for symbolic matrices | symbolic layer | Direct consequence of dropping `x * 0 → 0`. Recovering `A` needs a simplifier that tracks domains. `multiplyTransforms` restores the identity **for whole blocks only** — inside a genuine rotation product, per-cell terms like `cos(q)·0` still survive. |
| Factory ownership is value-semantic | both builders | `JointTransformBuilder` and `ForwardKinematicsBuilder` each take `ExpressionFactory` **by value**, so passing the same object to both still produces two copies. Harmless today — the class has no members — but if `ExpressionFactory` ever gains state (a hash-consing cache, a node counter, tolerance configuration), passing one object will **not** share it. Sharing would require redesigning ownership: a shared state object, a reference, or a `shared_ptr`. Do not assume the current constructors already enable it. |
| `UrdfLoadFailed` carries only a string | `IkEquationBuilder`, `UrdfModelLoader` | The facade preserves the chain builder's typed `KinematicChainError`, but has nothing equivalent for the loader: `UrdfModelLoader` consumes its structured `LoadError`/`LoadErrorCode` internally, assembles a message and throws `std::runtime_error`. No typed code survives to be preserved, so the asymmetry in `IkEquationBuilderError` is a consequence, not an oversight. Fixing it means giving `UrdfModelLoader` a typed error type — its own change. Same root cause as the row below. |
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
| `proposal-expression-evaluator-architecture.md` | Numeric evaluation design | approved (v2) |
| `proposal-expression-evaluator-implementation.md` | Numeric evaluation code | implemented |
| `proposal-numeric-fk-validation-architecture.md` | Independent FK cross-check design | approved (v2) |
| `proposal-numeric-fk-validation-implementation.md` | Quaternion reference + 17 tests | implemented |
| `proposal-ik-equation-builder-architecture.md` | Facade state, error model, accessors | approved |
| `proposal-ik-equation-builder-implementation.md` | Facade code + 17 tests | implemented |
| `proposal-tcp-transform-architecture.md` | Constant tip→TCP transform design | approved (v3) |
| `proposal-tcp-transform-implementation.md` | TCP code + 21 tests | implemented |
| `proposal-ik-equation-model-architecture.md` | Equation, target and system types | approved (v3) |
| `proposal-ik-equation-model-implementation.md` | Model code + 31 tests | implemented |
| `analysis-ik-pipeline-constraint-builder.md` | Where task-space constraints belong | open |

## Next step

Phase 1 is closed and Phase 2 is under way: the constant TCP transform (F2.1 and
F2.2) is implemented and numerically validated, and F2.3 — the domain model of
equations and targets — is now implemented. `Equation` is `lhs = rhs` stored as
given, the right-hand side is an `Expression`, a target rotation is a plain
row-major 3×3 of doubles, and validation rejects rather than repairs.

**Next is F2.4 — `ConstraintBuilder` for `PositionOnly`**: the translation
column of `T_base_tcp` against a `PositionTarget`, three equations, assembled
into an `IkEquationSystem`. That is the first point at which the project
*formulates* IK rather than describing geometry. The model it emits into now
exists, so the builder has a contract to satisfy instead of one to invent.

Worth stating plainly before starting: everything built so far was *mechanical
translation with a known correct answer* — parse, walk, compose, evaluate. What
follows is not. `EquationSolver` has no single algorithm; it is case analysis,
and for some robots a closed-form solution does not exist at all. Measured by
effort rather than component count, Phase 1 is closer to a quarter of the whole
than to half.

`EquationSimplifier` is the other underestimated piece: dropping `x * 0 → 0`
means it has to **track domains** to remove those terms correctly, and the
`1e-16` trig-folding noise forces a tolerance decision that becomes part of the
symbolic semantics.
