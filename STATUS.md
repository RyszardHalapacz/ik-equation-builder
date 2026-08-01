# Project Status

**Last updated:** 2026-08-01
**Branch:** `main`
**Build:** clean · **Tests:** 117/117 passing

A snapshot of what actually exists in this repo today, and what is still
declaration-only. The pipeline this project is working toward is described in
[`README.md`](README.md); this file tracks how far along it is.

## How far along, honestly

Two numbers, because they differ a lot depending on where the finish line is:

| Scope | Progress |
|---|---|
| **Phase 1** — URDF → symbolic forward kinematics | **~50%** |
| **Full vision** — including IK solver and code generation | **~30%** by component count |

The second number is misleading and worth stating plainly: **what is done is the
easy part.** Parsing XML, walking a tree and building an expression engine are
solved, well-understood problems. What remains includes symbolically solving IK
equations — research-grade work, plausibly larger on its own than everything
built so far. Measured by effort rather than component count, the honest figure
is closer to **15–20%**.

The upside: the foundation is solid. 117 tests, every architectural decision
written down and reviewed, and a long list of traps caught *before* they reached
the code — expression-domain loss from `x * 0 → 0`, trig-folding noise, a
`make_shared` symbol clash under `-static-libstdc++`, `std::hypot` losing
subnormal axes, and a parser that silently rewrote malformed vectors.

## Pipeline progress

```
URDF
  → UrdfModelLoader          ✅ implemented + tested + validating
  → RobotDescription         ✅ complete, canonical geometry guaranteed
  → KinematicChainBuilder    ✅ implemented + tested
  → KinematicChain           ✅ complete
  → Symbolic layer           ✅ implemented + tested
  → JointTransformBuilder    🟡 architecture approved, unblocked, not written
  → ForwardKinematicsBuilder ⬜ header only
  → Symbolic FK              ⬜ not reachable yet
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

### Supporting layers

- `mt::kinematics` (`robot_model`, `robot_model_loader`) — implemented.
- `mt::DiagnosticBag` — implemented; populated by `load_urdf` on success, and
  discarded at the `UrdfModelLoader` boundary.
- Model structs — complete aggregates, no behaviour.

## Not done

### `JointTransformBuilder` — designed, unblocked, not written

Architecture approved (`proposal/proposal-joint-transform-builder-architecture.md`).
The loader work that blocked it is now merged, so the canonical-geometry
invariant its assertions rely on actually holds. Settled decisions:

- `T_parent_child(q) = T_origin · T_motion(q)`, with `T_origin =
  Translation · RotationRPY`
- `R_rpy = Rz(yaw) · Ry(pitch) · Rx(roll)` — URDF fixed-axis convention
- Rodrigues for arbitrary axes, with a fast path for `±X`, `±Y`, `±Z`
  (measured: 33 → 7 nodes, and the general formula leaves cells that are
  mathematically zero but not `Constant(0)`)
- Blocks are composed directly rather than multiplying 4×4 matrices — a full
  4×4 product destroys the canonical form of the homogeneous last row

Next step: an implementation proposal with full code.

### `ForwardKinematicsBuilder` — declaration only

`build(const KinematicChain&, const JointTransformBuilder&) -> SymbolicTransform`
is declared; there is no `.cpp`.

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
| `continuous` joints unsupported | `robot_model_loader.cpp` | `mt::kinematics::JointType` has no `continuous` value, so `type="continuous"` rejects the whole file. `KinematicChainBuilder` and the `JointTransformBuilder` design already handle `JointType::Continuous`, making it currently unreachable. Fixing it also touches limit semantics (URDF requires no `lower`/`upper` for continuous), so it needs its own proposal. Pinned by `ReportsContinuousAsUnsupportedUntilImplemented`. |
| No `ExpressionEvaluator` | — | Blocks the strongest possible tests: substituting `q` and comparing against a numeric matrix. In particular, the equivalence of `JointTransformBuilder`'s axis-aligned fast path with the general Rodrigues formula **cannot be tested** without it — `structurallyEqual` must return false, since the trees differ by design. |
| Trig folding leaves numeric noise | symbolic layer | `sin(π) = 1.22e-16`, `cos(π/2) = 6.12e-17`. For `kr4_r600.urdf` this blocks even `x + 0 → x`, since those cells hold near-zero rather than zero. Whether to canonicalize is deliberately deferred — a tolerance would become part of the symbolic semantics. |
| `A · I ≠ A` for symbolic matrices | symbolic layer | Direct consequence of dropping `x * 0 → 0`. Recovering `A` needs a simplifier that tracks domains. |
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
| `proposal-joint-transform-builder-architecture.md` | Joint transform design | **approved, awaiting implementation proposal** |
| `analysis-ik-pipeline-constraint-builder.md` | Where task-space constraints belong | open |

## Next step

An implementation proposal for `JointTransformBuilder`. Together with
`ForwardKinematicsBuilder` it takes Phase 1 from ~50% to roughly 85% — at which
point the project can derive a symbolic forward-kinematics equation from a URDF
end to end, which is the milestone the whole design has been aiming at.
