# Project Status

**Last updated:** 2026-11-16
**Working branch state:** Phase 2 complete; Phase 3 F3.1–F3.9 and F3.12 implemented in the current workspace; F3.10 is complete for joint limits and spherical-wrist singularities but still has one explicit shoulder-axis singular-family gap.  
**Full CMake/CTest gate:** **PASS — 639/639 tests in Release, Debug, and the `KINEMAFORGE_WERROR=ON` configuration** (613 after F3.R2, 579 before it), on Windows/MinGW GCC 13.1.0. The seven defects of the 2026-11-15 audit and the eight of the 2026-11-16 audit have been corrected. The shoulder-axis singular family remains outside the supported solver scope.

> Current verification is recorded below. Component descriptions retain the
> Phase 1–3 history; the original September standalone evidence is explicitly
> dated and does not replace the November full-project test run.

## ▶ Where we are right now

**Stage F3.R3 — third audit (2026-11-16), eight findings: implemented test-first, not yet committed.**
Every fix was preceded by a test that failed for the reported reason and passes after it.
Gate: **639/639** in Release, Debug and `KINEMAFORGE_WERROR=ON` (613 before), Windows/MinGW GCC 13.1.0.

| # | Finding | Fix |
|---|---|---|
| 5 | `add_subdirectory` consumer broke (tests always added, data paths from `CMAKE_SOURCE_DIR`) | Tests gated by `KINEMAFORGE_BUILD_TESTS` (default `PROJECT_IS_TOP_LEVEL`) and `BUILD_TESTING`; paths use `PROJECT_SOURCE_DIR`. CTest case `cmake_subproject_consumer` configures a real consumer. |
| 6 | Empty / duplicate joint names accepted | Topology invariant T0c → `invalid_joint_declaration`. |
| 1 | `<mimic>` ignored, joint treated as independent | Rejected → `unsupported_mimic_joint` (coupling not modelled yet). |
| 8 | Turn range beyond `int64_t` treated as empty → NoSolution | `TurnRange::saturated`; count = SIZE_MAX, which exceeds every budget → `ResourceLimitExceeded`. |
| 3 | Generated solver: shoulder-axis singularity reported as NoSolution | New `generated::Status::UnsupportedSingularity`, same test/tolerance as the general solver. |
| 7 | Barely-touching singular-family intervals joined at midpoint, ~1e-13 outside limits | `detail::coupledFreeInterval` (shared): free bounds and dependent values at them lie within limits exactly; offset nudged by ≤ ~tolerance. |
| 4 | Nearly straight elbow on a limit: angle ~1e-9 outside, target rejected | Elbow angle within `kElbowLimitSnapMargin` (1e-7) of a limit is put on it and q2 re-solved for it; FK check (2e-9) decides. The ill-conditioning is inherent (~sqrt(eps)), so replacing `acos` alone would not fix it. |
| 2 | Equal links, wrist on joint-2 axis: one arbitrary q2, rest of family lost | Reported as unsupported (general: `UnsupportedKinematicStructure`, generated: `UnsupportedSingularity`). Test robot `tests/data/equal_links_6r.urdf`. |

Still open: parameterising the shoulder-axis and elbow-folded families, and real mimic support.

**Stage F3.R2 — follow-up to the second audit (2026-11-15): implemented, not yet committed.**
Design record: `proposal/F3.R2-audit-followup/proposal-audit-followup-architecture.md`.

| Finding | Fix |
|---|---|
| Wide joint limits made the analytic solver hang | Geometric branches are verified and deduplicated before full-turn expansion; the branch count is computed from the limits before allocating; over budget (`AnalyticIkSolverOptions::maxBranches`, default 4096) returns `ResourceLimitExceeded` with the required count — never a truncated list. Same in the generated runtime (`solve(parameters, target, maxBranches)`). |
| A target accepted by validation (1e-8) failed every FK round-trip (2e-9) | Both solvers project the validated matrix onto its nearest rotation (`nearestRotation()`, public) and solve/verify against that; the distance is returned as `orientationCorrection` and bounded by `kMaxOrientationCorrection` (2e-8). Validation itself still rewrites nothing. |
| Trigonometric simplifier skipped signed/scaled pairs and needed two passes | Terms are matched as `sign * cofactors * t(u)^2`; pairs are looked for before and after simplifying children; `simplify()` iterates to a fixed point. |
| Canonical-form guarantees were unstated | Contract written down and pinned by two tests: the divisor set is preserved (algebraic), evaluation in `double` is not guaranteed identical; constants already rounded by `ExpressionFactory` are not recovered. No code change. |
| Returned joint values could exceed a limit by ~1.6e-13 | Values within the 1e-12 margin are snapped onto the limit before the final FK check. |

Measured after the change (Windows/MinGW GCC 13.1.0): **613/613** CTest cases in
Release, Debug and `KINEMAFORGE_WERROR=ON` (34 new). Re-running the audit probes:
±1000 rad on two axes and ±1e5 rad on one return `ResourceLimitExceeded`
immediately (previously no result in 120 s); targets rounded to eight decimals
that pass validation are all solved (KR640 763/763, KR4 766/766; previously 8 and
5); the symbolic fuzzer finds 0 non-idempotent results in 300 000 expressions
(previously 22); no returned joint value lies outside its limits.

Still open, unchanged: the shoulder-axis singular family, `continuous` URDF
loading, and a "base branch + turn ranges" result API that would remove the
branch product at its source. `ConstraintBuilder` still receives the target
matrix exactly as written (call `nearestRotation()` first for an exactly
orthonormal symbolic target).

Next action: review and commit the working tree.

## Executive status

The project now contains two connected capabilities:

```text
URDF
  -> validated RobotDescription
  -> selected KinematicChain
  -> symbolic FK / TCP FK
  -> explicit IkEquationSystem
  -> canonical/trigonometric symbolic preparation
```

and, for the first explicitly supported analytic family:

```text
KinematicChain + PoseTarget (+ optional TCP)
  -> base-frame axis geometry
  -> supported-pattern detection
  -> analytic 6R solver
  -> all regular branches inside limits
  -> explicit spherical-wrist singular families
  -> robot-specialized generated C++23 solver
```

The second pipeline is deliberately **not** advertised as arbitrary-URDF analytic IK. The current analytic family requires exactly six rotary actuated joints, parallel joint-2/joint-3 axes, a spherical wrist, and the supported XYX-equivalent wrist arrangement (axis 4 parallel to axis 6; axis 5 perpendicular).

## Verification — 2026-11-15

The initial audit reproduced four failing tests in Release and Debug, a failed
warning-as-error build, and incorrect or missing analytic solutions near wrist
singularities. After remediation, all three configurations build and pass all
579 discovered CTest cases, including 25 additional regression cases.

The fixes cover stable XYX wrist decomposition; FK/limit verification of returned
regular branches and sampled singular families in both backends; retention of
distinct full-turn configurations; singular coupling intervals derived from
both joints' limits; locale-independent code generation and keyword rejection;
zero-product sign canonicalization without dropping divisor guards; and the
incorrect test fixtures and warning failures found during the audit.

`tests/test_analytic_ik_regressions.cpp` runs the general solver and newly generated
solvers against independent quaternion FK for KR640, KR4, and KR640 with a
translated/rotated TCP. Each variant includes 1000 regular configurations,
1000 aligned-wrist configurations, and 200 anti-aligned configurations, plus
near-singularity, full-turn, shifted-limit, and continuous-dependent-axis cases.
Regular branches are checked individually; singular intervals are sampled at
five points including both endpoints. Recovery checks preserve actual bounded
joint angles rather than comparing only modulo a full turn.

The CMake test build now runs `kinemaforge_generate_test_fixtures` and compiles its
output. The fixture generator intentionally uses a decimal-comma global locale,
so compilation also verifies that generated numeric literals remain valid C++.
The generated runtime includes the numeric-only helper
`ik_equations/solver/detail/AnalyticIkNumerics.hpp`; deployments need that header
alongside `GeneratedIkRuntime.hpp`.

These checks ran on Windows/MinGW; MSVC, Linux, a physical robot, and an external
kinematics oracle were not part of this validation.

## Historical verification snapshot — 2026-09-13

The following was recorded during the September implementation session:

```text
GCC 14, C++23, -Wall -Wextra -Wswitch -Werror
  25 product translation units independent of pugixml: PASS

new/changed GoogleTest translation units
  syntax/compile check with an offline gtest shim: PASS

analytic solver standalone property gate
  KR640: 1000/1000 random generating configurations recovered
  KR4:   1000/1000 random generating configurations recovered
  every returned regular branch independently checked with quaternion FK

code-generated solver standalone property gate
  KR640: 1000/1000 random targets recovered
  KR4:   1000/1000 random targets recovered
  KR640 + non-trivial translated/rotated TCP: 500/500 recovered
  generated KR640/KR4 headers: actually compiled

canonicalization gate
  debug + NDEBUG smoke checks
  randomized expression equivalence/definedness/idempotence checks
```

That snapshot counted 554 source test definitions and could not run full CTest in
its offline environment. The current executed test count and results are given above.

---

# Phase 1 — URDF -> symbolic forward kinematics

**State: complete before this work session.**

Implemented components include:

- `UrdfModelLoader` with strict structural/finite geometry validation;
- `KinematicChainBuilder`;
- symbolic `Expression`/`ExpressionFactory`/`SymbolicMatrix` layer;
- `JointTransformBuilder`;
- homogeneous `multiplyTransforms`;
- `ForwardKinematicsBuilder`;
- `ExpressionEvaluator` with identity memoization;
- quaternion-based numeric FK cross-check support;
- the stateful `IkEquationBuilder` facade.

The existing important semantic decision remains unchanged: general `x * 0 -> 0` is not a valid unconditional rewrite because it can erase domain information, e.g. `(1/q) * 0` at `q = 0`.

---

# Phase 2 — formulate IK equations

**State: complete before this work session.**

Implemented:

- constant chain-tip -> TCP transform;
- IK equation/target model;
- `PositionOnly` constraints;
- `FullPose` constraints;
- quaternion target boundary;
- directional/angle constraints;
- facade integration.

Phase 2's output remains an explicit `IkEquationSystem`; it does not itself solve for `q`.

---

# Phase 3

## F3.1 — RewriteEngine

**State: complete before this work session.**

The engine owns rule application semantics, fixed-point behavior and domain-aware rewrite support. It remains the low-level rewrite mechanism rather than the analytic solver itself.

## F3.2 — AC canonical form

**State: implemented and heavily property-checked in this workspace.**

Added/updated:

```text
src/ik_equations/symbolic/ExpressionCanonicalizer.hpp/.cpp
src/ik_equations/symbolic/ExpressionIdentityMap.hpp
src/ik_equations/builders/EquationSimplifier.hpp/.cpp
tests/test_expression_canonicalizer.cpp
```

Purpose: make associative/commutative `Add` and `Multiply` structure deterministic so later matching does not depend on accidental FK construction order.

Key contracts:

- deterministic flatten/sort/rebuild for representable AC chains;
- exact-only constant grouping;
- representability barriers preserve original subtrees when a canonical reconstruction would violate the expression factory's finite/domain contracts;
- repeated canonicalization is idempotent;
- a fresh session can recover the same canonical node for an already canonical form.

A concrete bug found during implementation was the sign/domain interaction around `-(0/q)`. The implementation was corrected so normalization does not silently replace a partial expression with a total zero.

## F3.3 — trigonometric simplification

**State: implemented for a deliberately small domain-safe rule set.**

Added:

```text
src/ik_equations/symbolic/TrigonometricSimplifier.hpp/.cpp
tests/test_trigonometric_simplifier.cpp
```

Implemented rules include odd/even sign normalization and a guarded `sin^2(x)+cos^2(x)->1`. The Pythagorean identity fires only when replacing it cannot widen symbolic definedness; in particular, a partial argument containing division is not collapsed to total `1`.

This is not a general CAS and intentionally does not expand angle sums, distribute products, or cancel partial terms across equation sides.

## F3.4 — kinematic axis geometry

**State: implemented.**

Added:

```text
src/ik_equations/model/KinematicGeometry.hpp
src/ik_equations/builders/KinematicGeometryBuilder.hpp/.cpp
tests/test_kinematic_geometry.cpp
```

Every actuated joint becomes a base-frame `AxisLine` at the zero configuration. Fixed joints contribute to the accumulated frame but do not create an axis entry. Approximate geometry is isolated from exact symbolic equality and uses named angular/linear tolerances.

Queries cover parallelism, perpendicularity, line distance, coincidence and intersection.

## F3.5 — IK pattern detector

**State: implemented conservatively for the first solver family.**

Added:

```text
src/ik_equations/model/IkPattern.hpp
src/ik_equations/solver/IkPatternDetector.hpp/.cpp
tests/test_ik_pattern_detector.cpp
```

Detected evidence includes:

- spherical wrist;
- parallel shoulder/elbow axes;
- composite six-rotary spherical-wrist support marker.

The detector returns evidence/metrics rather than guessing a closest family.

## F3.6 — analytic solution IR

**State: implemented.**

Added `model/AnalyticIkSolution.hpp` with:

- multiple regular branches;
- shoulder/elbow/wrist branch labels;
- explicit singularity classification;
- free-joint intervals;
- coupled free-parameter relation for singular families;
- statuses for solutions/no solution/infinite family/unsupported/invalid target.

The critical policy is that a singularity is not resolved by inventing an arbitrary joint value.

## F3.7 — primitive solvers

**State: implemented and smoke-tested.**

Added `solver/PrimitiveSolvers.hpp/.cpp` and tests for:

- linear equations;
- sine/cosine equations;
- `a*sin(x)+b*cos(x)=c`;
- generalized planar 2R;
- modulo-`2*pi` normalization and branch deduplication.

## F3.8 — position solver for the first 6R family

**State: implemented and property-tested on both shipped robots.**

`AnalyticIkSolver` derives the target wrist center from the actual home geometry and the full target pose, solves shoulder candidates from axis geometry, inverse-rotates the wrist-center target, and solves joints 2/3 as a generalized planar 2R problem.

No KUKA model name or KR4/KR640 link-length table is hard-coded.

## F3.9 — spherical-wrist orientation solver

**State: implemented and property-tested.**

The solver computes residual wrist orientation and maps the actual wrist axes into an XYX-equivalent basis. Primary and wrist-flip branches are returned. URDF `origin.rpy` rotations are respected through the base-frame geometry instead of assuming world-aligned joint frames.

This is why the same implementation recovers both KR640 and the rotated-origin KR4 cases.

## F3.10 — singularities, degeneracies and limits

**State: partially complete.**

Implemented:

- all regular `2*pi` equivalent solutions inside finite joint limits;
- continuous-joint normalization where applicable to the supported rotary family;
- wrist aligned/anti-aligned singularities as `InfiniteFamily` results;
- free `q4` interval plus coupled `q6 = offset + multiplier*q4 (mod 2*pi)` relation;
- non-finite candidate rejection.

**Known gap:** if the wrist center lies on the shoulder axis, the shoulder equation loses rank and `q1` becomes free. The current solver does not yet parameterize the remaining position solution as a function of free `q1`; it returns `UnsupportedKinematicStructure` rather than choosing an arbitrary angle. The lightweight generated backend has the same limitation.

## F3.11 — end-to-end analytic IK validation + facade

**State: full CTest passed in all three configurations on 2026-11-15.**

`IkEquationBuilder` now exposes:

```cpp
std::expected<AnalyticIkSolution, IkEquationBuilderError>
solveAnalyticIk(const PoseTarget& target) const;
```

The selected chain and optional TCP are reused. Typed `AnalyticIkSolverError` is preserved inside the facade error.

Standalone random round trips actually executed in this session:

```text
KR640: 1000/1000
KR4:   1000/1000
```

For every returned regular branch, an independent quaternion FK implementation was used to verify the target pose. A non-trivial translated and rotated TCP also passed a dedicated smoke/property path.

## F3.12 — C++23 code generator

**State: implemented and standalone-generated output compiled/property-tested.**

Added:

```text
src/ik_equations/codegen/CppAnalyticIkGenerator.hpp/.cpp
src/ik_equations/codegen/GeneratedIkRuntime.hpp
tests/test_code_generator.cpp
tests/generated/kr640_analytic_ik.hpp
tests/generated/kr4_analytic_ik.hpp
```

The generator consumes the selected chain and optional TCP, validates support, derives robot-specific numeric geometry, and emits deterministic C++ text containing constexpr parameters plus one lightweight runtime include.

Generated runtime deliberately has no URDF parser, `Expression` DAG or general `AnalyticIkSolver` dependency.

Observed generated-code property results:

```text
KR640:                         1000/1000
KR4:                           1000/1000
KR640 with translated/rotated TCP: 500/500
```

---

# Known gaps / release boundaries

| Gap | Current behavior | Why it matters |
|---|---|---|
| Shoulder-axis singular family | Main solver returns `UnsupportedKinematicStructure`; generated runtime produces no shoulder branch | `q1` is genuinely free and the remaining position problem must be parameterized rather than assigned an arbitrary representative. |
| Analytic family is intentionally narrow | Unsupported geometry returns typed failure | No false claim that arbitrary 6R/arbitrary URDF has closed-form IK. |
| Generated code uses `GeneratedIkRuntime.hpp` and its numeric helper | Generated file is specialized but not literally one-file standalone | Deploy both headers on the include path; the runtime remains symbolic/URDF-free. |
| Generated singular metadata is leaner than the general IR | Supports wrist free relation, not the full future solution-condition language | Sufficient for the first family, not a universal solver IR. |
| Existing Phase-1 cross-check begins at `KinematicChain` | Loader has its own tests | A parser/order bug would not be detected by the quaternion-vs-symbolic comparison alone. |
| RPY convention cross-check is internally independent but not externally authoritative | Existing hand oracles + two implementations | External KDL/Pinocchio oracle would close the convention gap. |
| Continuous-joint URDF loading is not supported | The loader rejects `continuous`; hand-built downstream types can represent it | Do not claim continuous URDF support end-to-end. |

---

# Build and release gate

Normal developer-machine gate remains:

```powershell
cmake -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

For a strict warning gate configure with the project's `KINEMAFORGE_WERROR` option as documented by CMake/CI.

The full Windows/MinGW CMake/CTest gate is green. The workspace remains a
**v1.0 analytic-IK release candidate** while complete handling of the shoulder-axis
singular family is still an open support requirement.

# Next engineering step

Before extending the supported robot families:

1. close the shoulder-axis singular family if v1.0 promises complete singular handling for the supported family;
2. validate any additional deployment toolchains and an external FK oracle;
3. move the verified F3 proposals into `proposal/done/` and tag/release when that scope is complete;
4. then add another geometric family or a more general condition-bearing solver IR.
