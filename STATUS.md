# Project Status

**Last updated:** 2026-09-13  
**Working branch state:** Phase 2 complete; Phase 3 F3.1–F3.9 and F3.12 implemented in the current workspace; F3.10 is complete for joint limits and spherical-wrist singularities but still has one explicit shoulder-axis singular-family gap.  
**Full CMake/CTest release gate:** **not run in this sandbox** — `FetchContent` needs GitHub for pugixml/googletest and outbound DNS/network is unavailable here.

> This file describes the code that is actually present in the repository on
> 2026-09-13. The older August status that stopped at F3.2 has been retired.
> Evidence below distinguishes direct compilation/property tests that were
> actually executed from the full project CTest gate that still has to be run
> in a networked/dependency-populated build environment.

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

## Verification snapshot from this work session

The following was actually run against the files in this workspace:

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

There are **554 `TEST(...)`/GoogleTest test definitions** in `tests/` in the current workspace. That is a source count, not a claim that all 554 were executed here. The full `ctest --output-on-failure` gate remains required outside this offline sandbox.

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

**State: working-branch gate passed; full CTest still pending.**

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
| Full CMake/CTest not executed in this sandbox | Direct GCC/Werror and standalone property gates only | FetchContent needs network for pugixml/googletest; release must rerun `cmake --build` + `ctest` in the normal developer environment. |
| Shoulder-axis singular family | Main solver returns `UnsupportedKinematicStructure`; generated runtime produces no shoulder branch | `q1` is genuinely free and the remaining position problem must be parameterized rather than assigned an arbitrary representative. |
| Analytic family is intentionally narrow | Unsupported geometry returns typed failure | No false claim that arbitrary 6R/arbitrary URDF has closed-form IK. |
| Generated code uses `GeneratedIkRuntime.hpp` | Generated file is specialized but not literally one-file standalone | Runtime is lightweight and symbolic/URDF-free; a future backend may inline it if deployment requires a single translation unit. |
| Generated singular metadata is leaner than the general IR | Supports wrist free relation, not the full future solution-condition language | Sufficient for the first family, not a universal solver IR. |
| Existing Phase-1 cross-check begins at `KinematicChain` | Loader has its own tests | A parser/order bug would not be detected by the quaternion-vs-symbolic comparison alone. |
| RPY convention cross-check is internally independent but not externally authoritative | Existing hand oracles + two implementations | External KDL/Pinocchio oracle would close the convention gap. |
| Continuous-joint end-to-end loader support remains a historical gap if still absent at the legacy boundary | Hand-built downstream types can represent it | Should be verified before claiming continuous URDF support end-to-end. |

---

# Build and release gate

Normal developer-machine gate remains:

```powershell
cmake -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

For a strict warning gate configure with the project's `KINEMAFORGE_WERROR` option as documented by CMake/CI.

The current workspace should be considered a **v1.0 analytic-IK release candidate**, not a green release, until the full normal CMake/CTest run succeeds with dependencies available.

# Next engineering step

Do **not** add another robot family before closing the release gate. The next order should be:

1. run clean configure/build/full CTest on the normal Windows/MinGW or CI toolchain;
2. fix any integration-only failures;
3. close the shoulder-axis singular family if v1.0 promises complete singular handling for the supported family;
4. only then move the verified F3 proposals into `proposal/done/` and tag/release;
5. after that, add another geometric family or a more general condition-bearing solver IR.
