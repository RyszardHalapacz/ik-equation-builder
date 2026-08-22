# IkEquationBuilder

Part of the KinemaForge project.

Most robot software treats kinematics as a black box: you feed in joint angles, a solver crunches numbers, and a pose comes out. Somewhere underneath, someone once derived the actual equations — usually by hand, on a whiteboard, for one specific robot. **KinemaForge doesn't want to do that by hand anymore.**

The goal of this project is to build a system that, given nothing but a robot's **URDF description**, automatically derives its **symbolic forward-kinematics equations** — not a number, an *equation*. Feed it a KUKA, feed it a UR arm, feed it something that doesn't exist yet — as long as it has a URDF, the same machinery should be able to walk the kinematic chain and produce the math.

> **How far along is it really?** This file describes the pipeline the project is
> working toward. [`STATUS.md`](STATUS.md) tracks what actually exists today,
> component by component, along with every known gap and deferred decision.

## Why bother with symbolic instead of numeric?

Numeric forward kinematics (plug in angles, get a matrix out) is easy and every robotics library does it. The interesting — and much harder — problem is what comes *after*: solving that formula backwards (inverse kinematics), simplifying it, spotting structural patterns (spherical wrist? offset shoulder?), and eventually turning it into fast generated code. All of that is dramatically easier if you have the actual symbolic expression tree to work with, instead of a numeric black box you can only sample.

So the bet here is: **derive the equation once, symbolically, and everything downstream — simplification, solving, code generation — becomes a math problem instead of a guessing game.**

## What this project actually is

This repo is the home of `IkEquationBuilder`: a small, self-contained C++ library that owns the whole pipeline from *URDF file* to *symbolic forward-kinematics formula*.

```
                            URDF
                              │
                              ▼
                      IkEquationBuilder            ← the only public surface
                              │
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
 UrdfModelLoader    KinematicChainBuilder   ForwardKinematicsBuilder
                                             (JointTransformBuilder
                                              once per joint)
                                                    │
                                                    ▼
                                          T_base_chain_tip(q)
                                                    │
                                                    │ · T_chain_tip_tcp
                                                    │   (optional TCP offset)
                                                    ▼
                                             T_base_tcp(q)
                                                    │
                                                    ▼
                                          ExpressionEvaluator
                                        (substitute q → numbers)

                                     T_base_tcp(q) + a task target
                                                    │
                                                    ▼
                                            ConstraintBuilder
                                                    │
                                                    ▼
                                            IkEquationSystem
                                       (3 equations, or 12 for a pose)
```

Everything under the facade is a private implementation detail. From the outside, using the whole pipeline looks like this:

```cpp
using namespace kinemaforge::ik;

IkEquationBuilder builder;

if (const auto result = builder.loadRobotModel("kr640.urdf"); !result)
{
    std::cerr << result.error().message << '\n';
    return;
}

if (const auto result = builder.selectChain("base_link", "tool0"); !result)
{
    // ChainBuildFailed also carries result.error().chainError
    std::cerr << result.error().message << '\n';
    return;
}

if (const auto result = builder.buildForwardKinematics(); !result)
{
    std::cerr << result.error().message << '\n';
    return;
}

const SymbolicTransform* fk = builder.forwardKinematics(); // a symbolic 4x4 transform

// Optional: a constant offset from the chain tip to the real tool centre point.
// Translation in metres, fixed-axis RPY in radians.
if (const auto result = builder.setTcp(FixedRigidTransform{{0.0, 0.0, 0.120},
                                                          {0.0, 0.0, 0.0}}); !result)
{
    std::cerr << result.error().message << '\n';
    return;
}

if (const auto result = builder.buildTcpForwardKinematics(); !result)
{
    std::cerr << result.error().message << '\n';
    return;
}

const SymbolicTransform* tcpFk = builder.tcpForwardKinematics();

// Ask the model a question: where must the tool be? The target's own values are
// in the BASE frame; the TCP decides which frame of the robot is constrained.
const auto system = builder.buildEquationSystem(PositionTarget{{2.0, 0.5, 1.5}});
if (!system)
{
    std::cerr << system.error().message << '
';   // error().constraintError has the detail
    return;
}

// Three equations, lhs = rhs, in the joint variables q1..qn.
for (const auto& equation : system->equations())
    use(equation.lhs, equation.rhs);

// Or substitute a joint configuration into the transform directly. One
// evaluator is one substitution; all sixteen cells share its cache.
ExpressionEvaluator evaluator{{{"q1", 0.0}, {"q2", -1.57}, /* ... */}};
const auto x = evaluator.evaluate((*tcpFk)(0, 3));
```

`buildEquationSystem` is a **query**: it is `const`, it returns the system by
value, and the facade never stores it. A target is a question asked of the model,
not a part of it — so one prepared robot answers as many targets as you like, and
each answer stays valid however you use the facade afterwards.

Accessors return `nullptr` until the corresponding step has succeeded, and a failed call leaves the object exactly as it was — results are built into locals and committed only once they exist.

### State is a graph, not a chain

Each successful step invalidates what it makes obsolete, and nothing stale survives:

```
        RobotDescription
               │
         KinematicChain
          /          \
ForwardKinematics    TCP
          \          /
     TcpForwardKinematics
```

The important consequence is that **the TCP is a sibling of the transform, not a descendant.** So it may be set before or after `buildForwardKinematics`, and rebuilding the transform keeps it. Changing the chain, on the other hand, clears the TCP: the same three numbers would name a different physical point once the tip changes, and a plausible wrong answer is worse than a loud absence.

### Core Components

- **`UrdfModelLoader`** — turns a URDF file into a plain data description of the robot (links, joints, limits, axes). No kinematics here, but it is not a bare parser either: it **guarantees canonical geometry** downstream, so origins hold only finite values and an actuated joint's axis is finite, non-zero and numerically normalized. That guarantee is load-bearing — later stages assert on it rather than re-validating.
- **`KinematicChainBuilder`** — takes that raw description plus a `baseLink`/`toolLink` pair and produces an ordered chain of joints, with actuated joints assigned symbolic variables (`q1`, `q2`, ...). Fixed joints stay in the chain, because they carry real offsets.
- **`JointTransformBuilder`** — converts a single joint into its symbolic 4×4 transform (translation, fixed rotation from the URDF origin, and — for actuated joints — a rotation parameterized by that joint's own symbolic variable).
- **`ForwardKinematicsBuilder`** — multiplies the whole chain of transforms together into one final symbolic transform: the forward kinematics equation.
- **`FixedRigidTransform` + `setTcp`** — an optional constant offset from the end of the selected chain to the real tool centre point, composed as `T_base_tcp(q) = T_base_chain_tip(q) · T_chain_tip_tcp`. Given as translation plus fixed-axis RPY, in metres and radians, and always measured from whichever link you chose as the chain tip — no link name is hard-coded.
- **The symbolic model** (`Expression`, `SymbolicMatrix`, `multiplyTransforms`) — a small expression-tree engine (constants, symbols, `+ - * /`, `sin`/`cos`) strong enough to represent everything the builders above produce, and future-proof enough for the next stage of the project.
- **`ExpressionEvaluator`** — substitutes concrete joint values back into a symbolic tree and returns a `double`. Memoized by node identity, which is not an optimization but a requirement: a composed FK transform for `kr640.urdf` is 281 unique nodes but 21 882 counted with multiplicity. One instance is one substitution, so a cache filled for one configuration can never answer for another.
- **The equation model** (`Equation`, `IkEquationSystem`, `IkTarget`) — what an IK equation and a task target *are*. Types only, and deliberately written **before** anything that builds them, so the constraint builder had a contract to satisfy instead of one to invent. An equation is stored as `lhs = rhs`, never normalised to `lhs - rhs = 0`: subtracting would destroy which side came from the robot and which from the task, and that boundary cannot be recovered without algebraic decomposition.
- **`ConstraintBuilder`** — the first component that *formulates* IK rather than describing geometry. Given the symbolic transform, the chain it came from and a target, it emits an explicit `IkEquationSystem`: three equations for a `PositionTarget`, twelve for a `PoseTarget` (three of position, nine of orientation). Nine matrix cells for three rotational degrees of freedom is on purpose — no fixed subset of three is independent for every rotation, so **the equation count is not the constraint count**, and picking an independent subproblem belongs to the layer that knows the solution structure. It also guarantees that every symbol appearing in the equations is a declared unknown of the chain — checked by walking the equations, not assumed.

### Is the math right?

The symbolic forward kinematics is **cross-checked against an independently written quaternion implementation** that composes rotations without ever forming a matrix. Both robots in `data/urdf`, nine joint configurations each — 18 matrix comparisons, 288 cells — agreeing to within a few ULP against a `1e-12` bound.

Two limits stay attached to that claim and are worth knowing before trusting it: the cross-check starts at `KinematicChain` rather than at the URDF file, and the RPY convention rests on an understanding shared by both implementations rather than on an external reference. Both are recorded in [`STATUS.md`](STATUS.md) under known gaps.

### Where this is headed

The pipeline above is deliberately the *first* phase — parsing and forward kinematics. The same facade is meant to grow into the rest of the story:

```
IkEquationBuilder
│
├── (done) URDF → chain → symbolic forward kinematics, numerically cross-checked
├── (done) constant TCP offset → T_base_tcp(q)
├── (done) the equation and target model — what an equation and a target *are*
├── (done) ConstraintBuilder — FK + a target → an explicit system of IK equations
├── (done) quaternion targets, directional targets, angle-to-a-direction targets
├── (done) all of it behind IkEquationBuilder — PHASE 2 COMPLETE
│
├── (done) a rewrite engine with machine-checked domain safety — no rules yet
│
└── (future)
    ├── EquationSimplifier   — collapse the raw symbolic mess into something readable
    ├── EquationSolver       — actually solve the equation for inverse kinematics
    ├── IkPatternDetector    — recognize known structural shortcuts (e.g. spherical wrist)
    └── CodeGenerator        — emit fast, specialized code straight from the derived formula
```

The project now **states** an IK problem end to end, through one public surface: a URDF file, a chain, an optional tool offset and a target go in; an explicit system of symbolic equations in the joint variables comes out. Six kinds of task are expressible — position, full pose, the direction of a tool axis, and the angle between that axis and a reference direction, the last two with and without position — and a target orientation may be given as a rotation matrix or as a quaternion.

It does **not** solve them. That is the honest shape of the milestone: the project can formulate inverse kinematics, and deriving `q` is the next phase and the larger one.

One thing worth saying plainly: everything built so far was mechanical translation with a known correct answer — parse, walk, compose, evaluate. What follows is not. `EquationSolver` has no single algorithm; it is case analysis, and for some robots a closed-form solution does not exist at all. Measured by effort rather than by component count, the first phase is closer to a quarter of the whole than to half.

## A quiet word about where this is going

`IkEquationBuilder` isn't being built in a vacuum — it's the kinematics-equation engine for a larger private robotics/motion-control project called **MotionBridge**. That project's internals aren't part of this repo and aren't described here, but the reason this module is built as a single clean facade (rather than a pile of loosely related classes) is precisely so that MotionBridge — or any other robotics application — can consume it without ever needing to know how URDF parsing, expression trees, or symbolic matrices actually work underneath.

## Design philosophy

`IkEquationBuilder` is intentionally designed around symbolic mathematics rather than numerical evaluation.

Every stage transforms one representation into another:

```
URDF
  → Robot model
  → Kinematic chain
  → Symbolic transforms (one per joint)
  → Symbolic forward kinematics
  → Symbolic forward kinematics to the TCP
  → An explicit system of IK equations
  → (next) simplification, then a solution
```

Each stage owns exactly one responsibility. The goal is to keep the architecture modular enough that future stages — simplification, inverse kinematics solving, and code generation — can be added without changing the earlier pipeline. So far it has held: the TCP offset, the equation model and the constraint builder were all pure additions, with no change to any earlier stage.

Two decisions shape the symbolic layer more than any others, and both cost something on purpose:

- **`x * 0` is not folded to `0`.** It would erase domain information — `(1/q) * 0` is undefined at `q = 0`, but a folded `0` claims otherwise. FK trees grow noticeably as a result, accepted so that a future simplifier can remove those terms correctly rather than blindly.
- **Validation rejects, it never repairs.** A target rotation that isn't quite a rotation is an error, not something to quietly re-orthonormalize, because repairing it would change the task the caller asked for and hide their mistake.

## Building & testing

See [`COMMANDS.md`](COMMANDS.md) for the day-to-day cheat sheet (CMake configure/build, running tests, PowerShell shortcuts), and [`STATUS.md`](STATUS.md) for the current component-by-component state.

Non-trivial changes go through a written proposal in [`proposal/`](proposal/) before any code is written: the prompt, the current state, and the full code of the change, reviewed and approved first. Once a stage is implemented and green, its proposals move to [`proposal/done/`](proposal/done/README.md) — so the top of `proposal/` is always what is in flight, and the archive keeps the reasoning behind decisions the code no longer shows, including the variants that were rejected.
