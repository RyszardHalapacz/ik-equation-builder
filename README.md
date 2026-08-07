# IkEquationBuilder

Part of the KinemaForge project.

Most robot software treats kinematics as a black box: you feed in joint angles, a solver crunches numbers, and a pose comes out. Somewhere underneath, someone once derived the actual equations — usually by hand, on a whiteboard, for one specific robot. **KinemaForge doesn't want to do that by hand anymore.**

The goal of this project is to build a system that, given nothing but a robot's **URDF description**, automatically derives its **symbolic forward-kinematics equations** — not a number, an *equation*. Feed it a KUKA, feed it a UR arm, feed it something that doesn't exist yet — as long as it has a URDF, the same machinery should be able to walk the kinematic chain and produce the math.

## Why bother with symbolic instead of numeric?

Numeric forward kinematics (plug in angles, get a matrix out) is easy and every robotics library does it. The interesting — and much harder — problem is what comes *after*: solving that formula backwards (inverse kinematics), simplifying it, spotting structural patterns (spherical wrist? offset shoulder?), and eventually turning it into fast generated code. All of that is dramatically easier if you have the actual symbolic expression tree to work with, instead of a numeric black box you can only sample.

So the bet here is: **derive the equation once, symbolically, and everything downstream — simplification, solving, code generation — becomes a math problem instead of a guessing game.**

## What this project actually is

This repo is the home of `IkEquationBuilder`: a small, self-contained C++ library that owns the whole pipeline from *URDF file* to *symbolic forward-kinematics formula*.

```
                URDF
                  │
                  ▼
        IkEquationBuilder
                  │
     ┌────────────┼────────────┐
     ▼            ▼            ▼
UrdfLoader   ChainBuilder   FKBuilder
                  │
                  ▼
         Symbolic Transform
```

Everything under the facade is a private implementation detail. From the outside, using the whole pipeline looks like this:

```cpp
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
```

State is explicit and cascading: loading a robot clears the chain and the transform, selecting a chain clears the transform. Nothing stale survives. Point it at a URDF, ask for the chain you care about, and it derives the formula fresh, every time.

### Core Components

- **`UrdfModelLoader`** — turns a URDF file into a plain data description of the robot (links, joints, limits, axes). Parsing only; no math happens here.
- **`KinematicChainBuilder`** — takes that raw description plus a `baseLink`/`toolLink` pair and produces an ordered chain of joints, with actuated joints assigned symbolic variables (`q1`, `q2`, ...).
- **`JointTransformBuilder`** — converts a single joint into its symbolic 4×4 transform (translation, fixed rotation from the URDF origin, and — for actuated joints — a rotation parameterized by that joint's own symbolic variable).
- **`ForwardKinematicsBuilder`** — multiplies the whole chain of transforms together into one final symbolic transform: the forward kinematics equation.
- **`FixedRigidTransform` + `setTcp`** — an optional constant offset from the end of the selected chain to the real tool centre point, composed as `T_base_tcp(q) = T_base_chain_tip(q) · T_chain_tip_tcp`. Given as translation plus fixed-axis RPY, in metres and radians, and always measured from whichever link you chose as the chain tip — no link name is hard-coded.
- **The symbolic model** (`Expression`, `SymbolicMatrix`) — a small expression-tree engine (constants, symbols, `+ - * /`, `sin`/`cos`) strong enough to represent everything the builders above produce, and future-proof enough for the next stage of the project.

### Where this is headed

The pipeline above is deliberately the *first* phase — parsing and forward kinematics only. Once that's solid, the same facade is meant to grow into the rest of the story:

```
IkEquationBuilder
│
├── (done / in progress) URDF → chain → symbolic forward kinematics
│
└── (future)
    ├── EquationSimplifier   — collapse the raw symbolic mess into something readable
    ├── EquationSolver       — actually solve the equation for inverse kinematics
    ├── IkPatternDetector    — recognize known structural shortcuts (e.g. spherical wrist)
    └── CodeGenerator        — emit fast, specialized code straight from the derived formula
```

None of that later half exists yet, on purpose — there's no point designing a solver before the equations it's supposed to solve actually exist.

## A quiet word about where this is going

`IkEquationBuilder` isn't being built in a vacuum — it's the kinematics-equation engine for a larger private robotics/motion-control project called **MotionBridge**. That project's internals aren't part of this repo and aren't described here, but the reason this module is built as a single clean facade (rather than a pile of loosely related classes) is precisely so that MotionBridge — or any other robotics application — can consume it without ever needing to know how URDF parsing, expression trees, or symbolic matrices actually work underneath.

## Design philosophy

`IkEquationBuilder` is intentionally designed around symbolic mathematics rather than numerical evaluation.

Every stage transforms one representation into another:

```
URDF
  → Robot model
  → Kinematic chain
  → Symbolic transforms
  → Symbolic forward kinematics
```

Each stage owns exactly one responsibility. The goal is to keep the architecture modular enough that future stages — simplification, inverse kinematics solving, and code generation — can be added without changing the earlier pipeline.

## Building & testing

See [`COMMANDS.md`](COMMANDS.md) for the day-to-day cheat sheet (CMake configure/build, running tests, PowerShell shortcuts).
