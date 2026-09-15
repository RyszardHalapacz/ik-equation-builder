# F3.4 — Kinematic geometry model — architecture

**State:** approved for implementation in this working branch  
**Date:** 2026-09-13

## Goal

Translate an already selected `KinematicChain` into a small numeric geometry model that a future IK pattern detector can query without inspecting symbolic expression trees.

At the zero joint configuration, for every actuated joint the model records the joint axis as an infinite line in the chain base frame:

```text
AxisLine = point_on_axis + t * unit_direction
```

The component also provides explicit approximate geometric predicates (parallel, perpendicular, coincident, distance and line intersection). This is the first deliberately approximate layer in the IK pipeline; it must stay separate from exact symbolic rewriting.

## Contracts

1. Geometry is expressed in the selected chain's **base frame**.
2. Fixed joints contribute their origin transform but do not produce an axis entry.
3. Revolute, continuous and prismatic joints all produce an axis entry; pattern detectors decide which kinds they accept.
4. The URDF convention is identical to FK: `R_origin = Rz(yaw) * Ry(pitch) * Rx(roll)`.
5. At `q = 0`, joint motion is identity, so the child zero frame is obtained only by the fixed origin transform.
6. Actuated axes are expected to be finite and non-zero. Unlike `JointTransformBuilder`, this component returns an error for a violated hand-built-chain precondition because a pattern detector must not classify invalid geometry.
7. Approximate geometry uses named tolerances and never feeds a tolerance back into symbolic equality.

## Tolerances

```text
angular: 1e-10
linear:  1e-9 m
```

Directions are unit vectors, therefore cross-product magnitude and dot product are directly usable as angular residuals. One nanometre is deliberately much larger than floating-point propagation through the shipped robot origins while still far below manufacturing-scale geometry.

## Public model

```cpp
struct AxisLine { Vector3 point; Vector3 direction; };

struct JointAxisGeometry {
    std::size_t chainPosition;
    std::size_t sourceJointIndex;
    std::string name;
    JointType type;
    AxisLine axis;
};

struct KinematicGeometry {
    std::string baseLink;
    std::string toolLink;
    std::vector<JointAxisGeometry> actuatedJoints;
};
```

`KinematicGeometryBuilder::build(chain)` returns `std::expected<KinematicGeometry, KinematicGeometryError>`.

## Query operations

- `axesParallel`
- `axesPerpendicular`
- `axisDistance`
- `axesCoincident`
- `axisIntersectionPoint`
- `pointToAxisDistance`

`axisIntersectionPoint` returns `nullopt` for parallel/coincident/skew lines. A coincident pair has infinitely many intersections and must not be represented by one arbitrary point.

## Non-goals

- pattern classification;
- spherical-wrist detection;
- DH parameters;
- tolerance inference from robot scale;
- solving any equations.

## Gate

- synthetic chains pin RPY and fixed-joint accumulation;
- invalid axes/non-finite origins are rejected;
- relation primitives are unit-tested;
- KR4 and KR640 geometry can be built from their selected chains;
- the last three revolute axes of both shipped robots are pairwise intersecting within the linear tolerance (the classification itself belongs to F3.5).
