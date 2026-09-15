# F3.5 — IK pattern detector — architecture

**State:** approved for implementation in this working branch  
**Date:** 2026-09-13

## Goal

Classify only geometric structures for which the implementation can provide explicit evidence. The first supported family is the one exercised by both shipped KUKA robots: six rotary joints, a spherical wrist on joints 4–6, and parallel shoulder/elbow axes on joints 2–3.

The detector is deliberately conservative. Failure to match yields no pattern, never a best-effort guess.

## Patterns

```text
SphericalWrist
ParallelShoulderElbow
SixRotarySphericalWrist
```

`SixRotarySphericalWrist` is a composite support marker used by the first analytic solver. It requires:

- exactly six actuated joints;
- every actuated joint is Revolute or Continuous;
- axes 4, 5 and 6 share one point within `kGeometryLinearTolerance`;
- axes 2 and 3 are parallel within `kGeometryAngularTolerance`.

## Evidence

A detected pattern carries metrics, not only an enum:

```cpp
struct PatternEvidence {
    double maximumAxisResidual;
    std::optional<Vector3> wristCenter;
};
```

`maximumAxisResidual` is the worst distance/angle residual used by that pattern. This makes classification inspectable and prevents a future caller from treating a tolerance-based result as exact geometry.

## Spherical-wrist algorithm

For three rotary axes:

1. require every pair of lines to have distance <= linear tolerance;
2. choose a non-parallel pair and compute its intersection;
3. require the candidate point to be within linear tolerance of all three axes;
4. if all three axes are coincident there is no unique wrist center and the pattern is rejected for now.

The last restriction is intentional: the future wrist solver needs a concrete center.

## Non-goals

- arbitrary 6R classification;
- planar-chain decomposition;
- DH extraction;
- solver selection beyond the first supported family;
- probabilistic/ranked matching.
