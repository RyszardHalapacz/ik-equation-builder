# F3.6 — analytic IK solution model — architecture

**State:** approved for implementation in this working branch  
**Date:** 2026-09-13

## Goal

Define the result IR before writing a 6R solver. The model must represent multiple regular branches and must not hide singular families behind arbitrary joint values.

## Model

`AnalyticIkSolution` contains a status, zero or more `SolutionBranch` values and a diagnostic message. A regular branch carries explicit joint values and shoulder/elbow/wrist branch labels. A singular branch may instead carry `FreeJointParameter` and `CoupledFreeParameter` records.

The first coupled relation is deliberately simple and sufficient for a spherical wrist:

```text
dependent = offset + multiplier * free    (mod 2*pi)
```

This lets the solver report a continuum such as `q6 = offset - q4` without choosing an arbitrary `q4`.

## Status and singularities

Statuses are `Solutions`, `NoSolution`, `InfiniteFamily`, `UnsupportedKinematicStructure`, and `InvalidTarget`. Singularities are classified separately (`ShoulderAxis`, elbow folded/extended, wrist aligned/anti-aligned) so the caller can distinguish mathematical multiplicity from ordinary failure.

## Non-goals

No symbolic conditions, no general polynomial root representation, no optimizer for selecting one branch, and no joint-limit policy beyond representing the admissible interval of a free parameter.
