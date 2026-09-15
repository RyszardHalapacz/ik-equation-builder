# F3.7 — primitive solvers — architecture

**State:** approved for implementation in this working branch  
**Date:** 2026-09-13

## Goal

Keep the first 6R solver out of a monolithic block by extracting small complete subproblem solvers with explicit branch handling.

## Supported primitives

- linear `a*x+b=0`;
- `sin(x)=c`;
- `cos(x)=c`;
- `a*sin(x)+b*cos(x)=c`;
- generalized planar 2R geometry;
- angle normalization modulo `2*pi`.

Each angular primitive returns all distinct solutions in the principal interval and rejects unreachable values rather than clamping outside a named tolerance. The planar 2R primitive returns both elbow branches when they are distinct.

## Non-goals

No general symbolic equation solving, root isolation, interval arithmetic, or branch selection policy.
