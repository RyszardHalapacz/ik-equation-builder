# F3.9 — spherical-wrist orientation solver — implementation

**State:** implemented and property-tested in this working branch  
**Date:** 2026-09-13

The implementation lives in `AnalyticIkSolver.cpp` and shares the same geometry-derived constants as the position solver. Regular XYX decomposition returns primary and flip branches. Every complete six-joint branch goes through the solver's FK round-trip check before being returned.
