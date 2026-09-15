# F3.11 — analytic IK end-to-end validation — architecture

**State:** implemented as a working-branch validation gate  
**Date:** 2026-09-13

## Required round trip

```text
q -> independent numeric FK -> target -> analytic IK -> every returned q' -> FK(q') -> target
```

The key rule is that branch generation and verification must not rely only on the same algebraic derivation. The repository's quaternion FK support is used as an independent representation for external property checks.

## Coverage

- KR640 and KR4;
- random configurations inside usable joint ranges;
- multiple regular branches;
- non-zero TCP translation and rotation;
- unreachable targets;
- invalid rotations;
- wrist singularity;
- joint limits;
- facade entry point.

## Environment caveat

A complete CMake/CTest gate cannot be executed in the current sandbox because `FetchContent` requires GitHub for pugixml/googletest and DNS/network access is disabled. The working branch therefore records direct `g++ -Werror` compilation plus standalone property harnesses separately from the still-required full project gate.
