#pragma once

// The single tolerance model of the analytic IK pipeline.
//
// Both the general solver and the generated runtime read their thresholds from
// here; older names (kGeometryLinearTolerance, detail::kLimitTolerance,
// generated::kLinearTolerance, ...) are aliases. Two independent sets of
// literals used to drift apart, and two of the 2026-11-16 audit findings came
// from how separately chosen thresholds interacted.
//
// Dependency-free on purpose: it is included by the model layer, the solver
// and generated code alike.
//
// The thresholds form a ladder, from round-off to ill-conditioning. Every
// repair that moves a joint value is bounded so that it cannot by itself use
// up the FK acceptance check, which has the final word on every branch:
//
//   kRoundOff (1e-12)         noise of one primitive computation; joins
//                             intervals that miss by this much, absorbs
//                             rounding when counting full turns
//   kLimitAdmission (1e-10)   a joint value this close outside a limit is the
//                             limit itself and is snapped onto it. Realistic
//                             upstream round-off at the joints is ~1e-12
//                             (1.29e-12 measured on KR640; 1e-12 used to be
//                             the margin and lost that target)
//   kSingularLimitSnap (1e-7) an ILL-CONDITIONED angle (elbow near straight,
//                             singular-family q5) this close outside a limit
//                             is snapped and the dependent joints re-solved
//   kWristSnapOrientationBudget (1e-10)
//                             near the wrist singularity q4/q6 may be snapped
//                             by up to budget/|sin q5|; the resulting
//                             orientation change is at most this budget
//
//   kFkPosition / kFkOrientation (2e-9)
//                             acceptance of every returned branch: position
//                             distance and element-wise rotation distance
//
// Singularity detection is a separate axis:
//   kSingularLinear (1e-9)    wrist centre on joint-1 axis, equal links with
//                             the wrist on the joint-2 axis, degenerate axes
//   kSingularAngular (1e-10)  |sin q5| below this: the wrist is a family
//
// Input validation:
//   kOrientationValidation (1e-8)    orthogonality/determinant of the target
//   kMaxOrientationCorrection (2e-8) largest distance to the nearest rotation
//                                    the solvers project the target onto

namespace kinemaforge::ik::tolerance {

// Largest distance from the base at which the margins below are guaranteed to
// stay inside the FK budget. Industrial arms reach ~0.5-4 m.
inline constexpr double kMaxSupportedReach = 10.0; // m

inline constexpr double kFkPosition = 2e-9;    // m
inline constexpr double kFkOrientation = 2e-9; // rotation matrix element

inline constexpr double kRoundOff = 1e-12;
inline constexpr double kLimitAdmission = 1e-10;
inline constexpr double kSingularLimitSnap = 1e-7;
inline constexpr double kWristSnapOrientationBudget = 1e-10;

// Two branches closer than this in every joint (modulo 2*pi) are one branch.
inline constexpr double kSameBranch = 1e-9;

inline constexpr double kSingularLinear = 1e-9;   // m
inline constexpr double kSingularAngular = 1e-10; // rad

inline constexpr double kOrientationValidation = 1e-8;
inline constexpr double kMaxOrientationCorrection = 2e-8;

// --- Relations the numerics rely on -----------------------------------------
static_assert(kLimitAdmission * kMaxSupportedReach <= kFkPosition / 2.0,
              "snapping a joint onto its limit must not use up the FK position budget");
static_assert(kLimitAdmission <= kFkOrientation / 2.0,
              "snapping a joint onto its limit must not use up the FK orientation budget");
static_assert(10.0 * kWristSnapOrientationBudget <= kFkOrientation,
              "the wrist snap must stay well inside the FK orientation budget");
static_assert(10.0 * kSingularAngular <= kFkOrientation,
              "treating a near-singular wrist as singular must stay inside the FK budget");
static_assert(kRoundOff < kLimitAdmission && kLimitAdmission < kSingularLimitSnap,
              "margins are ordered from round-off to ill-conditioning");
static_assert(kSameBranch > kLimitAdmission,
              "distinct branches must differ by more than any admission repair");
static_assert(kMaxOrientationCorrection >= kOrientationValidation,
              "the solvers must be able to correct every target validation accepts");

} // namespace kinemaforge::ik::tolerance
