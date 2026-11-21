#include <gtest/gtest.h>

#include "fresh_generated/kr640_fresh.hpp"
#include "ik_equations/model/IkTolerances.hpp"
#include "ik_equations/model/KinematicGeometry.hpp"
#include "ik_equations/model/TargetValidation.hpp"
#include "ik_equations/solver/PrimitiveSolvers.hpp"
#include "ik_equations/solver/detail/AnalyticIkNumerics.hpp"

// One tolerance model for both solvers (2026-11-16 follow-up #7). These tests
// pin the relations the numerics rely on, and that every older name is an
// alias of the single source -- so the general and the generated solver can
// no longer drift apart one literal at a time.

namespace ik = kinemaforge::ik;
namespace tol = kinemaforge::ik::tolerance;
namespace gen = kinemaforge::ik::generated;

TEST(IkTolerances, SnappingAJointCannotConsumeTheFkBudget)
{
    // A joint moved onto its limit by kLimitAdmission moves a point at the
    // largest supported reach by at most half the FK position tolerance.
    EXPECT_LE(tol::kLimitAdmission * tol::kMaxSupportedReach, tol::kFkPosition / 2.0);
    EXPECT_LE(tol::kLimitAdmission, tol::kFkOrientation / 2.0);
    // The wrist snap is bounded by its orientation budget, well inside FK.
    EXPECT_LE(10.0 * tol::kWristSnapOrientationBudget, tol::kFkOrientation);
    // Treating |q5| <= kSingularAngular as exactly singular costs at most that
    // much orientation error.
    EXPECT_LE(10.0 * tol::kSingularAngular, tol::kFkOrientation);
}

TEST(IkTolerances, MarginsAreOrderedFromRoundOffToIllConditioning)
{
    EXPECT_LT(tol::kRoundOff, tol::kLimitAdmission);
    EXPECT_LT(tol::kLimitAdmission, tol::kSingularLimitSnap);
    // Distinct branches must differ by more than any admission repair.
    EXPECT_GT(tol::kSameBranch, tol::kLimitAdmission);
    // Realistic upstream round-off at the joints is ~1e-12 (measured 1.29e-12
    // on KR640); the admission margin must cover it with room to spare.
    EXPECT_GE(tol::kLimitAdmission, 10.0 * 1.29e-12);
    // The solver may correct at least what validation accepts.
    EXPECT_GE(tol::kMaxOrientationCorrection, tol::kOrientationValidation);
}

TEST(IkTolerances, EveryOlderNameAliasesTheSingleSource)
{
    EXPECT_EQ(ik::kGeometryLinearTolerance, tol::kSingularLinear);
    EXPECT_EQ(ik::kGeometryAngularTolerance, tol::kSingularAngular);
    EXPECT_EQ(ik::kPrimitiveSolverTolerance, tol::kRoundOff);
    EXPECT_EQ(ik::kOrientationTolerance, tol::kOrientationValidation);
    EXPECT_EQ(ik::kMaxOrientationCorrection, tol::kMaxOrientationCorrection);
    EXPECT_EQ(ik::detail::kLimitTolerance, tol::kLimitAdmission);
    EXPECT_EQ(ik::detail::kSingularLimitSnapMargin, tol::kSingularLimitSnap);
    EXPECT_EQ(ik::detail::kMaxOrientationCorrection, tol::kMaxOrientationCorrection);
    EXPECT_EQ(gen::detail::kLinearTolerance, tol::kSingularLinear);
    EXPECT_EQ(gen::detail::kAngularTolerance, tol::kSingularAngular);
    EXPECT_EQ(gen::detail::kOrientationTolerance, tol::kOrientationValidation);
}
