#pragma once

#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <array>
#include <optional>
#include <string_view>

namespace kinemaforge::testsupport {

using Matrix4 = std::array<std::array<double, 4>, 4>;

// Candidate tolerance, measured once and approved: the worst observed error
// across 18 matrix comparisons was 5.55e-16. Exceeding it is a finding for
// review, NOT a licence to raise the number.
inline constexpr double kAbsoluteTolerance = 1e-12;
inline constexpr double kRelativeTolerance = 1e-12;

bool withinTolerance(double actual, double expected);

Matrix4 toMatrix4(const RigidTransform& transform);

// One evaluator for all sixteen cells -- that is the whole reason
// ExpressionEvaluator is a session. Returns nullopt and records a gtest
// failure if any cell fails to evaluate.
std::optional<Matrix4> evaluateSymbolic(const ik::SymbolicTransform& transform,
                                        const ik::SymbolValues& values);

// Per-cell comparison; prints the worst rotation and translation error plus
// the orientation angle unconditionally, so reports can quote the run.
void expectMatrixMatches(const Matrix4& actual, const Matrix4& expected,
                         std::string_view label);

} // namespace kinemaforge::testsupport
