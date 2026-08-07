#include "support/TransformComparison.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>

namespace kinemaforge::testsupport {

namespace {

// R_error = R_expected^T * R_actual ; angle = acos((trace - 1) / 2).
//
// Diagnostic only -- the pass/fail condition stays per-cell. This answers the
// question a single cell difference cannot: is the discrepancy a real
// orientation error, or noise in one entry?
double orientationAngleError(const Matrix4& actual, const Matrix4& expected)
{
    // trace(R_expected^T * R_actual) = sum_i sum_k R_expected(k,i) * R_actual(k,i)
    double trace = 0.0;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t k = 0; k < 3; ++k)
            trace += expected[k][i] * actual[k][i];

    // clamp: with errors near 1e-16 the argument can leave [-1, 1].
    return std::acos(std::clamp(0.5 * (trace - 1.0), -1.0, 1.0));
}

} // namespace

bool withinTolerance(double actual, double expected)
{
    return std::abs(actual - expected)
           <= kAbsoluteTolerance + kRelativeTolerance * std::abs(expected);
}

Matrix4 toMatrix4(const RigidTransform& transform)
{
    // Unqualified: this translation unit is already inside
    // kinemaforge::testsupport, where toRotationMatrix lives.
    const Matrix3 rotation = toRotationMatrix(transform.rotation);

    Matrix4 result{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            result[row][column] = rotation[row][column];

    result[0][3] = transform.translation.x;
    result[1][3] = transform.translation.y;
    result[2][3] = transform.translation.z;
    result[3][3] = 1.0;
    return result;
}

std::optional<Matrix4> evaluateSymbolic(const ik::SymbolicTransform& transform,
                                        const ik::SymbolValues& values)
{
    ik::ExpressionEvaluator evaluator{values};

    Matrix4 numeric{};
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            const auto value = evaluator.evaluate(transform(row, column));
            if (!value)
            {
                ADD_FAILURE() << "evaluation failed at cell (" << row << ", " << column
                              << "), code " << static_cast<int>(value.error().code)
                              << " symbol '" << value.error().symbolName << "'";
                return std::nullopt;
            }
            numeric[row][column] = *value;
        }
    return numeric;
}

void expectMatrixMatches(const Matrix4& actual, const Matrix4& expected,
                         std::string_view label)
{
    double maxRotationError = 0.0;
    double maxTranslationError = 0.0;

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message()
                         << label << " cell (" << row << ", " << column << ")");

            const double error = std::abs(actual[row][column] - expected[row][column]);
            if (row < 3 && column < 3)
                maxRotationError = std::max(maxRotationError, error);
            else if (row < 3)
                maxTranslationError = std::max(maxTranslationError, error);

            EXPECT_TRUE(withinTolerance(actual[row][column], expected[row][column]))
                << "actual=" << actual[row][column]
                << " expected=" << expected[row][column]
                << " error=" << error;
        }

    // Printed unconditionally: the implementation report quotes these.
    std::cout << "[ MEASURE  ] " << label
              << "  rotation=" << maxRotationError
              << "  translation=" << maxTranslationError
              << "  angle=" << orientationAngleError(actual, expected) << "\n";
}

} // namespace kinemaforge::testsupport
