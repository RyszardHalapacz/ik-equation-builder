#pragma once

namespace kinemaforge::ik {

// (w, x, y, z) -- scalar FIRST. ROS/tf2 and Eigen::Quaterniond::coeffs()
// serialise (x, y, z, w); copying the four numbers across without reordering
// yields a rotation that is a perfectly valid rotation, just a different one.
// The norm is untouched by that mistake, so validation cannot catch it and does
// not pretend to -- when filling this from an external source, use designated
// initialisers:
//
//     Quaternion{.w = qw, .x = qx, .y = qy, .z = qz};
//
// Defaults to the identity rotation. Validity (finite components, unit norm) is
// NOT a construction condition -- same rule as RotationMatrix3: validate()
// decides.
struct Quaternion
{
    double w{1.0};
    double x{};
    double y{};
    double z{};
};

} // namespace kinemaforge::ik
