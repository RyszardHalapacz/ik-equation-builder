#pragma once

#include "ik_equations/symbolic/SymbolicMatrix.hpp"

namespace kinemaforge::ik {

using SymbolicRotation = SymbolicMatrix<3, 3>;
using SymbolicVector3 = SymbolicMatrix<3, 1>;
using SymbolicTransform = SymbolicMatrix<4, 4>;

} // namespace kinemaforge::ik
