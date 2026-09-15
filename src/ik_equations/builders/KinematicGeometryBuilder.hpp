#pragma once

#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/model/KinematicGeometry.hpp"

#include <expected>
#include <string>

namespace kinemaforge::ik {

enum class KinematicGeometryErrorCode
{
    NonFiniteOrigin,
    InvalidActuatedAxis
};

struct KinematicGeometryError
{
    KinematicGeometryErrorCode code{};
    std::string jointName;
    std::string message;
};

class KinematicGeometryBuilder
{
public:
    [[nodiscard]] std::expected<KinematicGeometry, KinematicGeometryError>
    build(const KinematicChain& chain) const;
};

} // namespace kinemaforge::ik
