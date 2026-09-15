#pragma once

#include "ik_equations/model/FixedRigidTransform.hpp"
#include "ik_equations/model/KinematicChain.hpp"

#include <expected>
#include <optional>
#include <string>
#include <string_view>

namespace kinemaforge::ik {

enum class CodeGenerationErrorCode
{
    UnsupportedKinematicStructure,
    InvalidKinematicGeometry,
    InvalidNamespaceName,
    NonFiniteParameter
};

struct CodeGenerationError
{
    CodeGenerationErrorCode code{};
    std::string message;
};

class CppAnalyticIkGenerator
{
public:
    [[nodiscard]] std::expected<std::string, CodeGenerationError>
    generate(const KinematicChain& chain,
             const std::optional<FixedRigidTransform>& tcp = std::nullopt,
             std::string_view namespaceName = "kinemaforge_generated_ik") const;
};

} // namespace kinemaforge::ik
