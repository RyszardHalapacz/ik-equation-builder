#pragma once
#include "common/source_location.hpp"
#include <cstdint>
#include <string>
#include <type_traits>

namespace mt {

enum class DiagnosticSeverity : uint8_t {
    Info,
    Warning,
    Error
};

// Trimmed from MotionBridge's common/diagnostic.hpp: only the Core and
// Kinematics ranges are relevant to this repo (IkEquationBuilder has no
// frontend/backend/extrusion/feasibility/placement/workspace concerns).
enum class DiagnosticCode : uint16_t {
    Core_InternalError = 0,

    // Kinematics [400–499]
    Kinematics_InvalidRobotModel     = 400,
    Kinematics_UnreachablePose       = 401,
    Kinematics_JointLimitExceeded    = 402,
    Kinematics_SingularConfiguration = 403,
    Kinematics_IkNoSolution          = 404,
    Kinematics_DiscontinuityDetected = 405,
    Kinematics_JointJumpRejected     = 406,
};

struct Diagnostic {
    DiagnosticSeverity severity = DiagnosticSeverity::Info;
    DiagnosticCode     code     = DiagnosticCode::Core_InternalError;
    SourceRange        location {};
    std::string        message;
};

static_assert(!std::is_polymorphic_v<Diagnostic>, "Virtual dispatch is forbidden");

} // namespace mt
