#pragma once

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/model/KinematicChainError.hpp"
#include "ik_equations/model/RobotDescription.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <expected>
#include <filesystem>
#include <optional>
#include <string>

namespace kinemaforge::ik {

enum class IkEquationBuilderErrorCode
{
    RobotModelNotLoaded,
    KinematicChainNotSelected,
    UrdfLoadFailed,
    ChainBuildFailed
};

// Errors returned by IkEquationBuilder satisfy this invariant: chainError has
// a value if and only if code == ChainBuildFailed. The type is a public
// aggregate, so a caller can hand-build an inconsistent value; that is not
// worth a variant or a factory, but the guarantee is about what this class
// produces, not about what the struct can hold.
//
// UrdfLoadFailed carries only `message`, and that is not an oversight:
// UrdfModelLoader consumes its structured LoadError internally and throws a
// std::runtime_error carrying an already-assembled string, so no typed code
// survives to be preserved here. Giving the facade a typed load error means
// changing UrdfModelLoader -- a separate change, recorded in STATUS.md.
struct IkEquationBuilderError
{
    IkEquationBuilderErrorCode code{};
    std::string message;
    std::optional<KinematicChainError> chainError;
};

// The public entry point of this module: URDF file -> symbolic forward
// kinematics, with everything underneath kept private.
//
// State machine, with each successful step invalidating what it obsoletes:
//
//     loadRobotModel         -> new model, clears chain and transform
//     selectChain            -> new chain, clears transform
//     buildForwardKinematics -> new transform
//
// A KinematicChain names links of one specific robot and a transform carries
// symbols of one specific chain, so anything surviving a change upstream would
// be an answer to a question no longer being asked.
//
// Failure leaves the object untouched: results are built into locals and only
// committed once they exist. That promise covers domain errors -- a bad path,
// a missing link -- not catastrophic ones such as std::bad_alloc.
//
// Not safe for concurrent modification, like any object with state.
class IkEquationBuilder
{
public:
    IkEquationBuilder();

    [[nodiscard]] std::expected<void, IkEquationBuilderError>
    loadRobotModel(const std::filesystem::path& urdfPath);

    [[nodiscard]] std::expected<void, IkEquationBuilderError>
    selectChain(const std::string& baseLink, const std::string& toolLink);

    [[nodiscard]] std::expected<void, IkEquationBuilderError>
    buildForwardKinematics();

    // Returns nullptr until the corresponding step has succeeded.
    //
    // Returned pointers are non-owning and may be invalidated by any
    // successful state-changing operation on this object, as well as by its
    // destruction. Do not hold one across a call to loadRobotModel,
    // selectChain or buildForwardKinematics.
    [[nodiscard]] const KinematicChain* kinematicChain() const noexcept;
    [[nodiscard]] const SymbolicTransform* forwardKinematics() const noexcept;

private:
    UrdfModelLoader urdfLoader_;
    KinematicChainBuilder chainBuilder_;
    JointTransformBuilder jointTransformBuilder_;
    ForwardKinematicsBuilder fkBuilder_;

    std::optional<RobotDescription> robotDescription_;
    std::optional<KinematicChain> kinematicChain_;
    std::optional<SymbolicTransform> forwardKinematics_;
};

} // namespace kinemaforge::ik
