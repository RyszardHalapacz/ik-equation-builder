#pragma once

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/model/FixedRigidTransform.hpp"
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
    ChainBuildFailed,
    ForwardKinematicsNotBuilt,
    TcpNotSet,
    InvalidTcpTransform
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
// State is a dependency graph, not a linear chain:
//
//              RobotDescription
//                     |
//               KinematicChain
//                /          \
//     ForwardKinematics    TCP configuration
//                \          /
//              TcpForwardKinematics
//
// A successful step sets its node and clears that node's descendants. The TCP
// is a sibling of the transform, not a descendant, so it may be set before or
// after buildForwardKinematics -- and rebuilding the transform keeps it.
//
// A KinematicChain names links of one specific robot, a transform carries
// symbols of one specific chain, and a TCP offset is measured from one
// specific chain tip -- so anything surviving a change upstream would be an
// answer to a question no longer being asked.
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

    // A TCP offset is defined relative to the tip of the currently selected
    // chain, so a chain must be selected first. That is the only prerequisite:
    // it may be called before or after buildForwardKinematics, because the TCP
    // does not depend on the transform.
    [[nodiscard]] std::expected<void, IkEquationBuilderError>
    setTcp(const FixedRigidTransform& tcp);

    // Idempotent, and valid in any state: before a chain is selected, with no
    // TCP set, and repeatedly. Afterwards tcp() and tcpForwardKinematics() are
    // both null.
    void clearTcp() noexcept;

    // T_base_tcp = T_base_chain_tip * T_chain_tip_tcp.
    //
    // Reports the FIRST missing prerequisite -- chain, then transform, then
    // TCP -- so the caller is told the step to take next rather than the last
    // one in the sequence.
    [[nodiscard]] std::expected<void, IkEquationBuilderError>
    buildTcpForwardKinematics();

    // Accessors return nullptr until the corresponding step has succeeded.
    //
    // A pointer obtained earlier must not be treated as access to the previous
    // result after an operation that changes its node. This is a semantic
    // contract, not an address one: assigning into an already engaged
    // std::optional constructs in place, so the address can survive while the
    // value behind it becomes a different result. That is worse than a
    // dangling pointer, because nothing crashes -- the caller simply reads the
    // new value believing it holds the old one.
    //
    //     kinematicChain()       stale after loadRobotModel, selectChain
    //     forwardKinematics()    ... plus buildForwardKinematics
    //     tcp()                  after loadRobotModel, selectChain, setTcp,
    //                            clearTcp
    //     tcpForwardKinematics() after any of the six operations
    //
    // The converse is a promise, and it is tested: setTcp does NOT affect
    // forwardKinematics(), and buildForwardKinematics does NOT affect tcp().
    //
    // All pointers are non-owning and die with the facade.
    [[nodiscard]] const KinematicChain* kinematicChain() const noexcept;
    [[nodiscard]] const SymbolicTransform* forwardKinematics() const noexcept;
    [[nodiscard]] const FixedRigidTransform* tcp() const noexcept;
    [[nodiscard]] const SymbolicTransform* tcpForwardKinematics() const noexcept;

private:
    UrdfModelLoader urdfLoader_;
    KinematicChainBuilder chainBuilder_;
    JointTransformBuilder jointTransformBuilder_;
    ForwardKinematicsBuilder fkBuilder_;

    std::optional<RobotDescription> robotDescription_;
    std::optional<KinematicChain> kinematicChain_;
    std::optional<SymbolicTransform> forwardKinematics_;
    std::optional<FixedRigidTransform> tcp_;
    std::optional<SymbolicTransform> tcpForwardKinematics_;
};

} // namespace kinemaforge::ik
