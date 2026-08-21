#pragma once

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ConstraintBuilder.hpp"
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
    InvalidTcpTransform,
    TcpForwardKinematicsNotBuilt,
    ConstraintBuildFailed
};

// Errors returned by IkEquationBuilder satisfy this invariant: chainError has
// a value if and only if code == ChainBuildFailed, and constraintError has a
// value if and only if code == ConstraintBuildFailed. The type is a public
// aggregate, so a caller can hand-build an inconsistent value; that is not
// worth a variant or a factory, but the guarantee is about what this class
// produces, not about what the struct can hold.
//
// Two optionals side by side rather than a variant, for the same reason: the
// guarantee is about production, and a variant would buy enforcement the class
// does not need at the cost of a shape every caller has to unwrap.
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
    std::optional<ConstraintBuilderError> constraintError;
};

// The public entry point of this module: URDF file -> symbolic forward
// kinematics, with everything underneath kept private.
//
// State is a dependency graph, not a linear chain:
//
//              RobotDescription
//                     |
//               KinematicChain
//                 +---+---+
//                 |       |
//     ForwardKinematics   TCP configuration
//                 |       |
//                 +---+---+
//                     |
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

    // URDF + TCP + target -> an explicit system of IK equations.
    //
    // WHICH FRAME OF THE ROBOT is constrained follows from this object's own
    // state, and this is the one place in the project where that ambiguity can
    // be closed rather than documented:
    //
    //     a TCP is set      -> the TCP frame is constrained, and
    //                          tcpForwardKinematics() is used
    //     no TCP is set     -> the chain-tip frame is constrained, and
    //                          forwardKinematics() is used
    //
    // What does NOT change either way: the target's own quantities are
    // expressed in the BASE frame, always. A PoseTarget is T_base_target, a
    // direction is a direction in the base frame, and the equations read
    // T(0,3)(q) = target.x with both sides in base coordinates. Setting a TCP
    // does not mean "re-express the target in tool coordinates" -- doing that
    // would produce a constraint wrong by the whole tool transform. It means
    // "constrain the tool frame instead of the chain tip".
    //
    // For a ToolDirectionTarget both frames appear at once and neither may be
    // confused for the other: the axis names an axis OF THE CONSTRAINED FRAME,
    // while the direction it must point along is given in the base frame.
    //
    // Having no TCP is not a missing prerequisite -- it is the other legal
    // mode. But a TCP that is SET while its transform has not been built IS a
    // missing prerequisite and reports one: falling back to the chain-tip
    // transform would return a system wrong by exactly the tool offset, which
    // compiles, runs and lies.
    //
    // Prerequisites are reported in the order they must be satisfied: chain,
    // then forward kinematics, then -- only if a TCP is set -- the TCP
    // transform.
    //
    // const, and returning by value rather than storing: a target is a question
    // asked of the model, not a part of it. The returned system is a snapshot;
    // it stays valid and unchanged no matter what happens to this object
    // afterwards, because nothing here owns it. Copying one duplicates
    // Expression handles, never trees.
    [[nodiscard]] std::expected<IkEquationSystem, IkEquationBuilderError>
    buildEquationSystem(const IkTarget& target) const;

private:
    // Which transform the constraints are built on, or the first missing
    // prerequisite. Reads four members, so it belongs to the class rather than
    // being a free function taking all four.
    [[nodiscard]] std::expected<const SymbolicTransform*, IkEquationBuilderError>
    transformForConstraints() const;

    UrdfModelLoader urdfLoader_;
    KinematicChainBuilder chainBuilder_;
    JointTransformBuilder jointTransformBuilder_;
    ForwardKinematicsBuilder fkBuilder_;
    ConstraintBuilder constraintBuilder_;

    std::optional<RobotDescription> robotDescription_;
    std::optional<KinematicChain> kinematicChain_;
    std::optional<SymbolicTransform> forwardKinematics_;
    std::optional<FixedRigidTransform> tcp_;
    std::optional<SymbolicTransform> tcpForwardKinematics_;
};

} // namespace kinemaforge::ik
