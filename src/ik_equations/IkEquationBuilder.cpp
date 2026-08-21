#include "ik_equations/IkEquationBuilder.hpp"

#include "ik_equations/builders/RigidTransformConstruction.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace kinemaforge::ik {

namespace {

std::unexpected<IkEquationBuilderError> makeError(IkEquationBuilderErrorCode code,
                                                  std::string message)
{
    // Four initialisers, not three: -Wextra turns a missing one into
    // -Wmissing-field-initializers, and -Werror turns that into a failed build.
    // That is the warning policy doing its job -- every site that constructs an
    // error has to answer for a new field instead of receiving a default in
    // silence, and the answer here is the invariant from the header.
    return std::unexpected(
        IkEquationBuilderError{code, std::move(message), std::nullopt, std::nullopt});
}

// The nested error is preserved, not flattened: the caller gets
// ConstraintBuildFailed -> InvalidTarget -> NonFinitePosition, three layers
// deep, and none of them lost. The same treatment chainError has had since the
// facade first reported a chain failure.
std::unexpected<IkEquationBuilderError> constraintBuildFailed(ConstraintBuilderError error)
{
    auto message = "constraint build failed: " + error.message;
    return std::unexpected(IkEquationBuilderError{
        IkEquationBuilderErrorCode::ConstraintBuildFailed, std::move(message),
        std::nullopt, std::move(error)});
}

// Explicit overload set, no generic lambda: a seventh IkTarget alternative must
// stop the build here until somebody decides how the facade builds it. The same
// idiom as TargetVisitor, EquationKindVisitor and toolAxisOf.
struct ConstraintDispatch
{
    const ConstraintBuilder& builder;
    const SymbolicTransform& transform;
    const KinematicChain& chain;

    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    operator()(const PositionTarget& target) const
    {
        return builder.build(transform, chain, target);
    }

    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    operator()(const PoseTarget& target) const
    {
        return builder.build(transform, chain, target);
    }

    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    operator()(const ToolDirectionTarget& target) const
    {
        return builder.build(transform, chain, target);
    }

    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    operator()(const PositionAndToolDirectionTarget& target) const
    {
        return builder.build(transform, chain, target);
    }

    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    operator()(const DirectionAngleTarget& target) const
    {
        return builder.build(transform, chain, target);
    }

    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    operator()(const PositionAndDirectionAngleTarget& target) const
    {
        return builder.build(transform, chain, target);
    }
};

const char* describe(KinematicChainError error) noexcept
{
    switch (error)
    {
    case KinematicChainError::BaseLinkNotFound:        return "base link not found";
    case KinematicChainError::ToolLinkNotFound:        return "tool link not found";
    case KinematicChainError::NoPathFound:             return "no path from base to tool";
    case KinematicChainError::InvalidRobotDescription: return "invalid robot description";
    }
    return "unknown kinematic chain error";
}

// Names the first offending component so the message is actionable; the
// structured code alone cannot say which of the six numbers was wrong.
const char* firstNonFiniteComponent(const FixedRigidTransform& transform) noexcept
{
    if (!std::isfinite(transform.translation.x)) return "translation x";
    if (!std::isfinite(transform.translation.y)) return "translation y";
    if (!std::isfinite(transform.translation.z)) return "translation z";
    if (!std::isfinite(transform.rpy.x))         return "rpy roll";
    if (!std::isfinite(transform.rpy.y))         return "rpy pitch";
    if (!std::isfinite(transform.rpy.z))         return "rpy yaw";
    return nullptr;
}

} // namespace

IkEquationBuilder::IkEquationBuilder() = default;

std::expected<void, IkEquationBuilderError>
IkEquationBuilder::loadRobotModel(const std::filesystem::path& urdfPath)
{
    RobotDescription loaded;
    try
    {
        loaded = urdfLoader_.load(urdfPath);
    }
    catch (const std::runtime_error& error)
    {
        // Exactly std::runtime_error, which is what UrdfModelLoader throws.
        // Catching std::exception would also swallow std::bad_alloc and report
        // it as a URDF problem -- a lie the caller cannot see through. Anything
        // else propagates.
        return makeError(IkEquationBuilderErrorCode::UrdfLoadFailed, error.what());
    }

    // Committed only past the throw: on failure the previous model, chain and
    // transform all survive untouched. Clearing first would leave a failed load
    // worse off than no call at all.
    robotDescription_ = std::move(loaded);
    kinematicChain_.reset();
    forwardKinematics_.reset();
    tcp_.reset();
    tcpForwardKinematics_.reset();
    return {};
}

std::expected<void, IkEquationBuilderError>
IkEquationBuilder::selectChain(const std::string& baseLink, const std::string& toolLink)
{
    if (!robotDescription_)
        return makeError(IkEquationBuilderErrorCode::RobotModelNotLoaded,
                         "no robot model loaded; call loadRobotModel first");

    auto selected = chainBuilder_.build(*robotDescription_, baseLink, toolLink);
    if (!selected)
        return std::unexpected(IkEquationBuilderError{
            IkEquationBuilderErrorCode::ChainBuildFailed,
            "cannot select chain '" + baseLink + "' -> '" + toolLink + "': " +
                describe(selected.error()),
            selected.error(),
            std::nullopt});

    kinematicChain_ = std::move(*selected);
    forwardKinematics_.reset();
    // A TCP offset is measured from one specific chain tip: the same three
    // numbers would name a different physical point after the tip changes.
    tcp_.reset();
    tcpForwardKinematics_.reset();
    return {};
}

std::expected<void, IkEquationBuilderError>
IkEquationBuilder::buildForwardKinematics()
{
    if (!kinematicChain_)
        return makeError(IkEquationBuilderErrorCode::KinematicChainNotSelected,
                         "no kinematic chain selected; call selectChain first");

    // Cannot fail: an empty chain is a valid input yielding the identity.
    forwardKinematics_ = fkBuilder_.build(*kinematicChain_, jointTransformBuilder_);
    // Rebuilding creates fresh nodes, so a retained T_base_tcp would point at
    // the previous tree. The TCP itself is a sibling, not a descendant, and
    // survives.
    tcpForwardKinematics_.reset();
    return {};
}

std::expected<void, IkEquationBuilderError>
IkEquationBuilder::setTcp(const FixedRigidTransform& tcp)
{
    if (!kinematicChain_)
        return makeError(IkEquationBuilderErrorCode::KinematicChainNotSelected,
                         "no kinematic chain selected; a TCP offset is defined "
                         "relative to the chain tip");

    if (const char* offending = firstNonFiniteComponent(tcp))
        return makeError(IkEquationBuilderErrorCode::InvalidTcpTransform,
                         std::string("tcp ") + offending + " is not finite");

    // Validated before the commit: a rejected update leaves the previous TCP
    // and the previous T_base_tcp untouched.
    tcp_ = tcp;
    tcpForwardKinematics_.reset();
    return {};
}

void IkEquationBuilder::clearTcp() noexcept
{
    tcp_.reset();
    tcpForwardKinematics_.reset();
}

std::expected<void, IkEquationBuilderError>
IkEquationBuilder::buildTcpForwardKinematics()
{
    if (!kinematicChain_)
        return makeError(IkEquationBuilderErrorCode::KinematicChainNotSelected,
                         "no kinematic chain selected; call selectChain first");

    if (!forwardKinematics_)
        return makeError(IkEquationBuilderErrorCode::ForwardKinematicsNotBuilt,
                         "forward kinematics not built; call buildForwardKinematics first");

    if (!tcp_)
        return makeError(IkEquationBuilderErrorCode::TcpNotSet,
                         "no TCP set; call setTcp first");

    // Local and stateless. ExpressionFactory has no members, so a facade field
    // would share nothing while looking as though it did.
    const ExpressionFactory factory;

    const SymbolicTransform fixed = buildFixedRigidTransform(*tcp_, factory);
    tcpForwardKinematics_ = multiplyTransforms(*forwardKinematics_, fixed, factory);
    return {};
}

std::expected<const SymbolicTransform*, IkEquationBuilderError>
IkEquationBuilder::transformForConstraints() const
{
    if (!kinematicChain_)
        return makeError(IkEquationBuilderErrorCode::KinematicChainNotSelected,
                         "no kinematic chain selected; call selectChain first");

    if (!forwardKinematics_)
        return makeError(IkEquationBuilderErrorCode::ForwardKinematicsNotBuilt,
                         "forward kinematics not built; call buildForwardKinematics first");

    // Having no TCP is not a missing step: it is the other legal mode, and the
    // constraints then apply to the chain tip.
    if (!tcp_)
        return &*forwardKinematics_;

    // A TCP that is set but not built IS a missing step, and there is
    // deliberately no fallback branch beside this one. Returning the chain-tip
    // transform here would produce a system wrong by exactly the tool offset.
    if (!tcpForwardKinematics_)
        return makeError(IkEquationBuilderErrorCode::TcpForwardKinematicsNotBuilt,
                         "a TCP is set but its transform is not built; call "
                         "buildTcpForwardKinematics first");

    return &*tcpForwardKinematics_;
}

std::expected<IkEquationSystem, IkEquationBuilderError>
IkEquationBuilder::buildEquationSystem(const IkTarget& target) const
{
    const auto transform = transformForConstraints();
    if (!transform)
        return std::unexpected(transform.error());

    // *kinematicChain_ is safe: transformForConstraints checked the chain first.
    auto system = std::visit(
        ConstraintDispatch{constraintBuilder_, **transform, *kinematicChain_}, target);

    if (!system)
        return constraintBuildFailed(std::move(system.error()));

    return *std::move(system);
}

const KinematicChain* IkEquationBuilder::kinematicChain() const noexcept
{
    return kinematicChain_ ? &*kinematicChain_ : nullptr;
}

const SymbolicTransform* IkEquationBuilder::forwardKinematics() const noexcept
{
    return forwardKinematics_ ? &*forwardKinematics_ : nullptr;
}

const FixedRigidTransform* IkEquationBuilder::tcp() const noexcept
{
    return tcp_ ? &*tcp_ : nullptr;
}

const SymbolicTransform* IkEquationBuilder::tcpForwardKinematics() const noexcept
{
    return tcpForwardKinematics_ ? &*tcpForwardKinematics_ : nullptr;
}

} // namespace kinemaforge::ik
