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
    return std::unexpected(
        IkEquationBuilderError{code, std::move(message), std::nullopt});
}

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
            selected.error()});

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
