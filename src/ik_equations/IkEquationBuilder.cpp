#include "ik_equations/IkEquationBuilder.hpp"

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

} // namespace kinemaforge::ik
