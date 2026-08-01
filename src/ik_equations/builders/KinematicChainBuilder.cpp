#include "ik_equations/builders/KinematicChainBuilder.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace kinemaforge::ik {

namespace {

bool isActuated(JointType type)
{
    switch (type)
    {
    case JointType::Revolute:
    case JointType::Prismatic:
    case JointType::Continuous:
        return true;
    case JointType::Fixed:
        return false;
    }
    return false;
}

enum class PathSearchResult
{
    Found,
    NotFound,
    CycleDetected
};

// Iterative DFS (explicit stack, no recursion) over parent -> child edges.
// The URDF is a tree, so at most one path to toolLink can exist.
//
// Each Frame remembers the joint that led into it, so the path is not kept
// in sync with the stack on every push/pop — it's read off the stack once,
// at the single point where toolLink is found. `link` is a view: every name
// it can point to (baseLink/toolLink, or a joint's parentLink/childLink)
// outlives this whole call, so nothing is copied while walking the graph.
PathSearchResult findPath(
    std::string_view baseLink,
    std::string_view toolLink,
    const std::unordered_map<std::string_view, std::vector<std::size_t>>& childrenOf,
    const RobotDescription& robot,
    std::vector<std::size_t>& pathJointIndices)
{
    struct Frame
    {
        std::string_view link;
        std::size_t nextChildIndex = 0;
        std::optional<std::size_t> incomingJointIndex; // nullopt only for the root frame
    };

    std::unordered_set<std::string_view> visited{baseLink};
    std::vector<Frame> stack;
    stack.push_back(Frame{baseLink});

    while (!stack.empty())
    {
        Frame& frame = stack.back();
        auto it = childrenOf.find(frame.link);
        const std::size_t childCount = (it != childrenOf.end()) ? it->second.size() : 0;

        if (frame.nextChildIndex == childCount)
        {
            // No more children to try from here — backtrack.
            visited.erase(frame.link);
            stack.pop_back();
            continue;
        }

        const std::size_t jointIndex = it->second[frame.nextChildIndex];
        ++frame.nextChildIndex;
        const std::string_view child = robot.joints[jointIndex].childLink;

        if (visited.contains(child))
            return PathSearchResult::CycleDetected;

        if (child == toolLink)
        {
            // Root has no incoming joint (nullopt); every other frame does.
            for (std::size_t i = 1; i < stack.size(); ++i)
                pathJointIndices.push_back(*stack[i].incomingJointIndex);
            pathJointIndices.push_back(jointIndex);
            return PathSearchResult::Found;
        }

        visited.insert(child);
        stack.push_back(Frame{child, 0, jointIndex});
    }

    return PathSearchResult::NotFound;
}

KinematicJoint copyJointData(const UrdfJoint& joint, std::size_t index)
{
    KinematicJoint kinematicJoint;
    kinematicJoint.index = index;
    kinematicJoint.name = joint.name;
    kinematicJoint.parentLink = joint.parentLink;
    kinematicJoint.childLink = joint.childLink;
    kinematicJoint.type = joint.type;
    kinematicJoint.origin = joint.origin;
    kinematicJoint.axis = joint.axis;
    kinematicJoint.limits = joint.limits;
    return kinematicJoint;
}

} // namespace

std::expected<KinematicChain, KinematicChainError> KinematicChainBuilder::build(
    const RobotDescription& robot,
    const std::string& baseLink,
    const std::string& toolLink) const
{
    // robot_model_loader.cpp doesn't cross-check <link> declarations against
    // joint parent/child references, so RobotDescription.links alone isn't
    // a reliable "does this link exist" source — union with joint references.
    std::unordered_set<std::string> knownLinks;
    for (auto const& link : robot.links)
        knownLinks.insert(link.name);
    for (auto const& joint : robot.joints)
    {
        knownLinks.insert(joint.parentLink);
        knownLinks.insert(joint.childLink);
    }

    if (!knownLinks.contains(baseLink))
        return std::unexpected(KinematicChainError::BaseLinkNotFound);
    if (!knownLinks.contains(toolLink))
        return std::unexpected(KinematicChainError::ToolLinkNotFound);

    if (baseLink == toolLink)
        return KinematicChain{baseLink, toolLink, {}};

    // Keys are views into robot.joints[i].parentLink/childLink, valid for
    // the lifetime of this call (robot is not mutated here).
    std::unordered_map<std::string_view, std::vector<std::size_t>> childrenOf;
    std::unordered_set<std::string_view> childLinksWithParent;
    for (std::size_t i = 0; i < robot.joints.size(); ++i)
    {
        const auto& joint = robot.joints[i];
        // A link has at most one parent joint in a valid URDF tree.
        if (!childLinksWithParent.insert(joint.childLink).second)
            return std::unexpected(KinematicChainError::InvalidRobotDescription);

        childrenOf[joint.parentLink].push_back(i);
    }

    std::vector<std::size_t> pathJointIndices;
    const auto searchResult = findPath(baseLink, toolLink, childrenOf, robot, pathJointIndices);

    if (searchResult == PathSearchResult::CycleDetected)
        return std::unexpected(KinematicChainError::InvalidRobotDescription);
    if (searchResult == PathSearchResult::NotFound)
        return std::unexpected(KinematicChainError::NoPathFound);

    KinematicChain chain;
    chain.baseLink = baseLink;
    chain.toolLink = toolLink;

    std::size_t variableCount = 0;
    for (std::size_t position = 0; position < pathJointIndices.size(); ++position)
    {
        auto kinematicJoint = copyJointData(robot.joints[pathJointIndices[position]], position);
        if (isActuated(kinematicJoint.type))
        {
            ++variableCount;
            kinematicJoint.variable = JointVariable{"q" + std::to_string(variableCount), variableCount};
        }
        chain.joints.push_back(std::move(kinematicJoint));
    }

    return chain;
}

} // namespace kinemaforge::ik
