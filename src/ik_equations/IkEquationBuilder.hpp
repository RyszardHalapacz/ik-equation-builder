#pragma once

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/model/RobotDescription.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <filesystem>
#include <string>

namespace kinemaforge::ik {

class IkEquationBuilder
{
public:
    IkEquationBuilder();

    void loadRobotModel(const std::filesystem::path& urdfPath);

    void selectChain(
        const std::string& baseLink,
        const std::string& toolLink
    );

    void buildForwardKinematics();

    const KinematicChain& kinematicChain() const;
    const SymbolicTransform& forwardKinematics() const;

private:
    UrdfModelLoader urdfLoader_;
    KinematicChainBuilder chainBuilder_;
    JointTransformBuilder jointTransformBuilder_;
    ForwardKinematicsBuilder fkBuilder_;

    RobotDescription robotDescription_;
    KinematicChain kinematicChain_;
    SymbolicTransform forwardKinematics_;
};

} // namespace kinemaforge::ik
