#include "legacy/kinematics/robot_model.hpp"

#include <utility>  // std::move

namespace mt::kinematics {

std::string_view RobotModel::name() const noexcept                 { return name_; }
std::span<Link  const>        RobotModel::links()  const noexcept  { return links_; }
std::span<Joint const>        RobotModel::joints() const noexcept  { return joints_; }

void RobotModel::set_name(std::string n)                { name_ = std::move(n); }
void RobotModel::add_link(Link l)                       { links_.push_back(std::move(l)); }
void RobotModel::add_joint(Joint j)                     { joints_.push_back(std::move(j)); }

}  // namespace mt::kinematics
