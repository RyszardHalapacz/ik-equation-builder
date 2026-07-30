#include "kinematics/robot_model.hpp"

namespace mt::kinematics {

std::string_view RobotModel::name() const noexcept                 { return name_; }
std::span<Link  const>        RobotModel::links()  const noexcept  { return links_; }
std::span<Joint const>        RobotModel::joints() const noexcept  { return joints_; }
std::span<std::size_t const>  RobotModel::chain()  const noexcept  { return chain_; }

void RobotModel::set_name(std::string n)                { name_ = std::move(n); }
void RobotModel::add_link(Link l)                       { links_.push_back(std::move(l)); }
void RobotModel::add_joint(Joint j)                     { joints_.push_back(std::move(j)); }
void RobotModel::set_chain(std::vector<std::size_t> i)  { chain_ = std::move(i); }

bool RobotModel::within_limits(std::span<double const> joint_values) const noexcept {
    if (joint_values.size() != chain_.size()) return false;
    for (std::size_t k = 0; k < chain_.size(); ++k) {
        auto const& j = joints_[chain_[k]];
        auto const  v = joint_values[k];
        if (v < j.limits.lower || v > j.limits.upper) return false;
    }
    return true;
}

}  // namespace mt::kinematics
