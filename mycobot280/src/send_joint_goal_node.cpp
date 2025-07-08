#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <map>

class SendJointGoalNode
{
public:
  SendJointGoalNode(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    node_->declare_parameter<std::string>("planning_group", "arm");
    std::string planning_group = node_->get_parameter("planning_group").as_string();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group);

    std::map<std::string, double> target_joint_values;
    target_joint_values["J1"] = 0.0;
    target_joint_values["J2"] = 1.2144;
    target_joint_values["J3"] = 0.5898;
    target_joint_values["J4"] = 1.0409;
    target_joint_values["J5"] = 0.347;
    target_joint_values["J6"] = 2.9492;

    move_group_->setJointValueTarget(target_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(node_->get_logger(), "Planning successful. Executing...");
      move_group_->execute(plan);

      // ✨ Wait briefly to ensure robot has moved
      rclcpp::sleep_for(std::chrono::seconds(1));

    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Joint space planning failed.");
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("send_joint_goal_node");
  SendJointGoalNode spn(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
