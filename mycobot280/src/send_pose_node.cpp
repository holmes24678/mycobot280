#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

class SendPoseNode
{
public:
  SendPoseNode(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    node_->declare_parameter<std::string>("planning_group", "arm");
    std::string planning_group = node_->get_parameter("planning_group").as_string();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group);

    // Set correct end-effector pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.777;
    target_pose.position.y = -0.426;
    target_pose.position.z = 0.743;

    target_pose.orientation.x = -0.172;
    target_pose.orientation.y = -0.380;
    target_pose.orientation.z = 0.767;
    target_pose.orientation.w = -0.488;

    move_group_->setPoseTarget(target_pose);

    // Optional: Increase planning time and reattempts
    move_group_->setPlanningTime(10.0);
    move_group_->allowReplanning(true);
    move_group_->setGoalTolerance(0.01);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(node_->get_logger(), "Planning successful. Executing...");
      move_group_->execute(plan);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed.");
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("send_pose_node");
  SendPoseNode spn(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
