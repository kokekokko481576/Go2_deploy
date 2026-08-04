#include <functional>
#include <string>
#include <memory>

#include "go2_bt_plugins/is_collision_detected_condition.hpp"

namespace go2_bt_plugins
{

IsCollisionDetectedCondition::IsCollisionDetectedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_collision_detected_(false)
{
  getInput("topic_name", topic_name_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  collision_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_name_,
    rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&IsCollisionDetectedCondition::collisionCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsCollisionDetectedCondition::tick()
{
  callback_group_executor_.spin_some();
  return is_collision_detected_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void IsCollisionDetectedCondition::collisionCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  is_collision_detected_ = msg->data;
}

}  // namespace go2_bt_plugins

#define BT_PLUGIN_EXPORT
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<go2_bt_plugins::IsCollisionDetectedCondition>("IsCollisionDetected");
}
