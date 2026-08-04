#ifndef GO2_BT_PLUGINS__IS_COLLISION_DETECTED_CONDITION_HPP_
#define GO2_BT_PLUGINS__IS_COLLISION_DETECTED_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace go2_bt_plugins
{

/**
 * @brief A BT::ConditionNode that listens to collision_detector(#23)の
 * collision_detectedトピックを見て、接触中はSUCCESS・それ以外はFAILUREを返す。
 * nav2_behavior_tree::IsBatteryLowConditionと同じ構成(トピックsubscribe→BT条件)。
 */
class IsCollisionDetectedCondition : public BT::ConditionNode
{
public:
  IsCollisionDetectedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsCollisionDetectedCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic_name", std::string("collision_detected"), "接触検知トピック"),
    };
  }

private:
  void collisionCallback(std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  std::string topic_name_;
  bool is_collision_detected_;
};

}  // namespace go2_bt_plugins

#endif  // GO2_BT_PLUGINS__IS_COLLISION_DETECTED_CONDITION_HPP_
