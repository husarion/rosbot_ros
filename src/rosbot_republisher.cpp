#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ROSbotRepublisher : public rclcpp::Node
{
public:
  ROSbotRepublisher() : Node("rosbot_republisher")
  {
    base_link_tf_stamped.header.frame_id = "odom";
    base_link_tf_stamped.child_frame_id = "base_link";
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "odom", 1, std::bind(&ROSbotRepublisher::odom_callback, this, _1));
  }

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped base_link_tf_stamped;
  std::string global_frame_id_ = "null";
  std::string odom_frame_id_ = "null";

  void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    base_link_tf_stamped.header.stamp = now();
    base_link_tf_stamped.transform.translation.x = msg->pose.position.x;
    base_link_tf_stamped.transform.translation.y = msg->pose.position.y;
    base_link_tf_stamped.transform.translation.z = msg->pose.position.z;
    base_link_tf_stamped.transform.rotation.x = msg->pose.orientation.x;
    base_link_tf_stamped.transform.rotation.y = msg->pose.orientation.y;
    base_link_tf_stamped.transform.rotation.z = msg->pose.orientation.z;
    base_link_tf_stamped.transform.rotation.w = msg->pose.orientation.w;
    tf_broadcaster_->sendTransform(base_link_tf_stamped);
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSbotRepublisher>());
  rclcpp::shutdown();
  return 0;
}