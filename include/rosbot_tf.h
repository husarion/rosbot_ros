#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class ROSbotTFbroadcaster : public rclcpp::Node
{
public:
    ROSbotTFbroadcaster();
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::unique_ptr<std::string> pose_topic;
    std::unique_ptr<geometry_msgs::msg::TransformStamped> current_transform;
};
