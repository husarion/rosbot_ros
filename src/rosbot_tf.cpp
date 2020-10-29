#include "rosbot_tf.h"

ROSbotTFbroadcaster::ROSbotTFbroadcaster() : Node("rosbot_tf")
{
    pose_topic = std::make_unique<std::string>("/pose");
    current_transform = std::make_unique<geometry_msgs::msg::TransformStamped>();
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(*pose_topic, rclcpp::SensorDataQoS(), std::bind(&ROSbotTFbroadcaster::pose_callback, this, std::placeholders::_1));
}

void ROSbotTFbroadcaster::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
    current_transform->header.frame_id = "odom";
    current_transform->header = pose->header;
    current_transform->child_frame_id = "base_link";
    current_transform->transform.translation.x = pose->pose.position.x;
    current_transform->transform.translation.y = pose->pose.position.y;
    current_transform->transform.translation.z = pose->pose.position.z;
    current_transform->transform.rotation.x = pose->pose.orientation.x;
    current_transform->transform.rotation.y = pose->pose.orientation.y;
    current_transform->transform.rotation.z = pose->pose.orientation.z;
    current_transform->transform.rotation.w = pose->pose.orientation.w;
    tf_broadcaster->sendTransform(*current_transform);
}
