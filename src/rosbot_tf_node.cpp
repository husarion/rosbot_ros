#include "rosbot_tf.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSbotTFbroadcaster>());
  rclcpp::shutdown();
  return 0;
}
