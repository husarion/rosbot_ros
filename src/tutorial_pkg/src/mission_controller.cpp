#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_srvs/Empty.h>

ros::Publisher task_pub;

#define OBJECT_1_ID 8
#define OBJECT_2_ID 6
#define HOME_1_ID 12
#define HOME_2_ID 11

std::vector<int> objects;

uint8_t current_object = 0;
std_msgs::Char task;

bool object_found(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    current_object++;
    ROS_INFO("Current object: %d", current_object);
    return true;
}

int main(int argc, char **argv)
{
    objects.push_back(OBJECT_1_ID);
    objects.push_back(OBJECT_2_ID);
    objects.push_back(HOME_1_ID);
    objects.push_back(HOME_2_ID);
    ros::init(argc, argv, "mission_controller");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);
    ros::ServiceServer service = n.advertiseService("/object_found", object_found);
    task_pub = n.advertise<std_msgs::Char>("/task", 1);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        if (current_object < objects.size())
        {
            task.data = objects[current_object];
        }
        else
        {
            // mission finished
            task.data = 0;
        }
        task_pub.publish(task);
    }
}