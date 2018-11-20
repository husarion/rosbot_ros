#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Char.h>
#include <std_srvs/Empty.h>

ros::Publisher action_pub;
geometry_msgs::Twist set_vel;

float distL = 0;
float distR = 0;
float sensorL_max = 0;
float sensorR_max = 0;

u_char search_obj;
int objectID;
int homeID;

ros::ServiceClient client;

void distL_callback(const sensor_msgs::Range &range)
{
    distL = range.range;
    sensorL_max = range.max_range;
}

void distR_callback(const sensor_msgs::Range &range)
{
    distR = range.range;
    sensorR_max = range.max_range;
}

void task_callback(const std_msgs::Char &task)
{
    search_obj = task.data;
}

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
    if (object->data.size() > 0)
    {
        if (search_obj == object->data[0])
        {
            ROS_INFO("Object found, call service %s", client.getService().c_str());
            std_srvs::Empty srv;
            client.call(srv);
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "action_controller");
    ros::NodeHandle n("~");
    ros::Subscriber sub = n.subscribe("objects", 1, objectCallback);
    ros::Subscriber distL_sub = n.subscribe("range/fl", 1, distL_callback);
    ros::Subscriber distR_sub = n.subscribe("range/fr", 1, distR_callback);
    ros::Subscriber task_sub = n.subscribe("/task", 1, task_callback);

    n.param<int>("objectID", objectID, 0);
    n.param<int>("homeID", homeID, 0);
    client = n.serviceClient<std_srvs::Empty>("/object_found");

    ros::Rate loop_rate(10);
    action_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    set_vel.linear.x = 0;
    set_vel.linear.y = 0;
    set_vel.linear.z = 0;
    set_vel.angular.x = 0;
    set_vel.angular.y = 0;
    set_vel.angular.z = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        if (search_obj == objectID || search_obj == homeID)
        {
            if (distL > 1)
            {
                distL = 1;
            }
            if (distR > 1)
            {
                distR = 1;
            }
            if (distL > 0 && distR > 0)
            {
                set_vel.angular.z = (distL - distR) * 10;
                set_vel.linear.x = ((distL + distR) / 2) - ((sensorL_max + sensorR_max) / 4);
            }
            else if (distL > 0)
            {
                set_vel.angular.z = -1;
                set_vel.linear.x = -0.1;
            }
            else if (distR > 0)
            {
                set_vel.angular.z = 1;
                set_vel.linear.x = -0.1;
            }
            else
            {
                set_vel.linear.x = 0.2;
                set_vel.angular.z = 0;
            }
        }
        else
        {
            set_vel.linear.x = 0;
            set_vel.angular.z = 0;
        }
        action_pub.publish(set_vel);
    }
}