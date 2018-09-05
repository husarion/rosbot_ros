#include <math.h>
#include <cmath>

#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <frontier_exploration/ExploreTaskAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <SearchManager.h>

SearchManager *sm;
grid_map::GridMap *obstacles;
grid_map::GridMap *map;
std::vector<std::string> layers_to_import;
nav_msgs::OccupancyGrid *occupancyGridInput;
nav_msgs::OccupancyGrid occupancyGridResult;
nav_msgs::OccupancyGrid occupancyGridScanned;
nav_msgs::OccupancyGrid occupancyGridObstaclesFound;
nav_msgs::OccupancyGrid occupancyGridPendingObstacles;
geometry_msgs::PoseStamped exploration_goal;
frontier_exploration::ExploreTaskAction exploreTaskAction;

ros::Publisher publisher_obstacles_found;
ros::Publisher publisher_pending_obstacles;
ros::Publisher publisher_checked_obstacles;
ros::Publisher publisher_exploration_goal;
ros::Publisher vis_pub;
ros::Publisher goal_pub;
ros::Publisher explore_canceller;

double angle_min;
double angle_max;       //        # end angle of the scan [rad]
double angle_increment; //  # angular distance between measurements [rad]
double range_min;       //        # minimum range value [m]
double range_max;       //        # maximum range value [m]
std::uint16_t ranges_size;
double point_x;
double point_y;
tf::Quaternion tf_q, tf_dest_quaternion;
tf::TransformListener *listener;
tf::StampedTransform ROSbot2_base_to_map_transform;
geometry_msgs::Pose circle_element;
grid_map::Position robot_position;
grid_map::Position nearest_obstacle;
grid_map::Position robot_destination;
grid_map::Position current_obstacle;
tf::Matrix3x3 m, dest_orientation;
double x_dest_pos, y_dest_pos;
bool goal_accessible;
bool goal_reached;
bool destination_free_to_go;
bool exploration_in_progress;
bool exploration_failed;
bool object_search_in_progress;
bool object_found;
double camera_view_dist = 0.7;  // [meters]
double camera_view_depth = 0.2; // [meters]
std::vector<geometry_msgs::PoseStamped> poses(36);

double obstacle_bearing;
float min_dist = 0.15;
grid_map::Position current_robot_position;

void set_new_goal();
bool find_nearest_obstacle(grid_map::Position *new_pos);
bool find_nearest_obstacle(grid_map::Position *new_pos, grid_map::Position current_pos);
grid_map::Position get_optimal_pose(grid_map::Position obstacle);

void cancel_move_base_action();
void cancel_exploration_action();
