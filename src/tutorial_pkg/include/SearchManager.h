#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <grid_map_core/grid_map_core.hpp>
#include <frontier_exploration/ExploreTaskAction.h>

class SearchManager
{
  public:
    SearchManager();
    bool check_space_occupation(
        grid_map::Position *robot_dest,
        double bearing,
        float dist_from_obstacle,
        float min_dist,
        grid_map::Position current_obstacle,
        grid_map::GridMap *obstacles,
        grid_map::GridMap *map);

    bool check_obstacle_surrounding(
        grid_map::Position *robot_dest,
        double *obstacle_bearing,
        float dist_from_obstacle,
        float min_dist,
        grid_map::Position current_obstacle,
        grid_map::GridMap *obstacles,
        grid_map::GridMap *map);

    bool lookup_camera_transform(
        double *cam_x,
        double *cam_y,
        double *cam_yaw,
        ros::Time scan_time,
        tf::TransformListener *listener);

    /**
     * brief Mark given point as checked
     * param x X position of point to be marked
     * param y Y position of point to be marked
     * param gm GridMap at which point is to be marked
     * param obstacle Obstacle which is currently set as destination
     * return true if obstacle was inside the checked area
     */
    bool set_point_checked(float x, float y, grid_map::GridMap *gm, grid_map::Position obstacle);

    /**
     * brief Mark given point as checked
     * param x X position of point to be marked
     * param y Y position of point to be marked
     * param gm GridMap at which point is to be marked
     * param obstacle Obstacle which is currently set as destination
     * param radius Radius of the circle to be marked
     * return true if obstacle was inside the checked area
     */
    bool set_point_checked(float x, float y, grid_map::GridMap *gm, grid_map::Position obstacle, float radius);

    frontier_exploration::ExploreTaskGoal createExplorationGoal();

    bool is_goal_reached(geometry_msgs::PoseStamped goal, tf::StampedTransform current_tf, double linear_threshold, double angular_threshold);
};