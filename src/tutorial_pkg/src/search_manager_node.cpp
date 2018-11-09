#include <search_manager_node.h>

/**
 * brief Computes the bearing in degrees from the point A(a1,a2) to the point B(b1,b2).
 * param a1 x coordiante of point A
 * param a2 y coordinate of point A
 * param b1 x coordiante of point B
 * param b2 y coordinate of point B
 */
double bearing(double a1, double a2, double b1, double b2)
{
    static const double TWOPI = 6.2831853071795865;
    static const double RAD2DEG = 57.2957795130823209;
    double theta = atan2(b1 - a1, b2 - a2);
    if (theta < 0.0)
        theta += TWOPI;
    return theta;
}

bool is_area_free(grid_map::Position point, float radius)
{
    bool area_free = true;
    for (grid_map::CircleIterator it(*obstacles, point, radius); !it.isPastEnd(); ++it)
    {
        if (obstacles->at("obstacles_found", *it) == 1)
        {
            area_free = false;
        }
    }
    return area_free;
}

void clear_area(grid_map::Position point, float radius)
{
    for (grid_map::CircleIterator it(*obstacles, point, radius); !it.isPastEnd(); ++it)
    {
        obstacles->at("pending_obstacles", *it) = 0;
    }
}

bool is_point_single(grid_map::Position point)
{
    float res = obstacles->getResolution();
    bool single = true;
    grid_map::CircleIterator point_it(*obstacles, point, 0.01);
    grid_map::Position point_it_position;
    obstacles->getPosition(*point_it, point_it_position);

    grid_map::Position circle_it_position;

    for (grid_map::CircleIterator it(*obstacles, point, 10 * res); !it.isPastEnd(); ++it)
    {
        obstacles->getPosition(*it, circle_it_position);
        if (obstacles->at("pending_obstacles", *it) == 1)
        {
            float x_dist = point_it_position.x() - circle_it_position.x();
            float y_dist = point_it_position.y() - circle_it_position.y();
            float x_abs = std::abs(x_dist);
            float y_abs = std::abs(y_dist);
            if (x_abs > res && y_abs > res)
            {
                single = false;
            }
        }
    }
    return single;
}

bool find_nearest_obstacle(grid_map::Position *new_pos)
{
    grid_map::Position current_robot_position(robot_position[0], robot_position[1]);
    return find_nearest_obstacle(new_pos, current_robot_position);
}

bool find_nearest_obstacle(grid_map::Position *new_pos, grid_map::Position current_pos)
{
    grid_map::Position new_obstacle;
    bool new_iteration = true;
    bool obstacle_found = false;
    double distance_to_nearest_obstacle;
    double distance_to_current_obstacle;
    double x_dist, y_dist;
    grid_map::Position element_position;

    for (grid_map::GridMapIterator it(*obstacles); !it.isPastEnd(); ++it)
    {
        if (obstacles->at("pending_obstacles", *it) > 0.5)
        {
            obstacle_found = true;
            obstacles->getPosition(*it, element_position);
            x_dist = current_pos[0] - element_position[0];
            y_dist = current_pos[1] - element_position[1];
            distance_to_current_obstacle = sqrt((x_dist * x_dist) + (y_dist * y_dist));
            if (new_iteration)
            {
                distance_to_nearest_obstacle = distance_to_current_obstacle;
                new_obstacle = element_position;
                new_iteration = false;
            }
            else if (distance_to_current_obstacle < distance_to_nearest_obstacle)
            {
                distance_to_nearest_obstacle = distance_to_current_obstacle;
                new_obstacle = element_position;
            }
        }
    }
    *new_pos = new_obstacle;
    return obstacle_found;
}

bool check_pending_obstacles()
{
    for (grid_map::GridMapIterator it(*obstacles); !it.isPastEnd(); ++it)
    {
        if (obstacles->at("pending_obstacles", *it) > 0.5)
        {
            return true;
        }
    }
    ROS_INFO("No pending obstacles found");
    return false;
}

bool was_obstacle_checked(grid_map::Position obstacle_position)
{
    float radius = 0.1;
    bool obstacle_already_checked = false;
    for (grid_map::CircleIterator it(*obstacles, obstacle_position, radius); !it.isPastEnd(); ++it)
    {
        if (obstacles->at("pending_obstacles", *it) > 0.1)
        {
            obstacle_already_checked = true;
        }
    }
    return obstacle_already_checked;
}

void publish_exploration_goal(grid_map::Position goal_pos, float goal_angle)
{
    dest_orientation.setRPY(0, 0, goal_angle);
    dest_orientation.getRotation(tf_dest_quaternion);
    exploration_goal.header.frame_id = "map";
    exploration_goal.pose.position.x = goal_pos[0];
    exploration_goal.pose.position.y = goal_pos[1];
    exploration_goal.pose.position.z = 0;
    exploration_goal.pose.orientation.x = tf_dest_quaternion.x();
    exploration_goal.pose.orientation.y = tf_dest_quaternion.y();
    exploration_goal.pose.orientation.z = tf_dest_quaternion.z();
    exploration_goal.pose.orientation.w = tf_dest_quaternion.w();
    publisher_exploration_goal.publish(exploration_goal);
}

void status_callback(const actionlib_msgs::GoalStatusArrayConstPtr &status)
{
    int size = status->status_list.size();
    if (size > 0)
    {
        int status_value = status->status_list[size - 1].status;
        if (status_value == 1)
        {
            goal_accessible = true;
        }
        else if (status_value == 4)
        {
            goal_accessible = false;
        }
    }
    else
    {
        goal_accessible = true;
    }
}

void explore_status_callback(const actionlib_msgs::GoalStatusArrayConstPtr &status)
{
    int size = status->status_list.size();
    if (size > 0)
    {
        int status_value = status->status_list[size - 1].status;
        if (status_value == 1) // ACTIVE
        {
            exploration_in_progress = true;
        }
        else if (status_value == 3) // SUCCEEDED
        {
            exploration_in_progress = false;
            exploration_failed = false;
        }
        else if (status_value == 4) // ABORTED
        {
            exploration_in_progress = false;
            exploration_failed = true;
            ROS_ERROR("Exploration failed, will not search for object.");
        }
    }
}

void scanCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    angle_increment = msg->angle_increment;
    range_min = msg->range_min;
    range_max = msg->range_max;
    ranges_size = msg->ranges.size();
    uint32_t s = msg->header.stamp.sec;
    uint32_t ns = msg->header.stamp.nsec;
    ROS_INFO("Scan time: %d,%d", s, ns);

    double camera_x;
    double camera_y;
    double camera_yaw;

    if (sm->lookup_camera_transform(&camera_x, &camera_y, &camera_yaw, ros::Time(s, ns), listener))
    {
        for (int i = 0; i < ranges_size; i++)
        {
            if (msg->ranges[i] > (camera_view_dist - camera_view_depth) && msg->ranges[i] < (camera_view_dist + camera_view_depth))
            {
                point_x = msg->ranges[i] * cos((angle_min + i * angle_increment) + camera_yaw);
                point_y = msg->ranges[i] * sin((angle_min + i * angle_increment) + camera_yaw);
                if (sm->set_point_checked(point_x + camera_x, point_y + camera_y, obstacles, nearest_obstacle))
                {
                    set_new_goal();
                }
            }
        }
        grid_map::GridMapRosConverter::toOccupancyGrid(*obstacles, "rgbd_scan", 0.0, 1.0, occupancyGridResult);
        publisher_checked_obstacles.publish(occupancyGridResult);
    }
    else
    {
        ROS_WARN("Transform lookup failed, drop this scan");
    }
}

void gridMapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    occupancyGridInput = new nav_msgs::OccupancyGrid();
    occupancyGridInput->header = msg->header;
    occupancyGridInput->info = msg->info;
    occupancyGridInput->data = msg->data;

    map = new grid_map::GridMap({"input_og"});
    grid_map::GridMapRosConverter::fromOccupancyGrid(*occupancyGridInput, "input_og", *map);
    map->setFrameId("map");
    obstacles->addDataFrom(*map, true, true, true);

    for (grid_map::GridMapIterator it(*obstacles); !it.isPastEnd(); ++it)
    {
        if (obstacles->at("input_og", *it) > 0)
        {
            obstacles->at("obstacles_found", *it) = 1;
            if (obstacles->at("rgbd_scan", *it) > 0)
            {
                obstacles->at("pending_obstacles", *it) = 0;
            }
            else
            {
                obstacles->at("pending_obstacles", *it) = 1;
            }
        }
        else
        {
            obstacles->at("obstacles_found", *it) = 0;
            obstacles->at("pending_obstacles", *it) = 0;
        }
    }

    grid_map::GridMapRosConverter::toOccupancyGrid(*obstacles, "obstacles_found", 0.0, 1.0, occupancyGridResult);
    publisher_obstacles_found.publish(occupancyGridResult);
    grid_map::GridMapRosConverter::toOccupancyGrid(*obstacles, "pending_obstacles", 0.0, 1.0, occupancyGridResult);
    publisher_pending_obstacles.publish(occupancyGridResult);
    if (is_area_free(nearest_obstacle, 0.1))
    {
        set_new_goal();
    }
    if (!sm->check_obstacle_surrounding(&current_robot_position, &obstacle_bearing, camera_view_dist, min_dist, current_obstacle, obstacles, map))
    {
        set_new_goal();
    }
}

void objects_found_callback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    if (msg->data.size() > 0)
    {
        ROS_INFO("Object detected");
        object_found = true;
    }
}

void update_robot_pos()
{
    try
    {
        listener->lookupTransform("/map", "/base_link", ros::Time(0), ROSbot2_base_to_map_transform);
        robot_position[0] = ROSbot2_base_to_map_transform.getOrigin().x();
        robot_position[1] = ROSbot2_base_to_map_transform.getOrigin().y();
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

void cancel_move_base_action()
{
    actionlib_msgs::GoalID move_base_goal;
    move_base_goal.id = "";
    move_base_goal.stamp = ros::Time::now();
    goal_pub.publish(move_base_goal);
}

void cancel_exploration_action()
{
    actionlib_msgs::GoalID cancel_exploration;
    cancel_exploration.id = "";
    cancel_exploration.stamp = ros::Time::now();
    explore_canceller.publish(cancel_exploration);
}

void start_frontier_exploration()
{
    exploration_in_progress = true;
    actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> exploreClient("explore_server", true);
    exploreClient.waitForServer();
    ROS_INFO("Sending goal");
    exploreClient.sendGoal(sm->createExplorationGoal());
}

void set_new_goal()
{
    if (object_search_in_progress)
    {
        ROS_INFO("Set new point for object recognition.");
        if (find_nearest_obstacle(&nearest_obstacle, nearest_obstacle))
        {
            if (is_point_single(nearest_obstacle))
            {
                ROS_INFO("Point %f, %f has no surrounding obstacles, consider it as noise.", nearest_obstacle[0], nearest_obstacle[1]);
                clear_area(nearest_obstacle, 0.1);
                find_nearest_obstacle(&nearest_obstacle, nearest_obstacle);
            }
            ROS_INFO("New dest point %f, %f.", nearest_obstacle[0], nearest_obstacle[1]);

            robot_destination = get_optimal_pose(nearest_obstacle);
            double angle = bearing(robot_destination[1], robot_destination[0], nearest_obstacle[1], nearest_obstacle[0]);
            publish_exploration_goal(robot_destination, angle);
        }
        else
        {
            if (!check_pending_obstacles())
            {
                ROS_INFO("Object search finished, object not found");
                object_search_in_progress = false;
            }
        }
    }
}

grid_map::Position get_optimal_pose(grid_map::Position obstacle)
{
    obstacle_bearing = 180 * bearing(obstacle[0], obstacle[1], robot_position[0], robot_position[1]) / M_PI;
    ROS_INFO("Obstacle bearing: %f", obstacle_bearing);
    current_obstacle = obstacle;
    if (!sm->check_obstacle_surrounding(&current_robot_position, &obstacle_bearing, camera_view_dist, min_dist, current_obstacle, obstacles, map))
    {
        set_new_goal();
    }
    return current_robot_position;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "search_manager");
    ros::NodeHandle node("~");

    sm = new SearchManager();

    exploration_failed = false;
    object_search_in_progress = false;
    object_found = false;

    ros::Subscriber objects_found_sub = node.subscribe("/objects", 5, objects_found_callback);
    ros::Subscriber sub = node.subscribe("/proj_scan", 1, scanCallback);
    ros::Subscriber gridMapSub = node.subscribe("/map", 1, gridMapCallback);
    ros::Subscriber move_base_status_sub = node.subscribe("/move_base/status", 1, status_callback);
    ros::Subscriber exploration_status_sub = node.subscribe("/explore_server/status", 1, explore_status_callback);

    publisher_obstacles_found = node.advertise<nav_msgs::OccupancyGrid>("/obstacles/found", 1);
    publisher_checked_obstacles = node.advertise<nav_msgs::OccupancyGrid>("/obstacles/checked", 1);
    publisher_pending_obstacles = node.advertise<nav_msgs::OccupancyGrid>("/obstacles/pending", 1);
    publisher_exploration_goal = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    goal_pub = node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    explore_canceller = node.advertise<actionlib_msgs::GoalID>("/explore_server/cancel", 1);
    listener = new tf::TransformListener();

    map = new grid_map::GridMap({"input_og"});

    map->setFrameId("map");
    map->setGeometry(grid_map::Length(2.0, 2.0), 0.01);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             map->getLength().x(), map->getLength().y(),
             map->getSize()(0), map->getSize()(1));

    std::vector<std::string> map_layers;
    map_layers.push_back("rgbd_scan");
    map_layers.push_back("obstacles_found");
    map_layers.push_back("pending_obstacles");
    obstacles = new grid_map::GridMap(map_layers);
    obstacles->setFrameId("map");
    obstacles->setGeometry(map->getLength(), map->getResolution());
    obstacles->setPosition(map->getPosition());

    ros::Rate rate(50.0);

    start_frontier_exploration();

    while (exploration_in_progress && node.ok() && !object_found)
    {
        ros::spinOnce();
        update_robot_pos();
        rate.sleep();
    }

    if (object_found)
    {
        ROS_INFO("Object detected during exploration, stop exploration task");
        cancel_exploration_action();
        cancel_move_base_action();
        node.shutdown();
        return 0;
    }
    else
    {
        ROS_INFO("Detected exploration finsh");
    }

    if (exploration_failed)
    {
        ROS_ERROR("Shutting down node due to exploration error");
        node.shutdown();
        return 0;
    }

    ROS_INFO("Begin searching for object");
    object_search_in_progress = true;
    set_new_goal();

    while (node.ok() && object_search_in_progress && !object_found)
    {
        update_robot_pos();
        goal_reached = sm->is_goal_reached(exploration_goal, ROSbot2_base_to_map_transform, 0.3, 0.5);
        if (goal_reached)
        {
            ROS_INFO("Goal is reached, can set new destination");
            set_new_goal();
        }
        else if (goal_accessible)
        {
            // wait until goal is reached
        }
        else
        {
            set_new_goal();
        }

        ros::spinOnce();
        rate.sleep();
    }
    if (node.ok())
    {
        if (object_found)
        {
            ROS_INFO("Object search succeeded");
        }
        else
        {
            ROS_INFO("Object search stopped, all area checked, nothing found");
        }
        cancel_move_base_action();
        node.shutdown();
    }
    else
    {
        ROS_WARN("Object search cancelled with external signal");
    }
    return 0;
}