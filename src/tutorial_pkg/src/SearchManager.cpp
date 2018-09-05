#include <SearchManager.h>

SearchManager::SearchManager()
{
}

bool SearchManager::lookup_camera_transform(double *cam_x, double *cam_y, double *cam_yaw, ros::Time scan_time, tf::TransformListener *listener)
{
    try
    {
        tf::StampedTransform rgbd_scan_to_map_transform;
        listener->lookupTransform("/map", "/camera_link", scan_time, rgbd_scan_to_map_transform);
        std::cout << "LookupTime: " << scan_time.sec << ", " << scan_time.nsec << std::endl;
        *cam_x = rgbd_scan_to_map_transform.getOrigin().x();
        *cam_y = rgbd_scan_to_map_transform.getOrigin().y();
        tf::Quaternion quat = rgbd_scan_to_map_transform.getRotation();
        tf::Matrix3x3 rotation_matrix;
        double roll;
        double pitch;
        double yaw;
        rotation_matrix.setRotation(quat);
        rotation_matrix.getRPY(roll, pitch, yaw);
        *cam_yaw = yaw;
        return true;
    }
    catch (tf::TransformException ex)
    {
        return false;
    }
}

bool SearchManager::check_obstacle_surrounding(grid_map::Position *robot_dest, double *obstacle_bearing, float dist_from_obstacle, float min_dist, grid_map::Position current_obstacle, grid_map::GridMap *obstacles, grid_map::GridMap *map)
{
    double current_bearing;
    for (int deg = 0; deg < 36; deg++)
    {
        current_bearing = *obstacle_bearing + (deg * 10);

        if (check_space_occupation(robot_dest, current_bearing, dist_from_obstacle, min_dist, current_obstacle, obstacles, map))
        {
            *obstacle_bearing = current_bearing;
            return true;
        }
    }
    return false;
}

bool SearchManager::check_space_occupation(grid_map::Position *robot_dest, double bearing, float dist_from_obstacle, float min_dist, grid_map::Position current_obstacle, grid_map::GridMap *obstacles, grid_map::GridMap *map)
{
    bool polygon_free = true;
    grid_map::Position current_robot_destination;
    grid_map::Position middle_corner;
    grid_map::Position left_corner;
    grid_map::Position right_corner;
    std::vector<grid_map::Position> vertices;
    grid_map::Polygon polygon;

    // calculate possible position
    current_robot_destination[0] = current_obstacle[0] + dist_from_obstacle * (sin((bearing)*M_PI / 180));
    current_robot_destination[1] = current_obstacle[1] + dist_from_obstacle * (cos((bearing)*M_PI / 180));
    middle_corner[0] = current_obstacle[0] + min_dist * (sin((bearing)*M_PI / 180));
    middle_corner[1] = current_obstacle[1] + min_dist * (cos((bearing)*M_PI / 180));
    left_corner[0] = middle_corner[0] + min_dist * (sin((bearing + 90) * M_PI / 180));
    left_corner[1] = middle_corner[1] + min_dist * (cos((bearing + 90) * M_PI / 180));
    right_corner[0] = middle_corner[0] + min_dist * (sin((bearing - 90) * M_PI / 180));
    right_corner[1] = middle_corner[1] + min_dist * (cos((bearing - 90) * M_PI / 180));
    vertices.push_back(current_robot_destination);
    vertices.push_back(middle_corner);
    vertices.push_back(left_corner);
    vertices.push_back(right_corner);
    polygon = grid_map::Polygon(vertices);

    int checked_points = 0;
    for (grid_map::PolygonIterator it(*obstacles, polygon); !it.isPastEnd(); ++it)
    {
        checked_points++;
        if (obstacles->at("obstacles_found", *it) > 0.5)
        {
            polygon_free = false;
        }
    }

    for (grid_map::CircleIterator it(*map, current_robot_destination, 0.1); !it.isPastEnd(); ++it)
    {
        float map_val = map->at("input_og", *it);
        if (map_val >= 0)
        {
        }
        else
        {
            polygon_free = false;
            grid_map::Position known_pos;
            map->getPosition(*it, known_pos);
        }
    }

    if (polygon_free)
    {
        *robot_dest = current_robot_destination;
        return true;
    }
    else
    {
        return false;
    }
}

bool SearchManager::set_point_checked(float x, float y, grid_map::GridMap *gm, grid_map::Position obstacle)
{
    float radius = 3 * gm->getResolution();
    return set_point_checked(x, y, gm, obstacle, radius);
}

bool SearchManager::set_point_checked(float x, float y, grid_map::GridMap *gm, grid_map::Position obstacle, float radius)
{
    for (grid_map::CircleIterator it(*gm, grid_map::Position(x, y), radius); !it.isPastEnd(); ++it)
    {
        gm->at("rgbd_scan", *it) = 1;
        gm->at("pending_obstacles", *it) = 0;
    }
    float x_dist = obstacle.x() - x;
    float y_dist = obstacle.y() - y;
    float dist = sqrt(x_dist * x_dist + y_dist * y_dist);
    if (dist < radius)
    {
        // set_new_goal();
        return true;
    }
    return false;
}

frontier_exploration::ExploreTaskGoal SearchManager::createExplorationGoal()
{
    frontier_exploration::ExploreTaskGoal goal;
    geometry_msgs::PointStamped center;
    center.header.frame_id = "map";
    center.point.x = 1.0;
    center.point.y = 0;
    center.point.z = 0;
    goal.explore_center = center;
    geometry_msgs::PolygonStamped square;
    square.header.frame_id = "map";
    std::vector<geometry_msgs::Point32> square_points;
    geometry_msgs::Point32 point_a;
    point_a.x = 30;
    point_a.y = 30;
    point_a.z = 0;
    geometry_msgs::Point32 point_b;
    point_b.x = -30;
    point_b.y = 30;
    point_b.z = 0;
    geometry_msgs::Point32 point_c;
    point_c.x = -30;
    point_c.y = -30;
    point_c.z = 0;
    geometry_msgs::Point32 point_d;
    point_d.x = 30;
    point_d.y = -30;
    point_d.z = 0;
    square_points.push_back(point_a);
    square_points.push_back(point_b);
    square_points.push_back(point_c);
    square_points.push_back(point_d);
    square.polygon.points = square_points;
    goal.explore_boundary = square;
    return goal;
}

bool SearchManager::is_goal_reached(geometry_msgs::PoseStamped goal, tf::StampedTransform current_tf, double linear_threshold, double angular_threshold)
{
    tf::Matrix3x3 goal_m, current_m;
    tf::Quaternion goal_q;
    double current_roll, current_pitch, current_yaw;
    double goal_roll, goal_pitch, goal_yaw;

    goal_q.setX(goal.pose.orientation.x);
    goal_q.setY(goal.pose.orientation.y);
    goal_q.setZ(goal.pose.orientation.z);
    goal_q.setW(goal.pose.orientation.w);
    current_m.setRotation(current_tf.getRotation());
    current_m.getRPY(current_roll, current_pitch, current_yaw);
    goal_m.setRotation(goal_q);
    goal_m.getRPY(goal_roll, goal_pitch, goal_yaw);

    double x_distance = current_tf.getOrigin().x() - goal.pose.position.x;
    double y_distance = current_tf.getOrigin().y() - goal.pose.position.y;
    double yaw_distance = current_yaw - goal_yaw;

    if (x_distance < linear_threshold && x_distance > -linear_threshold)
    {
        if (y_distance < linear_threshold && y_distance > -linear_threshold)
        {
            if (yaw_distance < angular_threshold && yaw_distance > -linear_threshold)
            {
                return true;
            }
        }
    }
    return false;
}