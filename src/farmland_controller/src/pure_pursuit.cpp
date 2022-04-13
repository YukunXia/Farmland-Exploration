#include <farmland_controller/pure_pursuit.h>
#define PP PurePursuit
namespace farmland_controller
{
  std::pair<geometry_msgs::Twist, bool> PP::getCommand(gazebo_msgs::ModelState &robot_state,
                                                       const nav_msgs::Path &path)
  {
    float robot_speed;            // Translational speed of the robot
    float ld;                     // The lookahead distance
    geometry_msgs::Point tp;      // The lookahead point
    float alpha;                  // Delta between current robot heading and direction of target point
    float target_dist;            // Distance to target point
    float turning_radius;         // Turning radius the robot needs to get to tp
    geometry_msgs::Twist cmd_vel; // Command sent to robot

    size_t num_poses = path.poses.size();
    geometry_msgs::Point goal_point = path.poses[num_poses - 1].pose.position;
    robot_speed = robot_state.twist.linear.x;

    ld = getLookAheadDistance(robot_speed);
    tp = getTargetPoint(robot_state.pose, path, ld);
    alpha = getHeadingDelta(robot_state.pose, tp);
    target_dist = euclideanDistance2d(robot_state.pose.position, tp);
    turning_radius = getTurningRadius(target_dist, alpha);
    cmd_vel = getTwist(turning_radius, desired_speed);

    std::pair<geometry_msgs::Twist, bool> result;
    result.first = cmd_vel;

    result.second = euclideanDistance2d(robot_state.pose.position, goal_point) < goal_dist_epsilon;
    return result;
  }
  float PP::euclideanDistance2d(geometry_msgs::Point a, geometry_msgs::Point b)
  {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
  }

  /**
   * Extract yaw from a pose
   */
  float PP::yawFromPose(geometry_msgs::Pose &pose)
  {
    static tf::Quaternion quaternion;
    static tf::Matrix3x3 rot_matrix;
    static double roll, pitch, yaw;

    // Get robot state
    tf::quaternionMsgToTF(pose.orientation, quaternion);
    rot_matrix.setRotation(quaternion);
    rot_matrix.getRPY(roll, pitch, yaw);
    return yaw;
  }

  /**
   * Gets the lookahead distance for the robot
   */
  float PP::getLookAheadDistance(float robot_speed)
  {
    float ld = robot_speed * k_dd;
    if (ld < min_ld)
      ld = min_ld;
    if (ld > max_ld)
      ld = max_ld;
    return ld;
  }

  /**
   * Gets a lookahead point
   * TODO: Implement function stub
   */
  geometry_msgs::Point PP::getTargetPoint(geometry_msgs::Pose &robot_pose,
                                          const nav_msgs::Path &path, float ld)
  {
    geometry_msgs::Point point;
    bool found_target_candidate = false;
    bool at_start = PP::euclideanDistance2d(robot_pose.position, path.poses[0].pose.position) < ld;
    bool at_end = PP::euclideanDistance2d(robot_pose.position, path.poses.back().pose.position) < ld;

    if (at_end) {
      ROS_DEBUG("PP At end");
      // Return the last point
      return path.poses.back().pose.position;
    }

    if (at_start)
    {
      ROS_DEBUG("PP At Start");
      // Search from start to end, return last point le ld away
      for (int i = 0; i < path.poses.size(); i++)
      {
        if (PP::euclideanDistance2d(robot_pose.position, path.poses[i].pose.position) > ld)
        {
          break;
        }
        point = path.poses[i].pose.position;
      }
      return point;
    }

    // At this point robot is neither near start nor end of path.
    // Search from end to start, return last point ge ld away
    ROS_DEBUG("PP in Middle");
    bool found_point = false;
    for (int i = path.poses.size()-1; i >= 0; i--) {
      if (PP::euclideanDistance2d(robot_pose.position, path.poses[i].pose.position) <= ld)
      {
        point = path.poses[i].pose.position;
        found_point = true;
        break;
      }
    }
    if (!found_point) {
      return path.poses.back().pose.position;
    }
    return point;
  }

  /**
   * Gets the heading delta needed to have the robot face the target point
   * TODO: Implement function stub
   */
  float PP::getHeadingDelta(geometry_msgs::Pose &robot_pose,
                            geometry_msgs::Point target_point)
  {
    float yaw = yawFromPose(robot_pose);
    float delta_y = robot_pose.position.y - target_point.y;
    float delta_x = robot_pose.position.x - target_point.x;
    float angleFromRobotToTarget = atan2(delta_y, delta_x);
    float heading_delta = angleFromRobotToTarget - yaw;
    return heading_delta;
  }

  /**
   * Gets the turning radius needed to reach target point
   */
  float PP::getTurningRadius(float dist_to_target_point, float heading_delta)
  {
    return 0;
  }

  /**
   * Gets the twist (translational velocity and angular velocity) from a turning
   * radius
   */
  geometry_msgs::Twist PP::getTwist(float radius, float target_speed)
  {
    geometry_msgs::Twist twist;
    return twist;
  } // eg. ld = clip(k_dd * speed, min_ld, max_ld)
}