#include <farmland_controller/pure_pursuit.h>
#define PP PurePursuit
namespace farmland_controller {
std::pair<geometry_msgs::Twist, bool>
PP::getCommand(gazebo_msgs::ModelState &robot_state,
               const nav_msgs::Path &path) {
  cur_robot_state = robot_state;
  float robot_speed;       // Translational speed of the robot
  float ld;                // The lookahead distance
  geometry_msgs::Point tp; // The lookahead point
  float alpha; // Delta between current robot heading and direction of target
               // point
  float target_dist;            // Distance to target point
  float turning_radius;         // Turning radius the robot needs to get to tp
  geometry_msgs::Twist cmd_vel; // Command sent to robot

  marker_array.markers.clear();

  geometry_msgs::Point goal_point = path.poses.back().pose.position;
  robot_speed = sqrt(pow(robot_state.twist.linear.x, 2) +
                     pow(robot_state.twist.linear.y, 2));

  ld = getLookAheadDistance(robot_speed);
  tp = getTargetPoint(robot_state.pose, path, ld);
  alpha = getHeadingDelta(robot_state.pose, tp);
  target_dist = euclideanDistance2d(robot_state.pose.position, tp);
  turning_radius = getTurningRadius(target_dist, alpha);
  float dist_to_goal =
      euclideanDistance2d(robot_state.pose.position, goal_point);

  float target_linear_speed =
      getLinearCommand(target_dist, dist_to_goal, alpha, turning_radius);
  float target_angular_speed = getAngularCommand(desired_speed, turning_radius);

  ROS_INFO_THROTTLE(0.2, "PP: ld= %.4f, hd=%.4f, radius=%.4f", ld, alpha,
                    turning_radius);

  std::pair<geometry_msgs::Twist, bool> result;

  result.first = getTwist(target_linear_speed, target_angular_speed);
  result.second =
      dist_to_goal < goal_dist_epsilon && robot_speed < stationary_threshold;
  ROS_INFO_THROTTLE(0.2,
                    "dist_to_goal = %.4f, goal_dist_epsilon = %.4f, "
                    "robot_speed = %.4f, stationary_threshold = %.4f",
                    dist_to_goal, goal_dist_epsilon, robot_speed,
                    stationary_threshold);

  return result;
}
float PP::euclideanDistance2d(geometry_msgs::Point a, geometry_msgs::Point b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

/**
 * Extract yaw from a pose
 */
float PP::yawFromPose(geometry_msgs::Pose &pose) {
  static tf::Quaternion quaternion;
  static tf::Matrix3x3 rot_matrix;
  static double roll, pitch, yaw;

  // Get robot state
  tf::quaternionMsgToTF(pose.orientation, quaternion);
  rot_matrix.setRotation(quaternion);
  rot_matrix.getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::Quaternion PP::quatFromYaw(float yaw) {
  geometry_msgs::Quaternion msg_quaternion;
  static tf::Quaternion tf_quaternion;
  static tf::Matrix3x3 rot_matrix;
  rot_matrix.setRPY(0, 0, yaw);
  rot_matrix.getRotation(tf_quaternion);
  tf::quaternionTFToMsg(tf_quaternion, msg_quaternion);
  return msg_quaternion;
}

/**
 * Gets the lookahead distance for the robot
 */
float PP::getLookAheadDistance(float robot_speed) {
  float ld = robot_speed * k_dd;
  if (ld < min_ld)
    ld = min_ld;
  if (ld > max_ld)
    ld = max_ld;

  return ld;
}

/**
 * Gets a lookahead point
 */
geometry_msgs::Point PP::getTargetPoint(geometry_msgs::Pose &robot_pose,
                                        const nav_msgs::Path &path, float ld) {
  geometry_msgs::Point point;
  bool found_target_candidate = false;
  bool at_start = PP::euclideanDistance2d(robot_pose.position,
                                          path.poses[0].pose.position) < ld;
  bool at_end = PP::euclideanDistance2d(robot_pose.position,
                                        path.poses.back().pose.position) < ld;
  bool found_tp = false;
  if (at_end) {
    ROS_INFO_THROTTLE(0.2, "PP At end");
    // Return the last point
    point = path.poses.back().pose.position;
    found_tp = true;
  }

  if (at_start && !found_tp) {
    ROS_INFO_THROTTLE(0.2, "PP At Start");
    // Search from start to end, return last point le ld away
    for (int i = 0; i < path.poses.size(); i++) {
      if (PP::euclideanDistance2d(robot_pose.position,
                                  path.poses[i].pose.position) > ld) {
        found_tp = true;
        break;
      }
      point = path.poses[i].pose.position;
    }
  }

  // At this point robot is neither near start nor end of path.
  // Search from end to start, return last point ge ld away
  if (!found_tp) {
    ROS_INFO_THROTTLE(0.2, "PP in Middle");
    for (int i = path.poses.size() - 1; i >= 0; i--) {
      if (PP::euclideanDistance2d(robot_pose.position,
                                  path.poses[i].pose.position) <= ld) {
        point = path.poses[i].pose.position;
        found_tp = true;
        break;
      }
    }
    if (!found_tp) {
      point = path.poses.back().pose.position;
      found_tp = true;
    }
  }

  // tp;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "world";
  marker.frame_locked = true;
  marker.ns = PP_MARKER_NS;
  marker.id = 10;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = point;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.5;
  marker.lifetime = ros::Duration(PP_MARKER_LIFETIME);
  marker_array.markers.push_back(marker);

  return point;
}

/**
 * Gets the heading delta needed to have the robot face the target point
 * ie robot_heading + heading_delta = angle robot needs to face target
 */
float PP::getHeadingDelta(geometry_msgs::Pose &robot_pose,
                          geometry_msgs::Point target_point) {
  float yaw = yawFromPose(robot_pose);
  float delta_y = target_point.y - robot_pose.position.y;
  float delta_x = target_point.x - robot_pose.position.x;
  float angleFromRobotToTarget = atan2(delta_y, delta_x);
  float heading_delta = angleFromRobotToTarget - yaw;

  ROS_INFO_THROTTLE(
      0.2, "angleFromRobotToTarget = %.4f, yaw = %.4f, heading_delta = %.4f",
      angleFromRobotToTarget, yaw, heading_delta);
  /*
  yaw -> e0(cos yaw, sin yaw)
  angleFromRobotToTarget -> e1 (cos.., sin..)
  cosine = e0 \dot e1
  sin ...
  atain2 (sin, cos)

  */
  // Constrain heading_delta to range [-pi, pi)
  while (heading_delta < -M_PI) {
    ROS_WARN_THROTTLE(2, "PP: Heading less than -pi");
    heading_delta += 2*M_PI;
  }
  while (heading_delta >= M_PI) {
    ROS_WARN_THROTTLE(2, "PP: Heading greater than +pi");
    heading_delta -= 2*M_PI;
  }
  return heading_delta;
}

/**
 * Gets the turning radius needed to reach target point
 */
float PP::getTurningRadius(float dist_to_target_point, float heading_delta) {
  return dist_to_target_point / (2.0 * sin(heading_delta));
}

float PP::getLinearCommand(float dist_to_target_point, float dist_to_goal_point,
                           float heading_delta, float turning_radius) {
  bool at_goal = dist_to_goal_point < goal_dist_epsilon;
  bool approaching = dist_to_goal_point < approach_dist;

  float speed = desired_speed;
  if (approaching)
    speed = approach_speed;
  if (at_goal)
    speed = 0;

  visualization_msgs::Marker marker;

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "base_link";
  marker.frame_locked = true;
  marker.ns = PP_MARKER_NS;
  marker.id = 20;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1; // shaft_diameter
  marker.scale.y = 0.2; // head diameter
  marker.scale.z = 0.1; // head length
  geometry_msgs::Point start_point;
  start_point.x = 0;
  start_point.y = 0;
  start_point.z = 0.5;
  marker.points.push_back(start_point);
  geometry_msgs::Point end_point;
  end_point.x = speed * 2;
  end_point.y = 0;
  end_point.z = 0.5;
  marker.points.push_back(end_point);
  marker.color.r = 0;
  marker.color.g = 1;
  marker.color.b = 0;
  marker.color.a = 1;
  marker.lifetime = ros::Duration(PP_MARKER_LIFETIME);
  marker_array.markers.push_back(marker);

  return speed;
} // namespace farmland_controller

float PP::getAngularCommand(float speed, float radius) {
  float cmd = speed / radius;

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "base_link";
  marker.frame_locked = true;
  marker.ns = PP_MARKER_NS;
  marker.id = 30;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1; // shaft_diameter
  marker.scale.y = 0.2; // head diameter
  marker.scale.z = 0.1; // head length

  geometry_msgs::Point start_point;
  start_point.x = 0;
  start_point.y = 0;
  start_point.z = 0.5;
  marker.points.push_back(start_point);
  geometry_msgs::Point end_point;
  end_point.x = 0;
  end_point.y = cmd;
  end_point.z = 0.5;
  marker.points.push_back(end_point);
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1;
  marker.lifetime = ros::Duration(PP_MARKER_LIFETIME);
  marker_array.markers.push_back(marker);

  return cmd;
}

/**
 * Gets the twist (translational velocity and angular velocity) from a
 * turning radius
 */
geometry_msgs::Twist PP::getTwist(float linear_speed, float angular_speed) {
  geometry_msgs::Twist twist;
  twist.linear.x = linear_speed;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = angular_speed;
  return twist;
} // eg. ld = clip(k_dd * speed, min_ld, max_ld)
} // namespace farmland_controller