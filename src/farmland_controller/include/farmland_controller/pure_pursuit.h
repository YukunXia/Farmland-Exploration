#include <cmath>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <ros/duration.h>

#include <nav_msgs/Path.h>

#define PP_MARKER_NS "pure_pursuit_markers"
#define PP_MARKER_LIFETIME 1
#define MAX_ANGULAR_VEL 2

namespace farmland_controller
{
  class PurePursuit
  {
  public:
    /** --------- hyper parameters ---------------*/
    float desired_speed;     // desried speed of the robot
    float goal_dist_epsilon; // the distance which the robot needs to get to the
                             // goal point
    float stationary_threshold; // Speed at which robot is considered stationary                         
    float approach_dist;
    float approach_speed;
    float max_ld;            // max lookahead distance
    float min_ld;            // Min lookahead distance
    float k_dd;              // Lookahead speed mulitplier.
    gazebo_msgs::ModelState cur_robot_state;

    visualization_msgs::MarkerArray marker_array;

    PurePursuit():
      desired_speed(1),
      goal_dist_epsilon(1.0),
      max_ld(2),
      min_ld(0.5),
      k_dd(1),
      approach_dist(1.5),
      approach_speed(0.3),
      stationary_threshold(0.1){
    }

    PurePursuit(float desired_speed, float goal_dist_epsilon,
                float max_ld, float min_ld, float k_dd,
                float stationary_threshold):
      desired_speed(desired_speed),
      goal_dist_epsilon(goal_dist_epsilon),
      max_ld(max_ld),
      min_ld(min_ld),
      k_dd(k_dd),
      stationary_threshold(stationary_threshold){
    }
    /**
     * @brief Get the twist to control to a path
     * 
     * @param robot_pose 
     * @param path 
     * @return geometry_msgs::Twist, bool. Twist is the cmd to the robot, bool is
     * true if the robot is within goal_dist_epsilon of the end of the path
     */
    std::pair<geometry_msgs::Twist,bool> getCommand(gazebo_msgs::ModelState &robot_state,
                                  const nav_msgs::Path &path);

    static float euclideanDistance2d(geometry_msgs::Point a, geometry_msgs::Point b);

    /**
     * Extract yaw from a pose
     */
    float yawFromPose(geometry_msgs::Pose &pose);

    /**
     * @brief 
     * 
     * @param yaw 
     * @return geometry_msgs::Quaternion 
     */
    geometry_msgs::Quaternion quatFromYaw(float yaw);

    /**
     * Gets the lookahead distance for the robot
     */
    float getLookAheadDistance(float robot_speed);

    /**
     * Gets a lookahead point
     */
    geometry_msgs::Point getTargetPoint(geometry_msgs::Pose &robot_pose,
                                        const nav_msgs::Path &path, float ld);

    /**
     * Gets the heading delta needed to have the robot face the target point
     */
    float getHeadingDelta(geometry_msgs::Pose &robot_pose,
                          geometry_msgs::Point target_point);

    /**
     * Gets the turning radius needed to reach target point
     */
    float getTurningRadius(float dist_to_target_point, float heading_delta);

    float getLinearCommand(float dist_to_target_point, float dist_to_goal_point,
                          float heading_delta, float turning_radius);

    float getAngularCommand(float speed, float radius, float heading_delta);

    /**
     * Gets the twist (translational velocity and angular velocity) from a turning
     * radius
     */
    geometry_msgs::Twist getTwist(float linear_speed, float angular_speed);
  };
}