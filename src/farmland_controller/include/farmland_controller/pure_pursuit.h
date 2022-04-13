#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>

#include <nav_msgs/Path.h>

namespace farmland_controller
{
  class PurePursuit
  {
  public:
    /** --------- hyper parameters ---------------*/
    float desired_speed;     // desried speed of the robot
    float goal_dist_epsilon; // the distance which the robot needs to get to the
                             // goal point
    float max_ld;            // max lookahead distance
    float min_ld;            // Min lookahead distance
    float k_dd;              // Lookahead speed mulitplier.

    PurePursuit():
      desired_speed(1),
      goal_dist_epsilon(1.0),
      max_ld(2),
      min_ld(0.5),
      k_dd(1){
    }

    PurePursuit(float desired_speed, float goal_dist_epsilon,
                float max_ld, float min_ld, float k_dd):
      desired_speed(desired_speed),
      goal_dist_epsilon(goal_dist_epsilon),
      max_ld(max_ld),
      min_ld(min_ld),
      k_dd(k_dd){
    }
    /**
     * @brief Get the twist to control to a path
     * 
     * @param robot_pose 
     * @param path 
     * @return geometry_msgs::Twist 
     */
    std::pair<geometry_msgs::Twist,bool> getCommand(gazebo_msgs::ModelState &robot_state,
                                  const nav_msgs::Path &path);

    float euclideanDistance2d(geometry_msgs::Point a, geometry_msgs::Point b);

    /**
     * Extract yaw from a pose
     */
    float yawFromPose(geometry_msgs::Pose &pose);

    /**
     * Gets the lookahead distance for the robot
     */
    float getLookAheadDistance(float robot_speed);

    /**
     * Gets a lookahead point
     * TODO: Implement function stub
     */
    geometry_msgs::Point getTargetPoint(geometry_msgs::Pose &robot_pose,
                                        const nav_msgs::Path &path, float ld);

    /**
     * Gets the heading delta needed to have the robot face the target point
     * TODO: Implement function stub
     */
    float getHeadingDelta(geometry_msgs::Pose &robot_pose,
                          geometry_msgs::Point target_point);

    /**
     * Gets the turning radius needed to reach target point
     */
    float getTurningRadius(float dist_to_target_point, float heading_delta);

    /**
     * Gets the twist (translational velocity and angular velocity) from a turning
     * radius
     */
    geometry_msgs::Twist getTwist(float radius, float target_speed);
  };
}