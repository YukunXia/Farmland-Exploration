#include <cmath>

#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <farmland_controller/pure_pursuitAction.h>
#include <farmland_controller/pure_pursuitFeedback.h>
#include <farmland_controller/pure_pursuitResult.h>

/** ----------- defines --------------------- */
#define NODE_NAME "pure_pursuit_node" // note: this should be same as file name
#define SIMULATION_ROBOT_NAME "robot" // name of the robot in gazebo
#define MAIN_LOOP_RATE 100            // loop rate for main loop

typedef actionlib::SimpleActionServer<farmland_controller::pure_pursuitAction>
    Server;

/** --------- global variables -------------- */
gazebo_msgs::ModelState robot_state; // current state of the robot
bool robot_state_is_initialized =
    false; // set to true if robot state received, else false
ros::ServiceClient global_path_client; // service client to call global planner

/** --------- hyper parameters ---------------*/
float goal_dist_epsilon; // the distance which the robot needs to get to the
                           // goal point
float max_ld;              // max lookahead distance
float min_ld;              // Min lookahead distance
float k_dd; // Lookahead speed mulitplier. 
            // eg. ld = clip(k_dd * speed, min_ld, max_ld)

/** -------------- Helper Functions ---------- */
/**
 * Convert (x,y) to pose stamped.
 */
geometry_msgs::PoseStamped ArgToPoseStamped(double x, double y) {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.seq = 0;
  pose_stamped.header.frame_id = "map";
  pose_stamped.pose.position.x = x;
  pose_stamped.pose.position.y = y;
  // No need to provide orientation, because global doesn't consider that

  return pose_stamped;
}

float euclideanDistance2d(geometry_msgs::Point a, geometry_msgs::Point b) {
  return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
}

/**
 * Extract yaw from a pose
 */
float yawFromPose(geometry_msgs::Pose::ConstPtr &pose) {
  static tf::Quaternion quaternion;
  static tf::Matrix3x3 rot_matrix;
  static double roll, pitch, yaw;

  // Get robot state
  tf::quaternionMsgToTF(pose->orientation, quaternion);
  rot_matrix.setRotation(quaternion);
  rot_matrix.getRPY(roll, pitch, yaw);
  return yaw;
}

/**
 * Gets the lookahead distance for the robot
 */
float getLookAheadDistance(float robot_speed, float min_ld, float max_ld,
                           float k_dd) {
  return 0;
}

/**
 * Gets a lookahead point
 * TODO: Implement function stub
 */
geometry_msgs::Point getTargetPoint(geometry_msgs::Pose &robot_pose,
                                    nav_msgs::Path path, float ld) {
  geometry_msgs::Point point;
  return point;
}

/**
 * Gets the heading delta needed to have the robot face the target point
 * TODO: Implement function stub
 */
float getHeadingDelta(geometry_msgs::Pose &robot_pose,
                      geometry_msgs::Point target_point) {
  return 0;
}

/**
 * Gets the turning radius needed to reach target point
 */
float getTurningRadius(float dist_to_target_point, float heading_delta) {
  return 0;
}

/**
 * Gets the twist (translational velocity and angular velocity) from a turning
 * radius
 */
float getTwist(float radius, float target_speed) { return 0; }

/** ---------------- Callbacks ------------- */
/**
 * Callback to get robot state from Gazebo model states
 */
void modelStateCallback(const gazebo_msgs::ModelState::ConstPtr &msg) {
  robot_state.model_name = msg->model_name;
  robot_state.pose = msg->pose;
  robot_state.twist = msg->twist;
  robot_state_is_initialized = true;
}

void execute(const farmland_controller::pure_pursuitGoalConstPtr goal, Server *as_ptr,
             ros::NodeHandle *nh) {
  farmland_controller::pure_pursuitResult result;
  farmland_controller::pure_pursuitFeedback feedback;

  // Goal point is end of path
  size_t num_poses = goal->path.poses.size();
  geometry_msgs::Point goal_point = goal->path.poses[num_poses-1].pose.position;

  while(euclideanDistance2d(robot_state.pose.position,goal_point) > goal_dist_epsilon) {
    if (as_ptr->isPreemptRequested() || !ros::ok()) {
      ROS_WARN("pre empted");
      result.success = false;
      break;
    }
    // TODO: remove break
    break;
  }

  if (result.success) {
    ROS_INFO("PP Node: Got to goal");
    as_ptr->setSucceeded(result);
  } else {
    as_ptr->setPreempted(result);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;
  ros::Rate main_loop_rate(MAIN_LOOP_RATE);

  global_path_client =
      nh.serviceClient<nav_msgs::GetPlan>("/planner/planner/make_plan");

  ros::Subscriber sub_model_states =
      nh.subscribe("/husky_cur_state", 1, modelStateCallback);

  Server server(nh, "pure_pursuit_server",
                boost::bind(&execute, _1, &server, &nh), false);
  server.start();

  while (ros::ok()) {
    // Do work
    ros::spinOnce();
    main_loop_rate.sleep();
  }
}