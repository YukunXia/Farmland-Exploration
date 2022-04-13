#include <cmath>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <farmland_controller/pure_pursuitAction.h>
#include <farmland_controller/pure_pursuitFeedback.h>
#include <farmland_controller/pure_pursuitResult.h>

/** ----------- defines --------------------- */
#define NODE_NAME "pure_pursuit_node" // note: this should be same as file name
#define SIMULATION_ROBOT_NAME "robot" // name of the robot in gazebo
#define MAIN_LOOP_RATE 100            // loop rate for main loop
#define PURE_PURSUIT_LOOP_RATE 100

typedef actionlib::SimpleActionServer<farmland_controller::pure_pursuitAction>
    Server;

/** --------- global variables -------------- */
gazebo_msgs::ModelState robot_state; // current state of the robot
bool robot_state_is_initialized =
    false; // set to true if robot state received, else false
ros::ServiceClient global_path_client; // service client to call global planner
ros::Publisher pub_cmd_vel;

/** --------- hyper parameters ---------------*/
float desired_speed;       // desried speed of the robot
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
float yawFromPose(geometry_msgs::Pose &pose) {
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
float getLookAheadDistance(float robot_speed, float min_ld, float max_ld,
                           float k_dd) {
  float ld = robot_speed * k_dd;
  if (ld < min_ld) ld = min_ld;
  if (ld > max_ld) ld = max_ld;
  return ld;
}

/**
 * Gets a lookahead point
 * TODO: Implement function stub
 */
geometry_msgs::Point getTargetPoint(geometry_msgs::Pose &robot_pose,
                                    const nav_msgs::Path &path, float ld) {
  geometry_msgs::Point point;
  return point;
}

/**
 * Gets the heading delta needed to have the robot face the target point
 * TODO: Implement function stub
 */
float getHeadingDelta(geometry_msgs::Pose &robot_pose,
                      geometry_msgs::Point target_point) {
  float yaw = yawFromPose(robot_pose);
  float delta_y = robot_pose.position.y - target_point.y;
  float delta_x = robot_pose.position.x - target_point.x;
  float angleFromRobotToTarget = atan2(delta_y,delta_x);
  float heading_delta = angleFromRobotToTarget - yaw;
  return heading_delta;
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
geometry_msgs::Twist getTwist(float radius, float target_speed) {
  geometry_msgs::Twist twist;
  return twist;
}

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
  ros::Rate loop_rate(PURE_PURSUIT_LOOP_RATE);

  // Goal point is end of path
  size_t num_poses = goal->path.poses.size();
  geometry_msgs::Point goal_point = goal->path.poses[num_poses-1].pose.position;
  float goal_dist; // Distance from robot to goal point
  bool reached_goal = false;


  while(true) {
    if (as_ptr->isPreemptRequested() || !ros::ok()) {
      ROS_WARN("pre empted");
      result.success = false;
      break;
    }
    float robot_speed; // Translational speed of the robot
    float ld; // The lookahead distance
    geometry_msgs::Point tp; // The lookahead point
    float alpha; // Delta between current robot heading and direction of target point
    float target_dist; // Distance to target point
    float turning_radius; // Turning radius the robot needs to get to tp
    geometry_msgs::Twist cmd_vel; // Command sent to robot


    robot_speed = robot_state.twist.linear.x;
    ld = getLookAheadDistance(robot_speed, min_ld, max_ld, k_dd);
    tp = getTargetPoint(robot_state.pose,goal->path,ld);
    alpha = getHeadingDelta(robot_state.pose,tp);
    target_dist = euclideanDistance2d(robot_state.pose.position, tp);
    turning_radius = getTurningRadius(target_dist, alpha);
    cmd_vel = getTwist(turning_radius, desired_speed);

    pub_cmd_vel.publish(cmd_vel);

    goal_dist = euclideanDistance2d(robot_state.pose.position,goal_point);
    reached_goal = goal_dist < goal_dist_epsilon;

    if (reached_goal) {
      result.success = true;
      break;
    }

    loop_rate.sleep();
  }

  if (result.success) {
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

  // /differential_drive_control/cmd_vel
  ros::Subscriber sub_model_states =
      nh.subscribe("/husky_curr_state", 1, modelStateCallback);

  pub_cmd_vel =
      nh.advertise<geometry_msgs::Twist>("/differential_drive_control/cmd_vel",5);

  Server server(nh, "pure_pursuit_server",
                boost::bind(&execute, _1, &server, &nh), false);
  server.start();

  while (ros::ok()) {
    // Do work
    ros::spinOnce();
    main_loop_rate.sleep();
  }
}