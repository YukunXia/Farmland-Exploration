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

#include <visualization_msgs/MarkerArray.h>
#include <farmland_controller/pure_pursuit.h>

/** ----------- defines --------------------- */
#define NODE_NAME                                                              \
  "farmland_controller_node"          // note: this should be same as file name
#define SIMULATION_ROBOT_NAME "husky" // name of the robot in gazebo
#define ACTION_SERVER_NAME "pure_pursuit_server"
#define MAIN_LOOP_RATE 100 // loop rate for main loop
#define PURE_PURSUIT_LOOP_RATE 100

typedef actionlib::SimpleActionServer<farmland_controller::pure_pursuitAction>
    Server;

/** --------- global variables -------------- */
gazebo_msgs::ModelState robot_state; // current state of the robot
bool robot_state_is_initialized =
    false; // set to true if robot state received, else false
ros::Publisher pub_cmd_vel;
ros::Publisher pub_marker_array;

/** --------- hyper parameters ---------------*/
float desired_speed = 1;        // desried speed of the robot
float goal_dist_epsilon = 0.5; // the distance which the robot needs to get to
                                // the goal point
float max_ld = 3;   // max lookahead distance
float min_ld = 0.5; // Min lookahead distance
float k_dd = 0.75;  // Lookahead speed mulitplier.
                    // eg. ld = clip(k_dd * speed, min_ld, max_ld)

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

void execute(const farmland_controller::pure_pursuitGoalConstPtr goal,
             Server *as_ptr, ros::NodeHandle *nh) {
  farmland_controller::pure_pursuitResult result;
  farmland_controller::pure_pursuitFeedback feedback;
  farmland_controller::PurePursuit pp;
  pp.desired_speed = desired_speed;
  pp.k_dd = k_dd;
  pp.goal_dist_epsilon = goal_dist_epsilon;
  pp.min_ld = min_ld;
  pp.max_ld = max_ld;
  ros::Rate loop_rate(PURE_PURSUIT_LOOP_RATE);

  // Goal point is end of path
  geometry_msgs::Point goal_point = goal->path.poses.back().pose.position;
  bool reached_goal = false;
  geometry_msgs::Twist cmd_vel;
  ROS_INFO("Controller: Beginning execution");

  while (true) {
    if (as_ptr->isPreemptRequested() || !ros::ok()) {
      ROS_WARN("Controller: pre empted");
      result.success = false;
      break;
    }

    std::tie(cmd_vel, reached_goal) = pp.getCommand(robot_state, goal->path);

    if (reached_goal) {
      result.success = true;
      break;
    }

    pub_cmd_vel.publish(cmd_vel);
    pub_marker_array.publish(pp.marker_array);

    loop_rate.sleep();
  }

  if (result.success) {
    ROS_INFO("Controller: Succeeded");
    as_ptr->setSucceeded(result);
  } else {
    ROS_WARN("Controller: Failed");
    as_ptr->setPreempted(result);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;
  ros::Rate main_loop_rate(MAIN_LOOP_RATE);

  // /differential_drive_control/cmd_vel
  ros::Subscriber sub_model_states =
      nh.subscribe("/husky_curr_state", 1, modelStateCallback);

  pub_cmd_vel = nh.advertise<geometry_msgs::Twist>(
      "/husky_velocity_controller/cmd_vel", 5);

  pub_marker_array = nh.advertise<visualization_msgs::MarkerArray>(
    "/farmland_controlller/marker_array", 5);

  Server server(nh, ACTION_SERVER_NAME, boost::bind(&execute, _1, &server, &nh),
                false);
  server.start();

  while (ros::ok()) {
    // Do work
    ros::spinOnce();
    main_loop_rate.sleep();
  }
}