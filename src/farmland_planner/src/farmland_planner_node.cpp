#include "nav_msgs/GetPlan.h"
#include "ros/ros.h"
#include <gazebo_msgs/ModelState.h>

ros::ServiceClient frontier_detection_client;
ros::ServiceClient global_planner_client;
nav_msgs::GetPlan global_planner_srv;
ros::Publisher global_path_pub;

ros::Subscriber robot_state_sub;
gazebo_msgs::ModelState robot_state; // current state of the robot
bool robot_state_is_initialized =
    false; // set to true if robot state received, else false

void robotStateCallback(const gazebo_msgs::ModelStateConstPtr robot_state_msg) {
  robot_state = *robot_state_msg;
  robot_state_is_initialized = true;

  geometry_msgs::PoseStamped start_pose_stamped;
  start_pose_stamped.header.stamp = ros::Time::now();
  start_pose_stamped.header.seq = 0;
  start_pose_stamped.header.frame_id = "world";
  start_pose_stamped.pose = robot_state.pose;

  geometry_msgs::PoseStamped goal_pose_stamped;
  goal_pose_stamped.header.stamp = ros::Time::now();
  goal_pose_stamped.header.seq = 0;
  goal_pose_stamped.header.frame_id = "world";
  goal_pose_stamped.pose.position.x = 0.0;
  goal_pose_stamped.pose.position.y = 0.0;

  global_planner_srv.request.start = start_pose_stamped;
  global_planner_srv.request.goal = goal_pose_stamped;

  if (global_planner_client.call(global_planner_srv)) {
    if (global_planner_srv.response.plan.poses.size() > 0) {
      ROS_INFO("A plan is made, and its length is %ld",
               global_planner_srv.response.plan.poses.size());
      // ROS_INFO(srv.response.plan.poses[0]);
    } else {
      ROS_INFO("No feasible plan founded");
    }
  } else {
    ROS_ERROR("Failed to call service /planner/planner/make_plan");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "farmland_planner_node");
  ros::NodeHandle nh;

  global_planner_client =
      nh.serviceClient<nav_msgs::GetPlan>("/planner/planner/make_plan");
  robot_state_sub = nh.subscribe("/husky_curr_state", 100, robotStateCallback);
  global_path_pub = nh.advertise<nav_msgs::Path>("/global_path", 100);

  ros::Rate loop_rate(100);
  ROS_INFO("In farmland_planner_node: looping starts\n");
  while (ros::ok()) {
    ros::spinOnce();

    if (robot_state_is_initialized) {
      global_path_pub.publish(global_planner_srv.response.plan);
    }

    loop_rate.sleep();
  }

  return 0;
}