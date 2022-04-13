#include "nav_msgs/GetPlan.h"
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <farmland_controller/pure_pursuitAction.h>
#include <farmland_controller/pure_pursuitFeedback.h>
#include <farmland_controller/pure_pursuitResult.h>
#include <gazebo_msgs/ModelState.h>

ros::ServiceClient frontier_detection_client;
ros::ServiceClient global_planner_client;
nav_msgs::GetPlan global_planner_srv;

std::vector<geometry_msgs::PoseStamped> goal_poses;
int cur_goal = 0;

ros::Subscriber robot_state_sub;
geometry_msgs::PoseStamped curr_pose_stamped;
bool robot_state_is_initialized =
    false; // set to true if robot state received, else false

typedef actionlib::SimpleActionClient<farmland_controller::pure_pursuitAction>
    PPClient;
std::shared_ptr<PPClient> ac_ptr_PP;
const std::string action_server_name = "pure_pursuit_server";

void robotStateCallback(const gazebo_msgs::ModelStateConstPtr robot_state_msg) {
  robot_state_is_initialized = true;

  curr_pose_stamped.header.stamp = ros::Time::now();
  curr_pose_stamped.header.seq = 0;
  curr_pose_stamped.header.frame_id = "world";
  curr_pose_stamped.pose = robot_state_msg->pose;

  ROS_INFO_THROTTLE(1,"robot vlinear = %.4f,  vyaw = %.4f",
           std::sqrt(std::pow(robot_state_msg->twist.linear.x, 2) +
                     std::pow(robot_state_msg->twist.linear.y, 2)),
           robot_state_msg->twist.angular.z);
}

void setGoalPoses() {
  geometry_msgs::PoseStamped goal_pose_stamped;
  // goal_pose_stamped.header.stamp = ros::Time::now();
  goal_pose_stamped.header.seq = 0;
  goal_pose_stamped.header.frame_id = "world";
  goal_pose_stamped.pose.position.x = 8.0;
  goal_pose_stamped.pose.position.y = 1.0;
  goal_poses.push_back(goal_pose_stamped);

  // goal_pose_stamped.header.stamp = ros::Time::now();
  goal_pose_stamped.header.seq = 0;
  goal_pose_stamped.header.frame_id = "world";
  goal_pose_stamped.pose.position.x = 0.0;
  goal_pose_stamped.pose.position.y = 0.0;
  goal_poses.push_back(goal_pose_stamped);
}
geometry_msgs::PoseStamped getGoalPose() {
  if (cur_goal >= goal_poses.size()) {
    cur_goal = goal_poses.size() - 1;
  }
  goal_poses[cur_goal].header.stamp = ros::Time::now();
  return goal_poses[cur_goal];
}

std::pair<nav_msgs::Path, bool>
getGlobalPlan(const geometry_msgs::PoseStamped &start_pose,
              const geometry_msgs::PoseStamped &goal_pose) {
  global_planner_srv.request.start = start_pose;
  global_planner_srv.request.goal = goal_pose;
  std::pair<nav_msgs::Path, bool> result;

  result.second = false;
  if (global_planner_client.call(global_planner_srv)) {
    if (global_planner_srv.response.plan.poses.size() > 0) {
      ROS_INFO("A plan is made, and its length is %ld",
               global_planner_srv.response.plan.poses.size());
      // ROS_INFO(srv.response.plan.poses[0]);
      result.first = global_planner_srv.response.plan;
      result.second = true;
      return result;
    } else {
      ROS_INFO("No feasible plan founded");
    }
  } else {
    ROS_ERROR("Failed to call service /planner/planner/make_plan");
  }
  return result;
}

void checkPurePuresuitReadiness() {
  ROS_INFO(
      "farmland_planner_node: Connecting to pure pursuit action server (%s)",
      action_server_name.c_str());
  ac_ptr_PP->waitForServer();
  ROS_INFO("farmland_planner_node: Pure pursuit action Server (%s) connected",
           action_server_name.c_str());
}

bool ac_ptr_PP_is_active() {
  return ac_ptr_PP->getState() == actionlib::SimpleClientGoalState::ACTIVE ||
         ac_ptr_PP->getState() == actionlib::SimpleClientGoalState::PENDING;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "farmland_planner_node");
  ros::NodeHandle nh;

  global_planner_client =
      nh.serviceClient<nav_msgs::GetPlan>("/planner/planner/make_plan");
  robot_state_sub = nh.subscribe("/husky_curr_state", 100, robotStateCallback);
  ac_ptr_PP = std::make_shared<PPClient>(action_server_name, true);
  checkPurePuresuitReadiness();
  setGoalPoses();

  ros::Rate loop_rate(100);
  ROS_INFO("Planner Node: looping starts\n");
  while (ros::ok()) {
    bool PP_has_started = false;

    if (!robot_state_is_initialized) {
        ros::spinOnce();
        loop_rate.sleep();
        continue;
    }

    if (!ac_ptr_PP_is_active()) {
      const geometry_msgs::PoseStamped goal_pose = getGoalPose();

      nav_msgs::Path path;
      bool global_plan_success;
      std::tie(path, global_plan_success) =
          getGlobalPlan(curr_pose_stamped, goal_pose);

      if (!global_plan_success) {
        ros::spinOnce();
        loop_rate.sleep();
        continue;
      }

      ROS_INFO("Planner Node: Sending goal %d to Pure Pursuit",cur_goal);
      farmland_controller::pure_pursuitGoal goal;
      goal.path = path;
      ac_ptr_PP->sendGoal(goal);
      PP_has_started = false;
    }

    // Wait for PP to finish
    while (ac_ptr_PP_is_active() || !PP_has_started) {
      if (ac_ptr_PP_is_active()) {
        PP_has_started = true;
      }
      ros::spinOnce();
      loop_rate.sleep();
    }

    ROS_INFO("Planner Node: Pure pursuit finished");
    if (ac_ptr_PP->getResult()->success) {
      ROS_INFO("Planner Node: Pure pursuit success");
      cur_goal++;
    }

    if (cur_goal >= goal_poses.size()) {
      ROS_INFO("Planner Node: Finished all goals");
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}