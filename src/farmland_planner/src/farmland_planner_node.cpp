#include "nav_msgs/GetPlan.h"
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <farmland_controller/pure_pursuit.h>
#include <farmland_controller/pure_pursuitAction.h>
#include <farmland_controller/pure_pursuitFeedback.h>
#include <farmland_controller/pure_pursuitResult.h>
#include <farmland_frontier_detection/GetFrontiers.h>
#include <gazebo_msgs/ModelState.h>
#include <limits>

ros::ServiceClient get_frontiers_client;
ros::ServiceClient global_planner_client;
nav_msgs::GetPlan global_planner_srv;

constexpr float DIST_TO_GOAL_THRESHOLD = 1.0f;

std::vector<geometry_msgs::PoseStamped> goal_poses;
int cur_goal = 0;
farmland_controller::pure_pursuitFeedback feedback;
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

  ROS_INFO_THROTTLE(1, "robot vlinear = %.4f,  vyaw = %.4f",
                    std::sqrt(std::pow(robot_state_msg->twist.linear.x, 2) +
                              std::pow(robot_state_msg->twist.linear.y, 2)),
                    robot_state_msg->twist.angular.z);
}

void PPDoneCb(const actionlib::SimpleClientGoalState &state,
              const farmland_controller::pure_pursuitResultConstPtr &result) {}

void PPActiveCb() {}

void PPFeedbackCb(const farmland_controller::pure_pursuitFeedbackConstPtr &fb) {
  feedback = *fb;
}

geometry_msgs::Point getBestFrontier(std::vector<geometry_msgs::Point> points) {
  geometry_msgs::Point result;
  float best_dist = std::numeric_limits<float>::infinity();

  for (const auto &point : points) {
    float dist = (farmland_controller::PurePursuit::euclideanDistance2d(
        curr_pose_stamped.pose.position, point));
    if (dist < best_dist) {
      result = point;
      best_dist = dist;
    }
  }
  return result;
}

// Returns true if there is another goal to go to
// Returns false if there are no more frontiers
// Crashes on error
std::pair<geometry_msgs::PoseStamped, bool> getGoalPose() {
  farmland_frontier_detection::GetFrontiers get_frontiers_srv;
  get_frontiers_srv.request.robot_position = curr_pose_stamped.pose.position;
  if (!get_frontiers_client.call(get_frontiers_srv)) {
    ROS_ERROR("Planner node: Failed to call /get_frontiers srv");
    // TODO: Get summary print out
    assert(false);
  }

  geometry_msgs::PoseStamped best_goal;
  bool get_goal_success;
  if (!get_frontiers_srv.response.points.points.empty()) {
    best_goal.pose.position =
        getBestFrontier(get_frontiers_srv.response.points.points);
    get_goal_success = true;
  } else {
    ROS_INFO("Planner node: No feasible goal founded");
    get_goal_success = false;
  }
  return {best_goal, get_goal_success};
}

std::pair<nav_msgs::Path, bool>
getGlobalPlan(const geometry_msgs::PoseStamped &start_pose,
              const geometry_msgs::PoseStamped &goal_pose) {
  global_planner_srv.request.start = start_pose;
  global_planner_srv.request.start.pose.position.z = 0;
  global_planner_srv.request.goal = goal_pose;
  global_planner_srv.request.goal.pose.position.z = 0;
  global_planner_srv.request.goal.header.frame_id = "world";
  ROS_INFO_THROTTLE(0.2,
                    "Planner: start_pose = (%.4f, %.4f, %.4f), goal_pose = "
                    "(%.4f, %.4f, %.4f)",
                    start_pose.pose.position.x, start_pose.pose.position.y,
                    start_pose.pose.position.z, goal_pose.pose.position.x,
                    goal_pose.pose.position.y, goal_pose.pose.position.z);
  std::pair<nav_msgs::Path, bool> result;

  result.second = false;
  if (global_planner_client.call(global_planner_srv)) {
    if (global_planner_srv.response.plan.poses.size() > 0) {
      ROS_INFO("Planner node: A plan is made, and its length is %ld",
               global_planner_srv.response.plan.poses.size());
      // ROS_INFO(srv.response.plan.poses[0]);
      result.first = global_planner_srv.response.plan;
      result.second = true;
      return result;
    } else {
      ROS_INFO("Planner node: No feasible plan founded");
    }
  } else {
    ROS_ERROR(
        "Planner node: Failed to call service /planner/planner/make_plan");
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
  get_frontiers_client =
      nh.serviceClient<farmland_frontier_detection::GetFrontiers>(
          "/get_frontiers");

  ros::Rate loop_rate(100);
  ROS_INFO("Planner Node: looping starts\n");
  while (ros::ok()) {
    bool PP_has_started = false;

    if (!robot_state_is_initialized) {
      ros::spinOnce();
      loop_rate.sleep();
      continue;
    }

    // send a new goal
    if (!ac_ptr_PP_is_active()) {
      geometry_msgs::PoseStamped goal_pose;
      bool get_goal_pose_succ;
      std::tie(goal_pose, get_goal_pose_succ) = getGoalPose();

      if (!get_goal_pose_succ) {
        ROS_INFO("Planner node: Failed to get a new frontier. Success?");
        break;
      }

      nav_msgs::Path path;
      bool global_plan_success;
      std::tie(path, global_plan_success) =
          getGlobalPlan(curr_pose_stamped, goal_pose);

      if (!global_plan_success) {
        ros::spinOnce();
        loop_rate.sleep();
        continue;
      }

      ROS_INFO("Planner Node: Sending goal %d to Pure Pursuit", cur_goal);
      farmland_controller::pure_pursuitGoal goal;
      goal.path = path;
      feedback.dist_to_goal = std::numeric_limits<float>::infinity();
      ac_ptr_PP->sendGoal(goal, &PPDoneCb, &PPActiveCb, &PPFeedbackCb);
      PP_has_started = false;
    }

    // Wait for PP to finish
    while (ac_ptr_PP_is_active() || !PP_has_started) {
      if (ac_ptr_PP_is_active()) {
        PP_has_started = true;
      }
      if (feedback.dist_to_goal < DIST_TO_GOAL_THRESHOLD) {
        ac_ptr_PP->cancelGoal();
      }
      ros::spinOnce();
      loop_rate.sleep();
    }

    if (ac_ptr_PP->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_INFO("Planner node: Successfully preempted pure pursuit");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}