#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetPlan.h"
#include "ros/ros.h"
#include "tf/tf.h"

geometry_msgs::PoseStamped ArgToPoseStamped(float x, float y) {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.seq = 0;
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose.position.x = x;
  pose_stamped.pose.position.y = y;
  // pose_stamped.pose.position.z = 0;
  // tf::Quaternion pose_stamped_q_tf = tf::createQuaternionFromYaw(yaw);
  // geometry_msgs::Quaternion pose_stamped_q_geo;
  // tf::quaternionTFToMsg(pose_stamped_q_tf, pose_stamped_q_geo);
  // pose_stamped.pose.orientation = pose_stamped_q_geo;

  return pose_stamped;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_planner_test");
  if (argc != 5) {
    ROS_INFO("usage: start_x, start_y, end_x, end_y");
    return 1;
  }

  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<nav_msgs::GetPlan>("/planner/planner/make_plan");

  nav_msgs::GetPlan srv;
  srv.request.start = ArgToPoseStamped(atof(argv[1]), atof(argv[2]));
  srv.request.goal = ArgToPoseStamped(atof(argv[3]), atof(argv[4]));
  // srv.request.tolerance = 0.01;

  if (client.call(srv)) {
    if (srv.response.plan.poses.size() > 0) {
      ROS_INFO("A plan is made, and its length is %ld",
               srv.response.plan.poses.size());
      // ROS_INFO(srv.response.plan.poses[0]);
    } else {
      ROS_INFO("No feasible plan founded");
    }
  } else {
    ROS_ERROR("Failed to call service /planner/planner/make_plan");
    return 1;
  }

  return 0;
}
