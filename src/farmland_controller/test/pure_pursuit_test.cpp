#include <cmath>
#include <farmland_controller/pure_pursuit.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/console.h>


#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>

#include <nav_msgs/Path.h>

geometry_msgs::Point get_point(float x, float y) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  return p;
}

TEST(PurePursuitTestSuite, euclideanDistance2d)
{
  farmland_controller::PurePursuit pp;
  geometry_msgs::Point p1 = get_point(0,0);
  geometry_msgs::Point p2 = get_point(1,0);
  geometry_msgs::Point p3 = get_point(1,1);
  EXPECT_FLOAT_EQ(0,pp.euclideanDistance2d(p1,p1));
  EXPECT_FLOAT_EQ(1,pp.euclideanDistance2d(p1,p2));
  EXPECT_FLOAT_EQ(sqrt(2),pp.euclideanDistance2d(p1,p3));
}

TEST(PurePursuitTestSuite, getTargetPoint)
{
  // Create path
  nav_msgs::Path path;
  float path_len = 10;
  float spacing = 0.01;
  int num_points = path_len/spacing;
  for (int i = 0; i < num_points; i++) {
    geometry_msgs::PoseStamped ps;
    ps.pose.position = get_point(i*spacing,0);
    path.poses.push_back(ps);
  }

  farmland_controller::PurePursuit pp;
  geometry_msgs::Pose robot_pose;


  robot_pose.position = get_point(0,0);
  float ld = 1;
  EXPECT_FLOAT_EQ(1, pp.getTargetPoint(robot_pose, path,ld).x);

  robot_pose.position = get_point(5,0);
  ld = 1;
  EXPECT_FLOAT_EQ(6, pp.getTargetPoint(robot_pose, path,ld).x);

  ld = 1;
  robot_pose.position = get_point(path_len - (ld/2.0),0);
  EXPECT_FLOAT_EQ(path.poses.back().pose.position.x, pp.getTargetPoint(robot_pose, path,ld).x);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pure_pursuit_test");
  ros::NodeHandle nh;
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();
  return RUN_ALL_TESTS();
}