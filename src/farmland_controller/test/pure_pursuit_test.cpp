#include <farmland_controller/pure_pursuit.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>

#include <nav_msgs/Path.h>

TEST(PurePursuitTestSuite, euclideanDistance2d)
{
  farmland_controller::PurePursuit pp;
  geometry_msgs::Point p1;
  p1.x = 0;
  p1.y = 0;

  EXPECT_EQ(0,pp.euclideanDistance2d(p1,p1));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pure_pursuit_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}