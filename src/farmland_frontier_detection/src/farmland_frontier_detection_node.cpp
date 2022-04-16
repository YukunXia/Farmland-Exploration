#include <Eigen/Dense>
#include <farmland_frontier_detection/frontier_detection.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

ros::Subscriber occupancy_grid_sub;

farmland_frontier_detection::FrontierDetector fd;
nav_msgs::OccupancyGrid occupancy_grid;

// occupancy_matrix has the same data as occupancy grid
Eigen::Map<farmland_frontier_detection::Matrix> occupancy_matrix(nullptr, 0, 0);

void occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr raw_map_msg) {
  occupancy_grid = *raw_map_msg;
  new (&occupancy_matrix) Eigen::Map<farmland_frontier_detection::Matrix>(
      occupancy_grid.data.data(), occupancy_grid.info.height,
      occupancy_grid.info.width);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "farmland_frontier_detection_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);

  occupancy_grid_sub = nh.subscribe<nav_msgs::OccupancyGrid>(
      "/projected_map", 100, occupancyGridCallback);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}