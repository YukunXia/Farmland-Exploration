#include <Eigen/Dense>
#include <farmland_frontier_detection/GetFrontiers.h>
#include <farmland_frontier_detection/frontier_detection.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define FD_MARKER_NS "frontier_detection_ns"
#define FD_MARKER_LIFETIME 5

#define COST_MAP "/planner/costmap/costmap"
// #define COST_MAP "/projected_map"

ros::Subscriber occupancy_grid_sub;
ros::ServiceServer service;
ros::Publisher pub_marker_array;

farmland_frontier_detection::FrontierDetector fd;
nav_msgs::OccupancyGrid occupancy_grid;
visualization_msgs::MarkerArray frontier_markers;

// occupancy_matrix has the same data as occupancy grid
Eigen::Map<farmland_frontier_detection::MatrixXi8> occupancy_matrix(nullptr, 0,
                                                                    0);

void occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr raw_map_msg) {
  occupancy_grid = *raw_map_msg;
  new (&occupancy_matrix) Eigen::Map<farmland_frontier_detection::MatrixXi8>(
      occupancy_grid.data.data(), occupancy_grid.info.height,
      occupancy_grid.info.width);
}

farmland_frontier_detection::MapLocation
getLocation(const geometry_msgs::Point &point) {
  farmland_frontier_detection::MapLocation result;

  // Calculate MapLocation row
  result.row =
      std::roundf((float(point.y) - occupancy_grid.info.origin.position.y) /
                  occupancy_grid.info.resolution);
  if (result.row >= occupancy_grid.info.height || result.row < 0) {
    ROS_WARN("farmland_frontier_detection_node: trying to transform point "
             "(%.4f, %.4f) to MapLocation, but row (%i) is out of valid range. "
             "Will clip row to between 0 and %i",
             point.x, point.y, result.row, int(occupancy_grid.info.height));
  }
  result.row =
      std::min(std::max(result.row, 0), int(occupancy_grid.info.height) - 1);

  // Calculate MapLocation col
  result.col =
      std::roundf((float(point.x) - occupancy_grid.info.origin.position.x) /
                  occupancy_grid.info.resolution);
  if (result.col >= occupancy_grid.info.width || result.col < 0) {
    ROS_WARN("farmland_frontier_detection_node: trying to transform point "
             "(%.4f, %.4f) to MapLocation, but col (%i) is out of valid range. "
             "Will clip row to between 0 and %i",
             point.x, point.y, result.col, int(occupancy_grid.info.width));
  }
  result.col =
      std::min(std::max(result.col, 0), int(occupancy_grid.info.width) - 1);

  return result;
}

geometry_msgs::Point
getPoint(const farmland_frontier_detection::MapLocation &location) {
  geometry_msgs::Point result;
  result.x = float(location.col) * occupancy_grid.info.resolution +
             occupancy_grid.info.origin.position.x;
  result.y = float(location.row) * occupancy_grid.info.resolution +
             occupancy_grid.info.origin.position.y;
  result.z = 0.0f;

  return result;
}

visualization_msgs::MarkerArray
getMarkers(std::vector<geometry_msgs::Point> &points) {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "world";
  marker.frame_locked = true;
  marker.ns = FD_MARKER_NS;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  for (const auto &point : points) {
    marker.points.push_back(point);
  }
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 0;
  marker.color.a = 1;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.5;
  marker.lifetime = ros::Duration(FD_MARKER_LIFETIME);
  markers.markers.push_back(marker);
  return markers;
}

bool getFrontiers(farmland_frontier_detection::GetFrontiers::Request &req,
                  farmland_frontier_detection::GetFrontiers::Response &res) {
  farmland_frontier_detection::MapLocation robot_location =
      getLocation(req.robot_position);
  farmland_frontier_detection::MapLocations frontier_locations =
      fd.getDetections(robot_location, occupancy_matrix);
  for (const auto &frontier_location : frontier_locations) {
    res.points.points.push_back(getPoint(frontier_location));
  }
  frontier_markers = getMarkers(res.points.points);

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "farmland_frontier_detection_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);

  occupancy_grid_sub = nh.subscribe<nav_msgs::OccupancyGrid>(
      COST_MAP, 100, occupancyGridCallback);
  service = nh.advertiseService("get_frontiers", getFrontiers);
  pub_marker_array = nh.advertise<visualization_msgs::MarkerArray>(
      "/farmland_frontier_detection/marker_array", 5);
      
  while (ros::ok()) {
    pub_marker_array.publish(frontier_markers);
    ros::spinOnce();
    loop_rate.sleep();
  }
}