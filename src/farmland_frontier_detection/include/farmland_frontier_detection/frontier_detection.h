#include <Eigen/Dense>
#include <farmland_frontier_detection/PointArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <ros/ros.h>

#define FREESPACE 0
#define UNKOWN -1

namespace farmland_frontier_detection {

typedef Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    MatrixXi8;

typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    MatrixXb;

class MapLocation {
public:
  int row;
  int col;
  MapLocation();
  MapLocation(MapLocation loc, int delta_row, int delta_col);
};

typedef std::vector<MapLocation> MapLocations;

class FrontierDetector {
public:
  MatrixXi8 occupancy_grid;
  MatrixXb wavefront_seen;

  FrontierDetector();

  MapLocations getDetections(const MapLocation &robot_location,
                             const MatrixXi8 &occupancy_grid);
  bool isValid(const MapLocation &loc);
  bool locationIsFrontier(const MapLocation &loc);
  std::pair<MapLocation, bool>
  getClosestFreeCell(const MapLocation &robot_location);
};
} // namespace farmland_frontier_detection