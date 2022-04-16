#include <Eigen/Dense>
#include <farmland_frontier_detection/PointArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#define MARKER_FRONTIER_Z 0 // Height of the markers for detected frontiers

namespace farmland_frontier_detection {

typedef Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
  Matrix;

class FrontierDetector {
public:
  Matrix occpancy_grid;
  Matrix wavefront_seen;
  visualization_msgs::MarkerArray marker_array;

  FrontierDetector();

  farmland_frontier_detection::PointArray
  getDetections(const Matrix &occupancy_grid);

  void addFrontierToMarkerArray(int x, int y);
};
} // namespace farmland_frontier_detection