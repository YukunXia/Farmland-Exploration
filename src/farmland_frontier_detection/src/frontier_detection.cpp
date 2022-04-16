#include <farmland_frontier_detection/frontier_detection.h>

namespace farmland_frontier_detection {
  // =========== Cals Frontier Detector ================
  FrontierDetector::FrontierDetector() {}

  farmland_frontier_detection::PointArray
    FrontierDetector::getDetections(const geometry_msgs::Point &robot_position, const Matrix &occupancy_grid) {
    marker_array.markers.clear();
    farmland_frontier_detection::PointArray points;
    return points;
  }

  void FrontierDetector::addFrontierToMarkerArray(int x, int y) {

  }

}