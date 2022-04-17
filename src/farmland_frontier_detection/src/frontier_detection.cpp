#include <farmland_frontier_detection/frontier_detection.h>

namespace farmland_frontier_detection {
// ============ Class Map Location =============
MapLocation::MapLocation() {}
MapLocation::MapLocation(MapLocation loc, int delta_row, int delta_col) {
  row = loc.row + delta_row;
  col = loc.col + delta_col;
}
// =========== Class Frontier Detector ================
FrontierDetector::FrontierDetector() {}

MapLocations FrontierDetector::getDetections(const MapLocation &robot_location,
                                             const MatrixXi8 &grid) {
  occupancy_grid = grid;
  MapLocations frontiers;

  // Initialize wave_front seen to False
  wavefront_seen = MatrixXb::Zero(occupancy_grid.rows(), occupancy_grid.cols());

  std::queue<MapLocation> Q;
  Q.push(robot_location);

  int offset = 10;
  Q.push(MapLocation(robot_location, offset, 0));
  Q.push(MapLocation(robot_location, offset, offset));
  Q.push(MapLocation(robot_location, 0     , offset));
  Q.push(MapLocation(robot_location,-offset, offset));
  Q.push(MapLocation(robot_location,-offset, 0));
  Q.push(MapLocation(robot_location,-offset,-offset));
  Q.push(MapLocation(robot_location, 0     ,-offset));
  Q.push(MapLocation(robot_location, offset,-offset));

  while (!Q.empty()) {
    MapLocation cur_loc = Q.front();
    Q.pop();

    if (!isValid(cur_loc) 
      || wavefront_seen(cur_loc.row, cur_loc.col)
      || occupancy_grid(cur_loc.row, cur_loc.col) != FREESPACE)
      continue;

    wavefront_seen(cur_loc.row, cur_loc.col) = true;

    if (locationIsFrontier(cur_loc)) {
      frontiers.push_back(cur_loc);
    }

    Q.push(MapLocation(cur_loc, 1, 0));
    Q.push(MapLocation(cur_loc, -1, 0));
    Q.push(MapLocation(cur_loc, 0, 1));
    Q.push(MapLocation(cur_loc, 0, -1));
  }

  return frontiers;
}

bool FrontierDetector::isValid(const MapLocation &loc) {
  return 0 <= loc.row && loc.row < occupancy_grid.rows() && 0 <= loc.col &&
         loc.col < occupancy_grid.cols();
}

bool FrontierDetector::locationIsFrontier(const MapLocation &loc) {
  if (!isValid(loc) || occupancy_grid(loc.row, loc.col) != FREESPACE)
    return false;

  MapLocations locs;
  locs.push_back(MapLocation(loc, 1, 0));
  locs.push_back(MapLocation(loc, -1, 0));
  locs.push_back(MapLocation(loc, 0, 1));
  locs.push_back(MapLocation(loc, 0, -1));
  for (const auto &adj : locs) {
    if (!isValid(adj))
      continue;
    if (occupancy_grid(adj.row, adj.col) == UNKOWN)
      return true;
  }
  
  return false;
}

} // namespace farmland_frontier_detection