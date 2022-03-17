#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

nav_msgs::OccupancyGrid occupancy_grid;
bool robot_state_inited = false;
int robot_state_index = -1;
float robot_height = -1.0f;
gazebo_msgs::ModelState robot_state;

bool initOccupancyGrid() {
  if (!ros::param::has("/world_size"))
    return false;
  float world_size = -1.0;
  ros::param::get("/world_size", world_size);
  assert(world_size > 0.0f);

  if (!ros::param::has("/wall_thickness"))
    return false;
  float wall_thickness = -1.0;
  ros::param::get("/wall_thickness", wall_thickness);
  assert(wall_thickness > 0.0f);

  if (!ros::param::has("/occupancy_grid_resolution"))
    return false;
  float occupancy_grid_resolution = -1.0;
  ros::param::get("/occupancy_grid_resolution", occupancy_grid_resolution);
  assert(occupancy_grid_resolution > 0.0f);

  // Default setting: width = height
  occupancy_grid.info.width =
      // std::ceil((world_size + wall_thickness * 2) /
      // occupancy_map_resolution);
      std::ceil(world_size / occupancy_grid_resolution);
  occupancy_grid.info.height = occupancy_grid.info.width;
  // https://answers.ros.org/question/207914/occupancy-grid-coordinates/
  occupancy_grid.info.origin.position.x = -world_size / 2.0f;
  occupancy_grid.info.origin.position.y = -world_size / 2.0f;
  occupancy_grid.info.origin.position.z = 0.0;
  occupancy_grid.info.origin.orientation.x = 0.0;
  occupancy_grid.info.origin.orientation.y = 0.0;
  occupancy_grid.info.origin.orientation.z = 0.0;
  occupancy_grid.info.origin.orientation.w = 1.0;

  occupancy_grid.header.frame_id = "/world";

  occupancy_grid.data = std::vector<int8_t>();
  occupancy_grid.data.resize(
      occupancy_grid.info.width * occupancy_grid.info.height, -1);

  return true;
}

// bool initRobotState() {
//   if (!ros::param::has("/robot_init_x"))
//     return false;
//   float robot_init_x = -1.0;
//   ros::param::get("/robot_init_x", robot_init_x);
//   assert(robot_init_x > 0.0f);

//   if (!ros::param::has("/robot_init_y"))
//     return false;
//   float robot_init_y = -1.0;
//   ros::param::get("/robot_init_y", robot_init_y);
//   assert(robot_init_y > 0.0f);

//   if (!ros::param::has("/robot_init_z"))
//     return false;
//   float robot_init_z = -1.0;
//   ros::param::get("/robot_init_z", robot_init_z);
//   assert(robot_init_z > 0.0f);

//   if (!ros::param::has("/robot_init_yaw"))
//     return false;
//   float robot_init_yaw = -1.0;
//   ros::param::get("/robot_init_yaw", robot_init_yaw);
//   assert(robot_init_yaw > 0.0f);

//   robot_state.pose.position.x = robot_init_x;
//   robot_state.pose.position.y = robot_init_y;
//   robot_state.pose.position.z = robot_init_z;

//   return true;
// }

void velodynePointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr point_cloud_msg) {
  if (!robot_state_inited)
    return;

  pcl::PCLPointCloud2 pcl_pointcloud2_intermediate;
  pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;

  pcl_conversions::toPCL(*point_cloud_msg, pcl_pointcloud2_intermediate);
  pcl::fromPCLPointCloud2(pcl_pointcloud2_intermediate, pcl_point_cloud);

  // TODO: ues point cloud and robot pose to update occupancy grid
}

// TODO: Use localization method to replace the gazebo magic
void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr msg) {
  if (!robot_state_inited) {
    if (!ros::param::has("/robot_name")) {
      ROS_ERROR("Failed to read rosparam `/robot_name`");
      return;
    }
    std::string robot_name;
    ros::param::get("/robot_name", robot_name);
    assert(!robot_name.empty());

    if (!ros::param::has("/robot_bbox_z")) {
      ROS_ERROR("Failed to read rosparam `/robot_bbox_z`");
      return;
    }
    ros::param::get("/robot_bbox_z", robot_height);
    assert(robot_height > 0.0f);

    for (int i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == robot_name) {
        robot_state_index = i;
      }
    }
    assert(robot_state_index >= 0);

    robot_state.model_name = robot_name;
    robot_state.pose = msg->pose.at(robot_state_index);
    robot_state.twist = msg->twist.at(robot_state_index);
    robot_state.reference_frame = "/world";

    // position:
    //   x: 20.000076757763342
    //   y: 19.99997709945979
    //   z: 0.13227261495344791
    // orientation:
    //   x: 3.2100818533505595e-06
    //   y: 4.226063762422636e-08
    //   z: 2.1034668619371185e-06
    //   w: 0.9999999999926346
  }

  robot_state.pose = msg->pose.at(robot_state_index);
  robot_state.twist = msg->twist.at(robot_state_index);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "farmland_mapping_node");
  if (!initOccupancyGrid()) {
    ROS_ERROR("Failed to initialize occupancy grid");
    return -1;
  }

  ros::NodeHandle nh;

  ros::Subscriber velodyne_point_cloud_sub =
      nh.subscribe("/points", 100, velodynePointCloudCallback);
  ros::Subscriber robot_state_sub =
      nh.subscribe("/gazebo/model_states", 100, modelStatesCallback);

  ros::Rate loop_rate(100);
  ROS_INFO("In farmland_mapping_node\n");
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}