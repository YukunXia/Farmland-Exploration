#include <eigen_conversions/eigen_msg.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include <iostream>

nav_msgs::OccupancyGrid occupancy_grid;
// occupancy_matrix has the same data as occupancy grid
Eigen::Map<
    Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
    occupancy_matrix(nullptr, 0, 0);
bool robot_state_inited = false;
int robot_state_index = -1;
float robot_height = -1.0f;
float robot_mobility_max_height = -1.0f;
// robot_state.pose -> geometry_msgs::Pose
gazebo_msgs::ModelState robot_state;
// tf2::Transform base_link__tf2__velodyne;
Eigen::Isometry3d base_link__eigenT__velodyne;
Eigen::Isometry3d world__eigenT__velodyne;

/*
  Initialization stage functions
*/

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
  // occupancy_matrix.rows() == height (y range in gazebo world)
  // cols() == width (x range in gazebo world)
  // occupancy_matrix((y_w - y_omap) / res, (x_w - x_omap) / res)
  new (&occupancy_matrix) Eigen::Map<
      Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      occupancy_grid.data.data(), occupancy_grid.info.height,
      occupancy_grid.info.width);

  return true;
}

bool initRobotTF() {
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // base_link is the root link of the robot
  geometry_msgs::TransformStamped base_link__geo_stamped_tf__velodyne;
  try {
    base_link__geo_stamped_tf__velodyne = tf_buffer.lookupTransform(
        "base_link", "velodyne", ros::Time(0), ros::Duration(60.0));
  } catch (tf2::TransformException &e) {
    ROS_ERROR("tf2::TransformException error message: %s", e.what());
    return false;
  }

  // tf2::fromMsg(base_link__geo_stamped_tf__velodyne.transform,
  //              base_link__tf2__velodyne);
  base_link__eigenT__velodyne =
      tf2::transformToEigen(base_link__geo_stamped_tf__velodyne);

  return true;
}

/*
  Helper functions
*/

void mapPointCloudToCostmap(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {

  pcl::PassThrough<pcl::PointXYZ> point_cloud_height_filter;
  point_cloud_height_filter.setInputCloud(point_cloud);
  point_cloud_height_filter.setFilterFieldName("z");
  point_cloud_height_filter.setFilterLimits(robot_mobility_max_height,
                                            robot_height);

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_blocking(
      new pcl::PointCloud<pcl::PointXYZ>);
  point_cloud_height_filter.filter(*point_cloud_blocking);
}

/*
  Callback functions
*/

void velodynePointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr point_cloud_msg) {
  if (!robot_state_inited)
    return;
  pcl::PCLPointCloud2 pcl_pointcloud2_intermediate;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl_conversions::toPCL(*point_cloud_msg, pcl_pointcloud2_intermediate);
  pcl::fromPCLPointCloud2(pcl_pointcloud2_intermediate, *pcl_point_cloud);
  // source: in velodyne coords; target: in world coords
  pcl::transformPointCloud(*pcl_point_cloud, *pcl_point_cloud,
                           world__eigenT__velodyne.matrix());

  // TODO: ues point cloud and robot pose to update occupancy grid
  mapPointCloudToCostmap(pcl_point_cloud);
}

// TODO: Use localization method to replace the gazebo magic
void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr msg) {
  // if not inited from gazebo magic: go through gazebo model names one by one
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

    if (!ros::param::has("/robot_wheel_radius")) {
      ROS_ERROR("Failed to read rosparam `/robot_wheel_radius`");
      return;
    }
    float robot_wheel_radius = -1.0f;
    ros::param::get("/robot_wheel_radius", robot_wheel_radius);
    if (!ros::param::has("/robot_wheel_radius")) {
      ROS_ERROR("Failed to read rosparam `/robot_wheel_radius`");
      return;
    }
    assert(robot_wheel_radius > 0.0f);
    float robot_mobility_height_ratio = -1.0f;
    ros::param::get("/robot_mobility_height_ratio",
                    robot_mobility_height_ratio);
    assert(robot_mobility_height_ratio > 0.0f);
    robot_mobility_max_height =
        robot_wheel_radius * robot_mobility_height_ratio;
    assert(robot_mobility_max_height > 0.0f);

    for (int i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == robot_name) {
        robot_state_index = i;
      }
    }
    assert(robot_state_index >= 0);

    robot_state.model_name = robot_name;
    robot_state.reference_frame = "/world";

    robot_state_inited = true;
  }

  robot_state.pose = msg->pose.at(robot_state_index);
  robot_state.twist = msg->twist.at(robot_state_index);

  Eigen::Isometry3d world__eigenT__base_link;
  tf::poseMsgToEigen(robot_state.pose, world__eigenT__base_link);
  world__eigenT__velodyne =
      world__eigenT__base_link * base_link__eigenT__velodyne;
}

/*
  Main function
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "farmland_mapping_node");
  ros::NodeHandle nh;

  if (!initOccupancyGrid()) {
    ROS_ERROR("Failed to initialize occupancy grid");
    return -1;
  }
  if (!initRobotTF()) {
    ROS_ERROR("Failed to initialize robot tf");
    return -1;
  }

  ros::Subscriber velodyne_point_cloud_sub =
      nh.subscribe("/points", 100, velodynePointCloudCallback);
  ros::Subscriber robot_state_sub =
      nh.subscribe("/gazebo/model_states", 100, modelStatesCallback);

  ros::Rate loop_rate(100);
  ROS_INFO("In farmland_mapping_node: looping starts\n");
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}