#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// increase FREQ to have a more accurate integration result
constexpr int FREQ = 200;
constexpr double DELTA_T = 1.0 / double(FREQ);
constexpr double MAX_VX = 2.0;
constexpr double MAX_VYAW = M_PI * 1.0 / 3.0;
constexpr double MAX_AX = 0.5;
constexpr double MAX_AYAW = M_PI * 1.0 / 9.0;

ros::Subscriber gazebo_states_sub;
ros::Publisher robot_curr_state_pub;
geometry_msgs::TransformStamped robot_odometry_tf;
gazebo_msgs::ModelState robot_target_state;
gazebo_msgs::ModelState robot_curr_state;
double robot_odometry_tf_last_time = -1.0;
bool robot_state_inited = false;
int robot_state_index = -1;
std::string robot_name;

ros::Publisher robot_target_state_pub;
ros::Subscriber robot_cmdvel_sub;
geometry_msgs::Twist robot_cmdvel;
double robot_cmdvel_last_time = -1.0;
// double robot_target_state_pub_last_time = -1.0;
bool robot_cmdvel_inited = false;
constexpr double CMDVEL_TIMEOUT = 0.1;

// TODO: Use localization method to replace the gazebo magic
void gazeboStatesCallback(const gazebo_msgs::ModelStatesConstPtr msg) {
  // if not inited from gazebo magic: go through gazebo model names one by one
  if (!robot_state_inited) {
    if (!ros::param::has("/robot_name")) {
      ROS_ERROR("Failed to read rosparam `/robot_name`");
      return;
    }
    ros::param::get("/robot_name", robot_name);
    assert(!robot_name.empty());

    for (int i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == robot_name) {
        robot_state_index = i;
      }
    }

    if (robot_state_index >= 0) {
      robot_target_state.model_name = robot_name;
      robot_target_state.pose = msg->pose.at(robot_state_index);
      robot_target_state.reference_frame = "world";
      robot_state_inited = true;
    }
  }

  static tf::TransformBroadcaster br;

  ros::Time curr_time = ros::Time::now();
  if (curr_time.toSec() >= robot_odometry_tf_last_time + DELTA_T / 10.0) {
    robot_odometry_tf.header.frame_id = "world";
    robot_odometry_tf.header.stamp = curr_time;
    robot_odometry_tf.child_frame_id = "base_link";
    robot_odometry_tf.transform.rotation =
        msg->pose.at(robot_state_index).orientation;
    robot_odometry_tf.transform.translation.x =
        msg->pose.at(robot_state_index).position.x;
    robot_odometry_tf.transform.translation.y =
        msg->pose.at(robot_state_index).position.y;
    robot_odometry_tf.transform.translation.z =
        msg->pose.at(robot_state_index).position.z;
    br.sendTransform(robot_odometry_tf);

    robot_curr_state.model_name = robot_name;
    robot_curr_state.pose = msg->pose.at(robot_state_index);
    robot_curr_state.reference_frame = "world";
    robot_curr_state.twist = msg->twist.at(robot_state_index);
    robot_curr_state_pub.publish(robot_curr_state);

    robot_odometry_tf_last_time = curr_time.toSec();
  }
}

void robotCmdvelCallback(const geometry_msgs::TwistConstPtr cmd_vel_ptr) {
  robot_cmdvel = *cmd_vel_ptr;

  if (!robot_cmdvel_inited) {
    robot_cmdvel_inited = true;
  }
  ros::Time curr_time = ros::Time::now();
  robot_cmdvel_last_time = curr_time.toSec();
}

void pubRobotState() {
  assert(robot_state_inited && robot_cmdvel_inited);

  // robot only takes velocity of x(front direction) and yaw
  ros::Time curr_time = ros::Time::now();

  double target_vx = 0.0;
  double target_vyaw = 0.0;
  if (curr_time.toSec() < robot_cmdvel_last_time + CMDVEL_TIMEOUT) {
    target_vx = std::max(-MAX_VX, std::min(MAX_VX, robot_cmdvel.linear.x));
    target_vyaw =
        std::max(-MAX_VYAW, std::min(MAX_VYAW, robot_cmdvel.angular.z));
  }

  // real delta t could be close to DELTA_T on performant machines
  // const double real_delta_t =
  //     robot_state_pub_last_time < 0.0
  //         ? DELTA_T
  //         : curr_time.toSec() - robot_state_pub_last_time;
  const double real_delta_t = DELTA_T;

  // use the last pose read from gazebo to estimate the current yaw
  tf::Quaternion q(
      robot_target_state.pose.orientation.x, robot_target_state.pose.orientation.y,
      robot_target_state.pose.orientation.z, robot_target_state.pose.orientation.w);
  const tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // rotate the robot velocity to global velodicy, and accelerate v_x
  const double curr_vx = std::sqrt(std::pow(robot_target_state.twist.linear.x, 2) +
                                   std::pow(robot_target_state.twist.linear.y, 2));
  const bool increase_vx = target_vx > curr_vx;
  double next_vx = curr_vx + (increase_vx ? 1.0 : -1.0) * MAX_AX * DELTA_T;
  if (increase_vx) {
    next_vx = std::min(next_vx, target_vx);
  } else {
    next_vx = std::max(next_vx, target_vx);
  }
  next_vx = std::max(-MAX_VX, std::min(MAX_VX, next_vx));
  robot_target_state.twist.linear.x = next_vx * std::cos(yaw);
  robot_target_state.twist.linear.y = next_vx * std::sin(yaw);
  robot_target_state.pose.position.x += robot_target_state.twist.linear.x * real_delta_t;
  robot_target_state.pose.position.y += robot_target_state.twist.linear.y * real_delta_t;

  // accelerate v_yaw
  const bool increase_vyaw = target_vyaw > robot_target_state.twist.angular.z;
  double next_vyaw = robot_target_state.twist.angular.z +
                     (increase_vyaw ? 1.0 : -1.0) * MAX_AYAW * real_delta_t;
  if (increase_vyaw) {
    next_vyaw = std::min(next_vyaw, target_vyaw);
  } else {
    next_vyaw = std::max(next_vyaw, target_vyaw);
  }
  next_vyaw = std::max(-MAX_VYAW, std::min(MAX_VYAW, next_vyaw));
  robot_target_state.twist.angular.z = next_vyaw;
  double next_yaw = yaw + next_vyaw * real_delta_t;
  q.setRPY(roll, pitch, next_yaw);
  robot_target_state.pose.orientation.x = q.x();
  robot_target_state.pose.orientation.y = q.y();
  robot_target_state.pose.orientation.z = q.z();
  robot_target_state.pose.orientation.w = q.w();

  // assume terrian is flat
  robot_target_state.twist.angular.x = 0.0;
  robot_target_state.twist.angular.y = 0.0;
  robot_target_state.twist.linear.z = 0.0;

  // finally, publish
  robot_target_state_pub.publish(robot_target_state);
  // curr_time = ros::Time::now();
  // robot_target_state_pub_last_time = curr_time.toSec();
}

/*
  Main function
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "differential_drive_husky");
  ros::NodeHandle nh;

  gazebo_states_sub =
      nh.subscribe("/gazebo/model_states", FREQ, gazeboStatesCallback);
  robot_target_state_pub =
      nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);
  robot_curr_state_pub =
      nh.advertise<gazebo_msgs::ModelState>("/husky_curr_state", 100);
  robot_cmdvel_sub = nh.subscribe("/differential_drive_control/cmd_vel", FREQ,
                                  robotCmdvelCallback);

  ros::Rate loop_rate(FREQ);
  ROS_INFO("In differential_drive_husky: looping starts\n");
  while (ros::ok()) {
    ros::spinOnce();

    if (robot_state_inited && robot_cmdvel_inited) {
      pubRobotState();
    }

    loop_rate.sleep();
  }

  return 0;
}