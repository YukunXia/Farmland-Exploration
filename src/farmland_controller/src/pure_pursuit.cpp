#include <farmland_controller/pure_pursuit.h>
#define PP PurePursuit
namespace farmland_controller {
    float PP::euclideanDistance2d(geometry_msgs::Point a, geometry_msgs::Point b) {
        return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
    }

    /**
     * Extract yaw from a pose
     */
    float PP::yawFromPose(geometry_msgs::Pose &pose) {
        static tf::Quaternion quaternion;
        static tf::Matrix3x3 rot_matrix;
        static double roll, pitch, yaw;

        // Get robot state
        tf::quaternionMsgToTF(pose.orientation, quaternion);
        rot_matrix.setRotation(quaternion);
        rot_matrix.getRPY(roll, pitch, yaw);
        return yaw;
    }

    /**
     * Gets the lookahead distance for the robot
     */
    float PP::getLookAheadDistance(float robot_speed, float min_ld, float max_ld,
                            float k_dd) {
        float ld = robot_speed * k_dd;
        if (ld < min_ld) ld = min_ld;
        if (ld > max_ld) ld = max_ld;
        return ld;
    }

    /**
     * Gets a lookahead point
     * TODO: Implement function stub
     */
    geometry_msgs::Point PP::getTargetPoint(geometry_msgs::Pose &robot_pose,
                                        const nav_msgs::Path &path, float ld) {
        geometry_msgs::Point point;
        return point;
    }

    /**
     * Gets the heading delta needed to have the robot face the target point
     * TODO: Implement function stub
     */
    float PP::getHeadingDelta(geometry_msgs::Pose &robot_pose,
                        geometry_msgs::Point target_point) {
        float yaw = yawFromPose(robot_pose);
        float delta_y = robot_pose.position.y - target_point.y;
        float delta_x = robot_pose.position.x - target_point.x;
        float angleFromRobotToTarget = atan2(delta_y,delta_x);
        float heading_delta = angleFromRobotToTarget - yaw;
        return heading_delta;
    }

    /**
     * Gets the turning radius needed to reach target point
     */
    float PP::getTurningRadius(float dist_to_target_point, float heading_delta) {
        return 0;
    }

    /**
     * Gets the twist (translational velocity and angular velocity) from a turning
     * radius
     */
    geometry_msgs::Twist PP::getTwist(float radius, float target_speed) {
        geometry_msgs::Twist twist;
        return twist;
    }               // eg. ld = clip(k_dd * speed, min_ld, max_ld)
}