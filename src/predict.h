/* File: predict.h
 * Author: andfro
 * Description: Header file for prediction node.
 */

// Defs
#define RATE 10 // Update rate of prediction node
#define WHEEL_RADIUS 0.035 // Radius of wheels on vehicle
#define WHEEL_DISTANCE 0.23 // Radius of wheels on vehicle
#define PI 3.141592654
// Includes
// ROS
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "tf/LinearMath/Quaternion.h"
#include "ekf_slam/Pose2DWithCovariance.h"
// Generic
#include <math.h>
#include <Eigen/Dense>


// Function prototypes
void joint_callback(const sensor_msgs::JointState &);
inline double get_joint_diff(const sensor_msgs::JointState &, const sensor_msgs::JointState &, int);
inline double get_time_diff(const sensor_msgs::JointState &, const sensor_msgs::JointState &);
double trans_speed_from_wheeldistance(const double w_left, const double w_right, double dt);
double rot_speed_from_wheeldistance(const double w_left, const double w_right, double dt);
