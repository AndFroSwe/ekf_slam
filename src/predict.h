/* File: predict.h
 * Author: andfro
 * Description: Header file for prediction node.
 */

// Defs
#define RATE 1 // Update rate of prediction node
// Includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"

// Function prototypes
void joint_callback(const sensor_msgs::JointState &);
inline double get_joint_diff(const sensor_msgs::JointState &, const sensor_msgs::JointState &, int);
inline double get_time_diff(const sensor_msgs::JointState &, const sensor_msgs::JointState &);
