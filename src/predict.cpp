/*
 * TODO: Motion model
 * TODO: Time discrepancies
 */

/*
 * EKF SLAM prediction step.
 * Takes old mean and covariance as input and outputs new estimated mean
 * and covariance. 
 */

#include "predict.h"

// Globals
sensor_msgs::JointState old_state, current_state; // Save the joint state data. Need to be global for callback function
double left_wheel_speed, right_wheel_speed; // Used for motion model

int main(int argc, char *argv[])
{
    // Parameters
    const char node_name[] = "ekf_slam_prediction";
    const char joint_topic[] = "/joint_states"; // Topic to listen to for joint states

    // Init ROS
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    ROS_INFO("[EKF SLAM] Node %s started", node_name);

    // Create subscriber to states 
    ros::Subscriber joint_sub = n.subscribe(joint_topic, 1, joint_callback); // Subscribes to the latest joint state

    // Create empty joint state. Needed for first execution
    current_state.header.stamp = ros::Time::now(); // This isn't correct since time in message is Gazebo time. Who cares.
    current_state.name.resize(2);
    current_state.name[0] = "left_dummy";
    current_state.name[1] = "right_dummy";
    current_state.position.resize(2);
    current_state.position[0] = 0;
    current_state.position[1] = 0;

    // Main loop
    ros::Rate r(RATE); // Update rate
    while(ros::ok())
    {
        ros::spinOnce(); // Get messages

        r.sleep();
    }

    return 0;
}

void joint_callback(const sensor_msgs::JointState &msg)
{
    // enum for getting left and right
    // This is used for more intuitive indexing of joint state message
    typedef enum wheel_index {
        left = 0,
        right = 1
    } Wheel_index;
    // Update states
    old_state = current_state;
    current_state = msg;

    // Get wheel position difference
    double left_dtheta, right_dtheta; // Number of radians passed since last instance
    left_dtheta = get_joint_diff(current_state, old_state, left);
    right_dtheta = get_joint_diff(current_state, old_state, right);

    // Get time diff
    double dt = get_time_diff(current_state, old_state);

    // Get rotational speed
    ROS_INFO("Time passed: %f", dt);
    ROS_INFO("Left wheel speed: %f", left_dtheta/dt);
    ROS_INFO("right wheel speed: %f", right_dtheta/dt);
}

inline double get_joint_diff(const sensor_msgs::JointState &curr, const sensor_msgs::JointState &old, int i)
{
    return (double) (curr.position[i] - old.position[i]);
}

inline double get_time_diff(const sensor_msgs::JointState &curr, const sensor_msgs::JointState &old)
{

    double curr_sec, old_sec; // Seconds for each state 
    curr_sec = curr.header.stamp.sec + curr.header.stamp.nsec*1e-9; // Convert time to seconds
    old_sec = old.header.stamp.sec + old.header.stamp.nsec*1e-9; // Convert time to seconds
    return (curr_sec - old_sec);
}
