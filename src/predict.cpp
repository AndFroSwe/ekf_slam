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
double x = 0;
double y = 0;
double theta = 0;

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
} // end main

void joint_callback(const sensor_msgs::JointState &msg)
{
    static bool first_time = true; // Variable to remove first bad reading

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

    if (!first_time) // Don't execute this loop first time 
    {

        // Get time diff
        double dt = get_time_diff(current_state, old_state);


        // Calculate new position from odometry
        double v = trans_speed_from_wheeldistance(left_dtheta, right_dtheta, dt); // Get speed around ICC
        double w = rot_speed_from_wheeldistance(left_dtheta, right_dtheta, dt); // Get rot speed around ICC
        ROS_INFO("Speed v: %f", v);
        ROS_INFO("Rot speed w: %f", w);

        // Update values
        if (abs(w) > 0.001)
        { // Special clause when rotational speed is 0
            x = x - v/w*sin(theta) + v/w*sin(theta + w*dt);
            y = y + v/w*cos(theta) - v/w*cos(theta + w*dt);
            theta = theta + w*dt;
        } else {
            x = x - v*dt*sin(theta) + v*dt*sin(theta + w*dt);
            y = y + v*dt*cos(theta) - v*dt*cos(theta + w*dt);
            theta = theta;
        }

        ROS_INFO("Time passed: %f", dt);
        ROS_INFO("X: %f", x);
        ROS_INFO("Y: %f", y);
        ROS_INFO("Theta: %f", theta);
        ROS_INFO("**************");

    } else {
        first_time = false;
        // This is just temporary to give an estimate to x and y to start from.
        // This should be expanded to take real position as first value.
        x = 0;
        y = 0;
        theta = 0;
    }
} // end callback

inline double get_joint_diff(const sensor_msgs::JointState &curr, const sensor_msgs::JointState &old, int i)
{
    // Help function for getting difference in joint position
    return (double) (curr.position[i] - old.position[i]);
}

inline double get_time_diff(const sensor_msgs::JointState &curr, const sensor_msgs::JointState &old)
{
    // Help function for getting dt from wheel status messages
    double curr_sec, old_sec; // Seconds for each state 
    curr_sec = curr.header.stamp.sec + curr.header.stamp.nsec*1e-9; // Convert time to seconds
    old_sec = old.header.stamp.sec + old.header.stamp.nsec*1e-9; // Convert time to seconds
    return (curr_sec - old_sec);
}

double trans_speed_from_wheeldistance(const double w_left, const double w_right, double dt)
{
    // Calculates momentary speed of vehicle based on wheel odometry of left and right wheels
    // This is taken from kinematic model of vehicle
    return ((w_left + w_right)*WHEEL_RADIUS/2/dt);
}

double rot_speed_from_wheeldistance(const double w_left, const double w_right, double dt)
{
    // Calculates momentary rotational speed of vehicle around Instantaneous Curvature Center
    // based on rotated radians rotated since last time instance w_* and time passed dt [s]
    // This is taken from kinematic model of vehicle
    return (WHEEL_RADIUS/WHEEL_DISTANCE*(w_right - w_left)/dt);
}
