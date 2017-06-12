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

// Initiate the covariance matrices
Eigen::Matrix3d P = Eigen::Matrix3d::Zero(); // Pose covariance
Eigen::Matrix2d Q = Eigen::Matrix2d::Zero(); // Control covariance
Eigen::Matrix3d Jx = Eigen::Matrix3d::Zero(); // Jacobian for pose
Eigen::MatrixXd Ju = Eigen::MatrixXd::Zero(3,2); // Jacobian for control


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

    // Give values to covariance matrices
    // State
    P(0,0) = 0.1;
    P(1,1) = 0.1;
    P(2,2) = 0.1;

    // Control
    Q(0,0) = 0.1;
    Q(1,1) = 0.1;


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
    double left_dtheta, right_dtheta, left_ds, right_ds; // Number of radians/meters passed since last instance
    left_dtheta = get_joint_diff(current_state, old_state, left);
    right_dtheta = get_joint_diff(current_state, old_state, right);
    left_ds = left_dtheta*WHEEL_RADIUS;
    right_ds = right_dtheta*WHEEL_RADIUS;



    if (!first_time) // Don't execute this loop first time 
    {
        double ds = (right_ds + left_ds)/2;
        double dtheta = (right_ds - left_ds)/WHEEL_DISTANCE;
        double dt = get_time_diff(current_state, old_state);

        // Update values based on odometry
        x = x + ds*cos(theta + dtheta/2);
        //x = x + ds*cos(theta);
        y = y + ds*sin(theta + dtheta/2);
        //y = y + ds*sin(theta);
        theta = theta + dtheta;
        // Adjust dtheta to be within -pi to pi interval
        // This introduces truncation errors, works when time interval is small
        if (theta < -PI)
        {
            theta = PI;
        } else if (theta > PI) {
            theta = -PI;
        }

        // Update covariances
        // Precalculate values
        double _s, _c;
        _s = sin(theta + dtheta/2);
        _c = cos(theta + dtheta/2);

        // State
        Jx(0,0) = 1;
        Jx(1,1) = 1;
        Jx(2,2) = 1;
        Jx(0,2) = -ds*_s;
        Jx(1,2) = ds*_c;

        // Control
        Ju(0,0) = _c;
        Ju(0,1) = -ds/2*_s;
        Ju(1,0) = _s;
        Ju(1,1) = ds/2*_c;
        Ju(2,0) = 0;
        Ju(2,1) = 1;

        // Make the update
        P = Jx*P*Jx.transpose() + Ju*Q*Ju.transpose();

        ROS_INFO("Time passed: %f", dt);
        ROS_INFO("dS: %f", ds);
        ROS_INFO("dtheta: %f", dtheta);
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
    return (-1)*(WHEEL_RADIUS/WHEEL_DISTANCE*(w_right - w_left)/dt);
}
