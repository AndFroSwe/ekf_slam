/* File: predict.cpp
 * Author: andfro
 * Description: Update step of an EKF on a differential drive robot.
 * This node uses the Eigen Matrix package for calculation.
 */

#include "update.h"


// Globals
ekf_slam::Pose2DWithCovariance pose;

// Function prototypes
void update_callback(const ekf_slam::Pose2DWithCovariance &msg)
{
    // Callback updates the value
    pose.pose.x = msg.pose.x;
    pose.pose.y = msg.pose.y;
    pose.pose.theta = msg.pose.theta;
    pose.covariance = msg.covariance;
}

int main(int argc, char *argv[])
{
    // Parameters
    char node_name[] = "ekf_slam_update";
    char update_sub_topic[] = "/predicted_z";

    // Initiate ROS
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    ROS_INFO("[EKF_SLAM] Node %s started", node_name);

    // Listeners
    ros::Subscriber update_sub = n.subscribe(update_sub_topic, 1, update_callback);

    // Main loop
    ros::Rate r(1.0);
    while(ros::ok())
    {
        ros::spinOnce(); // Get message data

        // Kalman update step
        // Naming from Thrun 2005
        // Loop through all observed landmarks
        // Get landmark data
        double m_x = 0.0; // Landmark x position
        double m_y = 0.0; // Landmark y position
        double m_r = 0.0; // Distance to landmark
        double m_theta = 0.0; // Angle to landmark
        double m_s = 1; // Landmark id

        // Pre calculations
        double q = pow((m_y + pose.pose.y), 2) + pow((m_x + pose.pose.x), 2);

        // Create measurement
        struct _z {
            double R;
            double THETA;
            double S;
        } z;

        z.R = sqrt(q); // Distance to landmark
        z.THETA = atan2(m_y - pose.pose.y, m_x - pose.pose.x) - pose.pose.theta; // Angle to landmark in local frame
        z.S = m_s;

        // Calculate Jacobian
        double _x = (m_x - pose.pose.x);
        double _y = (m_y - pose.pose.y);
        Eigen::Matrix3d J_H; // Create Jacobian
        J_H(0,0) = -_x/sqrt(q);
        J_H(0,1) = -_y/sqrt(q);
        J_H(0,2) = 0;
        J_H(1,0) = _y/q;
        J_H(1,1) = -_x/q;
        J_H(1,2) = -1;
        J_H(2,0) = 0;
        J_H(2,1) = 0;
        J_H(2,2) = 0;

        // Add covariance from pose into matrix
        Eigen::Matrix3d sigma;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                sigma(row, col) = pose.covariance[col + 3*row];
            }
        }

        // Measurement covariance
        Eigen::Matrix3d Q = Eigen::Matrix3d::Zero(); // Contruct with diagonal elements
        Q(0, 0) = 0.1;
        Q(1, 1) = 0.1;
        Q(2, 2) = 0.1;

        // Calculate Kalman gain
        Eigen::Matrix3d S = J_H*sigma*J_H.transpose() + Q; // Kalman gain nominator
        Eigen::Matrix3d K = J_H*sigma*J_H.transpose() * S.inverse();

        // Add landmarks to vectors
        // Observation
        Eigen::Vector3d z_obs;
        z_obs(0) = m_r;
        z_obs(1) = m_theta;
        z_obs(2) = m_s;
        // Estimation
        Eigen::Vector3d z_est;
        z_est(0) = z.R;
        z_est(1) = z.THETA;
        z_est(2) = z.S;


        // Update mean and covariance
        Eigen::Vector3d mu; // Mean vector
        mu(0) = pose.pose.x;
        mu(1) = pose.pose.y;
        mu(2) = pose.pose.theta;
        
        // Integrate estimation and observation
        mu = mu + K*(z_obs - z_est);
        sigma = sigma - K*J_H*sigma;

    }
    return 0;
}
