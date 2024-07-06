#ifndef FRONTAL_HUMAN_FOLLOWING_H
#define FRONTAL_HUMAN_FOLLOWING_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>

class FrontalHumanFollowing {
public:
    FrontalHumanFollowing();  // Constructor declaration
    void run();  // Main run function declaration

private:
    void dead_man_Callback(const std_msgs::Bool::ConstPtr& msg);
    void human_pose_odom_Callback(const nav_msgs::Odometry::ConstPtr& msg);
    void robot_pose_odom_Callback(const nav_msgs::Odometry::ConstPtr& msg);
    double heading_calibration(double yaw_original);
    double set_limit_velocity_by_velocity_increment(double vel_input, double vel_limit, double acc);
    double set_upper_lower_limits_velocity(double velocity_input, double upper, double lower);
    ros::Subscriber sub_dead_man;
    ros::Subscriber sub_human_pose;
    ros::Subscriber sub_robot_pose;
    ros::Publisher pub_vel_robot;
    // Constants
    #define INF (1.0 / 0.0)  // Infinity
    #define PI 3.14159265  // Value of pi

    // Global variables
    geometry_msgs::Twist vel_robot_real_;  // Velocity output for the robot
    double safety_radius;  // Safety zone radius
    double vel_robot_linear_max_x, vel_robot_linear_max_y, vel_robot_angular_max_z;  // Maximum velocity limits
    double vel_robot_linear_min_x, vel_robot_linear_min_y, vel_robot_angular_min_z;  // Minimum velocity limits
    double human_x_odom, human_y_odom, human_yaw_odom;  // Human pose in odom
    double human_vel_x_odom, human_vel_y_odom, human_vel_yaw_odom;  // Human velocity in odom
    double human_x_robot, human_y_robot, human_yaw_robot;  // Human pose in robot coordinates
    double robot_x_odom, robot_y_odom, siny_cosp, cosy_cosp, robot_yaw_odom;  // Robot pose in odom
    double robot_x_human, robot_y_human, robot_yaw_human;  // Robot pose in human coordinates
    double gain_repulsive, gain_attractive;  // Gain values for Artificial Potential Field
    double force_repulsive_x, force_repulsive_y, force_attractive_x, force_attractive_y, force_total_x, force_total_y;  // Forces in Artificial Potential Field
    double reference_velocity_magnitude, velocity_magnitude_real;  // Velocity magnitude
    double gain_p_yaw, gain_p, gain_d;  // PD gain values
    double vel_robot_recommand_by_velocity_field_x, vel_robot_recommand_by_velocity_field_y, vel_robot_recommand_by_velocity_field_z;  // Velocity recommended by velocity field
    double vel_robot_real_x, vel_robot_real_y, vel_robot_real_z;  // Velocity applied to the robot
    double vel_robot_real_x_past, vel_robot_real_y_past, vel_robot_real_z_past;  // Past robot velocity
    double robot_velocity_direction_human, robot_velocity_direction_robot;  // Velocity direction in human and robot coordinates
    double distance2destination_past, difference_yaw_human_robot_past;
    bool move_sideway;  // Sideways flag
    bool human_odom_init;  // Human pose initialization flag
    bool dead_man;  // Deadman switch flag
};

#endif


