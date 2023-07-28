#ifndef _FRONTAL_HUMAN_FOLLOWING_H
#define _FRONTAL_HUMAN_FOLLOWING_H

#include "ros/ros.h"
// topics
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>

using namespace std;

#define INF     (1.0/0.0)
#define PI      3.14159265

geometry_msgs::Twist vel_robot_real_;                                                                                               // velocity output for robot
double safety_radius = 1.0;                                                                                                         // safety zone radius
double vel_robot_linear_max_x = 0.7,    vel_robot_linear_max_y = 0.7,   vel_robot_angular_max_z = 0.9;                              // velocity limit max
double vel_robot_linear_min_x = 0.01,   vel_robot_linear_min_y = 0.01,  vel_robot_angular_min_z = 0.01;                             // velocity limit min
double human_x_odom, human_y_odom, human_yaw_odom;                                                                                  // human pose in odom
double human_vel_x_odom, human_vel_y_odom, human_vel_yaw_odom;                                                                      // human velocity in odom
double human_x_robot, human_y_robot, human_yaw_robot;                                                                               // human pose in robot coordinate
double robot_x_odom, robot_y_odom, siny_cosp, cosy_cosp, robot_yaw_odom;                                                            // robot pose in odom
double robot_x_human, robot_y_human, robot_yaw_human;                                                                               // robot pose in human coordinate
double gain_repulsive =10, gain_attractive = 1;                                                                                     // gain of Artificial Potential Field
double force_repulsive_x, force_repulsive_y, force_attractive_x, force_attractive_y, force_total_x, force_total_y;                  // force of Artificial Potential Field
double reference_velocity_magnitude, velocity_magnitude_real;                                                                       // velocity magnitude
double gain_p_yaw, gain_p, gain_d;                                                                                                  // gain pd
double vel_robot_recommand_by_velocity_field_x, vel_robot_recommand_by_velocity_field_y, vel_robot_recommand_by_velocity_field_z;   // velocity recommand by velocity field
double vel_robot_real_x, vel_robot_real_y, vel_robot_real_z;                                                                        // velocity applied to robot 
double vel_robot_real_x_past, vel_robot_real_y_past, vel_robot_real_z_past;                                                         // robot past velocity 
double robot_velocity_direction_human, robot_velocity_direction_robot;                                                              // velocity direction in human and robot coordinate
double distance2destination_past, difference_yaw_human_robot_past;
bool move_sideway = false;                                                                                                          // sideways index
bool human_odom_init = false;                                                                                                       // human pose initialization
bool dead_man = true;

#endif


double heading_calibration(double yaw_original){
    while (yaw_original > PI){
        yaw_original = yaw_original - 2 * PI;
    }
    while (yaw_original < - PI){
        yaw_original = yaw_original + 2 * PI;
    }
    return yaw_original;
}

double set_limit_velocity_by_velocity_increment(double vel_input, double vel_limit, double acc){
    if (acc > 0){
        if (vel_input > vel_limit){
            vel_input = vel_limit;
        }
    }else{
        if (vel_input < vel_limit){
            vel_input = vel_limit;
        }
    }
    return vel_input;
}

double set_upper_lower_limits_velocity(double velocity_input, double upper, double lower){
    if(velocity_input > upper){
        velocity_input = upper;
    }else if (velocity_input < - upper){
        velocity_input= - upper;
    }else if (velocity_input< lower && velocity_input > - lower){
        velocity_input = 0;
    }
    return velocity_input;
}


