#ifndef HUMAN_TRACKING_MODULE_H
#define HUMAN_TRACKING_MODULE_H

#include <ros/ros.h>
// topics
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// optimization library
#include <nlopt.h>
// Kalman filter
#include <Eigen/Dense>
#include <Eigen/Core>
#include <frontal_human_following/Kalman.h>

using namespace std;
using namespace chrono;

class HumanTrackingModule {
public:
    HumanTrackingModule();

    void run(); 

private:
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void robotPoseOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void deadManCallback(const std_msgs::Bool::ConstPtr& msg);
    

    // For heading calibration
    static double headingCalibration(double yaw_original);

    // Calculate the average of elements in an array
    static double getAverage(double arr[], int start, int end);

    // Calculate the variance of elements in an array
    static double getVariance(double arr[], int start, int end);


    // For human orientation estimation via the relative position of the legs
    static double humanOrientationEstimation(double distance_between_legs_x_human, double distance_between_legs_y_human, double tolerance_x, double tolerance_y, double tolerance_turn);

    // Function for optimizing leg position
    static double legPositionOptimizationFunction(unsigned n, const double *x, double *grad, void *data);

    // Function for optimizing gait parameters
    static double gaitParametersOptimizationFunction(unsigned n, const double *x, double *grad, void *data);

    ros::Subscriber subLaserScan;
    ros::Subscriber subOdometry;
    ros::Subscriber subDeadMan;
    ros::Publisher pubOdomHuman;
    // Constants
    #define INF (1.0 / 0.0)  // Infinity
    #define PI 3.14159265  // Value of pi

    // Laser Scan
    static double delta_t_laser;
    static double bias_lidar_robot, laser_range_max, laserScan_rad_range;
    // static double LaserScan_filtered_x[512], LaserScan_filtered_y[512];
    static double LaserScan_filtered_x[513], LaserScan_filtered_y[513];
    static double leg_1_x_odom, leg_1_y_odom, leg_2_x_odom, leg_2_y_odom;
    static int LaserScan_size, LaserScan_filtered_number_clusters, number_legs, legs_points_1, legs_points_2;
    static int counter_nombre_clusters, LaserScan_filtered_clusters_start_end[100];

    // Robot & System Monitoring
    static double robot_x_odom, robot_y_odom, robot_yaw_odom;
    static bool dead_man, robot_odom_init, human_odom_init, kf_init;
    static int counter_no_legs, counter_dead_man;

    // Human Pose Estimation
    static double left_leg_measure_x, left_leg_measure_y, right_leg_measure_x, right_leg_measure_y;
    static double human_x_odom, human_y_odom, human_yaw_odom, human_vel_x_odom, human_vel_y_odom, human_vel_yaw_odom;
    static double human_vel_x_human, human_vel_y_human;
    static double human_x_in_robot, human_y_in_robot, human_yaw_in_robot;
    static double center_human_x_measure, center_human_y_measure, pseudo_human_yaw_odom;
    static double left_leg_measure_x_past, left_leg_measure_y_past, right_leg_measure_x_past, right_leg_measure_y_past;
    static double left_leg_predict_x_odom, left_leg_predict_y_odom, right_leg_predict_x_odom, right_leg_predict_y_odom;
    static double variance_distance_between_legs_y_history;
    static double left_leg_measure_x_history[30], left_leg_measure_y_history[30];
    static double right_leg_measure_x_history[30], right_leg_measure_y_history[30];
    static double distance_between_legs_y_history[30], dis_between_legs[30];
    static int index_leg_measure_history, counter_occu, counter_dbl;
    static bool turn_in_place, move_sideway, go_straight, go_straight_slight_turn, inconnu_move, stand_still;

    // Motion Intentions
    static double radius_zone_inconnu, step_width, tolerance_x_max, tolerance_x_min, tolerance_y_average, tolerence_turn;

    // Gait Parameters
    static double step_length_history[20];
    static double T_random, R_random, p_random, offset_random;
    static double initial_R, initial_T, initial_p, initial_offset;
    static double tolerance_x, tolerance_y;
    static int index_step_length_history, max_number_record;


    // Kalman Filter
    static int stateSize, measSize, controlSize;
    static KalmanFilter kf;
    static Eigen::MatrixXd A, B, H, P, R, Q;
    static Eigen::VectorXd x, u, z, res, res2;
    static double variance_measure_x, variance_measure_y, variance_measure_yaw;
    static double variance_measure_yaw_max, variance_measure_yaw_min;
    static double Var_pro_KF, Var_pro_KF_yaw;

    // Frontal Human Following
    static nav_msgs::Odometry odom_human_for_human_following_module;
    static double yaw_history[10], moyenne_yaw_history_past, human_yaw_odom_for_human_following_module;
    static int counter_yaw_history;
};
#endif
