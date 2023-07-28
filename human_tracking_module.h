#ifndef _HUMAN_TRACKING_MODULE_H
#define _HUMAN_TRACKING_MODULE_H

#include "ros/ros.h"
// topics
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// optimization library
#include <nlopt.h>
// Kalman filter
#include <Eigen/Dense>
#include <Eigen/Core>
#include "Kalman.h"

using namespace std;
using namespace chrono;

#define INF     (1.0/0.0)
#define PI      3.14159265

// leg detection from laser scan
double delta_t_laser                = 0.1;                      // time elapsed between measurements [sec]
double bias_lidar_robot             = 0.045;                    // bias of Lidar and robot center [m]
double laser_range_max              = 4;                        // scan distance range [m]
double LaserScan_rad_range          = PI /1.3;                  // scan angle range [rad]
double LaserScan_filtered_x [512];                              // point cloud in Cartesian coordinate system [m]
double LaserScan_filtered_y [512];                              // point cloud in Cartesian coordinate system [m]
double leg_1_x_odom, leg_1_y_odom, leg_2_x_odom, leg_2_y_odom;  // extracted legs [m]
int LaserScan_size                      = 512;                  // number of point cloud
int LaserScan_filtered_number_clusters  = 1;                    // number of clusters
int number_legs                         = 0;                    // number of legs
int legs_points_1, legs_points_2;                               // number of points associated with the legs
int counter_nombre_clusters;                                    // counter of clusters
int LaserScan_filtered_clusters_start_end [100];                // clusters


// robot state & system monitoring
double robot_x_odom, robot_y_odom, robot_yaw_odom;              // robot pose [m]
bool dead_man               = true;                             // safety buttons on the joystick
bool robot_odom_init        = false;                            // initialization of robot pose
bool human_odom_init        = false;                            // initialization of human pose
bool kf_init                = false;                            // initialization of kalman filter
int counter_no_legs         = 0;                                // duration of undetected legs [0.1 sec]
int counter_dead_man        = 0;                                // duration of safety buttons off [0.1 sec]


// human pose estimation parameters
double left_leg_measure_x, left_leg_measure_y, right_leg_measure_x, right_leg_measure_y;                    // measurements of leg position [m]
double human_x_odom, human_y_odom, human_yaw_odom, human_vel_x_odom, human_vel_y_odom, human_vel_yaw_odom;  // human pose estimation in odom [m] [m/s]
double human_vel_x_human, human_vel_y_human;                                                                // estimation of human forward and lateral movement velocity [m/s]
double human_x_in_robot, human_y_in_robot, human_yaw_in_robot;                                              // human pose estimation in robot coordinate [m]
double center_human_x_measure, center_human_y_measure, pseudo_human_yaw_odom;                               // measurements of human pose [m]                                                  
double left_leg_measure_x_past, left_leg_measure_y_past, right_leg_measure_x_past, right_leg_measure_y_past;// past measurements of leg position [m]
double left_leg_predict_x_odom, left_leg_predict_y_odom, right_leg_predict_x_odom, right_leg_predict_y_odom;// prediction of the position of the legs [m]
double variance_distance_between_legs_y_history;                                                            // sideways index [m^2]
double left_leg_measure_x_history [30], left_leg_measure_y_history [30];                                    // record of left leg position [m]                             
double right_leg_measure_x_history [30], right_leg_measure_y_history [30];                                  // record of right leg position [m]
double distance_between_legs_y_history [30], dis_between_legs [30];                                         // record of distance between two legs [m]
int index_leg_measure_history = 0;                                                                          // index of record
int counter_occu = 0;                                                                                       // duration of one detected leg [0.1 sec]
int counter_dbl = 0;                                                                                        // index of record
bool turn_in_place              = false;                                                                    // motion intention
bool move_sideway               = false;
bool go_straight                = false;
bool go_straight_slight_turn    = false;
bool inconnu_move               = false;
bool stand_still                = false;


// parameters of motion intentions zone
double radius_zone_inconnu          = 0.15;             // radius of the inconnu zone
double step_width                   = 0.216;            // initial step width
double tolerance_x_max              = 0.04;             // tolerance of max step width
double tolerance_x_min              = 0.01;             // tolerance of min step width
double tolerance_y_average          = 0.1;              // tolerance of average move_sideway zone
double tolerence_turn               = 0.04;             // tolerance of turning zone

// parameters of gait
double step_length_history [20];                            // record of step_length [m]
double T_random, R_random, p_random, offset_random;         // Gait parameter random value
double initial_R, initial_T, initial_p, initial_offset;     // Gait parameter initialization value
double tolerance_x, tolerance_y;                            // tolerance of step [m]    
int index_step_length_history   = 0;                        // index of step_length record
int max_number_record           = 20;                       // max number of step_length record


// parameters of Kalman filter
int stateSize                           = 9;                // kalman filter matrix size
int measSize                            = 3;
int controlSize                         = 0;
KalmanFilter        kf(stateSize, measSize, controlSize);   // kalman filter
Eigen::MatrixXd     A(stateSize, stateSize);               
Eigen::MatrixXd     B(0,0);
Eigen::MatrixXd     H(measSize, stateSize);
Eigen::MatrixXd     P(stateSize, stateSize);
Eigen::MatrixXd     R(measSize, measSize);
Eigen::MatrixXd     Q(stateSize, stateSize);
Eigen::VectorXd     x(stateSize);
Eigen::VectorXd     u(0);
Eigen::VectorXd     z(measSize);
Eigen::VectorXd     res(stateSize);
Eigen::VectorXd     res2(stateSize);
double variance_measure_x               = 0.0009;            // measurement noise x
double variance_measure_y               = 0.0009;            // measurement noise y
double variance_measure_yaw             = 0.0441 ;           // measurement noise yaw
double variance_measure_yaw_max         = 0.09 ;             // max yaw measurement noise 
double variance_measure_yaw_min         = 0.0441 ;           // min yaw measurement noise 
double Var_pro_KF                       = 100;               // process noise x,y
double Var_pro_KF_yaw                   = 9.8696;            // process noise yaw


// frontal human following
nav_msgs::Odometry odom_human_for_human_following_module;
double yaw_history [10];
double moyenne_yaw_history_past;
double human_yaw_odom_for_human_following_module;
int counter_yaw_history = 0;

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


double getAverage(double arr[], int start, int end){
    int i = start; 
    double sum = 0;       
    double avg = 0;          

    for (i = start; i < end + 1; ++i){
        sum += arr[i];
    }
 
    avg = sum / (end - start + 1) ;

    return avg;
}

double getVariacnce(double arr[], int start, int end){
    int i = start; 
    double avg = 0;          
    double variance = 0;     

    avg = getAverage(arr, start, end);

    for (i = start; i < end + 1; ++i)
    {
        variance += pow(arr[i] - avg, 2);
    }

    return variance;
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


// human orientation estimation via the relative position of the legs
double human_orientation_estimation(double distance_between_legs_x_human, double distance_between_legs_y_human, double tolerance_x, double tolerance_y,double tolerence_turn){
    //calcul yaw of human
    double yaw_human_change, A, B, C, D, D2, limit_changement, scaling_ratio; 
    limit_changement    = PI/10;
    scaling_ratio       = 0.9;
    A                   = distance_between_legs_x_human;
    B                   = distance_between_legs_y_human;
    C                   = step_width;
    D                   = (B + sqrt(B * B + A * A - C * C)) / (A + C);
    D2                  = (B - sqrt(B * B + A * A - C * C)) / (A + C);

    if (A > tolerance_y){
        //right leg ahead
        if (- B < C + tolerance_x  && - B >  C - tolerance_x){
            //in zone go ahead
            yaw_human_change = 0;
            go_straight = true;
        }else if (- B > C + tolerance_x){
            //in zone turn right and go ahead
            yaw_human_change = 2 * atan (D2) + PI/2;
            go_straight_slight_turn = true;
        }else{
            if ((pow(B,2) + pow(A,2)) > pow(C + tolerence_turn,2)){
                //in zone turn left and go ahead
                yaw_human_change = 2 * atan (D2) + PI / 2;
                go_straight_slight_turn = true;
            }else if (pow(B,2) + pow(A,2) <  pow(radius_zone_inconnu,2)){
                //in zone inconnu
                yaw_human_change = 0;
                inconnu_move = true;
            }else{
                //in zone turn left
                yaw_human_change = atan2(B, A) + PI / 2;
                turn_in_place = true;
            }
        }
    }else if (A < tolerance_y && A > - tolerance_y){
        //in zone step sideway
        yaw_human_change = 0;
        if (- B > C + 0.06  || - B <  C - 0.06){
            if(abs(human_vel_y_human) > 0.1){
                move_sideway = true;
            }else{
                stand_still = true;
            }
        }else{
            stand_still = true;
        }
    }else{
        //left leg ahead
        if (- B < C + tolerance_x && - B > C - tolerance_x){
            //in zone go ahead
            yaw_human_change = 0;
            go_straight = true;
        }else if (- B > C + tolerance_x){
            //in zone turn left and go ahead
            yaw_human_change = 2 * atan (D) + PI / 2;

            go_straight_slight_turn = true;
        }else{
            if ((pow(B,2) + pow(A,2)) > pow(C + tolerence_turn,2)){
                //in zone turn right and go ahead
                yaw_human_change = 2 * atan (D) + PI / 2;
                go_straight_slight_turn = true;
            }else if ((pow(B,2) + pow(A,2)) < pow(radius_zone_inconnu,2)){
                //in zone inconnu
                yaw_human_change = 0;
                inconnu_move = true;
            }else{
                //in zone turn right
                yaw_human_change = atan2(B, A) + PI / 2;
                turn_in_place = true;
            }
        }
    }

    if (isnan(yaw_human_change) || abs(yaw_human_change) > PI / 2){
        yaw_human_change = 0.0;
    }

  return yaw_human_change/2 * 1.2;
}

double leg_position_opt_function(unsigned n, const double *x, double *grad, void *data)
{
    double objectf, objectf_b, objectf_s, e;
    int i = counter_nombre_clusters;
    
    // calcul objectif functon
    objectf = 0;
    for (int k = LaserScan_filtered_clusters_start_end[2 * i]; k < LaserScan_filtered_clusters_start_end[2 * i + 1]; ++k){
      objectf = objectf - sqrt((( LaserScan_filtered_x[k] - x[0])*( LaserScan_filtered_x[k] - x[0]) + (LaserScan_filtered_y[k] - x[1])*(LaserScan_filtered_y[k] - x[1]) - (x[2])*(x[2]))*(( LaserScan_filtered_x[k] - x[0])*( LaserScan_filtered_x[k] - x[0]) + (LaserScan_filtered_y[k] - x[1])*(LaserScan_filtered_y[k] - x[1]) - (x[2])*(x[2])));
    }

    e = 0.00001;

    // calcul gradient x0
    objectf_b = 0;
    objectf_s = 0;
    for (int k = LaserScan_filtered_clusters_start_end[2 * i]; k < LaserScan_filtered_clusters_start_end[2 * i + 1]; ++k){
      objectf_b = objectf_b - sqrt((( LaserScan_filtered_x[k] - (x[0]+e))*( LaserScan_filtered_x[k] - (x[0]+e)) + (LaserScan_filtered_y[k] - x[1])*(LaserScan_filtered_y[k] - x[1]) - (x[2])*(x[2]))*(( LaserScan_filtered_x[k] - (x[0]+e))*( LaserScan_filtered_x[k] - (x[0]+e)) + (LaserScan_filtered_y[k] - x[1])*(LaserScan_filtered_y[k] - x[1]) - (x[2])*(x[2])));
    }
    for (int k = LaserScan_filtered_clusters_start_end[2 * i]; k < LaserScan_filtered_clusters_start_end[2 * i + 1]; ++k){
      objectf_s = objectf_s - sqrt((( LaserScan_filtered_x[k] - (x[0]-e))*( LaserScan_filtered_x[k] - (x[0]-e)) + (LaserScan_filtered_y[k] - x[1])*(LaserScan_filtered_y[k] - x[1]) - (x[2])*(x[2]))*(( LaserScan_filtered_x[k] - (x[0]-e))*( LaserScan_filtered_x[k] - (x[0]-e)) + (LaserScan_filtered_y[k] - x[1])*(LaserScan_filtered_y[k] - x[1]) - (x[2])*(x[2])));
    }
    grad[0]= (objectf_b - objectf_s) / 2 * e;

    // calcul gradient x1
    objectf_b = 0;
    objectf_s = 0;
    for (int k = LaserScan_filtered_clusters_start_end[2 * i]; k < LaserScan_filtered_clusters_start_end[2 * i + 1]; ++k){
      objectf_b = objectf_b - sqrt((( LaserScan_filtered_x[k] - x[0])*( LaserScan_filtered_x[k] - x[0]) + (LaserScan_filtered_y[k] - (x[1] + e))*(LaserScan_filtered_y[k] - (x[1] + e)) - (x[2])*(x[2]))*(( LaserScan_filtered_x[k] - x[0])*( LaserScan_filtered_x[k] - x[0]) + (LaserScan_filtered_y[k] - (x[1] + e))*(LaserScan_filtered_y[k] - (x[1] + e)) - (x[2])*(x[2])));
    }
    for (int k = LaserScan_filtered_clusters_start_end[2 * i]; k < LaserScan_filtered_clusters_start_end[2 * i + 1]; ++k){
      objectf_s = objectf_s - sqrt((( LaserScan_filtered_x[k] - x[0])*( LaserScan_filtered_x[k] - x[0]) + (LaserScan_filtered_y[k] - (x[1] - e))*(LaserScan_filtered_y[k] - (x[1] - e)) - (x[2])*(x[2]))*(( LaserScan_filtered_x[k] - x[0])*( LaserScan_filtered_x[k] - x[0]) + (LaserScan_filtered_y[k] - (x[1] - e))*(LaserScan_filtered_y[k] - (x[1] - e)) - (x[2])*(x[2])));
    }
    grad[1]= (objectf_b - objectf_s) / 2 * e;

    // calcul gradient x2
    objectf_b = 0;
    objectf_s = 0;
    for (int k = LaserScan_filtered_clusters_start_end[2 * i]; k < LaserScan_filtered_clusters_start_end[2 * i + 1]; ++k){
      objectf_b = objectf_b - sqrt((( LaserScan_filtered_x[k] - x[0])*( LaserScan_filtered_x[k] - x[0]) + (LaserScan_filtered_y[k] - (x[1]))*(LaserScan_filtered_y[k] - (x[1])) - (x[2]+ e)*(x[2]+ e))*(( LaserScan_filtered_x[k] - x[0])*( LaserScan_filtered_x[k] - x[0]) + (LaserScan_filtered_y[k] - (x[1]))*(LaserScan_filtered_y[k] - (x[1])) - (x[2]+ e)*(x[2]+ e)));
    }
    for (int k = LaserScan_filtered_clusters_start_end[2 * i]; k < LaserScan_filtered_clusters_start_end[2 * i + 1]; ++k){
      objectf_s = objectf_s - sqrt((( LaserScan_filtered_x[k] - x[0])*( LaserScan_filtered_x[k] - x[0]) + (LaserScan_filtered_y[k] - (x[1]))*(LaserScan_filtered_y[k] - (x[1])) - (x[2]- e)*(x[2]- e))*(( LaserScan_filtered_x[k] - x[0])*( LaserScan_filtered_x[k] - x[0]) + (LaserScan_filtered_y[k] - (x[1]))*(LaserScan_filtered_y[k] - (x[1])) - (x[2]- e)*(x[2]- e)));
    }
    grad[2]= (objectf_b - objectf_s) / 2 * e;
    return objectf;
}

double gait_parameters_opt_function(unsigned n, const double *x, double *grad, void *data)
{
    // calcul objectif functon
    // x : R_rand, T_rand, p_rand, offset_rand
    double objectf, error;
    objectf = 0;

    for (int j = 0; j < max_number_record; ++j){
        if ((index_step_length_history + j) > (max_number_record - 1)){
            error = pow((step_length_history[index_step_length_history + j - max_number_record] - (x[0]) * cos(2*PI*j*delta_t_laser/ (x[1]) + (x[2])) - (x[3])) * sqrt(j), 2);
        }else{
            error = pow((step_length_history[index_step_length_history + j] - (x[0]) * cos(2*PI*j*delta_t_laser/ (x[1]) + (x[2])) - (x[3])) * sqrt(j), 2);
        }
        objectf = objectf - error;
    }

    return objectf;
}

