#include "human_tracking_module.h"


// leg detection from laser scan
double HumanTrackingModule::delta_t_laser                = 0.1;                      // time elapsed between measurements [sec]
double HumanTrackingModule::bias_lidar_robot             = 0.045;                    // bias of Lidar and robot center [m]
double HumanTrackingModule::laser_range_max              = 4;                        // scan distance range [m]
double HumanTrackingModule::laserScan_rad_range          = PI /1.3;                  // scan angle range [rad]
double HumanTrackingModule::LaserScan_filtered_x [512];                              // point cloud in Cartesian coordinate system [m]
double HumanTrackingModule::LaserScan_filtered_y [512];                              // point cloud in Cartesian coordinate system [m]
double HumanTrackingModule::leg_1_x_odom, HumanTrackingModule::leg_1_y_odom, HumanTrackingModule::leg_2_x_odom, HumanTrackingModule::leg_2_y_odom;  // extracted legs [m]
int HumanTrackingModule::LaserScan_size                      = 512;                  // number of point cloud
int HumanTrackingModule::LaserScan_filtered_number_clusters  = 1;                    // number of clusters
int HumanTrackingModule::number_legs                         = 0;                    // number of legs
int HumanTrackingModule::legs_points_1, HumanTrackingModule::legs_points_2;                               // number of points associated with the legs
int HumanTrackingModule::counter_nombre_clusters;                                    // counter of clusters
int HumanTrackingModule::LaserScan_filtered_clusters_start_end [100];                // clusters


// robot state & system monitoring
double HumanTrackingModule::robot_x_odom, HumanTrackingModule::robot_y_odom, HumanTrackingModule::robot_yaw_odom;              // robot pose [m]
bool HumanTrackingModule::dead_man               = true;                             // safety buttons on the joystick
bool HumanTrackingModule::robot_odom_init        = false;                            // initialization of robot pose
bool HumanTrackingModule::human_odom_init        = false;                            // initialization of human pose
bool HumanTrackingModule::kf_init                = false;                            // initialization of kalman filter
int HumanTrackingModule::counter_no_legs         = 0;                                // duration of undetected legs [0.1 sec]
int HumanTrackingModule::counter_dead_man        = 0;                                // duration of safety buttons off [0.1 sec]


// human pose estimation parameters
double HumanTrackingModule::left_leg_measure_x, HumanTrackingModule::left_leg_measure_y, HumanTrackingModule::right_leg_measure_x, HumanTrackingModule::right_leg_measure_y;                    // measurements of leg position [m]
double HumanTrackingModule::human_x_odom, HumanTrackingModule::human_y_odom, HumanTrackingModule::human_yaw_odom, HumanTrackingModule::human_vel_x_odom, HumanTrackingModule::human_vel_y_odom, HumanTrackingModule::human_vel_yaw_odom;  // human pose estimation in odom [m] [m/s]
double HumanTrackingModule::human_vel_x_human, HumanTrackingModule::human_vel_y_human;                                                                // estimation of human forward and lateral movement velocity [m/s]
double HumanTrackingModule::human_x_in_robot, HumanTrackingModule::human_y_in_robot, HumanTrackingModule::human_yaw_in_robot;                                              // human pose estimation in robot coordinate [m]
double HumanTrackingModule::center_human_x_measure, HumanTrackingModule::center_human_y_measure, HumanTrackingModule::pseudo_human_yaw_odom;                               // measurements of human pose [m]                                                  
double HumanTrackingModule::left_leg_measure_x_past, HumanTrackingModule::left_leg_measure_y_past, HumanTrackingModule::right_leg_measure_x_past, HumanTrackingModule::right_leg_measure_y_past;// past measurements of leg position [m]
double HumanTrackingModule::left_leg_predict_x_odom, HumanTrackingModule::left_leg_predict_y_odom, HumanTrackingModule::right_leg_predict_x_odom, HumanTrackingModule::right_leg_predict_y_odom;// prediction of the position of the legs [m]
double HumanTrackingModule::variance_distance_between_legs_y_history;                                                            // sideways index [m^2]
double HumanTrackingModule::left_leg_measure_x_history [30], HumanTrackingModule::left_leg_measure_y_history [30];                                    // record of left leg position [m]                             
double HumanTrackingModule::right_leg_measure_x_history [30], HumanTrackingModule::right_leg_measure_y_history [30];                                  // record of right leg position [m]
double HumanTrackingModule::distance_between_legs_y_history [30], HumanTrackingModule::dis_between_legs [30];                                         // record of distance between two legs [m]
int HumanTrackingModule::index_leg_measure_history = 0;                                                                          // index of record
int HumanTrackingModule::counter_occu = 0;                                                                                       // duration of one detected leg [0.1 sec]
int HumanTrackingModule::counter_dbl = 0;                                                                                        // index of record
bool HumanTrackingModule::turn_in_place              = false;                                                                    // motion intention
bool HumanTrackingModule::move_sideway               = false;
bool HumanTrackingModule::go_straight                = false;
bool HumanTrackingModule::go_straight_slight_turn    = false;
bool HumanTrackingModule::inconnu_move               = false;
bool HumanTrackingModule::stand_still                = false;


// parameters of motion intentions zone
double HumanTrackingModule::radius_zone_inconnu          = 0.15;             // radius of the inconnu zone
double HumanTrackingModule::step_width                   = 0.216;            // initial step width
double HumanTrackingModule::tolerance_x_max              = 0.04;             // tolerance of max step width
double HumanTrackingModule::tolerance_x_min              = 0.01;             // tolerance of min step width
double HumanTrackingModule::tolerance_y_average          = 0.1;              // tolerance of average move_sideway zone
double HumanTrackingModule::tolerence_turn               = 0.04;             // tolerance of turning zone

// parameters of gait
double HumanTrackingModule::step_length_history [20];                            // record of step_length [m]
double HumanTrackingModule::T_random, HumanTrackingModule::R_random, HumanTrackingModule::p_random, HumanTrackingModule::offset_random;         // Gait parameter random value
double HumanTrackingModule::initial_R, HumanTrackingModule::initial_T, HumanTrackingModule::initial_p, HumanTrackingModule::initial_offset;     // Gait parameter initialization value
double HumanTrackingModule::tolerance_x, HumanTrackingModule::tolerance_y;                            // tolerance of step [m]    
int HumanTrackingModule::index_step_length_history   = 0;                        // index of step_length record
int HumanTrackingModule::max_number_record           = 20;                       // max number of step_length record


// parameters of Kalman filter
int HumanTrackingModule::stateSize                           = 9;                // kalman filter matrix size
int HumanTrackingModule::measSize                            = 3;
int HumanTrackingModule::controlSize                         = 0;
KalmanFilter        HumanTrackingModule::kf(stateSize, measSize, controlSize);   // kalman filter
Eigen::MatrixXd     HumanTrackingModule::A(stateSize, stateSize);               
Eigen::MatrixXd     HumanTrackingModule::B(0,0);
Eigen::MatrixXd     HumanTrackingModule::H(measSize, stateSize);
Eigen::MatrixXd     HumanTrackingModule::P(stateSize, stateSize);
Eigen::MatrixXd     HumanTrackingModule::R(measSize, measSize);
Eigen::MatrixXd     HumanTrackingModule::Q(stateSize, stateSize);
Eigen::VectorXd     HumanTrackingModule::x(stateSize);
Eigen::VectorXd     HumanTrackingModule::u(0);
Eigen::VectorXd     HumanTrackingModule::z(measSize);
Eigen::VectorXd     HumanTrackingModule::res(stateSize);
Eigen::VectorXd     HumanTrackingModule::res2(stateSize);
double HumanTrackingModule::variance_measure_x               = 0.0009;            // measurement noise x
double HumanTrackingModule::variance_measure_y               = 0.0009;            // measurement noise y
double HumanTrackingModule::variance_measure_yaw             = 0.0441 ;           // measurement noise yaw
double HumanTrackingModule::variance_measure_yaw_max         = 0.09 ;             // max yaw measurement noise 
double HumanTrackingModule::variance_measure_yaw_min         = 0.0441 ;           // min yaw measurement noise 
double HumanTrackingModule::Var_pro_KF                       = 100;               // process noise x,y
double HumanTrackingModule::Var_pro_KF_yaw                   = 9.8696;            // process noise yaw


// frontal human following
nav_msgs::Odometry HumanTrackingModule::odom_human_for_human_following_module;
double HumanTrackingModule::yaw_history [10];
double HumanTrackingModule::moyenne_yaw_history_past;
double HumanTrackingModule::human_yaw_odom_for_human_following_module;
int HumanTrackingModule::counter_yaw_history = 0;


HumanTrackingModule::HumanTrackingModule() {
    // Initialize the ROS node
    ros::NodeHandle nh;

    // Subscribe to ROS topics
    subLaserScan = nh.subscribe("/hokuyo1_laser/scan", 20, &HumanTrackingModule::laserScanCallback, this);
    subOdometry = nh.subscribe("/odom", 150, &HumanTrackingModule::robotPoseOdomCallback, this);
    subDeadMan = nh.subscribe("/dead_man", 150, &HumanTrackingModule::deadManCallback, this);

    // Create ROS topic publisher
    pubOdomHuman = nh.advertise<nav_msgs::Odometry>("/odom_human_for_human_following_module", 50);
}


void HumanTrackingModule::run() {
    ros::Rate loop_rate(150);  // Set the ROS node's loop rate to 150Hz

    while (ros::ok()) {
        // Perform the following actions in each loop iteration

        // 1. Publish the human pose in odom
        pubOdomHuman.publish(odom_human_for_human_following_module);

        // 2. Process subscribed messages
        ros::spinOnce();

        // 3. Allow the node to loop at the specified rate
        loop_rate.sleep();
    }
}


// Implementation of the heading calibration function
double HumanTrackingModule::headingCalibration(double yaw_original) {
    while (yaw_original > PI) {
        yaw_original = yaw_original - 2 * PI;
    }
    while (yaw_original < -PI) {
        yaw_original = yaw_original + 2 * PI;
    }
    return yaw_original;
}

// Implementation of the function to calculate the average
double HumanTrackingModule::getAverage(double arr[], int start, int end) {
    int i = start;
    double sum = 0;
    double avg = 0;

    for (i = start; i < end + 1; ++i) {
        sum += arr[i];
    }

    avg = sum / (end - start + 1);

    return avg;
}

// Implementation of the function to calculate the variance
double HumanTrackingModule::getVariance(double arr[], int start, int end) {
    int i = start;
    double avg = 0;
    double variance = 0;

    avg = getAverage(arr, start, end);

    for (i = start; i < end + 1; ++i) {
        variance += pow(arr[i] - avg, 2);
    }

    return variance;
}


// Implementation of the human orientation estimation function
double HumanTrackingModule::humanOrientationEstimation(double distance_between_legs_x_human, double distance_between_legs_y_human, double tolerance_x, double tolerance_y, double tolerance_turn) {
    double yaw_human_change, A, B, C, D, D2, limit_changement, scaling_ratio;
    limit_changement = PI / 10;
    scaling_ratio = 0.9;
    A = distance_between_legs_x_human;
    B = distance_between_legs_y_human;
    C = step_width;
    D = (B + sqrt(B * B + A * A - C * C)) / (A + C);
    D2 = (B - sqrt(B * B + A * A - C * C)) / (A + C);

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

    if (isnan(yaw_human_change) || abs(yaw_human_change) > PI / 2) {
        yaw_human_change = 0.0;
    }

    return yaw_human_change / 2 * 1.2;
}


// Implementation of leg position optimization function
double HumanTrackingModule::legPositionOptimizationFunction(
    unsigned n, const double *x, double *grad, void *data
) {
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

// Implementation of gait parameters optimization function
double HumanTrackingModule::gaitParametersOptimizationFunction(
    unsigned n, const double *x, double *grad, void *data
) {
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


// get information about the security buttons of the remote control
void HumanTrackingModule::deadManCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // dead_man = true : not safe
    dead_man = msg->data;

    // If the dead-man switch is engaged (not safe), increment a counter.
    if (dead_man == true) {
        counter_dead_man++;
    } else {
        // If the dead-man switch is disengaged (safe), reset the counter to 0.
        counter_dead_man = 0;
    }

    // If there is no security permission for a long time (counter_dead_man exceeds 10),
    // perform the following actions:

    if (counter_dead_man > 10) {
        // 1. Set the human_odom_init flag to false.
        human_odom_init = false;

        // 2. Set the kf_init flag to false. This likely refers to the initialization of a Kalman filter.
        kf_init = false;

        // 3. Set the robot_odom_init flag to false.
        robot_odom_init = false;

        // 4. Reset the counter_dbl variable to 0. It's unclear what this counter represents,
        //    but it's being reset when a safety condition is met.
        counter_dbl = 0;
    }
}

// get the pose information of the robot
void HumanTrackingModule::robotPoseOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Indicate that robot odometry information has been received and initialized.
    robot_odom_init = true;

    // Extract the robot's position information (x, y) from the received message.
    robot_x_odom = msg->pose.pose.position.x;
    robot_y_odom = msg->pose.pose.position.y;

    // Extract the robot's yaw (z-axis rotation) from the received message's orientation.
    // This involves some trigonometric calculations to convert the quaternion orientation to yaw.
    double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);

    // Calculate the yaw angle (robot_yaw_odom) using the arctangent of the above values.
    robot_yaw_odom = std::atan2(siny_cosp, cosy_cosp);
}


// extract human legs from point cloud and then estimate human pose
void HumanTrackingModule::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    auto start = system_clock::now(); // timer
    // check robot odom and dead_man_button
    if (robot_odom_init == true && dead_man == true){

        // check scan size
        if (LaserScan_size != msg->ranges.size()){
            cout << "inconsistent scan dimensions" << endl;
        }

        int LaserScan_filtered_number_points = 0; // initialize the number of filtered point clouds
        int LaserScan_filtered_sequence [512]; // index in the raw point cloud

        // convert point cloud to (x,y) according to preset scan range
        for (int i = (((msg->angle_max - msg->angle_min) / laserScan_rad_range) - 1 ) / ((msg->angle_max - msg->angle_min) / laserScan_rad_range) / 2 * LaserScan_size; i < (1 - (((msg->angle_max - msg->angle_min) / laserScan_rad_range) - 1 ) / ((msg->angle_max - msg->angle_min) / laserScan_rad_range) / 2) * LaserScan_size; ++i){
            if (msg->ranges[i] < laser_range_max){
                LaserScan_filtered_x[LaserScan_filtered_number_points] = msg->ranges[i] * cos(((msg->angle_max - msg->angle_min) * i / LaserScan_size + msg->angle_min));
                LaserScan_filtered_y[LaserScan_filtered_number_points] = msg->ranges[i] * sin(((msg->angle_max - msg->angle_min) * i / LaserScan_size + msg->angle_min));
                LaserScan_filtered_sequence[LaserScan_filtered_number_points] = i;
                LaserScan_filtered_number_points++;
            }
        }

        int LaserScan_filtered_number_clusters = 1;// initialize the number of filtered clusters

        LaserScan_filtered_clusters_start_end[0] = 0;// record the start and end points of clusters

        // form clusters based on continuity between adjacent point clouds (similar to density classification)
        for (int i = 0; i < LaserScan_filtered_number_points; ++i){
            if (LaserScan_filtered_sequence[i+1]-LaserScan_filtered_sequence[i] > 2 || sqrt(pow(LaserScan_filtered_x[i+1] - LaserScan_filtered_x[i],2) + pow(LaserScan_filtered_y[i+1] - LaserScan_filtered_y[i],2)) > 0.05) {
                LaserScan_filtered_clusters_start_end[2 * LaserScan_filtered_number_clusters - 1] = i;
                LaserScan_filtered_clusters_start_end[2 * LaserScan_filtered_number_clusters] = i+1;
                LaserScan_filtered_number_clusters++;
            }
        }
  
  
        LaserScan_filtered_clusters_start_end[2 * LaserScan_filtered_number_clusters -1] = LaserScan_filtered_number_points;

        int number_legs = 0;// Initialize the number of legs

        // initialize leg position and evaluation of objective function
        double legs_center_x [LaserScan_filtered_number_clusters], legs_center_y [LaserScan_filtered_number_clusters], legs_fx [LaserScan_filtered_number_clusters], legs_R[LaserScan_filtered_number_clusters], legs_ponits[LaserScan_filtered_number_clusters],legs_distange_to_lidar[LaserScan_filtered_number_clusters];

        // search for possible legs
        for (int i = 0; i < LaserScan_filtered_number_clusters; ++i){
            if ((LaserScan_filtered_sequence[LaserScan_filtered_clusters_start_end[2 * i + 1]] - LaserScan_filtered_sequence[LaserScan_filtered_clusters_start_end[2 * i]]) > 7){
      
                // Use the centers of the clusters to give the initial state of the optimization
                double moy_x = getAverage(LaserScan_filtered_x, LaserScan_filtered_clusters_start_end[2 * i], LaserScan_filtered_clusters_start_end[2 * i + 1] - 1);
                double moy_y = getAverage(LaserScan_filtered_y, LaserScan_filtered_clusters_start_end[2 * i], LaserScan_filtered_clusters_start_end[2 * i + 1] - 1);
                double correction_x = moy_x * 0.04 / sqrt(moy_y * moy_y + moy_x * moy_x);
                double correction_y = moy_y * 0.04 / sqrt(moy_y * moy_y + moy_x * moy_x);   
                double initial_state_x = moy_x + correction_x;
                double initial_state_y = moy_y + correction_y;

                if (human_odom_init == true){
                    // calcul the human pose in robot vue
                    human_x_in_robot = -(human_x_odom - robot_x_odom) * cos(-robot_yaw_odom) + (human_y_odom - robot_y_odom) * sin(-robot_yaw_odom);
                    human_y_in_robot = -(human_x_odom - robot_x_odom) * sin(-robot_yaw_odom) - (human_y_odom - robot_y_odom) * cos(-robot_yaw_odom);
                }

                if (pow(human_x_in_robot - initial_state_x - bias_lidar_robot,2) + pow(human_y_in_robot - initial_state_y,2) < pow(0.6, 2) || human_odom_init == false || human_odom_init == true){
        
                    auto start = system_clock::now(); // timer

                    // optimization
                    counter_nombre_clusters = i;
                    double tol=1e-9;
                    double lb[3]={-10,-4,0.03};
                    double ub[3]={+10,+4,0.08};
                    double x[3]={initial_state_x,initial_state_y,0.05};
                    double f_max=-INF;

                    // set up optimizer
                    nlopt_opt opt=nlopt_create(NLOPT_LD_MMA, 3);
                    nlopt_set_maxtime(opt, 0.002);

                    // lower and upper bound
                    nlopt_set_lower_bounds(opt, lb);
                    nlopt_set_upper_bounds(opt, ub);

                    // objective function
                    nlopt_set_max_objective(opt, legPositionOptimizationFunction, NULL);

                    // stopping criterion
                    nlopt_set_xtol_rel(opt, tol);
                    nlopt_set_ftol_rel(opt, tol);
                    nlopt_set_force_stop(opt, tol);

                    // optimize
                    nlopt_result result=nlopt_optimize(opt, x, &f_max);

                    // free
                    nlopt_destroy(opt);

                    // add detected Legs
                    if (abs(f_max) < 0.08){
                        legs_center_x[number_legs] = x[0] + bias_lidar_robot;
                        legs_center_y[number_legs] = x[1];
                        legs_R[number_legs] = x[2];
                        legs_fx[number_legs] = f_max;
                        legs_ponits[number_legs] = LaserScan_filtered_clusters_start_end[2 * i + 1] - LaserScan_filtered_clusters_start_end[2 * i];
                        legs_distange_to_lidar[number_legs] = sqrt(pow(x[0] + bias_lidar_robot,2)+ pow(x[1],2));
                        number_legs++;
                    }else{
                        //std::cout << "not a leg" <<  std::endl;
                    }
                }else{
                    //std::cout << "not a leg" <<  std::endl;
                }
            }else{
                //std::cout << "not a leg" <<  std::endl;
            }
        }

        // first step of selection of legs volunteers 
        int legs_detected = 0, index_leg_1 = 0, index_leg_2 = 0;
        // more than two legs detected
        if (number_legs > 2){
            index_leg_1 = 0;
            index_leg_2 = 0;
            for (int i = 0; i < number_legs; ++i){
                if (pow(legs_center_x[i],2) + pow(legs_center_y[i],2) < pow(legs_center_x[index_leg_1],2) + pow(legs_center_y[index_leg_1],2)){
                    index_leg_1 = i;
                }
            }
            if (index_leg_1 == 0){
                index_leg_2 = 1;
            }

            for (int i = 0; i < number_legs; ++i){
                if (i != index_leg_1){
                    if (pow(legs_center_x[i],2) + pow(legs_center_y[i],2) < pow(legs_center_x[index_leg_2],2) + pow(legs_center_y[index_leg_2],2)){
                        index_leg_2 = i;
                    }
                }
            }

            if((pow(legs_center_x[index_leg_1] - legs_center_x[index_leg_2],2) + pow(legs_center_y[index_leg_1] - legs_center_y[index_leg_2],2)) < 0.6 * 0.6){
                legs_detected = 2;
            }else{
                if (pow(human_x_in_robot - legs_center_x[index_leg_1],2) + pow(human_y_in_robot - legs_center_y[index_leg_1],2) < pow(human_x_in_robot - legs_center_x[index_leg_2],2) + pow(human_y_in_robot - legs_center_y[index_leg_2],2)){
                    if (pow(human_x_in_robot - legs_center_x[index_leg_1],2) + pow(human_y_in_robot - legs_center_y[index_leg_1],2) < 0.4 * 0.4){
                        legs_detected = 1;
                    }else{
                        legs_detected = 0;
                    }
                }else{
                    if (pow(human_x_in_robot - legs_center_x[index_leg_2],2) + pow(human_y_in_robot - legs_center_y[index_leg_2],2) < 0.4 * 0.4){
                        legs_detected = 1;
                        index_leg_1 = index_leg_2;
                    }else{
                        legs_detected = 0;
                    }
                }
            }
        // two legs detected
        }else if (number_legs == 2){
            index_leg_1 = 0;
            index_leg_2 = 1;
            if((pow(legs_center_x[index_leg_1] - legs_center_x[index_leg_2],2) + pow(legs_center_y[index_leg_1] - legs_center_y[index_leg_2],2)) < 0.6 * 0.6){
                legs_detected = 2;
            }else{
                if (pow(human_x_in_robot - legs_center_x[index_leg_1],2) + pow(human_y_in_robot - legs_center_y[index_leg_1],2) < pow(human_x_in_robot - legs_center_x[index_leg_2],2) + pow(human_y_in_robot - legs_center_y[index_leg_2],2)){
                    if (pow(human_x_in_robot - legs_center_x[index_leg_1],2) + pow(human_y_in_robot - legs_center_y[index_leg_1],2) < 0.4 * 0.4){
                        legs_detected = 1;
                    }else{
                        legs_detected = 0;
                    }
                }else{
                    if (pow(human_x_in_robot - legs_center_x[index_leg_2],2) + pow(human_y_in_robot - legs_center_y[index_leg_2],2) < 0.4 * 0.4){
                        legs_detected = 1;
                        index_leg_1 = index_leg_2;
                    }else{
                        legs_detected = 0;
                    }
                }
            }
        // one legs detected
        }else if (number_legs == 1){
            legs_detected = 1;
            index_leg_1 = 0;
        // no legs detected
        }else{
            legs_detected = 0;
        }

        // prepare leg detection position information
        if (legs_detected == 0){
            counter_no_legs++;
            leg_1_x_odom    = 0;
            leg_1_y_odom    = 0;
            leg_2_x_odom    = 0;
            leg_2_y_odom    = 0;
            legs_points_1   = 0;
            legs_points_2   = 0;
            // long duration without leg measurements, tracking module reboot
            if (counter_no_legs > 20){
                human_odom_init = false;
                kf_init = false;
                robot_odom_init = false;
                counter_no_legs = 0;
                cout<< "no legs for long time" << endl;
            }
        }else if (legs_detected == 1){
            counter_no_legs = 0;
            leg_1_x_odom    = robot_x_odom - (legs_center_x[index_leg_1] * cos(robot_yaw_odom) - legs_center_y[index_leg_1] * sin(robot_yaw_odom));
            leg_1_y_odom    = robot_y_odom - (legs_center_x[index_leg_1] * sin(robot_yaw_odom) + legs_center_y[index_leg_1] * cos(robot_yaw_odom));
            leg_2_x_odom    = 0;
            leg_2_y_odom    = 0;
            legs_points_1   = legs_ponits[index_leg_1];
            legs_points_2   = 0;
            //cout<< "one leg detected" << endl;
        }else if (legs_detected == 2){
            counter_no_legs = 0;
            leg_1_x_odom    = robot_x_odom - (legs_center_x[index_leg_1] * cos(robot_yaw_odom) - legs_center_y[index_leg_1] * sin(robot_yaw_odom));
            leg_1_y_odom    = robot_y_odom - (legs_center_x[index_leg_1] * sin(robot_yaw_odom) + legs_center_y[index_leg_1] * cos(robot_yaw_odom));
            leg_2_x_odom    = robot_x_odom - (legs_center_x[index_leg_2] * cos(robot_yaw_odom) - legs_center_y[index_leg_2] * sin(robot_yaw_odom));
            leg_2_y_odom    = robot_y_odom - (legs_center_x[index_leg_2] * sin(robot_yaw_odom) + legs_center_y[index_leg_2] * cos(robot_yaw_odom));
            legs_points_1   = legs_ponits[index_leg_1];
            legs_points_2   = legs_ponits[index_leg_2];
        }


        // initial human pose
        if (human_odom_init == false){
            // initialize the user only if the robot sees both legs
            if (legs_detected == 2){
                human_odom_init = true;
                double yaw_human_relative = atan2(leg_1_y_odom - leg_2_y_odom, leg_1_x_odom - leg_2_x_odom) + PI/2;
                double yaw_human_relative_1 = headingCalibration(yaw_human_relative - robot_yaw_odom);
                double yaw_human_relative_2 = headingCalibration(yaw_human_relative - robot_yaw_odom - PI);

                // assume that the user is standing facing the robot during initialization
                if (abs(yaw_human_relative_1) > abs(yaw_human_relative_2)){
                    human_yaw_odom = yaw_human_relative;
                    right_leg_measure_x = leg_1_x_odom;
                    right_leg_measure_y = leg_1_y_odom;
                    left_leg_measure_x = leg_2_x_odom;
                    left_leg_measure_y = leg_2_y_odom;
                // swap the labels of the legs, if the user's back is turned
                }else{
                    human_yaw_odom = yaw_human_relative - PI;
                    left_leg_measure_x = leg_1_x_odom;
                    left_leg_measure_y = leg_1_y_odom;
                    right_leg_measure_x = leg_2_x_odom;
                    right_leg_measure_y = leg_2_y_odom;
                }
                left_leg_measure_x_past = left_leg_measure_x;
                left_leg_measure_y_past = left_leg_measure_y;
                right_leg_measure_x_past = right_leg_measure_x;
                right_leg_measure_y_past = right_leg_measure_y;
                left_leg_predict_x_odom = left_leg_measure_x;
                left_leg_predict_y_odom = left_leg_measure_y;
                right_leg_predict_x_odom = right_leg_measure_x;
                right_leg_predict_y_odom = right_leg_measure_y;            
            }
        // human pose already initialized
        }else{
            // without leg occlusion
            if (legs_detected == 2){
                counter_occu = 0;
                if (pow(left_leg_measure_x_past - leg_1_x_odom,2) + pow(left_leg_measure_y_past - leg_1_y_odom,2) + pow(right_leg_measure_x_past - leg_2_x_odom,2) + pow(right_leg_measure_y_past - leg_2_y_odom,2)  < pow(right_leg_measure_x_past - leg_1_x_odom,2) + pow(right_leg_measure_y_past - leg_1_y_odom,2) + pow(left_leg_measure_x_past - leg_2_x_odom,2) + pow(left_leg_measure_y_past - leg_2_y_odom,2)){
                    left_leg_measure_x = leg_1_x_odom;
                    left_leg_measure_y = leg_1_y_odom;
                    right_leg_measure_x = leg_2_x_odom;
                    right_leg_measure_y = leg_2_y_odom;
                }else{
                    right_leg_measure_x = leg_1_x_odom;
                    right_leg_measure_y = leg_1_y_odom;
                    left_leg_measure_x = leg_2_x_odom;
                    left_leg_measure_y = leg_2_y_odom;
                }
                // 3 seconds to initialize the person's step width and construct a correspondingly scaled motion intention zone
                if (counter_dbl < 30){
                    dis_between_legs[counter_dbl] = sqrt(pow(left_leg_measure_x-right_leg_measure_x,2) + pow(left_leg_measure_y-right_leg_measure_y,2));
                    counter_dbl ++;
                }
                if (counter_dbl == 30){
                    step_width = getAverage(dis_between_legs,0,29);
                    radius_zone_inconnu = step_width / 0.216 * 0.15;
                }
      
            // one leg occlusion and occlusion processing
            }else if (legs_detected == 1){
                counter_occu++;
                double distance_left_leg_predict2robot      = sqrt(pow(left_leg_predict_x_odom-robot_x_odom,2) + pow(left_leg_predict_y_odom-robot_y_odom,2));
                double distance_right_leg_predict2robot     = sqrt(pow(right_leg_predict_x_odom-robot_x_odom,2) + pow(right_leg_predict_y_odom-robot_y_odom,2));
                double visible_leg_x, visible_leg_y;

                // determine visible legs
                if (distance_left_leg_predict2robot < distance_right_leg_predict2robot){
                    visible_leg_x = left_leg_predict_x_odom;
                    visible_leg_y = left_leg_predict_y_odom;
                }else{
                    visible_leg_x = right_leg_predict_x_odom;
                    visible_leg_y = right_leg_predict_y_odom;
                }

                // determine the occlusion angle
                double polar_angle_odom                     = atan2(visible_leg_y - robot_y_odom, visible_leg_x - robot_x_odom);
                double polar_angle_between_legs_odom        = atan2(right_leg_predict_y_odom - left_leg_predict_y_odom, right_leg_predict_x_odom - left_leg_predict_x_odom);
                double delta_polar_angle                    = headingCalibration(polar_angle_odom - polar_angle_between_legs_odom);
                if (delta_polar_angle > PI / 2){
                    delta_polar_angle = delta_polar_angle - PI;
                }else if(delta_polar_angle < - PI / 2){
                    delta_polar_angle = delta_polar_angle + PI;
                }

                double distance_between_legs_x_occlu        = right_leg_predict_x_odom - left_leg_predict_x_odom;
                double distance_between_legs_y_occlu        = right_leg_predict_y_odom - left_leg_predict_y_odom;
                double distance_between_legs_occlu          = sqrt(pow(distance_between_legs_x_occlu, 2) + pow(distance_between_legs_y_occlu, 2));

                // set up an occlusion zone
                double coeff            = 2 / (1 + pow(2.71828182846,-(counter_occu-1) / 0.5)) - 1;
                double coeff_2          = 2 / (1 + pow(2.71828182846,-(counter_occu-1) / 20)) - 1;
                double coeff_3          = (distance_between_legs_occlu - coeff_2 * (distance_between_legs_occlu - step_width)) / distance_between_legs_occlu;

                // improve the prediction of the previous moment in time
                double distance_between_legs_x_occlu_2 = coeff_3 * (distance_between_legs_x_occlu * cos(-coeff * delta_polar_angle) + distance_between_legs_y_occlu * sin(-coeff * delta_polar_angle));
                double distance_between_legs_y_occlu_2 = coeff_3 * (distance_between_legs_y_occlu * cos(-coeff * delta_polar_angle) - distance_between_legs_x_occlu * sin(-coeff * delta_polar_angle));

                // estimate the approximate position of the other leg based on the visible leg
                if (distance_left_leg_predict2robot < distance_right_leg_predict2robot){
                    right_leg_predict_x_odom     = left_leg_predict_x_odom + distance_between_legs_x_occlu_2;
                    right_leg_predict_y_odom     = left_leg_predict_y_odom + distance_between_legs_y_occlu_2;
                }else{
                    left_leg_predict_x_odom      = right_leg_predict_x_odom - distance_between_legs_x_occlu_2;
                    left_leg_predict_y_odom      = right_leg_predict_y_odom - distance_between_legs_y_occlu_2;
                }

                // second step of data correlation for leg measurements selection
                double distance_left_leg_measure2measure  = sqrt(pow(left_leg_measure_x_past - leg_1_x_odom, 2) + pow(left_leg_measure_y_past - leg_1_y_odom, 2));
                double distance_right_leg_measure2measure = sqrt(pow(right_leg_measure_x_past - leg_1_x_odom, 2) + pow(right_leg_measure_y_past - leg_1_y_odom, 2));
                if (distance_left_leg_measure2measure < distance_right_leg_measure2measure){
                    left_leg_measure_x = leg_1_x_odom;
                    left_leg_measure_y = leg_1_y_odom;
                    double distance_leftleft_leg_measure2predict        = sqrt(pow(left_leg_measure_x - left_leg_predict_x_odom,2) + pow(left_leg_measure_y - left_leg_predict_y_odom,2));
                    double distance_rightleft_leg_measure2predict       = sqrt(pow(left_leg_measure_x - right_leg_predict_x_odom,2) + pow(left_leg_measure_y - right_leg_predict_y_odom,2));
                    if (distance_leftleft_leg_measure2predict < distance_rightleft_leg_measure2predict){
                        right_leg_measure_x = right_leg_predict_x_odom + (left_leg_measure_x - left_leg_predict_x_odom);
                        right_leg_measure_y = right_leg_predict_y_odom + (left_leg_measure_y - left_leg_predict_y_odom);
                    }else{
                        right_leg_measure_x = left_leg_predict_x_odom + (left_leg_measure_x - right_leg_predict_x_odom);
                        right_leg_measure_y = left_leg_predict_y_odom + (left_leg_measure_y - right_leg_predict_y_odom); 
                    }

                }else{
                    right_leg_measure_x = leg_1_x_odom;
                    right_leg_measure_y = leg_1_y_odom;
                    double distance_rightright_leg_measure2predict      = sqrt(pow(right_leg_measure_x - right_leg_predict_x_odom,2) + pow(right_leg_measure_y - right_leg_predict_y_odom,2));
                    double distance_leftright_leg_measure2predict       = sqrt(pow(right_leg_measure_x - left_leg_predict_x_odom,2) + pow(right_leg_measure_y - left_leg_predict_y_odom,2));
                    if (distance_rightright_leg_measure2predict < distance_leftright_leg_measure2predict){
                        left_leg_measure_x = left_leg_predict_x_odom + (right_leg_measure_x - right_leg_predict_x_odom);
                        left_leg_measure_y = left_leg_predict_y_odom + (right_leg_measure_y - right_leg_predict_y_odom);
                    }else{
                        left_leg_measure_x = right_leg_predict_x_odom + (right_leg_measure_x - left_leg_predict_x_odom);
                        left_leg_measure_y = right_leg_predict_y_odom + (right_leg_measure_y - left_leg_predict_y_odom);
                    }
                }
            // no leg detected, using the measurement or the prediction from the previous moment
            }else if (legs_detected == 0){
                // left_leg_measure_x = left_leg_measure_x_past;
                // left_leg_measure_y = left_leg_measure_y_past;
                // right_leg_measure_x = right_leg_measure_x_past;
                // right_leg_measure_y = right_leg_measure_y_past;
                left_leg_measure_x = left_leg_predict_x_odom;
                left_leg_measure_y = left_leg_predict_y_odom;
                right_leg_measure_x = right_leg_predict_x_odom;
                right_leg_measure_y = right_leg_predict_y_odom;
            }
  

            // estimation of human orientation by relative position of legs
            // human position measure
            center_human_x_measure                  = (left_leg_measure_x + right_leg_measure_x) / 2;
            center_human_y_measure                  = (left_leg_measure_y + right_leg_measure_y) / 2;
            double distance_between_legs_x_odom     = right_leg_measure_x - left_leg_measure_x;
            double distance_between_legs_y_odom     = right_leg_measure_y - left_leg_measure_y;
            double distance_between_legs_x_human    = distance_between_legs_x_odom * cos(human_yaw_odom) + distance_between_legs_y_odom * sin(human_yaw_odom);
            double distance_between_legs_y_human    = distance_between_legs_y_odom * cos(human_yaw_odom) - distance_between_legs_x_odom * sin(human_yaw_odom);
            double human_velocity_index             = pow(human_vel_x_odom, 2) + pow(human_vel_y_odom, 2);

            // set the parameters of the motion intention zone based on situation
            if (variance_distance_between_legs_y_history > 0.015 && legs_detected != 1){
                tolerance_x = tolerance_x_min;
                tolerance_y = tolerance_y_average;
            }else{
                tolerance_x = tolerance_x_max;
                tolerance_y = tolerance_y_average;
            }
            if (turn_in_place == true && abs(human_vel_yaw_odom) > 0.6){
                tolerance_x = tolerance_x_min/2;
                tolerance_y = tolerance_y_average/5;
                if (legs_detected == 1){
                    tolerance_x = tolerance_x_max/2;
                    tolerance_y = tolerance_y_average/2;
                }
            }

            // Initializing Human Intention
            turn_in_place = false;
            go_straight = false;
            go_straight_slight_turn = false;
            inconnu_move = false;
            stand_still = false;
            move_sideway = false;

            // calcul human_orientation_increment with motion intention zoning
            double human_orientation_increment = humanOrientationEstimation(distance_between_legs_x_human, distance_between_legs_y_human, tolerance_x, tolerance_y, tolerence_turn);

            // set boundaries for human_orientation_increment
            int ratio = 15;
            if (abs(human_orientation_increment) > PI / 1.5){
                human_orientation_increment = 0; 
            }else if (human_orientation_increment > PI / ratio && human_orientation_increment <= PI / 1.5){
                human_orientation_increment = PI / ratio; 
            }else if (human_orientation_increment < - PI / ratio && human_orientation_increment >= - PI / 1.5){
                human_orientation_increment = - PI / ratio; 
            }
            if (isnan(human_orientation_increment)){
              human_orientation_increment = 0;
            }

            // calcul pseudo human orientaion as measurement
            double pseudo_human_yaw_odom    = human_yaw_odom + human_orientation_increment;

            left_leg_measure_x_past         = left_leg_measure_x;
            left_leg_measure_y_past         = left_leg_measure_y;
            right_leg_measure_x_past        = right_leg_measure_x;
            right_leg_measure_y_past        = right_leg_measure_y;

            // correct pseudo orientation with static orientation when the user stand_still
            // collect 3s historical data to determine human stand_still or not
            left_leg_measure_x_history[index_leg_measure_history] = left_leg_measure_x;
            left_leg_measure_y_history[index_leg_measure_history] = left_leg_measure_y;
            right_leg_measure_x_history[index_leg_measure_history] = right_leg_measure_x;
            right_leg_measure_y_history[index_leg_measure_history] = right_leg_measure_y;
            distance_between_legs_y_history[index_leg_measure_history] = distance_between_legs_y_human;

            index_leg_measure_history++;
            if (index_leg_measure_history > (30 - 1)){
                index_leg_measure_history -= 30;
            }

            variance_distance_between_legs_y_history = getVariance(distance_between_legs_y_history, 0, 29);
            double instruct_statique_1  = abs(left_leg_measure_x - getAverage(left_leg_measure_x_history, 0, 29));
            double instruct_statique_2  = abs(left_leg_measure_y - getAverage(left_leg_measure_y_history, 0, 29));
            double instruct_statique_3  = abs(right_leg_measure_x - getAverage(right_leg_measure_x_history, 0, 29));
            double instruct_statique_4  = abs(right_leg_measure_y - getAverage(right_leg_measure_y_history, 0, 29));
            double instruct_statique    = pow(instruct_statique_1+ instruct_statique_2 +instruct_statique_3+instruct_statique_4, 2) * 10;

            double gap_with_static_orientation_1 = headingCalibration(atan2(left_leg_measure_y - right_leg_measure_y, left_leg_measure_x - right_leg_measure_x) + PI/2 - pseudo_human_yaw_odom);
            double gap_with_static_orientation_2 = headingCalibration(atan2(left_leg_measure_y - right_leg_measure_y, left_leg_measure_x - right_leg_measure_x) - PI/2 - pseudo_human_yaw_odom);
            double yaw_cap_human_real;

            if (abs(gap_with_static_orientation_1) < abs(gap_with_static_orientation_2)){
                yaw_cap_human_real = gap_with_static_orientation_1;
            }else{
                yaw_cap_human_real = gap_with_static_orientation_2;
            }

            // first step correct pseudo orientation
            double static_yaw_ratio = 5;
            pseudo_human_yaw_odom = pseudo_human_yaw_odom + yaw_cap_human_real * 0.3 * (1 / (1 + pow(instruct_statique, 3) * static_yaw_ratio)); 


    
    
            // correct pseudo orientation with velocity direction when the user moves quickly
            double gap_with_velocity_orientation_1 = headingCalibration(atan2(human_vel_y_odom, human_vel_x_odom) - pseudo_human_yaw_odom);
            double gap_with_velocity_orientation_2 = headingCalibration(atan2(human_vel_y_odom, human_vel_x_odom) + PI - pseudo_human_yaw_odom);
            double yaw_cap_human_2_real;

            if (abs (gap_with_velocity_orientation_1) < abs(gap_with_velocity_orientation_2)){
                yaw_cap_human_2_real = gap_with_velocity_orientation_1;
            }else{
                yaw_cap_human_2_real = gap_with_velocity_orientation_2;
            }

            human_vel_x_human = human_vel_x_odom * cos(human_yaw_odom) + human_vel_y_odom * sin(human_yaw_odom);
            human_vel_y_human = human_vel_y_odom * cos(human_yaw_odom) - human_vel_x_odom * sin(human_yaw_odom);

            // correct pseudo orientation with velocity direction
            if (abs(yaw_cap_human_2_real) < PI / 9 && human_velocity_index > 0.08 && move_sideway == false && abs(human_vel_x_human) > 0.2){
                pseudo_human_yaw_odom = pseudo_human_yaw_odom +  yaw_cap_human_2_real * 0.4 * human_velocity_index;
            }






            // initializing the Kalman Filter

            if (kf_init == false){
                double T_1 = pow(delta_t_laser, 1), T_2 = pow(delta_t_laser, 2), T_3 = pow(delta_t_laser, 3), T_4 = pow(delta_t_laser, 4);

                A <<    1,      0,      0,      T_1,    0,      0,      0.5*T_2,    0,          0,
                        0,      1,      0,      0,      T_1,    0,      0,          0.5*T_2,    0,
                        0,      0,      1,      0,      0,      T_1,    0,          0,          0.5*T_2,
                        0,      0,      0,      1,      0,      0,      T_1,        0,          0,
                        0,      0,      0,      0,      1,      0,      0,          T_1,        0,
                        0,      0,      0,      0,      0,      1,      0,          0,          T_1,
                        0,      0,      0,      0,      0,      0,      1,          0,          0,
                        0,      0,      0,      0,      0,      0,      0,          1,          0,
                        0,      0,      0,      0,      0,      0,      0,          0,          1;

                 H <<   1, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 1, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 1, 0, 0, 0, 0, 0, 0;
      
                P.setIdentity();

                if (legs_detected < 2){
                    variance_measure_yaw = variance_measure_yaw_max;
                }else{
                    variance_measure_yaw = variance_measure_yaw_min;
                }

                // measure noise
                R <<    variance_measure_x,     0,                      0, 
                        0,                      variance_measure_y,     0,
                        0,                      0,                      variance_measure_yaw;

                // process noise
                Q <<    0.25*Var_pro_KF*T_4,     0,                       0,                            0.50*Var_pro_KF*T_3,     0,                       0,                            0.50*Var_pro_KF*T_2,   0,                     0,
                        0,                       0.25*Var_pro_KF*T_4,     0,                            0,                       0.50*Var_pro_KF*T_3,     0,                            0,                     0.50*Var_pro_KF*T_2,   0,
                        0,                       0,                       0.25*Var_pro_KF_yaw*T_4,      0,                       0,                       0.50*Var_pro_KF_yaw*T_3,      0,                     0,                     0.50*Var_pro_KF_yaw*T_2,
                        0.50*Var_pro_KF*T_3,     0,                       0,                            Var_pro_KF*T_2,          0,                       0,                            Var_pro_KF*T_1,        0,                     0,
                        0,                       0.50*Var_pro_KF*T_3,     0,                            0,                       Var_pro_KF*T_2,          0,                            0,                     Var_pro_KF*T_1,        0,
                        0,                       0,                       0.50*Var_pro_KF_yaw*T_3,      0,                       0,                       Var_pro_KF_yaw*T_2,           0,                     0,                     Var_pro_KF_yaw*T_1,
                        0.50*Var_pro_KF*T_2,     0,                       0,                            Var_pro_KF*T_1,          0,                       0,                            Var_pro_KF,            0,                     0,
                        0,                       0.50*Var_pro_KF*T_2,     0,                            0,                       Var_pro_KF*T_1,          0,                            0,                     Var_pro_KF,            0,
                        0,                       0,                       0.50*Var_pro_KF_yaw*T_2,      0,                       0,                       Var_pro_KF_yaw*T_1,           0,                     0,                     Var_pro_KF_yaw;


                z.setZero();
                x << center_human_x_measure, center_human_y_measure, pseudo_human_yaw_odom, 0, 0, 0, 0, 0, 0;
                kf.init(x, P, R, Q);
                kf_init = true;

            }
            // update process
            res     << kf.predict(A);
            z       << center_human_x_measure, center_human_y_measure, pseudo_human_yaw_odom;
            res2    << kf.update(H,z);

            human_x_odom        = res2[0];
            human_y_odom        = res2[1];
            human_yaw_odom      = res2[2];
            human_vel_x_odom    = res2[3];
            human_vel_y_odom    = res2[4];
            human_vel_yaw_odom  = res2[5];


            // gait parameters identification
            // collect historical gait data
            double distance_between_legs_x_human_2 = distance_between_legs_x_odom * cos(human_yaw_odom) + distance_between_legs_y_odom * sin(human_yaw_odom);
            double distance_between_legs_y_human_2 = distance_between_legs_y_odom * cos(human_yaw_odom) - distance_between_legs_x_odom * sin(human_yaw_odom);

            step_length_history[index_step_length_history] = (distance_between_legs_x_human_2);
            index_step_length_history++;
            if (index_step_length_history > (max_number_record-1)){
                index_step_length_history -= max_number_record;    
            }

            // using a gait model during the walkibg straight-ahead 
            if (human_velocity_index > 0.15 && abs(human_vel_yaw_odom) < 0.25){
                // initialize the value to be optimized with random way
                int max_iteration   = 200;
                double error        = 0;
                double sum_error    = 0;
                double min_error    = 100000;
                for (int i = 0; i < max_iteration; ++i){
                    T_random        = 1.3 + ((double) rand() / (double)(RAND_MAX))*1.5;
                    R_random        = 0.02+ ((double) rand() / (double)(RAND_MAX))*0.28;
                    p_random        = 0 + ((double) rand() / (double)(RAND_MAX))*2*PI;
                    offset_random   = - 0.2 + ((double) rand() / (double)(RAND_MAX))*0.4;
                    sum_error = 0;
                    for (int j = 0; j < max_number_record; ++j){
                        if ((index_step_length_history + j) > (max_number_record - 1)){
                            error = abs(step_length_history[index_step_length_history + j - max_number_record] - R_random * cos(2*PI*j*delta_t_laser/T_random + p_random) - offset_random);
                        }else{
                            error = abs(step_length_history[index_step_length_history + j] - R_random * cos(2*PI*j*delta_t_laser/T_random + p_random) - offset_random);
                        }
                        error = error * j;
                        sum_error = sum_error + error;
                    }
                    if (sum_error < min_error){
                        min_error       = sum_error;
                        initial_R       = R_random;
                        initial_T       = T_random;
                        initial_p       = p_random;
                        initial_offset  = offset_random;
                    }
                }

                // optimization with NLopt
                double tol      = 1e-9;
                double lb[4]    = {0,0.5,0,-0.5};
                double ub[4]    = {0.4,4,2*PI,0.5};
                double x[4]     = {initial_R,initial_T,initial_p,initial_offset};
                double f_max    = - INF;

                // set up optimizer
                nlopt_opt opt=nlopt_create(NLOPT_LN_COBYLA, 4);

                //nlopt_set_param(opt, "inner_maxeval", 100); NLOPT_LN_COBYLA  // NLOPT_LD_MMA
                nlopt_set_maxtime(opt, 0.0015);

                // lower and upper bound
                nlopt_set_lower_bounds(opt, lb);
                nlopt_set_upper_bounds(opt, ub);

                // objective function
                nlopt_set_max_objective(opt, gaitParametersOptimizationFunction, NULL);

                // stopping criterion
                nlopt_set_xtol_rel(opt, tol);
                nlopt_set_ftol_rel(opt, tol);
                nlopt_set_force_stop(opt, tol);

                // optimize
                nlopt_result result=nlopt_optimize(opt, x, &f_max);

                // free
                nlopt_destroy(opt);

                // prediction step length and relative position of legs
                double step_length_predict = x[0] * cos(2 * PI * (max_number_record+1) * delta_t_laser / x[1] + x[2]) + x[3];
                left_leg_predict_x_odom    = human_x_odom + human_vel_x_odom * delta_t_laser - sin(human_yaw_odom) * (- step_width/2) + cos(human_yaw_odom) * (step_length_predict)/2;
                left_leg_predict_y_odom    = human_y_odom + human_vel_y_odom * delta_t_laser + cos(human_yaw_odom) * (- step_width/2) + sin(human_yaw_odom) * (step_length_predict)/2;
                right_leg_predict_x_odom   = human_x_odom + human_vel_x_odom * delta_t_laser + sin(human_yaw_odom) * (- step_width/2) - cos(human_yaw_odom) * (step_length_predict)/2;
                right_leg_predict_y_odom   = human_y_odom + human_vel_y_odom * delta_t_laser - cos(human_yaw_odom) * (- step_width/2) - sin(human_yaw_odom) * (step_length_predict)/2;
            }else{
                // legs' position prediction by symmetry
                double distance_between_legs_x_odom_predict = distance_between_legs_x_odom * cos(- human_vel_yaw_odom * delta_t_laser) + distance_between_legs_y_odom * sin(- human_vel_yaw_odom * delta_t_laser);
                double distance_between_legs_y_odom_predict = distance_between_legs_y_odom * cos(- human_vel_yaw_odom * delta_t_laser) - distance_between_legs_x_odom * sin(- human_vel_yaw_odom * delta_t_laser);
                if (left_leg_measure_y < right_leg_measure_y){
                    right_leg_predict_x_odom     = left_leg_measure_x + distance_between_legs_x_odom_predict;
                    right_leg_predict_y_odom     = left_leg_measure_y + distance_between_legs_y_odom_predict;
                    left_leg_predict_x_odom      = left_leg_measure_x;
                    left_leg_predict_y_odom      = left_leg_measure_y;
                }else{
                    left_leg_predict_x_odom      = right_leg_measure_x - distance_between_legs_x_odom_predict;
                    left_leg_predict_y_odom      = right_leg_measure_y - distance_between_legs_y_odom_predict;
                    right_leg_predict_x_odom     = right_leg_measure_x;
                    right_leg_predict_y_odom     = right_leg_measure_y;
                }
            }


            if (counter_yaw_history < 10){
                yaw_history[counter_yaw_history] = human_yaw_odom;
            }else{
                counter_yaw_history = counter_yaw_history - 10;
                yaw_history[counter_yaw_history] = human_yaw_odom;
            }
            counter_yaw_history++;


            double moyenne_yaw_history = getAverage(yaw_history, 0, 9);
          
            double sum_yaw = 0.0;
            int smooth_yaw_counter = 0;

            for (int i = 0; i < 9; ++i){
                if (abs(yaw_history[i] - moyenne_yaw_history) < PI/6){
                    sum_yaw = sum_yaw + yaw_history[i];
                    smooth_yaw_counter++;
                }
            }

            double new_moyenne_yaw_history;

            if (smooth_yaw_counter > 4){
                new_moyenne_yaw_history = sum_yaw / smooth_yaw_counter;
            }else{
                new_moyenne_yaw_history = moyenne_yaw_history_past;
            }
            moyenne_yaw_history_past = new_moyenne_yaw_history;

            human_yaw_odom_for_human_following_module = headingCalibration(new_moyenne_yaw_history);

            // set odom human 
            odom_human_for_human_following_module.header.stamp                 = ros::Time::now();
            odom_human_for_human_following_module.header.frame_id              = "odom_human_for_human_following_module";
            odom_human_for_human_following_module.pose.pose.position.x         = human_x_odom;
            odom_human_for_human_following_module.pose.pose.position.y         = human_y_odom;
            odom_human_for_human_following_module.pose.pose.orientation.z      = human_yaw_odom_for_human_following_module;
            odom_human_for_human_following_module.twist.twist.linear.x         = human_vel_x_odom;
            odom_human_for_human_following_module.twist.twist.linear.y         = human_vel_y_odom;
            odom_human_for_human_following_module.twist.twist.angular.z        = human_vel_yaw_odom;
            odom_human_for_human_following_module.twist.twist.linear.z         = move_sideway;
          

            double left_x_in_robot     = -(human_x_odom - robot_x_odom) * cos(-robot_yaw_odom)   + (human_y_odom - robot_y_odom) * sin(-robot_yaw_odom);
            double left_y_in_robot     = -(human_x_odom - robot_x_odom) * sin(-robot_yaw_odom)   - (human_y_odom - robot_y_odom) * cos(-robot_yaw_odom);

            static tf::TransformBroadcaster br_41;
            tf::Transform transform_41;
            transform_41.setOrigin( tf::Vector3(left_x_in_robot, left_y_in_robot, 0.0) );
            tf::Quaternion q_41;
            q_41.setRPY(0, 0, human_yaw_odom-robot_yaw_odom);
            transform_41.setRotation(q_41);
            br_41.sendTransform(tf::StampedTransform(transform_41, ros::Time::now(), "hokuyo1_laser_link", "human"));

            if (isnan(human_orientation_increment)|| isnan(human_yaw_odom)){
                cout<<"bad calcul"<< endl;  
            }

        }
    }
    auto end        = system_clock::now();
    auto duration   = duration_cast<microseconds>(end - start);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "human_tracking_module");

    // Create a FrontalHumanFollowing object
    HumanTrackingModule HumanTrackingModule;

    // Call the run method to start the main loop of the ROS node
    HumanTrackingModule.run();

    return 0;
}
