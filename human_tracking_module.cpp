#include "human_tracking_module.h"

// get information about the security buttons of the remote control
void dead_man_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    // dead_man = true : not safe
    dead_man = msg->data;
    if (dead_man == true) {
        counter_dead_man++;
    }else{
        counter_dead_man = 0;
    }
    // if there is no security permission for a long time, restart the system
    if (counter_dead_man > 10){
        human_odom_init = false;
        kf_init = false;
        robot_odom_init = false;
        counter_dbl =0;
    }
}

// get the pose information of the robot
void robot_pose_odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Get the odometer data of the robot (x,y,yaw)
  robot_odom_init = true;
  robot_x_odom = msg->pose.pose.position.x;
  robot_y_odom = msg->pose.pose.position.y;

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
  double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
  robot_yaw_odom = std::atan2(siny_cosp, cosy_cosp);
}


// extract human legs from point cloud and then estimate human pose
void hokuyo_sacn_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
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
        for (int i = (((msg->angle_max - msg->angle_min) / LaserScan_rad_range) - 1 ) / ((msg->angle_max - msg->angle_min) / LaserScan_rad_range) / 2 * LaserScan_size; i < (1 - (((msg->angle_max - msg->angle_min) / LaserScan_rad_range) - 1 ) / ((msg->angle_max - msg->angle_min) / LaserScan_rad_range) / 2) * LaserScan_size; ++i){
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
                    nlopt_set_max_objective(opt, leg_position_opt_function, NULL);

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
                double yaw_human_relative_1 = heading_calibration(yaw_human_relative - robot_yaw_odom);
                double yaw_human_relative_2 = heading_calibration(yaw_human_relative - robot_yaw_odom - PI);

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
                double delta_polar_angle                    = heading_calibration(polar_angle_odom - polar_angle_between_legs_odom);
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
            double human_orientation_increment = human_orientation_estimation(distance_between_legs_x_human, distance_between_legs_y_human, tolerance_x, tolerance_y, tolerence_turn);

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

            variance_distance_between_legs_y_history = getVariacnce(distance_between_legs_y_history, 0, 29);
            double instruct_statique_1  = abs(left_leg_measure_x - getAverage(left_leg_measure_x_history, 0, 29));
            double instruct_statique_2  = abs(left_leg_measure_y - getAverage(left_leg_measure_y_history, 0, 29));
            double instruct_statique_3  = abs(right_leg_measure_x - getAverage(right_leg_measure_x_history, 0, 29));
            double instruct_statique_4  = abs(right_leg_measure_y - getAverage(right_leg_measure_y_history, 0, 29));
            double instruct_statique    = pow(instruct_statique_1+ instruct_statique_2 +instruct_statique_3+instruct_statique_4, 2) * 10;

            double gap_with_static_orientation_1 = heading_calibration(atan2(left_leg_measure_y - right_leg_measure_y, left_leg_measure_x - right_leg_measure_x) + PI/2 - pseudo_human_yaw_odom);
            double gap_with_static_orientation_2 = heading_calibration(atan2(left_leg_measure_y - right_leg_measure_y, left_leg_measure_x - right_leg_measure_x) - PI/2 - pseudo_human_yaw_odom);
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
            double gap_with_velocity_orientation_1 = heading_calibration(atan2(human_vel_y_odom, human_vel_x_odom) - pseudo_human_yaw_odom);
            double gap_with_velocity_orientation_2 = heading_calibration(atan2(human_vel_y_odom, human_vel_x_odom) + PI - pseudo_human_yaw_odom);
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
                nlopt_set_max_objective(opt, gait_parameters_opt_function, NULL);

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

            human_yaw_odom_for_human_following_module = heading_calibration(new_moyenne_yaw_history);

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


int main(int argc, char **argv){

    ros::init(argc, argv, "human_tracking_module");

    ros::NodeHandle n;
  
    ros::Subscriber sub_1 = n.subscribe("/hokuyo1_laser/scan", 20, hokuyo_sacn_Callback);
    ros::Subscriber sub_2 = n.subscribe("/odom", 150, robot_pose_odom_Callback);
    ros::Subscriber sub_3 = n.subscribe("/dead_man", 150, dead_man_Callback);
    //ros::Subscriber sub_4 = n.subscribe("/vrpn_client_node/summit_xl/pose", 1000, velCallback1);
    //ros::Subscriber sub_5 = n.subscribe("/vrpn_client_node/human/pose", 1000, velCallback2);
    ros::Publisher odom_human_for_human_following_module_ = n.advertise<nav_msgs::Odometry>("/odom_human_for_human_following_module", 50);
    ros::Rate loop_rate(150);

    while (ros::ok()){
        odom_human_for_human_following_module_.publish(odom_human_for_human_following_module);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


