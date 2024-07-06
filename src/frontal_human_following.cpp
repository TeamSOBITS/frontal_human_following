#include "frontal_human_following.h"

// Define and initialize the global variables
geometry_msgs::Twist vel_robot_real_;
double safety_radius = 1.0;
double vel_robot_linear_max_x = 0.7, vel_robot_linear_max_y = 0.7, vel_robot_angular_max_z = 0.9;
double vel_robot_linear_min_x = 0.01, vel_robot_linear_min_y = 0.01, vel_robot_angular_min_z = 0.01;
double human_x_odom, human_y_odom, human_yaw_odom;
double human_vel_x_odom, human_vel_y_odom, human_vel_yaw_odom;
double human_x_robot, human_y_robot, human_yaw_robot;
double robot_x_odom, robot_y_odom, siny_cosp, cosy_cosp, robot_yaw_odom;
double robot_x_human, robot_y_human, robot_yaw_human;
double gain_repulsive = 10, gain_attractive = 1;
double force_repulsive_x, force_repulsive_y, force_attractive_x, force_attractive_y, force_total_x, force_total_y;
double reference_velocity_magnitude, velocity_magnitude_real;
double gain_p_yaw, gain_p, gain_d;
double vel_robot_recommand_by_velocity_field_x, vel_robot_recommand_by_velocity_field_y, vel_robot_recommand_by_velocity_field_z;
double vel_robot_real_x, vel_robot_real_y, vel_robot_real_z;
double vel_robot_real_x_past, vel_robot_real_y_past, vel_robot_real_z_past;
double robot_velocity_direction_human, robot_velocity_direction_robot;
double distance2destination_past, difference_yaw_human_robot_past;
bool move_sideway = false;
bool human_odom_init = false;
bool dead_man = true;

FrontalHumanFollowing::FrontalHumanFollowing() {
    // Initialize the ROS node
    ros::NodeHandle nh;

    // Subscribe to ROS topics
    sub_dead_man = nh.subscribe("/dead_man", 150, &FrontalHumanFollowing::dead_man_Callback, this);
    sub_human_pose = nh.subscribe("/odom_human_for_human_following_module", 150, &FrontalHumanFollowing::human_pose_odom_Callback, this);
    sub_robot_pose = nh.subscribe("/odom", 150, &FrontalHumanFollowing::robot_pose_odom_Callback, this);

    // Create ROS topic publisher
    pub_vel_robot = nh.advertise<geometry_msgs::Twist>("/summit_xl_controller/cmd_vel", 150);
}


void FrontalHumanFollowing::run() {
    ros::Rate loop_rate(150);  // Set the ROS node's loop rate to 150Hz

    while (ros::ok()) {
        // Perform the following actions in each loop iteration

        // 1. Publish the robot's velocity message
        pub_vel_robot.publish(vel_robot_real_);

        // 2. Process subscribed messages
        ros::spinOnce();

        // 3. Allow the node to loop at the specified rate
        loop_rate.sleep();
    }
}



void FrontalHumanFollowing::dead_man_Callback(const std_msgs::Bool::ConstPtr& msg) {
    // Get the 'data' value from the message and store it in the class member variable 'dead_man'
    dead_man = msg->data;
}

void FrontalHumanFollowing::human_pose_odom_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Get human pose information from the message and store it in class member variables
    human_odom_init = true;
    human_x_odom = msg->pose.pose.position.x;
    human_y_odom = msg->pose.pose.position.y;
    human_yaw_odom = msg->pose.pose.orientation.z;
    human_vel_x_odom = msg->twist.twist.linear.x;
    human_vel_y_odom = msg->twist.twist.linear.y;
    human_vel_yaw_odom = msg->twist.twist.angular.z;

    // Perform additional logic based on the data in the message
    if (msg->twist.twist.linear.z == 1) {
        move_sideway = true;
    } else {
        move_sideway = false;
    }
}


void FrontalHumanFollowing::robot_pose_odom_Callback(const nav_msgs::Odometry::ConstPtr& msg){
    // Get the odometer data of the robot (x,y,yaw)
    robot_x_odom = msg->pose.pose.position.x;
    robot_y_odom = msg->pose.pose.position.y;

    // yaw (z-axis rotation)
    siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    robot_yaw_odom = std::atan2(siny_cosp, cosy_cosp);

    double distance2human = sqrt(pow(robot_y_human,2) + pow(robot_x_human,2));


    //Generate a velocity field based on the relative pose between human and robot
    if (human_odom_init == true){
        // relative robot pose
        robot_x_human = (robot_x_odom - human_x_odom) * cos(human_yaw_odom) + (robot_y_odom - human_y_odom) * sin(human_yaw_odom);
        robot_y_human = (robot_y_odom - human_y_odom) * cos(human_yaw_odom) - (robot_x_odom - human_x_odom) * sin(human_yaw_odom);
        robot_yaw_human = - human_yaw_odom + robot_yaw_odom;

        // relative human pose
        human_x_robot = - (robot_x_odom - human_x_odom) * cos(robot_yaw_odom) - (robot_y_odom - human_y_odom) * sin(robot_yaw_odom);
        human_y_robot = - (robot_y_odom - human_y_odom) * cos(robot_yaw_odom) + (robot_x_odom - human_x_odom) * sin(robot_yaw_odom);
        human_yaw_robot = human_yaw_odom - robot_yaw_odom;

        // robot outside the safety zone
        if (robot_x_human > safety_radius){
            robot_velocity_direction_human = PI - (PI / 2 - atan2(robot_x_human - safety_radius, - robot_y_human));
        }else if (robot_x_human < safety_radius && - robot_y_human > 0 && distance2human > safety_radius){
            if (- robot_y_human < safety_radius && robot_x_human < 0){
                robot_velocity_direction_human = - PI / 2 + atan2((- robot_y_human * robot_x_human - safety_radius * sqrt(pow(robot_y_human,2) + pow(robot_x_human,2) - pow(safety_radius,2))) / (pow(robot_y_human,2) - pow(safety_radius,2)), 1);
            }else{
                robot_velocity_direction_human = - PI / 2 + atan2(- (- robot_y_human * robot_x_human - safety_radius * sqrt(pow(robot_y_human,2) + pow(robot_x_human,2) - pow(safety_radius,2))) / (pow(robot_y_human,2) - pow(safety_radius,2)),- 1);
            }
        }else if (robot_x_human < safety_radius && - robot_y_human < 0 && distance2human > safety_radius){
            if (- robot_y_human > - safety_radius && robot_x_human < 0){
                robot_velocity_direction_human = - PI / 2 + atan2(- (- robot_y_human * robot_x_human + safety_radius * sqrt(pow(robot_y_human,2) + pow(robot_x_human,2) - pow(safety_radius,2))) / (pow(robot_y_human,2) - pow(safety_radius,2)),- 1);
            }else{
                robot_velocity_direction_human = - PI / 2 + atan2((- robot_y_human * robot_x_human + safety_radius * sqrt(pow(robot_y_human,2) + pow(robot_x_human,2) - pow(safety_radius,2))) / (pow(robot_y_human,2) - pow(safety_radius,2)), 1);
            }

        // robot inside the safety zone
        }else if (distance2human <= safety_radius){
            if (- robot_y_human > 0){
                force_repulsive_x = gain_repulsive * (- log(sqrt(pow(robot_y_human, 2) + pow(robot_x_human, 2))/safety_radius)) * (- robot_y_human / sqrt(pow(robot_y_human, 2) + pow(robot_x_human, 2)));
                force_repulsive_y = gain_repulsive * (- log(sqrt(pow(robot_y_human, 2) + pow(robot_x_human, 2))/safety_radius)) * (robot_x_human / sqrt(pow(robot_y_human, 2) + pow(robot_x_human, 2)));
                force_attractive_x = gain_attractive * (PI / 2 - atan2(robot_x_human , - robot_y_human)) * (- robot_x_human / distance2human);
                force_attractive_y = gain_attractive * (PI / 2 - atan2(robot_x_human , - robot_y_human)) * (- robot_y_human / distance2human);
                force_total_x = force_repulsive_x + force_attractive_x;
                force_total_y = force_repulsive_y + force_attractive_y;
                robot_velocity_direction_human = - PI / 2 + atan2(force_total_y , force_total_x);
            }else{
                force_repulsive_x = gain_repulsive * (- log(sqrt(pow(robot_y_human, 2) + pow(robot_x_human, 2))/safety_radius)) * (- robot_y_human / sqrt(pow(robot_y_human, 2) + pow(robot_x_human, 2)));
                force_repulsive_y = gain_repulsive * (- log(sqrt(pow(robot_y_human, 2) + pow(robot_x_human, 2))/safety_radius)) * (robot_x_human / sqrt(pow(robot_y_human, 2) + pow(robot_x_human, 2)));
                if (robot_x_human < 0){
                    force_attractive_x = gain_attractive * (PI * 3 / 2  + atan2(robot_x_human, - robot_y_human)) * (robot_x_human / sqrt(pow(robot_y_human, 2) + pow(robot_x_human, 2)));
                    force_attractive_y = gain_attractive * (PI * 3 / 2  + atan2(robot_x_human, - robot_y_human)) * (robot_y_human / sqrt(pow(robot_y_human, 2) + pow(robot_x_human, 2)));
                }else{
                    force_attractive_x = gain_attractive * (- PI / 2  + atan2(robot_x_human, - robot_y_human)) * (robot_x_human / sqrt(pow(robot_y_human, 2) + pow(robot_x_human, 2)));
                    force_attractive_y = gain_attractive * (- PI / 2  + atan2(robot_x_human, - robot_y_human)) * (robot_y_human / sqrt(pow(robot_y_human, 2) + pow(robot_x_human, 2)));
                }
                force_total_x = force_repulsive_x + force_attractive_x;
                force_total_y = force_repulsive_y + force_attractive_y;
                robot_velocity_direction_human = - PI / 2 + atan2(force_total_y , force_total_x);
            }
        }

        // convert velocity to the robot coordinate system
        robot_velocity_direction_robot = robot_velocity_direction_human - robot_yaw_human; 

        // velocity magnitude 
        reference_velocity_magnitude = 0.5;
        gain_p      = 2.5; 
        gain_p_yaw  = 0.6;
        gain_d      = 0.0; 

        double distance2destination = sqrt(pow(robot_y_human,2) + pow(robot_x_human - safety_radius,2));

        // keep the recommended velocity away from the target point
        if (distance2destination > 0.2){
            velocity_magnitude_real = reference_velocity_magnitude;
        }else{
        // add D to prevent oscillation at target point
            velocity_magnitude_real = distance2destination * gain_p - (distance2destination_past - distance2destination) * gain_d;
        }
        distance2destination_past = distance2destination;

        // quickly leave when the robot is very close to human
        if (distance2human < 1){
            velocity_magnitude_real = velocity_magnitude_real * 0.5 * (1 + 1 / pow(distance2human,1.6));
        }
        // approach slowly to the human
        if (distance2human > 1.5){
            velocity_magnitude_real = velocity_magnitude_real * 0.5 * (1 + 1 / pow(abs(distance2human- 2.5),0.8));
        }

        // set velocity recommended
        vel_robot_recommand_by_velocity_field_x = velocity_magnitude_real * cos(robot_velocity_direction_robot);
        vel_robot_recommand_by_velocity_field_y = velocity_magnitude_real * sin(robot_velocity_direction_robot);
        vel_robot_recommand_by_velocity_field_z =  atan(human_y_robot/ human_x_robot )* gain_p_yaw * 2;


        //if the yaw difference is not large, in the move_sideway zone and human turning velocity is low, then robot maintains the same yaw as human 
        double difference_yaw_human_robot = heading_calibration(human_yaw_robot);
        if ((abs(difference_yaw_human_robot)) < PI / 9 && abs(human_vel_yaw_odom) < PI / 18  && move_sideway == true){
            double alpha = 0.5;
            vel_robot_recommand_by_velocity_field_z =  difference_yaw_human_robot_past * alpha + (1 - alpha) * difference_yaw_human_robot * gain_p_yaw * 1.5;
        }
        difference_yaw_human_robot_past = difference_yaw_human_robot;

        // set limit to vel_robot_recommand_by_velocity_field_x
        vel_robot_recommand_by_velocity_field_x = set_upper_lower_limits_velocity(vel_robot_recommand_by_velocity_field_x, vel_robot_linear_max_x, vel_robot_linear_min_x);
        vel_robot_recommand_by_velocity_field_y = set_upper_lower_limits_velocity(vel_robot_recommand_by_velocity_field_y, vel_robot_linear_max_y, vel_robot_linear_min_y);
        vel_robot_recommand_by_velocity_field_z = set_upper_lower_limits_velocity(vel_robot_recommand_by_velocity_field_z, vel_robot_angular_max_z, vel_robot_angular_min_z);

        // robot stops when the safety key is released
        if (dead_man == true) {
            vel_robot_real_y_past = 0;
            vel_robot_real_x_past = 0;
            vel_robot_real_z_past = 0; 
        }

        // calcul velocity real by velocity past and velocity recommand
        double velocity_increment_x = set_upper_lower_limits_velocity(vel_robot_recommand_by_velocity_field_x - vel_robot_real_x_past, 0.5, 0.0);
        double velocity_increment_y = set_upper_lower_limits_velocity(vel_robot_recommand_by_velocity_field_y - vel_robot_real_y_past, 0.5, 0.0);
        double velocity_increment_z = set_upper_lower_limits_velocity(vel_robot_recommand_by_velocity_field_z - vel_robot_real_z_past, 0.6, 0.0);

        vel_robot_real_x = vel_robot_real_x_past + (velocity_increment_x) * 0.06; 
        vel_robot_real_y = vel_robot_real_y_past + (velocity_increment_y) * 0.06;
        vel_robot_real_z = vel_robot_real_z_past + (velocity_increment_z) * 0.06;  

        // set limit to velocity real, do not pass velocity recommand
        vel_robot_real_x = set_limit_velocity_by_velocity_increment(vel_robot_real_x, vel_robot_recommand_by_velocity_field_x, velocity_increment_x);
        vel_robot_real_y = set_limit_velocity_by_velocity_increment(vel_robot_real_y, vel_robot_recommand_by_velocity_field_y, velocity_increment_y);
        vel_robot_real_z = set_limit_velocity_by_velocity_increment(vel_robot_real_z, vel_robot_recommand_by_velocity_field_z, velocity_increment_z);

        // send vitesse to robot
        vel_robot_real_.angular.z =  vel_robot_real_z;
        vel_robot_real_.angular.x =  vel_robot_real_z;
        vel_robot_real_.angular.y =  vel_robot_real_z;
        vel_robot_real_.linear.x = vel_robot_real_x;
        vel_robot_real_.linear.y = vel_robot_real_y;

        // limit maximum de velocity applied to robot
        vel_robot_real_.linear.x = set_upper_lower_limits_velocity(vel_robot_real_.linear.x, vel_robot_linear_max_x, 0.0);
        vel_robot_real_.linear.y = set_upper_lower_limits_velocity(vel_robot_real_.linear.y, vel_robot_linear_max_y, 0.0);
        vel_robot_real_.angular.x = set_upper_lower_limits_velocity(vel_robot_real_.angular.x, vel_robot_angular_max_z, 0.0);
        vel_robot_real_.angular.y = set_upper_lower_limits_velocity(vel_robot_real_.angular.y, vel_robot_angular_max_z, 0.0);
        vel_robot_real_.angular.z = set_upper_lower_limits_velocity(vel_robot_real_.angular.z, vel_robot_angular_max_z, 0.0);

        vel_robot_real_x_past = vel_robot_real_.linear.x;
        vel_robot_real_y_past = vel_robot_real_.linear.y;
        vel_robot_real_z_past = vel_robot_real_.angular.z; 

        // update human pose by velocity constant model
        if (abs(human_vel_x_odom) > 0.2){
            human_x_odom = human_x_odom + human_vel_x_odom * 0.01;
        }
        if (abs(human_vel_y_odom) > 0.2){
            human_y_odom = human_y_odom + human_vel_y_odom * 0.01;
        }
        if (abs(human_vel_yaw_odom) > 0.4){
            human_yaw_odom = human_yaw_odom + human_vel_yaw_odom * 0.01;
        }
    }
}


double FrontalHumanFollowing::heading_calibration(double yaw_original) {
    while (yaw_original > PI) {
        yaw_original -= 2.0 * PI;
    }
    while (yaw_original < -PI) {
        yaw_original += 2.0 * PI;
    }
    return yaw_original;
}

double FrontalHumanFollowing::set_limit_velocity_by_velocity_increment(double vel_input, double vel_limit, double acc) {
    if (acc > 0) {
        if (vel_input > vel_limit) {
            vel_input = vel_limit;
        }
    } else {
        if (vel_input < vel_limit) {
            vel_input = vel_limit;
        }
    }
    return vel_input;
}


double FrontalHumanFollowing::set_upper_lower_limits_velocity(double velocity_input, double upper, double lower) {
    if (velocity_input > upper) {
        velocity_input = upper;
    } else if (velocity_input < -upper) {
        velocity_input = -upper;
    } else if (velocity_input < lower && velocity_input > -lower) {
        velocity_input = 0;
    }
    return velocity_input;
}




int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "frontal_human_following");

    // Create a FrontalHumanFollowing object
    FrontalHumanFollowing frontalHumanFollowing;

    // Call the run method to start the main loop of the ROS node
    frontalHumanFollowing.run();

    return 0;
}

