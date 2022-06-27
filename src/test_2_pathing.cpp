#include <iostream>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <vector>
#include <string>
#include <cmath>
#include <ctime>
#include <fstream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Twist.h"

struct PARAMS{
    double v_lin;
    double v_ang;
};

PARAMS params;

struct POSES{
    std::vector<double> odom_pose = {0,0,0};
    std::vector<double> gtruth_pose = {0,0,0};
    std::vector<double> old_asm = {0,0,0};
    std::vector<double> asm_pose = {0,0,0};
    bool asm_new = false;
};

POSES poses;

class CommandPublisher{
    
    public:
        ros::Publisher pub;

        CommandPublisher(ros::NodeHandle n){
            //Constructor
            this->pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        }
};

class OdomSubscriber{
    ros::Subscriber sub;
    public:
        static void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
            double odom_angle = atan2(2*(msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
                            msg->pose.pose.orientation.w*msg->pose.pose.orientation.w + msg->pose.pose.orientation.x*msg->pose.pose.orientation.x 
                            - msg->pose.pose.orientation.y*msg->pose.pose.orientation.y - msg->pose.pose.orientation.z*msg->pose.pose.orientation.z);
            poses.odom_pose[0] = msg->pose.pose.position.x;
            poses.odom_pose[1] = msg->pose.pose.position.y;
            poses.odom_pose[2] = 360*odom_angle/(2*M_PI);
        }
        OdomSubscriber(ros::NodeHandle n){
            //Constructor
            this->sub = n.subscribe("odom", 1000, odom_callback);
        }
};

class GTruthSubscriber{
    ros::Subscriber sub;
    public:
        static void gt_callback(const gazebo_msgs::ModelStates::ConstPtr &msg){
            double gtruth_angle = atan2(2*(msg->pose.data()->orientation.w * msg->pose.data()->orientation.z + msg->pose.data()->orientation.x * msg->pose.data()->orientation.y),
                            msg->pose.data()->orientation.w*msg->pose.data()->orientation.w + msg->pose.data()->orientation.x*msg->pose.data()->orientation.x 
                            - msg->pose.data()->orientation.y*msg->pose.data()->orientation.y - msg->pose.data()->orientation.z*msg->pose.data()->orientation.z);
            poses.gtruth_pose[0] = msg->pose.data()->position.x;
            poses.gtruth_pose[1] = msg->pose.data()->position.y;
            poses.gtruth_pose[2] = 360*gtruth_angle/(2*M_PI);
        }
        GTruthSubscriber(ros::NodeHandle n){
            //Constructor
            this->sub = n.subscribe("gazebo/model_states", 1000, gt_callback);
        }
};

class ASMSubscriber{
    ros::Subscriber sub;
    public:
        static void asm_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
            poses.old_asm = poses.asm_pose;
            double asm_angle = atan2(2*(msg->pose.orientation.w * msg->pose.orientation.z + msg->pose.orientation.x * msg->pose.orientation.y),
                            msg->pose.orientation.w*msg->pose.orientation.w + msg->pose.orientation.x*msg->pose.orientation.x 
                            - msg->pose.orientation.y*msg->pose.orientation.y - msg->pose.orientation.z*msg->pose.orientation.z);
            poses.asm_pose[0] = msg->pose.position.x;
            poses.asm_pose[1] = msg->pose.position.y;
            poses.asm_pose[2] = 360*asm_angle/(2*M_PI);
            if (poses.old_asm[0] != poses.asm_pose[1] || poses.old_asm[1] != poses.asm_pose[1] || poses.old_asm[2] || poses.asm_pose[2]){
                poses.asm_new = true;
            }
        }
        ASMSubscriber(ros::NodeHandle n){
            //Constructor
            this->sub = n.subscribe("asm_pos", 1000, asm_callback);
        }
};


int main(int argc, char **argv){
    //Creating the data file
    time_t now = time(0);
    char *date = ctime(&now);
    std::string sdate(date);
    std::string filename = "data/" + sdate + "_data_asm" + ".csv";
    std::ofstream dat_file{filename};

    //Starting ROS node
    ros::init(argc, argv, "evaluator");
    ros::NodeHandle n;
    ros::Rate rate(10);

    //Setting up parameters
    params.v_lin = n.param<double>("max_linear_velocity", 0.15);
    params.v_ang = n.param<double>("max_angular_velocity", 0.25);

    OdomSubscriber odom_sub(n);
    GTruthSubscriber gtruth_sub(n);
    ASMSubscriber asm_sub(n);

    CommandPublisher com_pub(n);
    geometry_msgs::Twist command;
    command.linear.x = params.v_lin;
    command.linear.y = 0;
    command.linear.z = 0;

    command.angular.x = 0;
    command.angular.y = 0;
    command.angular.z = 0;

    int step = 0;
    bool turning = false;
    double desired_heading = 0;
    int k = 0;

    while (ros::ok()) {

        std::string gtruth_dataline = std::to_string(k) + ',' + std::to_string(poses.gtruth_pose[0]) + ',' + std::to_string(poses.gtruth_pose[1]) + ',' + std::to_string(poses.gtruth_pose[2]);
        std::string odom_dataline = std::to_string(poses.odom_pose[0]) + ',' + std::to_string(poses.odom_pose[1]) + ',' + std::to_string(poses.odom_pose[2]);
        std::string asm_dataline = std::to_string(k) + ',' + std::to_string(poses.asm_pose[0]) + ',' + std::to_string(poses.asm_pose[1]) + ',' + std::to_string(poses.asm_pose[2]);
        
        if (poses.asm_new){
            ROS_INFO("New data obtained from asm!");
            std::string dataline = gtruth_dataline + ',' + odom_dataline + ',' + asm_dataline + '\n';
            dat_file << dataline;
            poses.asm_new = false;
        }
        k++;

        com_pub.pub.publish(command);

        if (poses.odom_pose[0] > 2.15 && step == 0){
            turning = true;
            command.linear.x = 0;
            command.angular.z = params.v_ang;
            step++;
        }
        else if (poses.odom_pose[2] > 90 && step == 1){
            turning = false;
            desired_heading = 90;
            command.linear.x = params.v_lin;
            command.linear.z = params.v_ang;
            step++;
        }
        else if (poses.odom_pose[1] > 1.65 && step == 2){
            turning = true;
            command.linear.x = 0;
            command.angular.z = params.v_ang;
            step++;
        }
        else if (poses.odom_pose[2] < 0 && step == 3){
            turning = false;
            desired_heading = 180;
            command.linear.x = params.v_lin;
            command.angular.z = params.v_ang;
            step++;
        }
        else if (poses.odom_pose[0] < 0.75 && step == 4){
            command.linear.x = -params.v_lin;
            step++;
        }
        else if (poses.odom_pose[0] > 1.60 && step == 5){
            turning = true;
            command.linear.x = 0;
            command.angular.z = -params.v_ang;
            step++;
        }
        else if (poses.odom_pose[2] > 0 && poses.odom_pose[2] < 90 && step == 6){
            turning = false;
            desired_heading = 90;
            command.linear.x = -params.v_lin;
            command.angular.z = 0;
            step++;
        }
        else if (poses.odom_pose[1] < 1 && step == 7){
            command.linear.x = 0;
            std::cout << "Routine Done!";
            break;
        }

        if (!turning){
            double heading_diff = 0;
            if (poses.odom_pose[2] < -150){
                heading_diff = desired_heading - 360 - poses.odom_pose[2];
            }
            else{
                heading_diff = desired_heading - poses.odom_pose[2];
            }
            if (heading_diff > 2){
                command.angular.z = 0.075;
            }
            else if (heading_diff < 2){
                command.angular.z = -0.075;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    dat_file.close();
    return 0;
}