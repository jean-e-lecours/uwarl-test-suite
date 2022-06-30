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
    double delay;
};

PARAMS params;

struct POSES{
    std::vector<double> odom_pose = {0,0,0};
    std::vector<double> gt_pose = {0,0,0};
    std::vector<double> old_asm = {0,0,0};
    std::vector<double> asm_pose = {0,0,0};
    bool asm_new = false;
};

POSES poses;

bool asm_ready = false;
bool las_ready = false;
bool gt_ready = false;

std::string gt_string = "Ground Truth: ";
std::string las_string = "Laser Data: ";
std::string asm_string = "ASM Estimate: ";

class LaserSubscriber{
  ros::Subscriber sub;

  public:
    static bool ready;

    static void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg){
        for (int i = 0; i < msg->ranges.size()-2; i++){
            las_string = las_string + std::to_string(msg->ranges[i]) + ", ";
        }
        las_string = las_string + std::to_string(msg->ranges[msg->ranges.size()-1]) + "\n";
        las_ready = true;
    }
    LaserSubscriber(ros::NodeHandle n){
      //Constructor
      this->sub = n.subscribe("scan", 1000, laser_callback);
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
            asm_string = asm_string + "[" + std::to_string(poses.asm_pose[0]) + ", " + std::to_string(poses.asm_pose[1]) + ", " + std::to_string(poses.asm_pose[2]) + "]\n";
            asm_ready = true;
        }
        ASMSubscriber(ros::NodeHandle n){
            //Constructor
            this->sub = n.subscribe("asm_pos", 1000, asm_callback);
        }
};

class GTruthSubscriber{
    ros::Subscriber sub;
    public:
        static void gt_callback(const gazebo_msgs::ModelStates::ConstPtr &msg){
            //WILL NEED TO CHANGE THE INDEX IF MORE MODELS ARE ADDED!
            double gtruth_angle = atan2(2*(msg->pose[1].orientation.w * msg->pose[1].orientation.z + msg->pose[1].orientation.x * msg->pose[1].orientation.y),
                            msg->pose[1].orientation.w*msg->pose[1].orientation.w + msg->pose[1].orientation.x*msg->pose[1].orientation.x 
                            - msg->pose[1].orientation.y*msg->pose[1].orientation.y - msg->pose[1].orientation.z*msg->pose[1].orientation.z);
            poses.gt_pose[0] = msg->pose[1].position.x;
            poses.gt_pose[1] = msg->pose[1].position.y;
            poses.gt_pose[2] = 360*gtruth_angle/(2*M_PI);
            gt_string = gt_string + "[" + std::to_string(poses.gt_pose[0]) + ", " + std::to_string(poses.gt_pose[1]) + ", " + std::to_string(poses.gt_pose[2]) + "]\n";
            gt_ready = true;
        }
        GTruthSubscriber(ros::NodeHandle n){
            //Constructor
            this->sub = n.subscribe("gazebo/model_states", 1000, gt_callback);
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "test_echo");
    ros::NodeHandle n;
    ros::Rate rate(10);

    params.delay = n.param<double>("time_delay_to_echo", 0);

    GTruthSubscriber gtruth_sub(n);
    ASMSubscriber asm_sub(n);
    LaserSubscriber las_sub(n);

    while(ros::ok()){
        ros::spinOnce();
        if (gt_ready && asm_ready && las_ready){
            std::cout << las_string << gt_string << asm_string;
            break;
        }
        rate.sleep();
    }
}