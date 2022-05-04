#!/usr/bin/env python3

import math
import time

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

pose_gazebo = [0,0,0]
pose_odom = [0,0,0]
quat_odom = [0,0,0,0]
pose_laser = [0,0,0]
pose_amcl = [0,0,0]
pose_asm = [0,0,0]
new_data = [False, False]

def laser_callback(message):
    old_laser = pose_laser[0]
    pose_laser[0] = message.x
    pose_laser[1] = message.y
    pose_laser[2] = message.theta
    if old_laser != pose_laser[0]:
        new_data[0] = True
def amcl_callback(message):
    old_amcl = pose_amcl[0]
    pose_amcl[0] = message.pose.pose.position.x
    pose_amcl[1] = message.pose.pose.position.y
    q1,q2,q3,qr = message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w
    pose_amcl[2] = 360*math.atan2(2*(qr*q3+q1*q2),qr*qr+q1*q1-q2*q2-q3*q3)/(2*math.pi)
    if old_amcl != pose_amcl[0]:
        new_data[1] = True
def odom_callback(message):
    pose_odom[0] = message.pose.pose.position.x
    pose_odom[1] = message.pose.pose.position.y
    q1,q2,q3,qr = message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w
    pose_odom[2] = 360*math.atan2(2*(qr*q3+q1*q2),qr*qr+q1*q1-q2*q2-q3*q3)/(2*math.pi)
def asm_callback(message):
    old_asm = pose_asm[0]
    pose_asm[0] = message.position.x
    pose_asm[1] = message.position.y
    q1,q2,q3,qr = message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w
    pose_asm[2] = 360*math.atan2(2*(qr*q3+q1*q2),qr*qr+q1*q1-q2*q2-q3*q3)/(2*math.pi)
    if old_asm != pose_asm[0]:
        new_data[1] = True
def gazebo_callback(message):
    pose_gazebo[0] = message.pose[1].position.x
    pose_gazebo[1] = message.pose[1].position.y
    q1,q2,q3,qr = message.pose[1].orientation.x, message.pose[1].orientation.y, message.pose[1].orientation.z, message.pose[1].orientation.w
    pose_gazebo[2] = 360*math.atan2(2*(qr*q3+q1*q2),qr*qr+q1*q1-q2*q2-q3*q3)/(2*math.pi)

def main():
    step = 0
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    rospy.Subscriber("gazebo/model_states", ModelStates, gazebo_callback)
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("pose2D", Pose2D, laser_callback)
    rospy.Subscriber("gmcl_pose", PoseWithCovarianceStamped, amcl_callback)
    rospy.Subscriber("scan_pos", Pose, asm_callback)
    rospy.init_node('path', anonymous=True)

    command = Twist(Vector3(0.15,0,0),Vector3(0,0,0))
    rate = rospy.Rate(10)
    desired_heading = 0
    turning = False
    k = 0

    while not rospy.is_shutdown():
        
        if pose_gazebo[2] < -150:
            corrected_gaz_t = 360 + pose_gazebo[2]
        else:
            corrected_gaz_t = pose_gazebo[2]
        if pose_odom[2] < -150:
            corrected_od_t = 360 + pose_odom[2]
        else:
            corrected_od_t = pose_odom[2]
        if pose_laser[2] < -150:
            corrected_las_t = 360 + pose_laser[2]
        else:
            corrected_las_t = pose_laser[2]   
        
        gazebo_dataline = str(k) + ',' + str(pose_gazebo[0]) + ',' + str(pose_gazebo[1]) + ',' +  str(corrected_gaz_t)
        odom_dataline =  str(pose_odom[0]) + ',' + str(pose_odom[1]) + ',' +  str(pose_odom[2]) + ',' +  str(pose_odom[0] - pose_gazebo[0]) + ',' +  str(pose_odom[1] - pose_gazebo[1]) + ',' +  str(corrected_od_t - corrected_gaz_t)
        laser_dataline = str(pose_laser[0]) + ',' + str(pose_laser[1]) + ',' +  str(corrected_las_t) + ',' +  str(pose_laser[0] - pose_gazebo[0]) + ',' +  str(pose_laser[1] - pose_gazebo[1]) + ',' +  str(corrected_las_t - corrected_gaz_t)
        asm_dataline = str(pose_asm[0]) + ',' + str(pose_asm[1]) + ',' +  str(pose_asm[2]) + ',' +  str(pose_asm[0] - pose_gazebo[0]) + ',' +  str(pose_asm[1] - pose_gazebo[1]) + ',' +  str(pose_asm[2] - corrected_gaz_t)
        
        if new_data[0]:
            dataline = gazebo_dataline + ',' + odom_dataline + ',' + laser_dataline + '\n'
            file_asm.write(dataline)
            print("New data obtained for plke!")
            new_data[0] = False
        if new_data[1]:
            dataline = gazebo_dataline + ',' + odom_dataline + ',' + asm_dataline + '\n'
            file_asm.write(dataline)
            print("New data obtained for asm!")
            new_data[1] = False
        k += 1

        pub.publish(command)

        if pose_odom[0] > 2.15 and step == 0:
            turning = True
            command = Twist(Vector3(0,0,0),Vector3(0,0,0.25))
            step += 1
        elif pose_odom[2] > 90 and step == 1:
            turning = False
            desired_heading = 90
            command = Twist(Vector3(0.15,0,0),Vector3(0,0,0))
            step += 1
        elif pose_odom[1] > 1.65 and step == 2:
            turning = True
            command = Twist(Vector3(0,0,0),Vector3(0,0,0.25))
            step += 1
        elif pose_odom[2] < 0 and step == 3:
            turning = False
            desired_heading = 180
            command = Twist(Vector3(0.15,0,0),Vector3(0,0,0))
            step += 1
        elif pose_odom[0] < 0.75 and step == 4:
            command = Twist(Vector3(-0.15,0,0),Vector3(0,0,0))
            step += 1
        elif pose_odom[0] > 1.60 and step == 5:
            turning = True
            command = Twist(Vector3(0,0,0),Vector3(0,0,-0.25))
            step += 1
        elif pose_odom[2] > 0 and pose_odom[2] < 90 and step == 6:
            turning = False
            desired_heading = 90
            command = Twist(Vector3(-0.15,0,0),Vector3(0,0,0))
            step += 1
        elif pose_odom[1] < 1 and step == 7:
            command = Twist(Vector3(0,0,0),Vector3(0,0,0))
            print("Routine Done!")
            break

        if not turning:
            if pose_odom[2] < -150:
                heading_diff = desired_heading - 360 - pose_odom[2]
            else:
                heading_diff = desired_heading - pose_odom[2]
            if heading_diff > 2:
                command.angular.z = 0.075
            elif heading_diff < 2:
                command.angular.z = -0.075

        rate.sleep()

if __name__ == '__main__':
    try:
        """ file_plke = open('data/' + time.asctime(time.localtime(time.time())) + '_data_plke' + '.csv', 'w') """
        file_asm = open('data/' + time.asctime(time.localtime(time.time())) + '_data_asm' + '.csv', 'w')
        """ file_plke.write('i,gz_x,gz_y,gz_t,od_x,od_y,od_t,odd_x,odd_y,odd_t,pl_x,pl_y,pl_t,pld_x,pld_y,pld_t\n') """
        file_asm.write('i,gz_x,gz_y,gz_t,od_x,od_y,od_t,odd_x,odd_y,odd_t,asm_x,asm_y,asm_t,asmd_x,asmd_y,asmd_t\n')
        """ file.write('i,gz_x,gz_y,gz_t,od_x,od_y,od_t,odd_x,odd_y,odd_t,gm_x,gm_y,am_t,amd_x,amd_y,amd_t,pl_x,pl_y,pl_t,pld_x,pld_y,pld_t\n') """
        main()
    except rospy.ROSInterruptException:
        pass