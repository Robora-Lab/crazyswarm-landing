#!/usr/bin/env python3

import rospy
import numpy as np
import csv
from geometry_msgs.msg import PoseStamped, TwistStamped
from crazyswarm.msg import GenericLogData
from std_msgs.msg import Float64 
import math
import os
import time

#store the data
cf_pose=np.zeros(3)
prev_pose=np.zeros(3)
cf_vel=np.zeros(3)
prev_vel=np.zeros(3)
cf_accel=np.zeros(3)
cf_goal=np.zeros(2)
h_pose=np.zeros(3)
h_goal= np.zeros(3)
tilt = 0
sys_time=0
extraZGoal=0
zAcc=0
h_vel= 0
plat_omega=0
prev_tilt = 0
last_tilt_time=time.time()

#csv
csv_directory = '/home/jess/crazyswarm/ros_ws/src/crazyswarm/scripts/MPC_take2/data'
csv_filename = os.path.join(csv_directory, 'node_data.csv')
uav_file = open(csv_filename, 'w', newline='')
uav_writer = csv.writer(uav_file)
uav_writer.writerow(['cf_px', 'cf_py', 'cf_pz','cf_gx', 'cf_gy', 'cf_gz', 'h_px', 'h_py', 'h_pz', 'h_gx', 'h_gy', 'hg_z', 'tiltx', 'global time', 'acc_z', 'h_vel', 'plat_omega'])


def custom_node():
    global sharedTime, start_time

    rospy.init_node('cf_sub_pub_node', anonymous=True)

    rospy.Subscriber('/goalPose', PoseStamped, hgCallback)
    rospy.Subscriber('/goalPoseCF', PoseStamped, cfgCallback)
    rospy.Subscriber('/vrpn_client_node/WillHusky/pose', PoseStamped, hpCallback)
    rospy.Subscriber('vrpn_client_node/cf1/pose', PoseStamped, cfpCallback)
    rospy.Subscriber('/tilts', PoseStamped, tiltCallback)
    rospy.Subscriber('/extraPub', Float64, extraCallback)
    rospy.Subscriber('/cf1/log1', GenericLogData, zAccCallback)
    rospy.Subscriber('/vrpn_client_node/WillHusky/twist', TwistStamped, hvCallback)

    sharedTime=rospy.Publisher('/systemTime', Float64, queue_size=10)

    start_time=float(rospy.Time.now().to_sec())
    #start_time=start_time+25
    #periodically publish time
    rospy.Timer(rospy.Duration(0.1), timePubCallback)

    rospy.spin()

def quatToYaw(x,y,z,w):
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    x/=norm
    y/=norm
    z/=norm
    w/=norm
    yaw=math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))
    return yaw

def timePubCallback(event):
    global start_time, sys_time
    current_time = float(rospy.Time.now().to_sec())
    elapsed_time = current_time - start_time
    time_msg = Float64()
    time_msg.data = elapsed_time
    sharedTime.publish(time_msg)
    sys_time = elapsed_time
    write_to_csv()

def hgCallback(msg):
    global h_goal
    position = msg.pose.position
    data_float = [position.x, position.y, 1.02]
    h_goal=np.array(data_float)


def hvCallback(msg):
    global h_vel
    twist = msg.twist.linear
    velx=twist.x
    vely=twist.y
    h_vel=math.sqrt(pow(velx,2)+pow(vely,2))
    # data_float = [twist.x, twist.y]
    # h_vel=np.array(data_float)

def cfgCallback(msg):
    global cf_goal
    position = msg.pose.position
    data_float = [position.x, position.y]#, position.z]
    cf_goal=np.array(data_float)

def extraCallback(msg):
    global extraZGoal
    extraZGoal=msg.data

def hpCallback(msg):
    global h_pose
    position = msg.pose.position
    ori=msg.pose.orientation
    position_offset = position

    yaw=quatToYaw(ori.x, ori.y, ori.z, ori.w)

    position_offset.x = position.x + (0.1 *math.cos(-yaw))
    position_offset.y = position.y - (0.1*math.sin(-yaw))

    x = float(position_offset.x)
    y = float(position_offset.y)
    z = float(position.z)
    z=1.02
    data_float = [x, y, z]
    h_pose=np.array(data_float)

# def cfpCallback(msg):
#     global cf_pose
#     position = msg.pose.position
#     data_float = [position.x, position.y, position.z]
#     cf_pose=np.array(data_float)

def cfpCallback(msg):
    global cf_pose#, prev_pose, cf_vel, prev_vel, cf_accel
    position = msg.pose.position
    data_float = [position.x, position.y, position.z]
    #prev_pose=cf_pose.copy()
    cf_pose=np.array(data_float)

    #calculations
    # cf_vel=(cf_pose-prev_pose)/0.1
    # cf_accel=(cf_vel-prev_vel)/0.1

    # prev_vel=cf_vel.copy()

def tiltCallback(msg):
    global tilt, prev_tilt, plat_omega, last_tilt_time
    current_time=time.time()
    if current_time - last_tilt_time >= 0.1:
        position = msg.pose.position
        tilt=abs(position.x)
        plat_omega=abs((tilt-prev_tilt)/0.1)         #angular velocity of platform
        prev_tilt=tilt
        last_tilt_time=current_time

def zAccCallback(msg):
    global zAcc
    zAcc=msg.values[0]



def write_to_csv():
    uav_writer.writerow(cf_pose.tolist() + cf_goal.tolist() + [extraZGoal]+ h_pose.tolist() + h_goal.tolist() + [tilt] + [sys_time] + [zAcc] + [h_vel]+ [plat_omega])


if __name__ == '__main__':
    try:
        custom_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        uav_file.close()