#!/usr/bin/env python3

"""
This script will read serial data from an IMU device reporting to port /dev/ttyUSB0 and publish the data to the topic 'imu_data'.

author: davidson.lu@northeastern.edu
"""

import rospy
import serial
import numpy as np
from lab3_package.msg import IMU_msg
from std_msgs.msg import Header

ser = serial.Serial('/dev/ttyUSB0',115200)

def euler_to_quarternion(roll,pitch,yaw):
    """
    Takes roll, pitch, yaw values as type float in degrees, converts to rad, 
    and calculates quaternion qx, qy, qz, qx as type float
    """
    roll = roll*np.pi/180
    pitch = pitch*np.pi/180
    yaw = yaw*np.pi/180
    qx = (np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)) - (np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
    qy = (np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)) + (np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2))
    qz = (np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)) - (np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2))
    qw = (np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)) + (np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
    return qx,qy,qz,qw

def imu_data_recorder():
    topic_pub = rospy.Publisher('imu_data', IMU_msg, queue_size=10)
    rospy.init_node('imu_data_publisher')
    rospy.logdebug('Reading IMU data from port /ttyUSB0 at Baud rate 115200')
    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        #split the data
        string_data = ser.readline().decode('utf-8')
        split_data = string_data.split(',')
        if split_data[0] != '$VNYMR':
            rospy.logwarn('Different data type than $VNYMR detected.')
        elif split_data[2] == '':
            rospy.logwarn('Read empty data. Check sensor output.')
        elif len(split_data) != 13:
            rospy.logwarn('Input data is in wrong shape! Too many or too few arguments.')
        else:
            #msg
            msg = IMU_msg()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            
            #convert split data to float and equate to msg params
            yaw = float(split_data[1])
            pitch = float(split_data[2])
            roll = float(split_data[3])
            msg.magnetic_field.x = float(split_data[4])*0.0001
            msg.magnetic_field.y = float(split_data[5])*0.0001
            msg.magnetic_field.z = float(split_data[6])*0.0001
            msg.linear_acceleration.x = float(split_data[7])
            msg.linear_acceleration.y = float(split_data[8])
            msg.linear_acceleration.z = float(split_data[9])
            msg.angular_velocity.x = float(split_data[10])
            msg.angular_velocity.y = float(split_data[11])
            # get rid of *XX\r\n at the end of the data list
            msg.angular_velocity.z = float(split_data[12].split('*')[0])
            split_gyro_z = list(split_data[12])
            new_gyro_z = "".join(split_gyro_z[:len(split_gyro_z)-5])
            msg.angular_velocity.z = float(new_gyro_z)

            #convert to quarternion
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = euler_to_quarternion(roll, pitch, yaw)
        
            #log info
            rospy.loginfo(msg)
            topic_pub.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        imu_data_recorder()
    except rospy.ROSInterruptException:
        pass
