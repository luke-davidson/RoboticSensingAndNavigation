#!/usr/bin/env python3.8

"""
This script will read serial data from a GPS device reporting to port /dev/ttyUSB0 and pusblish the data to the topic 'gps_data' with 
a custom message including a Header, latitude, longitude, altitude, UTM easting, UTM northing, UTM Zone, and UTM Letter

author: davidson.lu@northeastern.edu
"""

import rospy
import serial
import utm
from Lab2_package.msg import Lab2_msg
from std_msgs.msg import Header
import math

ser = serial.Serial('/dev/ttyACM0',57600)

def min_second_to_decimal(degree_min_s):
    
    #degree_min_s as type float

    deg_decimal_whole = math.trunc(degree_min_s/100)
    min_conversion = math.trunc((degree_min_s/100-deg_decimal_whole)*100)/60
    s_conversion = (degree_min_s-math.trunc(degree_min_s))*100/3600
    degree_decimal = deg_decimal_whole + min_conversion + s_conversion

    return degree_decimal

def data_recorder():

    topic_pub = rospy.Publisher('lab2_data',Lab2_msg,queue_size=10)
    rospy.init_node('lab2_data_pub')
    rospy.logdebug('Reading GPS data from port /ttyACM0 at 57600')
    rate = rospy.Rate(10) #??????????????????????????????????????????????????

    while not rospy.is_shutdown():
        #split the data so that it's readable
        string_data = ser.readline().decode('utf-8')
        split_data = string_data.split(',')
        d_type = split_data[0]

        if d_type == '$GNGGA':
            #parse lat long alt and directions from $GPGGA data
            if split_data[2] == '':
                rospy.logwarn('Data is empty, cannot get a reading.')
            else:
                latitude = float(split_data[2])
                lat_dir = split_data[3]
                longitude = float(split_data[4])
                long_dir = split_data[5]
                altitude = float(split_data[9])
                gps_fix = int(split_data[6])

                #convert to decimal
                latitude_decimal = min_second_to_decimal(latitude)
                longitude_decimal = min_second_to_decimal(longitude)

                if lat_dir == 'S':
                    latitude_decimal = -1*latitude_decimal
                elif long_dir == 'W':
                    longitude_decimal = -1*longitude_decimal
            
                #msg values
                msg = GPS_msg()
                # Header().stamp.secs = rospy.Time.now()
                msg.header = Header()
                msg.latitude = latitude_decimal
                msg.longitude = longitude_decimal
                msg.altitude = altitude
                msg.gps_fix = gps_fix
            
                #UTM conversion
                msg.utm_easting, msg.utm_northing, msg.zone, msg.letter = utm.from_latlon(msg.latitude, msg.longitude)

                #log info
                rospy.loginfo(msg)
                topic_pub.publish(msg)
                rate.sleep() #??????????????????????????????????????????????????


if __name__ == '__main__':
    try:
        data_recorder()
    except rospy.ROSInterruptException:
        pass
