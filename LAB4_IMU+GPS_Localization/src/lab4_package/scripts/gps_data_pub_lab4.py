#!/usr/bin/env python3.8

"""
This script will read serial data from a GPS device reporting to port specified in port_num and publish the data to the topic 'gps_data' with 
a custom message including a Header, latitude, longitude, altitude, UTM easting, UTM northing, UTM Zone, and UTM Letter

author: davidson.lu@northeastern.edu
"""

import rospy
import serial
import utm
from lab4_package.msg import GPS_msg
from std_msgs.msg import Header
import math

port_num = '1'
port = '/dev/ttyUSB' + port_num

ser = serial.Serial(port, 4800)

def min_second_to_decimal(degree_min_s):
    # degree_min_s as type float
    deg_decimal_whole = math.trunc(degree_min_s/100)
    min_conversion = math.trunc((degree_min_s/100-deg_decimal_whole)*100)/60
    s_conversion = (degree_min_s-math.trunc(degree_min_s))*100/3600
    degree_decimal = deg_decimal_whole + min_conversion + s_conversion
    return degree_decimal

def data_recorder():
    topic_pub = rospy.Publisher('gps_data', GPS_msg, queue_size=10)
    rospy.init_node('gps_data_pub')
    rospy.logdebug('Reading GPS data at Baud rate 4800')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #split the data so that it's readable
        string_data = ser.readline().decode('utf-8')
        split_data = string_data.split(',')
        d_type = split_data[0]
        if d_type == '$GPGGA':
            # parse lat long alt and directions from $GPGGA data
            if split_data[2] == '':
                rospy.logwarn('Data is empty, cannot get a reading.')
            else:
                latitude = float(split_data[2])
                lat_dir = split_data[3]
                longitude = float(split_data[4])
                long_dir = split_data[5]
                altitude = float(split_data[9])

                #convert to decimal
                latitude_decimal = min_second_to_decimal(latitude)
                longitude_decimal = min_second_to_decimal(longitude)

                if lat_dir == 'S':
                    latitude_decimal *= -1
                elif long_dir == 'W':
                    longitude_decimal *= -1
            
                #msg values
                msg = GPS_msg()
                Header().stamp.secs = rospy.Time.now()
                msg.header = Header()
                msg.latitude = latitude_decimal
                msg.longitude = longitude_decimal
                msg.altitude = altitude
            
                #UTM conversion
                msg.utm_easting, msg.utm_northing, msg.zone, msg.letter = utm.from_latlon(msg.latitude, msg.longitude)

                #log info
                rospy.loginfo(msg)
                topic_pub.publish(msg)
                rate.sleep()


if __name__ == '__main__':
    try:
        data_recorder()
    except rospy.ROSInterruptException:
        pass
