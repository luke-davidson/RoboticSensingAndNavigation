import rosbag
import numpy as np

bag = rosbag.Bag('moving_data.bag')

#initialize arrays for data
lat_data = np.empty((0))
long_data = np.empty((0))
alt_data = np.empty((0))
utmE_data = np.empty((0))
utmN_data = np.empty((0))

#write data
for topic, msg, t in bag.read_messages(topics=['/gps_data']):
    lat_data = np.append(lat_data, msg.latitude)
    long_data = np.append(long_data, msg.longitude)
    alt_data = np.append(alt_data, msg.altitude)
    utmE_data = np.append(utmE_data, msg.utm_easting)
    utmN_data = np.append(utmN_data, msg.utm_northing)
bag.close()

#Making sure shapes are correct
print('---------- Make sure shapes are correct ----------')
print('Latitude data shape: {}'.format(lat_data.shape))
print('Longitude data shape: {}'.format(long_data.shape))
print('Altitude data shape: {}'.format(alt_data.shape))
print('UTM E data shape: {}'.format(utmE_data.shape))
print('UTM N data shape: {}'.format(utmN_data.shape))
print('--------------------------------------------------')

#stack data in to one array
total_data = np.column_stack((lat_data, long_data, alt_data, utmE_data, utmN_data))
print('Total data shape: {}'.format(total_data.shape))

#write data to a .txt file
gps_data_txt = open('moving_data_txt.txt', 'w')
for row in total_data:
    np.savetxt(gps_data_txt, row)
gps_data_txt.close()