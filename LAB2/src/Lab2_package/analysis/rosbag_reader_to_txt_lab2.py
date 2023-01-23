import rosbag
import numpy as np

bag = rosbag.Bag('2022-02-20-14-23-06.bag')

#initialize arrays for data
lat_data = np.empty((0))
long_data = np.empty((0))
alt_data = np.empty((0))
utmE_data = np.empty((0))
utmN_data = np.empty((0))
gnss_fix = np.empty((0))

#write data
for topic, msg, t in bag.read_messages(topics=['/gpstopic']):
    lat_data = np.append(lat_data,msg.latitude.data)
    long_data = np.append(long_data,msg.longitude.data)
    alt_data = np.append(alt_data,msg.altitude.data)
    utmE_data = np.append(utmE_data,msg.utm_easting.data)
    utmN_data = np.append(utmN_data,msg.utm_northing.data)
    gnss_fix = np.append(gnss_fix,msg.fix_quality.data)
    # print(t) #see what this is
bag.close()

#Making sure shapes are correct
print('---------- Make sure shapes are correct ----------')
print('Latitude data shape: {}'.format(lat_data.shape))
print('Longitude data shape: {}'.format(long_data.shape))
print('Altitude data shape: {}'.format(alt_data.shape))
print('UTM E data shape: {}'.format(utmE_data.shape))
print('UTM N data shape: {}'.format(utmN_data.shape))
print('GNSS Fix data shape: {}'.format(gnss_fix.shape))
print('--------------------------------------------------')

print(lat_data.dtype)
print(long_data.dtype)
print(alt_data.dtype)
print(utmE_data.dtype)
print(utmN_data.dtype)
print(gnss_fix.dtype)

#stack data in to one array
total_data = np.column_stack((lat_data,long_data,alt_data,utmE_data,utmN_data,gnss_fix))
print('Total data shape: {}'.format(total_data.shape))

#write data to a .txt file
gps_data_txt = open('Open_field_stationary_txt.txt','w')
for row in total_data:
    np.savetxt(gps_data_txt,row)
gps_data_txt.close()