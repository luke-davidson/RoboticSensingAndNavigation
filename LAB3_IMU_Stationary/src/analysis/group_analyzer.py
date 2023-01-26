import pandas as pd
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np

# Read in to csv file
bag = bagreader('/home/luke/Downloads/2022-02-26-18-09-50.bag')
imu_data_fivehour = bag.message_by_topic(topic='/imu_data')
imu_data_5hr_csv = pd.read_csv(imu_data_fivehour)
imu_data_5hr_pd = pd.DataFrame(imu_data_5hr_csv, columns=['Time', 'magnetic_field.x', 'magnetic_field.y', 'magnetic_field.z', 'acceleration.x', 'acceleration.y', 'acceleration.z', 'gyro.x', 'gyro.y', 'gyro.z', 'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w']).astype(float)

# Convert quaternion back to roll, pitch, yaw
yaw = np.arctan2(2*(imu_data_5hr_pd['orientation.y']*imu_data_5hr_pd['orientation.z']+imu_data_5hr_pd['orientation.w']*imu_data_5hr_pd['orientation.x']), imu_data_5hr_pd['orientation.w']*imu_data_5hr_pd['orientation.w']-imu_data_5hr_pd['orientation.x']*imu_data_5hr_pd['orientation.x']-imu_data_5hr_pd['orientation.y']*imu_data_5hr_pd['orientation.y']+imu_data_5hr_pd['orientation.z']*imu_data_5hr_pd['orientation.z'])
pitch = np.arcsin(-2*(imu_data_5hr_pd['orientation.x']*imu_data_5hr_pd['orientation.z']-imu_data_5hr_pd['orientation.w']*imu_data_5hr_pd['orientation.y']))
roll = np.arctan2(2*(imu_data_5hr_pd['orientation.x']*imu_data_5hr_pd['orientation.y']+imu_data_5hr_pd['orientation.w']*imu_data_5hr_pd['orientation.z']), imu_data_5hr_pd['orientation.w']*imu_data_5hr_pd['orientation.w']+imu_data_5hr_pd['orientation.x']*imu_data_5hr_pd['orientation.x']-imu_data_5hr_pd['orientation.y']*imu_data_5hr_pd['orientation.y']-imu_data_5hr_pd['orientation.z']*imu_data_5hr_pd['orientation.z'])