from os import W_OK
import pandas as pd
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
from scipy import integrate
import math

########## Read in to csv file, convert to numpy array ##########
bag = bagreader('/home/luke/Downloads/lab4/all_data.bag')
imu_data = bag.message_by_topic(topic = '/imutopic')
gps_data = bag.message_by_topic(topic = '/gpstopic')
imu_data_csv = pd.read_csv(imu_data)
gps_data_csv = pd.read_csv(gps_data)
imu_data_pd = pd.DataFrame(imu_data_csv,columns=['Time','orientation.x','orientation.y','orientation.z','orientation.w','angular_velocity.x','angular_velocity.y','angular_velocity.z','linear_acceleration.x','linear_acceleration.y','linear_acceleration.z','magnetic_field.x','magnetic_field.y','magnetic_field.z']).astype(float)
gps_data_pd = pd.DataFrame(gps_data_csv,columns=['Time','latitude.data','longitude.data','altitude.data','utm_easting.data','utm_northing.data']).astype(float)
imu_data_np = pd.DataFrame.to_numpy(imu_data_pd)
gps_data_np = pd.DataFrame.to_numpy(gps_data_pd)

########## split data in to stationary, circles, moving ##########
stationary_gps = gps_data_np[:245,:]
circles_gps = gps_data_np[255:345,:]
moving_gps = gps_data_np[350:,:]
# imu_stationary_start, = np.where(np.absolute(imu_data_np[:,0] - gps_data_np[0,0]) == np.min(np.absolute(imu_data_np[:,0] - gps_data_np[0,0])))
imu_stationary_end, = np.where(np.absolute(imu_data_np[:,0] - gps_data_np[245,0]) == np.min(np.absolute(imu_data_np[:,0] - gps_data_np[245,0])))
imu_circles_start, = np.where(np.absolute(imu_data_np[:,0] - gps_data_np[255,0]) == np.min(np.absolute(imu_data_np[:,0] - gps_data_np[255,0])))
imu_circles_end, = np.where(np.absolute(imu_data_np[:,0] - gps_data_np[345,0]) == np.min(np.absolute(imu_data_np[:,0] - gps_data_np[345,0])))
imu_moving_start, = np.where(np.absolute(imu_data_np[:,0] - gps_data_np[350,0]) == np.min(np.absolute(imu_data_np[:,0] - gps_data_np[350,0])))
# imu_stationary_start = imu_stationary_start[0]
imu_stationary_end = imu_stationary_end[0]
imu_circles_start = imu_circles_start[0]
imu_circles_end = imu_circles_end[0]
imu_moving_start = imu_moving_start[0]
stationary_imu = imu_data_np[:imu_stationary_end,:]
circles_imu = imu_data_np[imu_circles_start:imu_circles_end,:]
moving_imu = imu_data_np[imu_moving_start:,:]

########## plotting data function ##########
def plot_data(x_data,y_data,xlabel,ylabel,title):
    plt.close()
    plt.plot(x_data,y_data)
    plt.grid()
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)
    plt.show()

########## plotting UTM data ##########
# plot_data(gps_data_np[:,4],gps_data_np[:,5],'UTM Easting (m)','UTM Northing (m)','UTM Data')

########## plotting corrected magnetometer data ##########
# # plot_data(circles_imu[:,11],circles_imu[:,12],'Mag - X','Mag - Y','X and Y Magnetometer Data')
circles_imu_avgx = np.mean(circles_imu[:,11])
circles_imu_avgy = np.mean(circles_imu[:,12])
corrected_circles_imu_magx = circles_imu[:,11] - circles_imu_avgx
corrected_circles_imu_magy = circles_imu[:,12] - circles_imu_avgy
# plt.close()
# plt.plot(circles_imu[:,11],circles_imu[:,12],'b-',label='Raw Data')
# plt.plot(circles_imu_avgx,circles_imu_avgy,'ro',label='Raw Data Center')
# plt.plot(corrected_circles_imu_magx,corrected_circles_imu_magy,'g-',label='Hard and Soft Iron Corrected Data')
# plt.grid()
# plt.legend()
# plt.title('X and Y Magnetometer Data')
# plt.ylabel('Mag - Y (tesla)')
# plt.xlabel('Mag - X (tesla)')
# plt.show()

########## correcting yaw from corrected mag data ##########
def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if np.any(t2 > +1.0) else t2
        t2 = -1.0 if np.any(t2 < -1.0) else t2
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return roll, pitch, yaw # in radians
roll,pitch,yaw = euler_from_quaternion(circles_imu[:,1],circles_imu[:,2],circles_imu[:,3],circles_imu[:,4])

def wrap_to_pi(data):
    data_wrapped_to_pi = np.remainder(data,2*np.pi)
    mask = np.abs(data_wrapped_to_pi)>np.pi
    data_wrapped_to_pi[mask] -= 2*np.pi*np.sign(data_wrapped_to_pi[mask])
    return data_wrapped_to_pi

corrected_yaw_mag = np.arctan2(corrected_circles_imu_magx,corrected_circles_imu_magy)
corrected_yaw_int = integrate.cumtrapz(circles_imu[:,7],circles_imu[:,0],initial=0)
##### subtract bias
corrected_yaw_int = corrected_yaw_int-(corrected_yaw_int[0]-corrected_yaw_mag[0])
corrected_yaw_int_final = wrap_to_pi(corrected_yaw_int)
# plt.close()
# plt.plot(circles_imu[:,0],corrected_yaw_mag,'b-',label='Mag Corrected Yaw')
# plt.plot(circles_imu[:,0],corrected_yaw_int_final,'g-',label='Angular Integrated Yaw')
# plt.grid()
# plt.legend()
# plt.title('Corrected Yaw Data')
# plt.ylabel('Yaw (rad)')
# plt.xlabel('Time (s)')
# plt.show()

########## Complementary filter ##########
corrected_yaw_mag_unwrapped = np.unwrap(corrected_yaw_mag)
corrected_complement = 0.9*corrected_yaw_int + 0.1*corrected_yaw_mag_unwrapped
corrected_complement_final = wrap_to_pi(corrected_complement)
# plt.close()
# plt.plot(circles_imu[:,0],corrected_yaw_mag,'b-',label='Mag Corrected Yaw')
# plt.plot(circles_imu[:,0],corrected_yaw_int_final,'g-',label='Angular Integrated Yaw')
# plt.plot(circles_imu[:,0],corrected_complement_final,'r-',label='Complementary Filter')
# plt.grid()
# plt.legend()
# plt.title('Corrected Yaw Data')
# plt.ylabel('Yaw (rad)')
# plt.xlabel('Time (s)')
# plt.show()

########## compare to raw yaw ##########
# corr_comp_final_unwrap = np.unwrap(corrected_complement_final)
# corr_comp_final_unwrap = corr_comp_final_unwrap - (corr_comp_final_unwrap[0]-yaw[0])
# corrected_complement_final_wrapped = wrap_to_pi(corr_comp_final_unwrap)
# plt.close()
# plt.plot(circles_imu[:,0],corrected_complement_final_wrapped,'r-',label='Complementary Filter')
# plt.plot(circles_imu[:,0],yaw,'b-',label='Raw Yaw')
# plt.grid()
# plt.legend()
# plt.title('Compared Yaw Data')
# plt.ylabel('Yaw (rad)')
# plt.xlabel('Time (s)')
# plt.show()



########## Velocity calculations ##########
integrated_linacc_x = integrate.cumtrapz(moving_imu[:,8],moving_imu[:,0],initial=0)
# integrated_linacc_x = integrated_linacc_x - np.mean(stationary_imu[:,8])
dist_traveled_gps = np.empty((0))
for i in range(moving_gps.shape[0]-1):
    dist = np.sqrt(np.square(moving_gps[i,4]-moving_gps[i+1,4]) + np.square(moving_gps[i,5]-moving_gps[i+1,5]))
    dist_traveled_gps = np.append(dist_traveled_gps,dist)

dt = np.empty((0))
for i in range(moving_gps.shape[0]-1):
    time_diff = moving_gps[i+1,0] - moving_gps[i,0]
    dt = np.append(dt,time_diff)
derived_position_gps = dist_traveled_gps/dt

x_data = np.array([moving_imu[500,0], moving_imu[-1,0]])
y_data = np.array([integrated_linacc_x[500], integrated_linacc_x[-1]])
m,b = np.polyfit(x_data,y_data,1)
dist_lobf = np.absolute(moving_imu[:,0]*m - integrated_linacc_x[:] + b)/(np.sqrt(m**2+1))
dist_lobf[12632:29632] = 0.5*dist_lobf[12632:29632]
# plt.close()
# plt.plot(moving_imu[:,0],integrated_linacc_x,'b-',label='Integrated IMU Lin Acc X')
# plt.plot(moving_imu[:,0],dist_lobf,'c-',label='Integrated Lin Acc X Compared to New Zero')
# plt.plot(moving_imu[:,0],m*moving_imu[:,0] + b,'r-',label='Integrated Lin Acc X Zero')
# plt.plot(moving_gps[:1160,0],derived_position_gps,'g-',label='Derivative of GPS Position')
# plt.grid()
# plt.legend()
# plt.title('Velocity vs. Time')
# plt.ylabel('Velocity (m/s)')
# plt.xlabel('Time (s)')
# plt.show()





########## Dead Reckoning ##########
corrected_moving_imu_magx = moving_imu[:,11] - circles_imu_avgx
corrected_moving_imu_magy = moving_imu[:,12] - circles_imu_avgy
corrected_moving_yaw_mag = np.arctan2(corrected_moving_imu_magx,corrected_moving_imu_magy)
corrected_moving_yaw_int = integrate.cumtrapz(moving_imu[:,7],moving_imu[:,0],initial=0)
corrected_moving_yaw_int_final = wrap_to_pi(corrected_moving_yaw_int)
corrected_moving_yaw_mag_unwrapped = np.unwrap(corrected_moving_yaw_mag)
corrected_moving_complement = 0.98*corrected_moving_yaw_int + 0.02*corrected_moving_yaw_mag_unwrapped
corrected_moving_complement_final = wrap_to_pi(corrected_moving_complement)
corrected_moving_complement_final = -1*corrected_moving_complement_final + 105*np.pi/180
# roll_moving,pitch_moving,yaw_moving = euler_from_quaternion(moving_imu[:,1],moving_imu[:,2],moving_imu[:,3],moving_imu[:,4])
# yaw_moving = -1*yaw_moving + 80*np.pi/180

# corrected_moving_yaw_mag = -1*corrected_moving_yaw_mag + 195*np.pi/180
# Ve = np.cos(corrected_moving_yaw_mag)
# Vn = np.sin(corrected_moving_yaw_mag)

Ve = np.cos(corrected_moving_complement_final)
Vn = np.sin(corrected_moving_complement_final)

Ve_mag = Ve * dist_lobf
Vn_mag = Vn * dist_lobf
pos_Ve = integrate.cumtrapz(Ve_mag,moving_imu[:,0],initial=0)
pos_Vn = integrate.cumtrapz(Vn_mag,moving_imu[:,0],initial=0)
pos_Ve = pos_Ve + moving_gps[0,4]
pos_Vn = pos_Vn + moving_gps[0,5]

# plt.close()
# plt.plot(moving_gps[:1000,4],moving_gps[:1000,5],'c-',label='GPS X and Y Data')
# plt.plot(pos_Ve[:41500],pos_Vn[:41500],'g-',label='IMU Integrated Data')
# plt.grid()
# plt.legend()
# plt.title('Position Data Using Complementary Filter')
# plt.ylabel('Y Position (m)')
# plt.xlabel('X Position (m)')
# plt.show()

########## 3.1 ##########
w_xdot = moving_imu[:,7]*integrated_linacc_x

# plt.close()
# plt.plot(moving_imu[:,0],moving_imu[:,9],'c-',label='Y double dot')
# plt.plot(moving_imu[:,0],w_xdot,'g-',label='W * Xdot')
# plt.grid()
# plt.legend()
# plt.title('Y double dot compared to W*Xdot')
# plt.ylabel('Acceleration (m/s^2)')
# plt.xlabel('Time (s)')
# plt.show()

########## Xc ##########
dt_moving = np.empty((0))
for i in range(moving_imu.shape[0]-1):
    time_diff = moving_imu[i+1,0] - moving_imu[i,0]
    dt_moving = np.append(dt_moving,time_diff)
derived_wz = moving_imu[:46416,7]/dt_moving
Xc_tot = (-1*(w_xdot[:46416]/derived_wz))
Xc_tot = Xc_tot[np.logical_not(np.isnan(Xc_tot))]
Xc = np.mean(Xc_tot)