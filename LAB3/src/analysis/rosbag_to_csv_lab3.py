import pandas as pd
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np

# Read in to csv file
bag = bagreader('/home/luke/Downloads/2022-02-26-15-41-54.bag')
imu_data = bag.message_by_topic(topic = '/imu_data')
imu_data_csv = pd.read_csv(imu_data)
imu_data_pd = pd.DataFrame(imu_data_csv,columns=['Time','magnetic_field.x','magnetic_field.y','magnetic_field.z','linear_acceleration.x','linear_acceleration.y','linear_acceleration.z','angular_velocity.x','angular_velocity.y','angular_velocity.z','orientation.x','orientation.y','orientation.z','orientation.w']).astype(float)

# Define time series plotting function
def plot_data_by_time(y_data,title,ylabel):
    """
    Plots data vs time.
    y_data: string, name of column to be plotted
    title: string, title of graph
    ylabel: string, Y-label of graph
    """
    plt.close()
    plt.plot(imu_data_pd['Time'],imu_data_pd[y_data])
    plt.grid()
    plt.title(title)
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    plt.show()

# # Plot each time series
# plot_data_by_time('magnetic_field.x','Magnetic Field-X vs Time','Magnetic Field-X (???)')
# plot_data_by_time('magnetic_field.y','Magnetic Field-Y vs Time','Magnetic Field-Y (???)')
# plot_data_by_time('magnetic_field.z','Magnetic Field-Z vs Time','Magnetic Field-Z (???)')
# plot_data_by_time('linear_acceleration.x','Linear Acceleration-X vs Time','Linear Acceleration-X (???)')
# plot_data_by_time('linear_acceleration.y','Linear Acceleration-Y vs Time','Linear Acceleration-Y (???)')
# plot_data_by_time('linear_acceleration.z','Linear Acceleration-Z vs Time','Linear Acceleration-Z (???)')
# plot_data_by_time('angular_velocity.x','Angular Velocity-X vs Time','Angular Velocity-X (???)')
# plot_data_by_time('angular_velocity.y','Angular Velocity-Y vs Time','Angular Velocity-Y (???)')
# plot_data_by_time('angular_velocity.z','Angular Velocity-Z vs Time','Angular Velocity-Z (???)')
# plot_data_by_time('orientation.x','Orientation-X vs Time','Orientation-X (???)')
# plot_data_by_time('orientation.y','Orientation-Y vs Time','Orientation-Y (???)')
# plot_data_by_time('orientation.z','Orientation-Z vs Time','Orientation-Z (???)')
# plot_data_by_time('orientation.w','Orientation-W vs Time','Orientation-W (???)')

# Define mean and standard deviation function
def mean_and_stddev(data):
    """
    Calculates mean and standard deviation of a dataset.
    data: string, name of dataframe column to analyze
    """
    mean = np.mean(imu_data_pd[data])
    std_dev = np.sqrt(np.sum(np.square(imu_data_pd[data]-mean))/30013)

    return mean,std_dev

# Initialize array to store means and standard deviaitons and calculate values
mean_stddev = np.zeros((13,2))
mean_stddev[0,:] = mean_and_stddev('magnetic_field.x')
mean_stddev[1,:] = mean_and_stddev('magnetic_field.y')
mean_stddev[2,:] = mean_and_stddev('magnetic_field.z')
mean_stddev[3,:] = mean_and_stddev('linear_acceleration.x')
mean_stddev[4,:] = mean_and_stddev('linear_acceleration.y')
mean_stddev[5,:] = mean_and_stddev('linear_acceleration.z')
mean_stddev[6,:] = mean_and_stddev('angular_velocity.x')
mean_stddev[7,:] = mean_and_stddev('angular_velocity.y')
mean_stddev[8,:] = mean_and_stddev('angular_velocity.z')
mean_stddev[9,:] = mean_and_stddev('orientation.x')
mean_stddev[10,:] = mean_and_stddev('orientation.y')
mean_stddev[11,:] = mean_and_stddev('orientation.z')
mean_stddev[12,:] = mean_and_stddev('orientation.w')

print(mean_stddev)