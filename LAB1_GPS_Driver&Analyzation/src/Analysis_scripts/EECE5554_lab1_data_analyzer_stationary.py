import numpy as np
import matplotlib.pyplot as plt

"""
Reads data in a .txt file and analyzes the data in the column structure
    0: latitude
    1: longitude
    2: altitude
    3: UTM easting
    4: UTM northing
author: davidson.lu@northeastern.edu
"""

########## load raw data ##########
raw_data = np.loadtxt('Downloads/gps_data_txt.txt').reshape(610, 5)

########## calculate averages ##########
averages = np.mean(raw_data, axis=0)

########## diff between values and averages ##########
differences = np.empty_like(raw_data)

for i in range(raw_data.shape[1]):
    differences[:, i] = raw_data[:, i] - averages[i,]
    diff_abs = np.absolute(differences)

avg_diff = np.mean(diff_abs, axis=0)

print('--------------- Average Difference ---------------')
print('Latitude: {}'.format(avg_diff[0]))
print('Longitude: {}'.format(avg_diff[1]))
print('Altitude: {}'.format(avg_diff[2]))
print('UTM Easting: {}'.format(avg_diff[3]))
print('UTM Northing: {}'.format(avg_diff[4]))
print('--------------------------------------------------')

########## plotting UTM data ##########
plt.close()
plt.scatter(differences[:, 3], differences[:, 4], s=50, c='blue', label='Raw Data')
plt.scatter(0, 0, s=300, c='red', label='Average')
plt.legend()
plt.grid()
plt.title('UTM Data Compared to Average')
plt.ylabel('UTM Northing (m)')
plt.xlabel('UTM Easting (m)')
plt.show()

########## plotting data vs trial ##########
def data_vs_trial(y_data, length, title, ylabel, xlabel):
    """
    y_data: array, y data to be plotted
    length: int, length of data for x range
    title: String, title
    ylabel: String, y-label
    xlabel: String, x-label
    """
    x_data = np.arange(length)
    plt.plot(x_data, y_data)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)
    plt.xlim(right=length)
    plt.show()

########## latitude vs trial ##########
data_vs_trial(raw_data[:, 0], 610, 'Latitude vs. Trial', 'Latitude (deg)', 'Trial #')
########## longitude vs trial ##########
data_vs_trial(raw_data[:, 1], 610, 'Longitude vs. Trial', 'Longitude (deg)', 'Trial #')
########## altitude vs trial ##########
data_vs_trial(raw_data[:, 2], 610, 'Altitude vs. Trial', 'Altitude (deg)', 'Trial #')
########## UTM Easting vs trial ##########
data_vs_trial(raw_data[:, 3], 610, 'UTM Easting vs. Trial', 'UTM Easting (m)', 'Trial #')
########## UTM Northing vs trial ##########
data_vs_trial(raw_data[:, 4], 610, 'UTM Northing vs. Trial', 'UTM Northing (m)', 'Trial #')

########## point to point distance from average UTM ##########
easting_diff = np.square(raw_data[:, 3] - averages[3])
northing_diff = np.square(raw_data[:, 4] - averages[4])
pt_to_pt_diff = np.sqrt(easting_diff + northing_diff)
print('Average distance: {}'.format(np.mean(pt_to_pt_diff)))
data_vs_trial(pt_to_pt_diff, 610, 'UTM Distance from Average', 'Distance from Average (m)', 'Trial #')