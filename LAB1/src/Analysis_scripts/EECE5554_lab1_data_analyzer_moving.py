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
raw_data = np.loadtxt('Downloads/moving_data_txt.txt').reshape(618,5)

########## resolving data recording error ##########
Lat_1 = raw_data[214,0]-raw_data[215,0]
Lat_2 = raw_data[549,0]-raw_data[550,0]
Long_1 = raw_data[214,1]-raw_data[215,1]
Long_2 = raw_data[549,1]-raw_data[550,1]
Alt_1 = raw_data[214,2]-raw_data[215,2]
Alt_2 = raw_data[549,2]-raw_data[550,2]
UTM_E_1 = raw_data[214,3]-raw_data[215,3]
UTM_E_2 = raw_data[549,3]-raw_data[550,3]
UTM_N_1 = raw_data[214,4]-raw_data[215,4]
UTM_N_2 = raw_data[549,4]-raw_data[550,4]

for i in range(618):
    if i > 214 and i <= 549:
        raw_data[i,0] = raw_data[i,0] + Lat_1
        raw_data[i,1] = raw_data[i,1] + Long_1
        raw_data[i,2] = raw_data[i,2] + Alt_1
        raw_data[i,3] = raw_data[i,3] + UTM_E_1
        raw_data[i,4] = raw_data[i,4] + UTM_N_1
    elif i > 549:
        raw_data[i,0] = raw_data[i,0] + Lat_1 + Lat_2
        raw_data[i,1] = raw_data[i,1] + Long_1 + Long_2
        raw_data[i,2] = raw_data[i,2] + Alt_1 + Alt_2
        raw_data[i,3] = raw_data[i,3] + UTM_E_1 + UTM_E_2
        raw_data[i,4] = raw_data[i,4] + UTM_N_1 + UTM_N_2

########## calculate averages ##########
averages = np.mean(raw_data,axis=0)

########## diff between values and averages ##########
differences = np.empty_like(raw_data)

for i in range(raw_data.shape[1]):
    differences[:,i] = raw_data[:,i] - averages[i,]
    diff_abs = np.absolute(differences)

avg_diff = np.mean(diff_abs,axis=0)

########## plotting UTM data ##########
plt.close()
plt.plot(raw_data[:,3],raw_data[:,4],'b-',label='Data')
m,b = np.polyfit(raw_data[:,3],raw_data[:,4],1)
plt.plot(raw_data[:,3],m*raw_data[:,3]+b,'r-',label='Line of Best Fit')
plt.grid()
plt.legend()
plt.title('UTM Data')
plt.ylabel('UTM Northing (m)')
plt.xlabel('UTM Easting (m)')
plt.show()

########## dist from line of best fit UTM ##########
dist_lobf = np.absolute(raw_data[:,3]*m - raw_data[:,4] + b)/(np.sqrt(m**2+1))
average_error = np.mean(dist_lobf)
print('Avg error: {}'.format(average_error))

########## plotting data vs trial ##########
def data_vs_trial(y_data,length,title,ylabel,xlabel):
    """
    y_data: array, y data to be plotted
    length: int, length of data for x range
    title: String, title
    ylabel: String, y-label
    xlabel: String, x-label
    """
    x_data = np.arange(length)
    m,b = np.polyfit(x_data,y_data,1)
    plt.plot(x_data,y_data,'b-',label='Data')
    plt.plot(x_data,m*x_data+b,'r-',label='Line of Best Fit')
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)
    plt.xlim(right=length)
    plt.legend()
    plt.show()

# ########## latitude vs trial ##########
# data_vs_trial(raw_data[:,0],618,'Latitude vs. Trial','Latitude (deg)','Trial #')
# ########## longitude vs trial ##########
# data_vs_trial(raw_data[:,1],618,'Longitude vs. Trial','Longitude (deg)','Trial #')
# ########## altitude vs trial ##########
# data_vs_trial(raw_data[:,2],618,'Altitude vs. Trial','Altitude (deg)','Trial #')
# ########## UTM Easting vs trial ##########
# data_vs_trial(raw_data[:,3],618,'UTM Easting vs. Trial','UTM Easting (m)','Trial #')
# ########## UTM Northing vs trial ##########
# data_vs_trial(raw_data[:,4],618,'UTM Northing vs. Trial','UTM Northing (m)','Trial #')