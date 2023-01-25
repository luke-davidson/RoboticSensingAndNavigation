import numpy as np
import matplotlib.pyplot as plt

"""
Reads Lab 2 moving data in a .txt file and analyzes the data in the column structure
    0: latitude
    1: longitude
    2: altitude
    3: UTM easting
    4: UTM northing
    5: GNSS Fix
author: davidson.lu@northeastern.edu
"""

########## load raw data ##########
mode = "isec"   # {"isec", "soccer"}
raw_data = np.loadtxt('Downloads/Soccer_field_moving_txt.txt').reshape(848, 6) if mode == "soccer" else np.loadtxt('Downloads/ISEC_moving_txt.txt').reshape(1259, 6)

########## calculate averages ##########
averages = np.mean(raw_data, axis=0)

########## diff between values and averages ##########
differences = np.empty_like(raw_data)

for i in range(raw_data.shape[1]):
    differences[:, i] = raw_data[:, i] - averages[i,]
    diff_abs = np.absolute(differences)

avg_diff = np.mean(diff_abs, axis=0)

########## plotting UTM data ########## Soccer == 0:275, 290:440, 455:675, 690: ##### ISEC == 0:160, 170:645, 660:765, 770:
plt.close()
plt.plot(raw_data[:, 3], raw_data[:, 4], 'b-', label='Data')

if mode == "soccer":
    # Soccer field
    m1, b1 = np.polyfit(raw_data[0:275, 3], raw_data[0:275, 4], 1)
    m2, b2 = np.polyfit(raw_data[290:440, 3], raw_data[290:440, 4], 1)
    m3, b3 = np.polyfit(raw_data[455:675, 3], raw_data[455:675, 4], 1)
    m4, b4 = np.polyfit(raw_data[690:, 3], raw_data[690:, 4], 1)
    plt.plot(raw_data[0:275, 3], m1*raw_data[0:275, 3]+b1, 'r-', label='Line of Best Fit 1')
    plt.plot(raw_data[290:440, 3], m2*raw_data[290:440, 3]+b2, 'g-', label='Line of Best Fit 2')
    plt.plot(raw_data[455:675, 3], m3*raw_data[455:675, 3]+b3, 'y-', label='Line of Best Fit 3')
    plt.plot(raw_data[690:, 3], m4*raw_data[690:, 3]+b4, 'c-', label='Line of Best Fit 4')
elif mode == "isec":
    # ISEC
    m1,b1 = np.polyfit(raw_data[0:160, 3], raw_data[0:160, 4], 1)
    m2,b2 = np.polyfit(raw_data[170:645, 3], raw_data[170:645, 4], 1)
    m3,b3 = np.polyfit(raw_data[660:765, 3], raw_data[660:765, 4], 1)
    m4,b4 = np.polyfit(raw_data[770:, 3], raw_data[770:, 4], 1)
    plt.plot(raw_data[0:160, 3], m1*raw_data[0:160, 3]+b1, 'r-', label='Line of Best Fit 1')
    plt.plot(raw_data[170:645, 3], m2*raw_data[170:645, 3]+b2, 'g-', label='Line of Best Fit 2')
    plt.plot(raw_data[660:765, 3], m3*raw_data[660:765, 3]+b3, 'y-', label='Line of Best Fit 3')
    plt.plot(raw_data[770:, 3], m4*raw_data[770:, 3]+b4, 'c-', label='Line of Best Fit 4')

plt.grid()
plt.legend()
plt.title('ISEC Transient UTM Data')
plt.ylabel('UTM Northing (m)')
plt.xlabel('UTM Easting (m)')
plt.show()

########## dist from line of best fit UTM ##########
def dist_lobf(x_data, y_data, m, b):
    dist_eq = np.absolute(x_data*m - y_data + b)/(np.sqrt(m**2+1))
    avgerage_dist_from_lobf = np.mean(dist_eq)
    return avgerage_dist_from_lobf
if mode == "soccer":
    # Soccer field
    avg_error_1 = dist_lobf(raw_data[0:275, 3], raw_data[0:275, 4], m1, b1)
    avg_error_2 = dist_lobf(raw_data[290:440, 3], raw_data[290:440, 4], m2, b2)
    avg_error_3 = dist_lobf(raw_data[455:675, 3], raw_data[455:675, 4], m3, b3)
    avg_error_4 = dist_lobf(raw_data[690:, 3], raw_data[690:, 4], m4, b4)
elif mode == "isec":
    # ISEC
    avg_error_1 = dist_lobf(raw_data[0:160, 3], raw_data[0:160, 4], m1, b1)
    avg_error_2 = dist_lobf(raw_data[170:645, 3], raw_data[170:645, 4], m2, b2)
    avg_error_3 = dist_lobf(raw_data[660:765, 3], raw_data[660:765, 4], m3, b3)
    avg_error_4 = dist_lobf(raw_data[770:, 3], raw_data[770:, 4], m4, b4)
print('Avg error 1: {}'.format(avg_error_1))
print('Avg error 2: {}'.format(avg_error_2))
print('Avg error 3: {}'.format(avg_error_3))
print('Avg error 4: {}'.format(avg_error_4))
print('Total Avg Error: {}'.format((avg_error_1+avg_error_2+avg_error_3+avg_error_4)/4))

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
    m,b = np.polyfit(x_data, y_data, 1)
    plt.plot(x_data, y_data, 'b-', label='Data')
    plt.plot(x_data, m*x_data+b, 'r-', label='Line of Best Fit')
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)
    plt.xlim(right=length)
    plt.legend()
    plt.grid()
    plt.show()

########## latitude vs trial ##########
data_vs_trial(raw_data[:, 0], 998, 'Latitude vs. Trial', 'Latitude (deg)', 'Trial #')
########## longitude vs trial ##########
data_vs_trial(raw_data[:, 1], 998, 'Longitude vs. Trial', 'Longitude (deg)', 'Trial #')
########## altitude vs trial ##########
data_vs_trial(raw_data[:, 2], 848, 'Soccer Field Transient Altitude vs. Trial', 'Altitude (m)', 'Trial #')
########## UTM Easting vs trial ##########
data_vs_trial(raw_data[:, 3], 848, 'UTM Easting vs. Trial', 'UTM Easting (m)', 'Trial #')
########## UTM Northing vs trial ##########
data_vs_trial(raw_data[:, 4], 848, 'UTM Northing vs. Trial', 'UTM Northing (m)', 'Trial #')
########## GNSS Fix vs trial ##########
data_vs_trial(raw_data[:, 5], 1259, 'GNSS Fix vs. Trial', 'GNSS Fix', 'Trial #')

for i in range(5):
    i += 1
    count = 0
    for j in range(848):
        if raw_data[j,5] == i:
            count += 1
    print('Number of {}s: {}'.format(i, count))

labels = ['Fix']
sizes = [100]
colors = ['g']
fig1, ax1 = plt.subplots()
ax1.pie(sizes, labels=labels, autopct='%1.1f%%', shadow=True, colors=colors, startangle=90)
ax1.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.
plt.title('Soccer Field Trasient Data GNSS Fix Status\n')
plt.show()