# EECE 5554 - Robotic Sensing and Navigation
This repo holds all labs completed for my Robotic Sensing and Navigation course (Spring 2022).

# Lab Descriptions

## Lab 1 --- GPS Analyzation
Error analyzation of both stationary and transient GPS data.

***Report:*** `LAB1_GPS_Driver&Analyzation/src/Lab1_Report.pdf`

### Structure
|Main Folders + Files|Description|
|--------------------|-----------|
|`src/Gps_driver/`|Main folder of the ROS package, directly holding the `CMakeLists.txt` and `package.xml` files|
|`src/Gps_driver/msg/GPS_msg.msg`|ROS `.msg` file used to publish the recorded data|
|`src/Gps_driver/scripts/LAB1_publisher.py`|Main ROS publisher script used to publish data to the ROS node|
|`src/analysis_scripts/`|Holds the scripts used to analyze the recorded data|
|`src/analysis_scripts/moving_analyzer.py`|Script used to analyze the transient GPS data|
|`src/analysis_scripts/stationary_analyzer.py`|Script used to analyze the stationary GPS data|
|`src/data/`|Holds the raw data files in `.bag` and `.txt` files|

## Lab 2 --- RTK-GPS Analyzation

Error analyzation of both stationary and transient RTK-GPS data.

***Report:*** `LAB2_RTK-GPS_Driver&Analyzation/Lab2_Report.pdf`

### Structure
|Main Folders + Files|Description|
|--------------------|-----------|
|`src/Lab2_package/`|Main folder of the ROS package, directly holding the `CMakeLists.txt` and `package.xml` files|
|`src/Lab2_package/msg/Lab2_msg.msg`|ROS `.msg` file used to publish the recorded data|
|`src/Lab2_package/scripts/Lab2_publisher.py`|Main ROS publisher script used to publish data to the ROS node|
|`src/analysis/`|Holds the scripts used to analyze the recorded data|
|`src/analysis/moving_analyzer.py`|Script used to analyze the transient GPS data|
|`src/analysis/stationary_analyzer.py`|Script used to analyze the stationary GPS data|
|`data/`|Holds the raw data files in `.bag` and `.txt` files|

## Lab 3 --- IMU Analyzation 

Error analyzation of stationary IMU data, including allan variance analysis.

***Report:*** `LAB3_IMU_Stationary/Lab3_Report.pdf`

### Structure
|Main Folders + Files|Description|
|--------------------|-----------|
|`src/lab3_package/`|Main folder of the ROS package, directly holding the `CMakeLists.txt` and `package.xml` files|
|`src/lab3_package/msg/IMU_msg.msg`|ROS `.msg` file used to publish the recorded data|
|`src/lab3_package/scripts/lab3_publisher.py`|Main ROS publisher script used to publish data to the ROS node|
|`src/analysis/`|Holds the scripts used to analyze the recorded data|
|`src/analysis/group_analyzer.py`|Script used to analyze the group collected stationary IMU data|
|`src/analysis/individual_analyzer.py`|Script used to analyze the individually collected transient IMU data|
|`src/analysis/allan_variance.m`|MATLAB script used to plot the allan variance plots of the stationary IMU data|
|`data/`|Holds the raw data files in `.bag` and `.csv` files|


## Lab 4 --- IMU Localization

Using IMU magnetometer, accelerometer, and angular data to plot the trajectory of a car ride. Implemented soft iron heading correction, yaw calculations from magnetometer data, velocity calculations from integrated accelerometer data, trajectory estimations via dead reckoning, and a calculation of the IMU placement with respect to the vehicle frame.

***Report:*** `LAB4_IMU+GPS_Localization/Lab4_Report.pdf`

### Structure
|Main Folders + Files|Description|
|--------------------|-----------|
|`src/lab4_package/`|Main folder of the ROS package, directly holding the `CMakeLists.txt` and `package.xml` files|
|`src/lab4_package/msg/IMU_msg.msg`|ROS `.msg` file used to publish the incoming IMU data|
|`src/lab4_package/msg/GPS_msg.msg`|ROS `.msg` file used to publish the incoming GPS data|
|`src/lab4_package/scripts/imu_data_pub_lab4.py`|ROS publisher script used to publish IMU data to the ROS node|
|`src/lab4_package/scripts/gps_data_pub_lab4.py`|ROS publisher script used to publish GPS data to the ROS node|
|`src/analysis/`|Holds the script used to analyze the recorded data|
|`src/analysis/analyzer.py`|Script used to perform all analyzations|
|`src/data/`|Holds the raw data files in `.bag` and `.csv` files|


## Lab 5 --- Image Stitching

Image stitching of murals across campus via Harris corner detectors and feature matching.

***Report:*** `Lab5_Image_Stitching/Lab5_Report.pdf`
***Code:*** `Lab5_Image_Stitching/EECE5554_Lab5.m`

## Mini Project
Mini project done on a meal delivery AMR, including feature, power, navigation and sensing analyses.

***Report:*** `Mini_Project/EECE5554_LukeDavidson_IndividualProject.pdf`
