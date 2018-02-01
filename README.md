# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

## Overview
This project implemented an Extended Kalman Filter for RADAR and LIDAR sensor data. The Udacity simulator generates noisy RADAR and LIDAR measurements of the position and velocity of an object. This project implemented the Extended Kalman Filter  to fusion those measurements to predict the position and velocity of the object. 

## Prerequisites

+ cmake >= 3.5
+ make >= 4.1
+ gcc/g++ >= 5.4
+ [Udacity's simulator](https://github.com/udacity/self-driving-car-sim/releases)

As described in [Udacity seed project](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project), when you developed in Ubuntu 16.04, you should run [install-ubuntu.sh](https://github.com/wuqianliang/CarND-EKF-Project/blob/master/install-ubuntu.sh) script to install uWebsocket and other required packages.

## Basic Build Instructions
+ Clone this repo and cd directory which include CMakeLists.txt
+ cmake .
+ make (This will create ExtendedKF executable) 

## Running Extended Kalman Filter
After launch the simulator, when executed ./ExtendedKF, output will be:
>     Listening to port 4567
>     Connected!!!


# Rubric points
## Accuracy
You can see in the above image,the final RMSE is:

dataset1 : RMSE of \[px,py,vx,vy\] is \[0.0946,0.0833,0.4499,0.4347\]
dataset1 : RMSE of \[px,py,vx,vy\] is \[0.0756,0.0958,0.4591,0.4941\]

## Follows the Correct Algorithm
### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
yes!!

### Your Kalman Filter algorithm handles the first measurements appropriately.
src/FusionEKF.cpp line 63 ~ 118.

### Your Kalman Filter algorithm first predicts then updates.
First predict at line 156 of src/FusionEKF.cpp then update at line 176 and line 183.

### Your Kalman Filter can handle radar and lidar measurements.
Implement measurement update at line 34 and line 53 of in src/kalman_filter.cpp

## Code Efficiency
These requirements also are satisfied!
