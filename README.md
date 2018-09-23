# Extended Kalman Filter Project Writeup
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## Rubric - Compilation
No changes have been made to the CMakeLists.txt. Simple `cmake` and `make` should be enough to compile

## Accuracy 
* For dataset 1, the RMSE (X,Y, VX,VY) are 0.0973, 0.0855, 0.4513, 0.4399 which is under the expected .11, .11, 0.52, 0.52 values.
* For dataset 2, the RMSE (X,Y, VX,VY) are 0.0726, 0.0967, 0.4579, 0.4966 which is under the expected .11, .11, 0.52, 0.52 values.

