#  **Extended Kalman Filter** 
## Udacity Self-Driving Car Nanodegree Project 5

**The goals / steps of this project are the following:**

* Establish communication between c++ program and simulator via uWebSocketIO
* Obtain lidar and radar measurement data on a bicycle and true ground value from simulator
* Utilize a kalman and extended kalman filter to estimate the state (position and velocity)
* Calculate and ensure RMSE value not greater than the tolerance( [.11, .11, 0.52, 0.52] ) outlined in the project rubric.
* Send RMSE values to simulator

[//]: # (Image References)

[image1]: ./examples/placeholder.png "Model Visualization"
[image2]: ./examples/placeholder.png "Grayscaling"
[image3]: ./examples/placeholder_small.png "Recovery Image"


### Environment and File Submitted

#### 1. Environment set up

* Download simulator from Udacity's repo on Github to local drive
* Install Windows Subsystem for Linux - WSL
* Install Ubuntu and set up username and password
* Follow the instruction of "Environment Setup (Linux)" to do followings:
 * install git
 * install cmake
 * install openssl 
 * install libssl-dev
 * clone project from Udacity's repo on Github
 * install uWebSocketIO


#### 2. Files submitted below:

File Name | Description
----------|-----------
FusionEKF.cpp | initialzation and fusion the kalman filter result of laser and radar sensor 
kalman_filter.cpp |prediction, update  function of kalman filter and extended kalmna filter
tools.cpp | calcaute RMSE and Jacobian Matrix
writeup_report.md | summary of the project and the same contents as README.MD



### Coding Process

There are only 3 cpp files below need to be finished. 
* FusionEKF.cpp
* kalman_filter.cpp
* tools.cpp

#### FusionEKF.cpp:

The main.cpp receives the following values from the simulator and passes these data in "meas_package" to FusionEKF.cpp
* Lidar measurement: x,y
* Radar measurement: rho, phi, rhodot
* Groundtruth: x, y, vx, vy
* Time: timestamp

The code need to do initialziation the first time it was called

Variable | Description
----------|-----------
R_laser_ |measurement covariance matrix - LIDAR 
R_radar_ |measurement covariance matrix - RADAR
H_laser_ | Measurement matrix - LIDAR
Hj_  | Measurement matrix - RADAR
ekf_.F_|Initial transition matrix F_
ekf_.P_|State covariance matrix P

Then predicts for both Lidar and Radar sensor, 
* set the state transitional matrix F
* set the process covariance matrix Q
* call predict fuctnion in kalman_filter.cpp

Then updates for Radar sensor
* call function in tools.cpp to get the Jacobian Matrix
* call udpateEKF fuction in kalman_filter.cpp to update state and P matrix

Or udpates for Lidar sensor
* call udpate fucntion on kalman_filter.cpp to udpate state and P matrix

At last the main.cpp will get RMSE from tools.cpp and passes following values to the simulator
* kalman filter estimated position x, y
* RMSE x,y,vx,vy


#### kalman_filter.cpp
There are three functions as below need to be updated:
* KalmanFilter Predict function for both Lidar and Radar
* KalmanFilter Update function for Lidar 
* Extended KalmanFilter Update function for Radar

#### tools.cpp
There are two functions as below need to be updated:
* Calculate RMSE for main.cpp
* Calculate Jacobian Matrix ( Measurement matrix - RADAR ) for FusionEKF.cpp


##  Compile, Build and Run 

* Make a build directory: `mkdir build && cd build`
* Compile: `cmake .. && make` 
* Run it: `./ExtendedKF ` and the command line will show "Listening to port 4567"
* Run simulator, click "play" button , select " Project 1/2: EKF and UKF " and bash command line will show "Connected!!!".
* Choose Dataset 1 and click "Start" Button to start and the simulator will show the RMSE data from FusionEKF code.




## Discussion

The challenge to me for this project was to set up the Linux environment and to understand the process flow. Fortunately the instruction from the class were very clear and with the help from my mentor, I successfuly set up the environment of WSL, Ubuntu and cmake. Thought this step took me many hours but it was great oppotunity to get familar with these environments.  

Then the next challenge was to understand the process flow among main.cpp, FusionEKF.cpp,kalman_filter.cpp and tools.cpp. There are sample code from class on kalman filter predict and update, extended kalman filter update fuction, RMSE and Jacobian calucation which was very helpful. The FusionEKF.cpp was the most difficult part for me andI have to read the class material back and forth to dive deeper. While the whole process have lots of pain and fun. Looking forward to the next challenge! 


