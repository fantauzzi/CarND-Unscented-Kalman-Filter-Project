# Unscented Kalman Filter Project
*Self-Driving Car Engineer Nanodegree Program*
---
In this project I implemented an Unscented Kalman Filter (UKF) to track a moving object with noisy lidar and radar measurements.  

The UKF is a state estimator for continuous space belonging to the family of Bayes filters. For the algorithm details, a good reference is "Probabilistic Robotics", written by Sebastian Thrun, Wolfram Burgard and Dieter Fox, The MIT Press. 

[//]: # (Image References)

[image1]: ./readme_figures/nis_lidar.png "NIS for Lidar"
[image2]: ./readme_figures/nis_radar.png "NIS for Radar"
[image3]: ./readme_figures/position_error.png "Position error"
[image4]: ./readme_figures/velocity_error.png "Velocity error"
[image5]: ./readme_figures/yaw_error.png "Yaw error"
[image6]: ./readme_figures/yaw_rate_error.png "Yaw rate error"
[image7]: ./readme_figures/simulator.png "Simulator"
[image8]: ./readme_figures/smooth.png "Tracking in the simulator"

## Dependencies
The program requires Udacity's simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). 

The simulator sends the program implementing the UKF noisy measurements from Lidar and Radar. The program estimates the state of the tracked object, and sends it back to the simulator for visualisation.

The following are also required:
* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

The program was tested under Ubuntu 16.04.

## Build and Run the Program
Once cloned (or downloaded) the project repository, in the project directory type:

1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`
5. `./UnscentedKF`

Run the simulator, make sure **Project 1/2: EKF and UKF** is selected in the simulator and click **SELECT**. On the next screen, you can choose between one of two datasets. Click **Start** to have the simulator begin to send data to the running UKF implementation. 

To stop the UKF program execution hit `Ctrl+C`.

In the screenshot of the simulator, below, the blue car represents the tracked object, sensors are in the origin of the reference system, the trail of blue dots is the series of Radar measurements, while red dots are Lidar measurements; green triangles are positions of the tracked object as estimated by the UKF. 

![alt text][image7]

The UKF program outputs a text file `data/out.txt`, with one line per sensors measurement, reporting the corresponding estimated state of the tracked object. On each line there is, tab separated:
- timestamp, as number of microseconds elapsed from an epoch;
- x and y coordinates of the object position in meters;
- tangential velocity in m/s;
- yaw in radians;
- yaw change rate in radians/s;
- computed normalised innovation square (NIS).

## Considerations on the Results

### Sensor Fusion
The UKF can be used to combine data from different sensors (sensor fusion), to achieve more accurate results. The simulator produces measurements from two kind of sensors, Radar and Lidar. Let's see the impact of either on the UKF accuracy. I will adopt, as metric, the Root Mean Square Error (RMSE), averaged over a whole simulation run, for the tracked object `x` and `y` position, and its velocity in the `x` and `y` direction.

I tried my UKF implementation using measurements from only Radar or Lidar, and then from both. Table below reports the RMSE of the simulation run on dataset 1 in the three cases.

| Variable| RMSE Radar only        | RMSE Lidar only   | RMSE Radar and Lidar combined|  
|:---:|:-------------:|:-------------:|:-------------:| 
|`x`| 0.2259      | 0.1580        | 0.0648| 
|`y`| 0.2589      | 0.1466        | 0.0992|
|`vx`| 0.4046     | 0.4897      | 0.3835|
|`vy`| 0.3689      | 0.2483      | 0.2240|

Table below reports the same metrics for a run on dataset 2.

| Variable| RMSE Radar only        | RMSE Lidar only   | RMSE Radar and Lidar combined|  
|:---:|:-------------:|:-------------:|:-------------:| 
|`x`| 0.2324      | 0.1509        | 0.0858| 
|`y`| 0.3174      | 0.1367        | 0.0805|
|`vx`| 0.4078     | 0.4227      | 0.3831|
|`vy`| 0.6449      | 0.2862      | 0.2336|

On both datasets, sensor fusion exhibits more accuracy, for each variable, than using measurements from one sensor type alone.

### Comparison with Extended Kalman Filter

It is interesting to compare the UKF implementation performance with those of an [EKF](https://github.com/fantauzzi/CarND-Extended-Kalman-Filter-Project). Table below compares the RMSE for the two algorithms on dataset 1.

| Variable| RMSE EKF        | RMSE UKF   |   
|:---:|:-------------:|:-------------:| 
|`x`| 0.0964      | 0.0648        | 
|`y`| 0.0853      | 0.0992        |
|`vx`| 0.4154     | 0.3835      | 
|`vy`| 0.4316      | 0.2240      |

Table below compares the respective performance on dataset 2.

| Variable| RMSE EKF        | RMSE UKF   |   
|:---:|:-------------:|:-------------:| 
|`x`| 0.0726      | 0.0858        | 
|`y`| 0.0965      | 0.0805        |
|`vx`| 0.4216     | 0.3831      | 
|`vy`| 0.4932      | 0.2336      |

RMSEs for the UKF are lower than for the EKF for 3 variables out of 4 with either dataset. 

### Initialisation and Parameters Tuning

Successful adoption of an UKF requires some parameters tuning, specifically, for the initial values of the state and its covariance, and for the process covariance.

My UKF implementation tracks the target x and y position, its tangential velocity, yaw (heading) and yaw change rate (rotation speed). I used the first measurement coming from either Radar or Lidar to initialise the x and y position accordingly, and initialised the other variables to 0. I charted the error for a run with dataset 1 (500 iterations of the algorithm); the error being the difference between the estimated variable value and the actual value (ground truth). 

Picture below charts the position error, computed as the distance between the estimated `(x, y)` position and the ground truth.
 
![alt text][image3]

Next charts show the error for the other tracked state variables.

![alt text][image4]
![alt text][image5]
![alt text][image6]

We can see that the error spikes early on for velocity, yaw and yaw rate, that are not initialised from sensors, but set to zero on the first iteration. Those errors then quickly reach the range they maintain for the rest of the simulation run.
  
Next two charts show the Normalised Innovation Square (NIS) for each sensor type, and also draw the line for the 95% percentile of its distribution. That is is a chi-square distribution with as many degrees of freedom as dimensions in sensor measurements. Lidar measurements have two dimension, `x` and `y`, while Radar measurements have three, that is range, bearing and radial velocity.

![alt text][image1]
![alt text][image2]

NIS charts may help tuning the process variance. Based on Radar NIS, I am currently underestimating the process variance. However, when I tuned the process variance trying to improve the Radar NIS, I found that tends to worsen the Lidar NIS and the RMSE. I therefore priviledged RMSE as the metric for the quality of my UKF, and tuned the process variance by trial-and-error based on RMSE alone.

As per the starting value of the state variance, I found that the identity matrix allows the algorithm to converge. However, I got even better RMSE by setting the initial variance for the `x` and `y` position to 1, but to 10 for the other state variables. 

The resulting UKF tracks the object with a smooth path that falls well within the trail of sensor measurements, as shown by the screenshot below.

![alt text][image8]

*Note:* I have plotted the charts with the Python 3 program `src/report.py`. Ground truth is available in file `data/obj_pose-laser-radar-synthetic-input.txt`.

