# Unscented Kalman Filter Project
**Self-Driving Car Engineer Nanodegree Program**

In this project I implemented an Unscented Kalman Filter (UKF) to track a moving object of interest with noisy lidar and radar measurements.  

The UKF is a state estimator for continuous space belonging to the family of Bayes filter. For the algorithm details, a good reference is "Probabilistic Robotics", written by Sebastian Thrun, Wolfram Burgard and Dieter Fox, The MIT Press, 

## Dependencies
The program requires Udacity's simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). The simulator sends the program implementing an UKF noisy measurements from lidar and radar; the program estimates the state of the tracked object, and sends it back to the simulator for visualisation.

It also requires:
* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

The program was tested under Ubuntu 16.04.

## Build and run the program
Once cloned (or downloaded) the project repository, in the project directory type:

1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`
5. `./UnscentedKF`

Run the simulator, make sure **Project 1/2: EKF and UKF** is selected in the simulator and click **SELECT**. On the next screen, you can choose between one of two datasets. Click **Start** to have the simulator begin to send data to the running UKF implementation. 

To stop the UKF program execution hit `Ctrl+C`.

>>> Screenshot here with explanation

The UKF program outputs a text file `data/out.txt`, with one line for every sensor measurement received from the simulator, reporting the corresponding estimated state of the tracked object. On each line there is, tab separated:
- timestamp, as number of microseconds elapsed from an epoch;
- x and y coordinates of the object position in meters;
- tangential velocity in m/s;
- yaw of the in radians;
- yaw change rate in radians/s;
- computed normalised innovation square (NIS);

## Considerations on the results

### Sensor fusion
The UKF can be used to combine data from different sensors (sensor fusion), to achieve more accurate results. The simulator provides measurements from two kind of sensors, RADAR and LIDAR. Let's see the impact of either on the UKF implementation. I will adopt, as metric, the Root Mean Square Error (RMSE), averaged over a whole simulation run, for the tracked object `x` and `y` position, and its velocity in the `x` and `y` direction.

I tried my UKF implementation using measurements from only Radar or Lidar, and then from both. Table below reports the RMSE of the simulation run on dataset 1 in the three cases, for the tracked object x and y coordinates, and for the x and y components of its velocity.


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

In both cases, sensor fusion exhibits more accuracy, for each variable, than using measurements from one sensor type alone.

### Comparison with Extended Kalman Filter

It is interesting to contrast the performance of the UKF implementation with those of an EKF. Table below compares the RMSE for the two algorithms on dataset 1.

Table below compares the respective performance for dataset 2.


### Parameters tuning

A successful run of the UKF required some parameter tuning, specifically, for the initial values of the state and its covariance, and for the process covariance.

The UKF implementation tracks the target x and y position, its tangential velocity, yaw (heading) and yaw change rate (rotation speed). I used the first measurement coming from either Radar or Lidar to initialise the x and y position with a first estimate, and initialised the other variables to 0. I charted the error for a run with dataset 1 (500 iterations of the algorithm); the error being the difference between the estimated variable value and the actual value (ground truth). 

The picture below charts the position error, computed as the distance between the estimated `(x, y)` position and the ground truth. 

Next charts show the error for the other tracked state variables.

We can see that the error spikes early on for velocity, yaw and yaw rate, that are not initialised from sensors, but set to zero. Those errors then quickly converge to the range they maintain for the rest of the simulation run.
  
Next two charts show the NIS for each sensor type, and also draw the line for the 95% percentile of its distribution. That is is a chi-square distribution with as many degrees of freedom as dimensions in the sensor measurements. Lidar measurements have two dimension, `x` and `y`, while Radar measurements have three, that is range, bearing and radial velocity.

NIS charts are supposed to help with tuning of the preocess variance. Based on the charts, I am currently over/under-estimating the... while over/under-estimating the.... However, when I tuned the process variance trying to improve the ... estimate, I found that by improving the estimate for either ridar or lidar, I would also worsen it for the other sensor type. I therefore proviledged RMSE as the metric of the quality of the tracking, and tuned the process variance by trial-and-error based on RMSE alone. 

As per the starting value of the state variance, I found that a diagonal matrix with all 1 on the main diagonal allows the alogirthm to converge. However, I got even better RMSE setting the initial variance for the `x` and `y` position to 1, but to 10 for the other state variables. 

