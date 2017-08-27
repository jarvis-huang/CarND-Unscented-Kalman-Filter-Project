# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the 
state of a moving object of interest with noisy lidar and radar measurements. 
Passing the project requires obtaining RMSE values that are lower that 
the tolerance outlined in the project reburic.

[image1]: ./figures/dataset_1_shot.png "Dataset 1 screenshot"
[image2]: ./figures/dataset_2_shot.png "Dataset 2 screenshot"
[image3]: ./figures/dataset_1.png "Dataset 1 error"
[image4]: ./figures/dataset_2.png "Dataset 2 error"
[image5]: ./figures/NIS_dataset_1.png "Dataset 1 NIS plot"
[image6]: ./figures/NIS_dataset_2.png "Dataset 2 NIS plot"

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` The current state uses i/o from the simulator.

## Editor Settings

## Project Implementation
I mostly followed the instructions from the UKF class materials. The only
special cases are process noise and covariance parameters and initializations.

### 1. Noise parameters

|    Noise   |    Value      | 
|:----------:|:-------------:| 
|  std_a     |      4.0      | 
| std_yawd   |      1.0      | 
| std_px     |      0.15     | 
| std_py     |      0.15     | 
| std_rho    |      0.2      | 
| std_phi    |      0.1      | 
| std_rhod   |      0.5      |

- For a bicycle, a reason linear acceleration is 2m/s^2. So I set std_a to be double that amount.
- For a bicycle, I assume it can complete a full circle in 6 seconds. That's equivalent
to 1 rad/s.
- I set laser and radar position error to fairly low, but set radar error to be higher than laser
to match reality.
- I set angular error of radar to 0.1 rad which is about 6 degrees.
- I set range rate error error to be a little large.

### 1. Initialization of state vector **x** and state covariance matrix **P**
- For P, I initialize to all zero, for lack of knowledge.
- For x, it is initialized upon receiving first measurement.
If the measurement is a laser measurement, I initialize px, py directly from measurement values.
If the measurement is a laser measurement, I initialize `px=rho*cos(phi)`, `py=rho*sin(phi)`.

## Results

### 1. RMSE
|   Metric   |Dataset 1 RMSE |Dataset 2 RMSE | 
|:----------:|:-------------:|:-------------:| 
|  px        |      0.10     |      0.08     | 
|  py        |      0.09     |      0.09     | 
|  vx        |      0.33     |      0.42     | 
|  vy        |      0.22     |      0.24     | 

**Dataset 1 screenshot**
![Dataset 1 screenshot][image1]
**Dataset 2 screenshot**
![Dataset 2 screenshot][image2]

### 2. Detailed error analysis
As the plots show, position estimates are generally very close to ground truth.
Velocity estimates are less accurate, but still follows ground truth mostly.
This is because we don't have direct measurement of the velocities.

**Dataset 1 error analysis**
![Dataset 1 error analysis][image3]
**Dataset 2 error analysis**
![Dataset 2 error analysis][image4]


### 3. NIS analysis

NIS plots show the NIS is mostly below 95% line, which shows the filter is very consistent.
The parameters are set properly so that we correctly estimate the uncertainty of the system.

**Dataset 1 NIS analysis**
![Dataset 1 NIS analysis][image5]
**Dataset 2 NIS analysis**
![Dataset 2 NIS analysis][image6]
