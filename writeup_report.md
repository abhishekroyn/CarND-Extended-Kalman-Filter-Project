# **Extended Kalman Filter** 
---

**Extended Kalman Filter Project**

The goals / steps of this project are the following:
* utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements
* use the simulator to test the model of RMSE values
* obtain RMSE values that are lower than the tolerance outlined in the project rubric
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./output_images/dataset_1_vel_init_0p0_lidar_radar_rmse.png "dataset_1_vel_init_0p0_lidar_radar_rmse"
[image2]: ./output_images/dataset_1_vel_init_0p0_lidar_only_rmse.png "dataset_1_vel_init_0p0_lidar_only_rmse"
[image3]: ./output_images/dataset_1_vel_init_0p0_radar_only_rmse.png "dataset_1_vel_init_0p0_radar_only_rmse"
[image4]: ./output_images/dataset_1_vel_init_0p1_lidar_radar_rmse.png "dataset_1_vel_init_0p1_lidar_radar_rmse"
[image5]: ./output_images/dataset_1_vel_init_0p1_lidar_only_rmse.png "dataset_1_vel_init_0p1_lidar_only_rmse"
[image6]: ./output_images/dataset_1_vel_init_0p1_radar_only_rmse.png "dataset_1_vel_init_0p1_radar_only_rmse"
[image7]: ./output_images/dataset_2_vel_init_0p0_lidar_radar_rmse.png "dataset_2_vel_init_0p0_lidar_radar_rmse"
[image8]: ./output_images/dataset_2_vel_init_0p0_lidar_only_rmse.png "dataset_2_vel_init_0p0_lidar_only_rmse"
[image9]: ./output_images/dataset_2_vel_init_0p0_radar_only_rmse.png "dataset_2_vel_init_0p0_radar_only_rmse"
[image10]: ./output_images/dataset_2_vel_init_0p1_lidar_radar_rmse.png "dataset_2_vel_init_0p1_lidar_radar_rmse"
[image11]: ./output_images/dataset_2_vel_init_0p1_lidar_only_rmse.png "dataset_2_vel_init_0p1_lidar_only_rmse"
[image12]: ./output_images/dataset_2_vel_init_0p1_radar_only_rmse.png "dataset_2_vel_init_0p1_radar_only_rmse"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/748/view) individually and describe how I addressed each point in my implementation.  

---
### Compiling

#### 1. Your code should compile.

My code compiled without errors with cmake and make.

### Accuracy

#### 1. px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt" which is the same data file the simulator uses for Dataset 1.

For Dataset 1, using both lidar and radar measurements for prediction and update, output coordinates for (px, py, vx, vy) have final RMSE value as [0.0973, 0.0855, 0.4513, 0.4399] which is less than [.11, .11, 0.52, 0.52], as required. Here is an image with the required state and values:

![alt text][image1]

### Follows the Correct Algorithm

#### 1. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

A well-defined set of steps was followed as described in the classroom lessons in order to successfully build a Kalman Filter. (lines 28-98 in the file ./src/kalman_filter.cpp) describes the primarily functions - Predict, Update, UpdateEKF, and UpdateAny, for building a Kalman Filter.

#### 2. Your Kalman Filter algorithm handles the first measurements appropriately.

(lines 68-102 in the file ./src/FusionEKF.cpp) use the first measurements to initialize the state vectors.
(lines 47-51, lines 28-35, and lines 120-126 in the file ./src/FusionEKF.cpp) describes the initialization of the covariance matrices, for state covariance matrix (P), measurement covariance matrix (R), and process covariance matrix (Q) respectively. 

#### 3. Your Kalman Filter algorithm first predicts then updates.

Upon receiving a measurement after the first, the algorithm predicts the object position to the current timestep (lines 128-129 in the file ./src/FusionEKF.cpp and lines 28-34 in the file ./src/kalman_filter.cpp). And then it updates the prediction using the new measurement (lines 135-149 in the file ./src/FusionEKF.cpp and lines 36-98 in the file ./src/kalman_filter.cpp).

#### 4. Your Kalman Filter can handle radar and lidar measurements.

The algorithm first check the sensor type and handles input measurement data accordingly (lines 70-93 in the file ./src/main.cpp). Thereafter, it checks the sensor type again and initialises the state vector accordingly (lines 76-89 in the file ./src/FusionEKF.cpp). For prediction, it uses common function for either sensor as expected (lines 128-129 in the file ./src/FusionEKF.cpp). For update, it calls the correct measurement function for a given sensor type (lines 135-149 in the file ./src/FusionEKF.cpp).

The algorithm here also sets up the appropriate matrices given the type of measurement for radar and lidar (lines 135-149 in the file ./src/FusionEKF.cpp).

### Code Efficiency

#### 1. Your algorithm should avoid unnecessary calculations.

The algorithm avoids any unnecessary calcuations in several ways.

* Keeping update function separate for lidar and radar partially, but combining rest of the calculations which are common in both Kalman Filter and Extended Kalman Filter by creating a new function `UpdateAny()` (lines 83-98 in the file ./src/kalman_filter.cpp) and calling the same from other functions - `Update` (lines 42-43 in the file ./src/kalman_filter.cpp) and `UpdateEKF` (lines 79-80 in the file ./src/kalman_filter.cpp).

* Avoiding unnecessary control flow checks by checking for divisibility by 0 only when new data is obtained and not when the same data is passed around in different files and different functions. Divisibilty by 0 was performed only once for each data and at two places for two different data sets, at the beginning of handling new data, once for measurement data(lines 59-62 in the file ./src/kalman_filter.cpp), and the other for predicted data (lines 65-69 in the file ./src/tools.cpp).

* Keeping simpler structures for loop based on simple conditions to keep Kalman filter angle value small between the range -pi and pi (lines 70-77 in the file ./src/kalman_filter.cpp).

* Running specific calculations only once, storing their values and then reusing the values later instead of re-calculating them again and again. To set the process covariance matrix Q, various orders of elapsed time was calcualed and stored first and them was simply used to set the Q matrix. That way repeated calculations for higher order multiplication of elapsed time was saved (lines 112-126 in the file ./src/FusionEKF.cpp).

### Additionally

#### 1. Additonal results, observations and analysis.

For both the datasets - dataset 1 and Dataset 2, I tried to utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements, sometimes using just one of those two sensor measurements. I also tried varying values for velocity (vx, vy) as (0, 0) and (0.1, 0.1) while initializing the state vectors using first measurements. 

Here are the final images with the required state and RMSE values on using the simulator to test the model:

* Results,
    - dataset 1,
        - initialize measurement for radar (vx, vy) as (0, 0)
            - lidar + radar
                - dataset_1_vel_init_0p0_lidar_radar_rmse
![alt text][image1]
            - lidar only
                - dataset_1_vel_init_0p0_lidar_only_rmse
![alt text][image2]
            - radar only
                - dataset_1_vel_init_0p0_radar_only_rmse
![alt text][image3]
        - initialize measurement for radar (vx, vy) as (0.1, 0.1)
            - lidar + radar
                - dataset_1_vel_init_0p1_lidar_radar_rmse
![alt text][image4]
            - lidar only
                - dataset_1_vel_init_0p1_lidar_only_rmse
![alt text][image5]
            - radar only
                - dataset_1_vel_init_0p1_radar_only_rmse
![alt text][image6]
    - dataset 2,
        - initialize measurement for radar (vx, vy) as (0, 0)
            - lidar + radar
                - dataset_2_vel_init_0p0_lidar_radar_rmse
![alt text][image7]
            - lidar only
                - dataset_2_vel_init_0p0_lidar_only_rmse
![alt text][image8]
            - radar only
                - dataset_2_vel_init_0p0_radar_only_rmse
![alt text][image9]
        - initialize measurement for radar (vx, vy) as (0.1, 0.1)
            - lidar + radar
                - dataset_2_vel_init_0p1_lidar_radar_rmse
![alt text][image10]
            - lidar only
                - dataset_2_vel_init_0p1_lidar_only_rmse
![alt text][image11]
            - radar only
                - dataset_2_vel_init_0p1_radar_only_rmse
![alt text][image12]

* Observations,
    - Using radar only, the rmse errors were high
    - using lidar only, the rmse errors were better as compared to radar only
    - using lidar and radar together, the rmse errors were the lowest
    - using initialize measurement for radar (vx, vy) as (0, 0) or (0.1, 0.1), doesn't change the rmse values for dataset 1, as it has initial measurement set by lidar measurement which doesn't take into account (vx, vy) values
    - using initialize measurement for radar (vx, vy) as (0, 0) or (0.1, 0.1), does change the rmse values for dataset 2, as it has initial measurement set by radar measurement which does take into account (vx, vy) values
    - using initialize measurement for radar (vx, vy) as (0, 0) produced better results with lower rmse values for vx and vy (though same rmse values for x and y) as compared to using (vx, vy) as (0.1, 0.1)

