# SFND_Unscented_Kalman_Filter
Fourth and final project of Udacity Sensor Fusion Nanodegree Program

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

The purpose of the project is to implement an Unscented Kalman Filter. Data is generated from a simulation of a car, the ego car, driving on a higway. The ego car is equipped with lidar and radar sensors and measures location of three cars in its environment as they move behind, in front and by it. The data is to be fused in a filter for best estimate of each car's two dimensional position, speed and turn rate.

Simulation and starter code is provided. The simulation code generates the measurement data from the surrounding cars. The simulation code also includes all necessary code for visualizing the simulation and its results (see gif above).

Referring to the visualization, the ego car is shown in green. Simluated lidar readings are red dots above and near each traffic car. Simulated lidar readings are purple arrows pointing at traffic cars. Actual velocity relative to the ego car is indicated.

The starter code provides header file for unscented filter implementation and prototypes for the main processing steps in a Kalman filter implementation, i.e. measurement processing, state prediction and measurement update. The student is to fill in the implementation, which was developed in detail in class room lectures.

Finally, the simulation code monitors filter implementation's ability to accurately estimate the cars' state. A successful implementation must pass RMSE (root mean square error) limits. At each step the RMSE is displayed and should the success criteria not be met, a display announcing the failure shows up. To meet the success criteria the student must implement the filter correctly and tune it to meet the accuracy criteria.

The filter implementation assumes the CTRV (Constant Turn Rate and Veolcity Magnitude) model. The simulated velocity and turn rates of traffic cars are non constant. The filter implementation accounts for that by modeling veolcity and rate changes as noise in the state equations. Tuning means setting values for two process noise variances, acceleration and yaw rate.

## Implementation

The Kalman Filter was implemented as instructed in the classroom and developed over a course of exercises. A few changes and additions were made.

### Command line arguments

Command line arguments were added to aid development and testing. The user can select to simulate any of the three traffic cars solely or all three. Simluations can use lidar or radar only or both. Values for the two tuning parameters can be set both or individually. Finally, a standard Kalman Filter update can be used instead of the sigma point measurement estimation method. If no command line options are set, defaults using all three traffic cars, both sensors, simga point measurement and the final tuning parameters is used. That is the configuration to run when verifying that the RMSE criteria have been met for the project.

<img src="media/options.png" width="700" />

### Performance plots

Two plot are presented during and after the simulation. The first is NIS (Normalized Innovation Squared). NIS is an indicator of proper tuning. Proper tuning is attained when the predicted measurement in the update step is within statistical bounds of the actual measurement. If the discrepancy is either consistently too large or too small the tuning is off. It is also possible that the modeling of the real process is wrong but that is a different matter. The NIS, a scalar, follows the chi-sqaured distribution, so expected targets can be set based on the number of measurement variables ("degrees of freedom"). Below is an example NIS plot. The simulation uses traffic car #3 only. The horizontal lines in teh NIS plot represent the Chi-squared limits applicable to the two measurement update types, liadr and radar.

<img src="media/NIS-C3.png" />

