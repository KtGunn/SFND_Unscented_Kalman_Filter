# SFND_Unscented_Kalman_Filter
Fourth and final project of Udacity Sensor Fusion Nanodegree Program

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

The purpose of the project is to implement an Unscented Kalman Filter. Data is generated from a simulation of a car, the ego car, driving on a higway. The ego car is equipped with lidar and radar sensors and measures location of three cars in its environment as they move behind, in front and by it. The data is to be fused in a filter for best estimate of each car's two dimensional position, speed and turn rate.

Simulation and starter code is provided. The simulation code generates the measurement data from the surrounding cars. The simulation code also includes all necessary code for visualizing the simulation and its results (see gif above).

Referring to the visualization, the ego car is shown in green. Simluated lidar readings are red dots above and near each traffic car. Simulated lidar readings are purple arrows pointing at traffic cars. Actual velocity relative to the ego car is indicated.

The starter code provides header file for unscented filter implementation and prototypes for the main processing steps in a Kalman filter implementation, i.e. measurement processing, state prediction and measurement update. The student is to fill in the implementation, which was developed in detail in class room lectures.

Finally, the simulation code monitors filter implementation's ability to accurately estimate the cars' state. A successful implementation must pass RMSE (root mean square error) limits. At each step the RMSE is displayed and should the success criteria not be met, a display announcing the failure shows up. To meet the success criteria the student must implement the filter correctly and tune it to meet the accuracy criteria.

The filter implementation assumes the CTRV (Constant Turn Rate and Veolcity Magnitude) model. The simulated velocity and turn rates of traffic cars are non constant. The filter implementation accounts for that by modeling veolcity and rate changes as noise in the state equations. Tuning means setting values for two process noise variances, acceleration and yaw rate.

