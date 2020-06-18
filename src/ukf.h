#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"
#include <iostream>

class UKF {
 public:

    //////////////////////////////////////////////////////
    /// Sensor accessor functions
    //
    void LidarState (const bool on) {
	use_laser_ = on;
    }
    void RadarState (const bool on) {
	use_radar_ = on;
    }

    // Set the yaw rate process noice variance
    void RateVariance (const double value) {
	std_yawdd_ = value;
    }

    // Set the yaw rate process noice variance
    void AccVariance (const double value) {
	std_a_ = value;
    }
    
    // Set the yaw rate process noice variance
    void Update (const bool value) {
	use_std_update_ = value;
    }
    
    void State (void) const {
	std::cout << "X (" << x_(PX_) << ", " << x_(PY_) << ") v=" << x_(V_) << " 0=" << x_(YAW_) <<
	    " w=" << x_(YAWDOT_) << std::endl;
    }
    void Lidar (MeasurementPackage m_pack) {
	std::cout << "L x=" << m_pack.raw_measurements_(0) << " y=" << m_pack.raw_measurements_(1) << std::endl;
    }
    void Radar (MeasurementPackage m_pack) {
	std::cout << "R r=" << m_pack.raw_measurements_(0) << " 0=" << m_pack.raw_measurements_(1) <<
	    " vR=" << m_pack.raw_measurements_(2) << std::endl;
    }

    
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar_std(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);


  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // select standard Kalman filter update instead of sigma point projection
  bool use_std_update_;

  enum AugmentedStateVector {PX_, PY_, V_, YAW_, YAWDOT_, NUA_, NUYAW_} enmCoords;

  // NIS values
  std::vector<double> vNIS_lidar_;

  std::vector<double> vNIS_radar_;

  
  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Lidar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;
};

#endif  // UKF_H
