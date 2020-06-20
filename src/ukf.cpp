#include "ukf.h"
#include "Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;
  std_a_ = 5; // 3 not bad
  std_a_ = 2; // 3 not bad
  std_a_ = 5; // 3 not bad


  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  std_yawdd_ = 3;
  std_yawdd_ = 1;

  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;
  
  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  
  /// Not initialized
  is_initialized_ = false;
  
  
  // State vector size
  n_x_ = 5;
  

  // Augmented state vector
  n_aug_ = n_x_+2;
  

  // Define the predicted sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);


  // Lambda
  lambda_ = 3-n_aug_;
  

  // Define Sigma point weights
  weights_ = Eigen::VectorXd (2*n_aug_+1);
  for (int i=0; i<weights_.size(); i++) {  // 2n+1 weights
      if (i == 0) {
	  weights_(i) = lambda_/(n_aug_+lambda_);
      } else {
	  weights_(i) = 0.5/(n_aug_+lambda_);
      }
  }

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage m_pack) {
    /**
     * TODO: Complete this function! Make sure you switch between lidar and radar
     * measurements.
     */

    /////////////////////////////////////////////////////////////////////////
    /// SINGLE sensor type simulation
    //
    if (m_pack.sensor_type_ == MeasurementPackage::LASER && !use_laser_) {
	return;
    }
    if (m_pack.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) {
	return;
    }
     

    /////////////////////////////////////////////////////////////////////////
    /// INITIALIZATIN
    //
    if (!is_initialized_) {
	
	is_initialized_ = true;  // We're now initialized
	time_us_ = m_pack.timestamp_; // Time stamped
	
	// Covariance matrix diagonalized
	P_ = MatrixXd::Identity(n_x_,n_x_);
	
 	// State vector to zero
	x_.fill(0.0);
	
	if (m_pack.sensor_type_ == MeasurementPackage::LASER) {
	    std::cout << " Initialization w/Laser data\n";
	    Lidar (m_pack);
	    x_(PX_) = m_pack.raw_measurements_(0);
	    x_(PY_) = m_pack.raw_measurements_(1);
	    x_(V_) = 0;
	    x_(YAW_) = 0; // Assume car is moving in the +x direction
	    
	    P_(0,0) = std_laspx_*std_laspx_;
	    P_(1,1) = std_laspy_*std_laspy_;

	} else {
	    std::cout << " Initializatin w/Radar data\n";
	    Radar (m_pack);

	    double rho = m_pack.raw_measurements_(0);
	    double phi = m_pack.raw_measurements_(1);
	    double dot = m_pack.raw_measurements_(2);

	    // ASSUMING rho defined relative to x-axis
	    x_(PX_) = rho*cos(phi);
	    x_(PY_) = rho*sin(phi);
	    x_(V_) = dot;

	    // Combining variance due to 'rho' and 'phi' uncertainties
	    double variance = std_radr_*std_radr_ + (rho*std_radphi_)*(rho*std_radphi_);
	    P_(0,0) = variance; // Reasonable initial values
	    P_(1,1) = P_(0,0);
	    P_(2,2) = std_radrd_*std_radrd_;

	}

	State ();
	std::cout << "Covariance:\n" << P_ << std::endl;
	return;
    }

    // Time step [sec]
    double dt = (m_pack.timestamp_-time_us_) / 1.0e6;
    time_us_ = m_pack.timestamp_; // Update only after update process
    
    if ( use_laser_ && m_pack.sensor_type_ == MeasurementPackage::LASER) {
	
	if (false)
	    Lidar (m_pack);           // Log measurement to console
	Prediction (dt);              // Predict state 'dt' forward
	if (use_std_update_) 
	    UpdateLidar_std (m_pack); // Standard Kalman update
	else 
	    UpdateLidar (m_pack);     // Sigma point estimation of measurement
	
    } else if ( use_radar_ && m_pack.sensor_type_ == MeasurementPackage::RADAR) {

	if (false)
	    Radar (m_pack);   // Log measurement to console
	Prediction (dt);      // Predict state 'dt' forward
	UpdateRadar (m_pack); // Update filter state

    } else {

	// Not execting to get here, but if I do let's not miss it
	throw ("Unexpected requset to process measurement");

    }

    if (false)
	State();  // Log state vector to console
    
    return;
}


void UKF::Prediction(double delta_t) {
    /**
     * TODO: Complete this function! Estimate the object's location. 
     * Modify the state vector, x_. Predict sigma points, the state, 
     * and the state covariance matrix.
     */
    
    ////////////////////////////////////////////////////////////////
    //*************************************************************/
    /// GENERATE sigma points

    /// Augmented state vector
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.segment(0, n_x_) = x_;
    x_aug.segment(n_x_, 2) << 0, 0;

    /// Augmented covariance matrix
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;

    MatrixXd Q = MatrixXd(2,2);
    Q << std_a_*std_a_, 0,
	0, std_yawdd_*std_yawdd_;
    P_aug.bottomRightCorner(2,2) = Q;

    /// Square root matrix
    MatrixXd A = P_aug.llt().matrixL();

    /// SIGMA points matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
    Xsig_aug.col(0) = x_aug;   // Column 1
    
    for (int i = 0; i < n_aug_; ++i) {
	Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
	Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);
    }
    
    ////////////////////////////////////////////////////////////////
    //*************************************************************/
    /// PREDICT sigma points
    double dt2 = delta_t*delta_t;
    
    // CYCLE over matrix columns. Each column is a SIGMA vector
    std::string type;
    for (short c=0; c < 2*n_aug_+1; c++) {

	VectorXd sig = VectorXd (n_aug_);  // SIGMA point
	sig = Xsig_aug.col(c);
	
	double vk = sig(V_);           // STATE variables
	double yawk = sig(YAW_);

	double yawDotk = sig(YAWDOT_);
	double yavk = sig(NUA_);
	double yavYwak = sig(NUYAW_);
	
	if (fabs(yawDotk) > 0.001) {
	    // Constant turn rate motion
	    sig(PX_) += vk/yawDotk*( sin(yawk+yawDotk*delta_t) - sin(yawk));
	    sig(PY_) += vk/yawDotk*(-cos(yawk+yawDotk*delta_t) + cos(yawk));
	} else {
	    // Straight line motion
	    sig(PX_) += vk*cos(yawk)*delta_t;
	    sig(PY_) += vk*sin(yawk)*delta_t;
	}
	
	sig(PX_) += 0.5*dt2*cos(yawk)*yavk;
	sig(PY_) += 0.5*dt2*sin(yawk)*yavk;
	
	sig(V_) += 0.0+delta_t*yavk;
	sig(YAW_) += yawDotk*delta_t + 0.5*dt2*yavYwak;
	sig(YAWDOT_) += 0.0 + delta_t*yavYwak;
	
	Xsig_pred_.col(c) = sig.head(n_x_);
    }

    ////////////////////////////////////////////////////////////////
    //*************************************************************/
    /// PREDICT Mean & Covariance
    x_.fill(0.0);
    for (int i = 0; i<2*n_aug_+1; i++) {  // iterate over sigma's
	x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    P_.fill(0.0);
    for (int i = 0; i<2*n_aug_+1; i++) {  // iterate over sigma's
	// state difference
	VectorXd x_diff = Xsig_pred_.col(i) - x_;
	
	// angle normalization
	while (x_diff(3) >  M_PI) x_diff(3)-=2.*M_PI;
	while (x_diff(3) < -M_PI) x_diff(3)+=2.*M_PI;
	
	P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }
    
    return;
}


void UKF::UpdateLidar (MeasurementPackage m_pack) {

    int n_z = 2; // Measurement dimension
     
    MatrixXd Zsig = MatrixXd (n_z,2*n_aug_+1); // Measurement Zigmas
    VectorXd z_pred = VectorXd(n_z);           // Mean predicted measurement
    MatrixXd S = MatrixXd(n_z,n_z);            // Measurement covariance
    
    for (short c=0; c<2*n_aug_+1; c++) { // transform zigma point into measurement space
	VectorXd Xsigma = Xsig_pred_.col(c);
	VectorXd Msigma = Xsigma.head(n_z);
	Zsig.col(c) = Msigma;
    }

    z_pred.fill(0.0);
    for (short c=0; c<2*n_aug_+1; c++) {  // Mean predicted measurement
	z_pred += weights_(c)*Zsig.col(c);
    }

    S.fill(0.0);
    for (short c=0; c<2*n_aug_+1; c++) {  // Predicted innovation covariance
	VectorXd dZ = Zsig.col(c) - z_pred;
	S += weights_(c)*dZ*dZ.transpose();
    }    

    MatrixXd R = MatrixXd(2,2);
    R << std_laspx_*std_laspx_, 0.0,
	0.0, std_laspy_*std_laspy_;

    S += R;  // Innovation covariance matrix

    VectorXd z = VectorXd(n_z);         //Measurment vector
    z << m_pack.raw_measurements_(0),  // px_
	m_pack.raw_measurements_(1);   // py

    MatrixXd Tc = MatrixXd (n_x_, n_z); // Cross-correlation matrix
    Tc.fill (0.0);

    for (short n=0; n<2*n_aug_+1; n++) {
	VectorXd dZ = Zsig.col(n) - z;       // Measurement spread
	VectorXd dX = Xsig_pred_.col(n) - x_; // State spread
	Tc += weights_(n)*dX*dZ.transpose();
    }

    MatrixXd K = MatrixXd(n_x_,n_z); // Kalman gain matrix
    K = Tc*S.inverse();

    VectorXd y = VectorXd(n_x_);
    y = z-z_pred;               // The 'error'

    x_ += K*y;  // State mean update
    P_ -= K*S*K.transpose();  // State covariance update

    vNIS_lidar_.push_back(y.transpose()*S.inverse()*y);

    return;
}

void UKF::UpdateLidar_std (MeasurementPackage m_pack) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
    std::cout << " s t d\n";
    
    MatrixXd H = MatrixXd(2,n_x_);
    H.fill(0.0);
    H(0,0) = 1.0;
    H(1,1) = 1.0;

    MatrixXd R = MatrixXd(2,2);
    R << std_laspx_*std_laspx_, 0.0,
	0.0, std_laspy_*std_laspy_;

    MatrixXd S = MatrixXd (2,2);
    S = H*P_*H.transpose() + R;

    MatrixXd K = MatrixXd (n_x_,2);
    K = P_*H.transpose()*S.inverse();

    VectorXd y = VectorXd (2);
    VectorXd z = VectorXd (2);
    z << m_pack.raw_measurements_(0), m_pack.raw_measurements_(1);
    y = z-H*x_;
    
    x_ = x_ + K*y;
    MatrixXd I = MatrixXd::Identity(n_x_,n_x_);
    P_ = (I-K*H)* P_;

    vNIS_lidar_.push_back(y.transpose()*S.inverse()*y);

    return;
}

void UKF::UpdateRadar(MeasurementPackage m_pack) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

    int n_z = 3; // Measurment dimension

    MatrixXd R = MatrixXd(n_z, n_z); // Uncertainty covariance
    R.fill (0.0);
    R(0,0) = std_radr_*std_radr_;
    R(1,1) = std_radphi_*std_radphi_;
    R(2,2) = std_radrd_*std_radrd_;

    MatrixXd Zsig = MatrixXd (n_z,2*n_aug_+1); // Measurement Zigmas
    MatrixXd z_pred = VectorXd (n_z);          // Mean predicted measurement
    MatrixXd S = MatrixXd (n_z, n_z);          // Measurement covariance (innovation)

    for (short col=0; col<2*n_aug_+1; col++) { // Predict measurement zigmas
	VectorXd Xsig = Xsig_pred_.col(col);     // Zigma point
	VectorXd zcol = VectorXd(n_z);           // Measurement zigma

	zcol(0) = sqrt (Xsig(PX_)*Xsig(PX_) + Xsig(PY_)*Xsig(PY_));
	zcol(1) = atan2 (Xsig(PY_), Xsig(PX_));
	zcol(2) = Xsig(V_)/zcol(0)*(Xsig(PX_)*cos(Xsig(YAW_)) + Xsig(PY_)*sin(Xsig(YAW_)));

	Zsig.col(col) = zcol;  // Measurement sigma
    }

    z_pred.fill (0.0);
    for (short col=0; col <2*n_aug_+1; col++) { // Mean predicted measurement
	z_pred += weights_(col)*Zsig.col(col);
    }

    S.fill (0.0);
    for (short col=0; col<2*n_aug_+1; col++) { // Innovation matrix
	VectorXd sig = Zsig.col(col)-z_pred;
	S += weights_(col)*sig*sig.transpose();
    }
    S += R;     // Combined measurement covariance

    
    VectorXd z = VectorXd (n_z);        // Measurement vector
    z << m_pack.raw_measurements_(0),
	m_pack.raw_measurements_(1),
	m_pack.raw_measurements_(2);

    MatrixXd Tc = MatrixXd (n_x_, n_z);   // Cross-correlation matrix
    Tc.fill (0.0);
    for (short col=0; col<2*n_aug_+1; col++) {
	VectorXd dZ = Zsig.col(col)-z_pred;
	while (dZ(1)> M_PI) dZ(1)-=2.0*M_PI;
	while (dZ(1)<-M_PI) dZ(1)+=2.0*M_PI;

	VectorXd dX = Xsig_pred_.col(col)-x_;
	while (dX(3)> M_PI) dX(3)-=2.0*M_PI;
	while (dX(3)<-M_PI) dX(3)+=2.0*M_PI;

	Tc += weights_(col)*dX*dZ.transpose();
    }

    MatrixXd K = Tc*S.inverse();    // Kalman gain matrix

    VectorXd dZ = z-z_pred;         // State measurement 'error'
    while (dZ(1)> M_PI) dZ(1)-=2.0*M_PI;
    while (dZ(1)<-M_PI) dZ(1)+=2.0*M_PI;

    x_ += K*dZ;               // State update
    P_ -= K*S*K.transpose();  //State covariance update

    double NIS = dZ.transpose() * S.inverse() * dZ;
    vNIS_radar_.push_back(NIS);

    return;
}
