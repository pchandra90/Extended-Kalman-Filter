#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::cerr;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

 // Laser mesurnmnet matrix
  H_laser_ << 1.0, 0.0, 0.0, 0.0,
  			 0.0, 1.0, 0.0, 0.0;
  
// Acceleration noise
  noise_ax_ = 9.0;
  noise_ay_ = 9.0;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    VectorXd x = VectorXd(4);
    float px, py;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      px = rho * cos(phi);
      py = rho * sin(phi);      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      px = measurement_pack.raw_measurements_[0];
      px = measurement_pack.raw_measurements_[1];
    }
    else {
      cerr << "Unknown sensor type. Exiting..." << endl;
      return;
    }
    x << px, py, 0, 0;  // initializing px and py. For vx and vy initializing with zero
   // State co-variance matrix
    MatrixXd P(4, 4);
    P << 1, 0, 0, 0,
    	 0, 1, 0, 0,
    	 0, 0, 1000, 0,
    	 0, 0, 0, 1000;
    
    // State transation matrix, delta_t = 0. So F(0,2) = 0 and F(1, 3) = 0
    MatrixXd F(4, 4);
    F << 1, 0, 0, 0,
    	 0, 1, 0, 0,
    	 0, 0, 1, 0,
    	 0, 0, 0, 1;
    
    // Process co-variance matrix. delta_t = 0. So all element will be zero
    MatrixXd Q(4, 4);  // default intialization is zero for each element 
	ekf_.Init(x, P, F, H_laser_, R_laser_, R_radar_, Q);
    cout << "Initialization is done." << endl
      << "First mesurnmnet is performed by: "
      << ((measurement_pack.sensor_type_ == MeasurementPackage::RADAR) ? "radar " : "lidar ") << endl;

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;  
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

    // TODO: YOUR CODE HERE
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
         	  0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
         	  dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
         	  0, dt_3/2*noise_ay_, 0, dt_2*noise_ay_;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  
  return;
}
