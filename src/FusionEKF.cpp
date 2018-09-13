#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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
  P_ = MatrixXd(4, 4);
  F_ = MatrixXd(4, 4);
  x_ = VectorXd(4);
  Q_ = MatrixXd(4, 4);


  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
	H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

  
  F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
	P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;
  x_ << 0,0,0,0;

  Q_ << 0, 0, 0, 0,
			  0, 0, 0, 0,
			  0, 0, 0, 0,
			  0, 0, 0, 0;

  ekf_ = KalmanFilter();
  ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
  cout << "Constructor finished!"<<endl;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  cout << "ProcessMeasurement started!"<<endl;
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF Init: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      float px = measurement_pack.raw_measurements_[0]* cos(measurement_pack.raw_measurements_[1]);
      float py = measurement_pack.raw_measurements_[0]* sin(measurement_pack.raw_measurements_[1]);
      float vx = measurement_pack.raw_measurements_[2]* cos(measurement_pack.raw_measurements_[1]);
      float vy = measurement_pack.raw_measurements_[2]* sin(measurement_pack.raw_measurements_[1]);
      cout <<"Radar Init:" <<endl;
      cout << px <<"," << py <<"," << vx <<"," << vy <<endl;
      ekf_.x_ << px,py,vx,vy;
      cout <<"Radar Init Complete" <<endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      float vx = 0;
      float vy = 0;

      cout <<"Lidar Init:" <<endl;
      cout << px <<"," << py <<"," << vx <<"," << vy <<endl;
      ekf_.x_ << px,py,vx,vy;
      cout <<"Lidar Init Complete" <<endl;
    }

  // done initializing, no need to predict or update
  cout<< "Completed Init 1" <<endl;
  is_initialized_ = true;
  cout<< "Completed Init 2" <<endl;
  //previous_timestamp_ = measurement_pack.timestamp_;
  cout<< "Completed Init";
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //compute the time elapsed between the current and previous measurements

  cout<< "Trying to predict" <<endl;
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;
	
  ekf_.F_(0,2) = dt;
	ekf_.F_(1,3) = dt;
	//2. Set the process covariance matrix Q
	float dt2 = dt * dt;
	float dt3 = dt2 * dt; //pow(dt, 3);
	float dt4 = dt3 * dt;
	//std::cout << "Modify Q" << std::endl;

  float noise_ax = 9;
  float noise_ay = 9;
	
	ekf_.Q_ << dt4*noise_ax/4, 0, dt3*noise_ax/2, 0,
	         0, dt4*noise_ay/4, 0, dt3*noise_ay/2,
	         dt3*noise_ax/2, 0, dt2*noise_ax, 0,
	         0, dt3*noise_ay/2, 0, dt2*noise_ay;

  ekf_.Predict();
  cout << "Post prediction: " <<endl;
  cout << ekf_.x_[0] << "," << ekf_.x_[1] << "," << ekf_.x_[2] << "," << ekf_.x_[3] <<endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    //Radar measurements are in (rho, theta and rho dot). Convert current predictions to this format
    cout << "Update radar" <<endl;

  } else {
    // Laser updates
    cout << "Update laser" <<endl;
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
