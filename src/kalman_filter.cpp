#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
  I_ = MatrixXd::Identity(2, 2);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {

 	x_ = F_* x_;
	P_ = F_* P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = VectorXd(2);
  MatrixXd S_;
  MatrixXd K_;

  y = z - H_ * x_;
  S_ = H_*P_*H_.transpose() + R_;
  K_ = P_ * H_.transpose()*S_.inverse();
  x_ = x_ + (K_ * y);
  P_ = (I_ - K_*H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  MatrixXd Hj(3,4);

	//recover state parameters
	float px = z(0);
	float py = z(1);
	float vx = z(2);
	float vy = z(3);

  float div = px*px + py*py;

	//TODO: YOUR CODE HERE 
 
	//check division by zero
	
	if(div < 0.0001){
	    // Hj << 0,0,0,0,
      // 0,0,0,0,
      // 0,0,0,0;

      return;
	} else{
    

    //compute the Jacobian matrix
    Hj << px / sqrt(div), py/sqrt(div), 0,0,
        -py/div, px/div, 0,0,
        py * (vx*py - vy*px) / pow(div, 3/2), px* (vy*px - vx*py) / pow(div, 3/2), px/sqrt(div), py/sqrt(div);
  }
	return;    
}
