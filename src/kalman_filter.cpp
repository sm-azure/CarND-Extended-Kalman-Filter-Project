#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

using namespace std;

KalmanFilter::KalmanFilter() {
  I_ = MatrixXd::Identity(4, 4);
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

  VectorXd y = z - H_ * x_;
  MatrixXd S_ = H_*P_*H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose()*S_.inverse();
  x_ = x_ + (K_ * y);
  P_ = (I_ - K_*H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  VectorXd z_ = VectorXd(3);
  z_ << z(0), z(1), z(2);
  float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

  float div = px*px + py*py;

  if(div < 0.0001){
      cout << "Small div!!!--------------------------------------" << endl;
	} else{
    // calculate rho, phi and rhodot
    VectorXd h_x_ = VectorXd(3);
    VectorXd y = VectorXd(3);
    MatrixXd S_;
    MatrixXd K_;

    MatrixXd Hj = tools.CalculateJacobian(x_);
    h_x_ << sqrt(div), atan2(py, px), (px*vx + py*vy)/sqrt(div);
    //validate phi is between -pi and +pi
    
    y = z_ - h_x_;
    if (y(1) > 3.1415 ){
      cout << "=============================================================== Booommmmmmmmmmmmmmm" << endl;
      y(1) = y(1) - 2*3.1415;
    }
    if (y(1) < -3.1415 ){
      cout << "=============================================================== Booommmmmmmmmmmmmmm" << endl;
      y(1) = y(1) + 2*3.1415;
    }

    S_ = Hj*P_*Hj.transpose() + R_;
    K_ = P_ * Hj.transpose()*S_.inverse();
    x_ = x_ + (K_ * y);
    P_ = (I_ - K_*Hj) * P_;
  }

   
}
