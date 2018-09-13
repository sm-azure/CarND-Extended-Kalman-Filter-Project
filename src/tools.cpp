#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

using namespace std;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
 VectorXd rmse = VectorXd(4);
 rmse << 0,0,0,0; 
 if (estimations.size() != 0 && estimations.size()== ground_truth.size()){
	    //accumulate squared residuals
    	for(int i=0; i < estimations.size(); ++i){
            rmse(0) = rmse(0) + pow(estimations[i](0) - ground_truth[i](0) ,2);
            rmse(1) = rmse(1) + pow(estimations[i](1)- ground_truth[i](1) ,2);
            rmse(2) = rmse(2) + pow(estimations[i](2) - ground_truth[i](2) ,2);
            rmse(3) = rmse(3) + pow(estimations[i](3) - ground_truth[i](3) ,2);
    	}
      cout <<"Sum:" << rmse <<endl;
    
    	//calculate the mean
    	rmse(0) = pow (rmse(0)/estimations.size(), 0.5);
    	rmse(1) = pow (rmse(1)/estimations.size(), 0.5);
    	rmse(2) = pow (rmse(2)/estimations.size(), 0.5);
    	rmse(3) = pow (rmse(3)/estimations.size(), 0.5);   
	}
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

  float div = px*px + py*py;

	//TODO: YOUR CODE HERE 
 
	//check division by zero
	
	if(div < 0.0001){
      return Hj;
	} else{
    //compute the Jacobian matrix
    Hj << px / sqrt(div), py/sqrt(div), 0,0,
        -py/div, px/div, 0,0,
        py * (vx*py - vy*px) / pow(div, 3/2), px* (vy*px - vx*py) / pow(div, 3/2), px/sqrt(div), py/sqrt(div);
  }
	return Hj;  
 
}
