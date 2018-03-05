#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd sum_squared = VectorXd(4);
  sum_squared << 0, 0, 0, 0;
  
  for (int i=0; i<estimations.size(); ++i) {
	  sum_squared = sum_squared.array() + (estimations[i] - ground_truth[i]).array().square();
  }

  VectorXd rmse = sum_squared / estimations.size();
  rmse = rmse.array().sqrt();
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  float divisor = std::pow(px,2) + std::pow(py,2);
  
  // check division by zero
  if(divisor == 0) {
	  cout << "CalculateJacobian () - Error - Division by Zero" << endl;
	  exit(0);
  }
  
  Hj << (px/std::sqrt(divisor)), (py/std::sqrt(divisor)), 0, 0,
		-(py/divisor), (px/divisor), 0, 0,
		(py*(vx*py - vy*px)/std::pow(divisor,1.5)), (px*(vy*px - vx*py)/std::pow(divisor,1.5)), (px/std::sqrt(divisor)), (py/std::sqrt(divisor));

  return Hj;
}
