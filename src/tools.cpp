#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4); // the size of our state vector
  rmse.setZero();

  if (estimations.size() == 0) {
      std::cout << "estimation vector size must be greater than zero";
      return rmse;
  }
  if (estimations.size() != ground_truth.size()) {
      std::cout << "the estimation vector size must equal the ground truth vector size";
      return rmse;
  }

  //accumulate squared residuals
  for(int i = 0; i < estimations.size(); i++) {
    rmse += (estimations[i] - ground_truth[i]).array().pow(2).matrix();
  }

  //calculate the mean
  rmse /= estimations.size();

  //take the square root
  rmse = rmse.array().sqrt().matrix();

  return rmse;
}
