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
  assert(estimations.size()>0);
  int n_x = estimations[0].size();
  VectorXd rmse(n_x); // px, py, vx, vy
  rmse.fill(0.0);
  int count = 0;
  for (unsigned i=0; i<estimations.size(); i++)
  {
      VectorXd est = estimations[i];
      VectorXd gt = ground_truth[i];
      VectorXd err = est - gt;
      rmse = rmse + (err.array()*err.array()).matrix();
      count++;
      //std::cout << "est=" << est << "\ngt=" << gt << std::endl;
  }
  
  rmse /= count;
  rmse = rmse.array().sqrt();
  return rmse;
}
