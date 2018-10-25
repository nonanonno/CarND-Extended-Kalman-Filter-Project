#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

namespace Tools
{

/**
   * @brief A helper method to calculate RMSE.
   * 
   * @param estimations 
   * @param ground_truth 
   * @return VectorXd 
   */
Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

/**
   * @brief A helper method to calculate Jacobians.
   * 
   * @param x_state 
   * @return MatrixXd 
   */
Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);
} // namespace Tools

#endif /* TOOLS_H_ */
