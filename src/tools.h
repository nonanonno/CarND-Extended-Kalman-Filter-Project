#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools
{
public:
  /**
   * @brief Construct a new Tools object
   * 
   */
  Tools();

  /**
   * @brief Destroy the Tools object
   * 
   */
  virtual ~Tools();

  /**
   * @brief A helper method to calculate RMSE.
   * 
   * @param estimations 
   * @param ground_truth 
   * @return VectorXd 
   */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
   * @brief A helper method to calculate Jacobians.
   * 
   * @param x_state 
   * @return MatrixXd 
   */
  MatrixXd CalculateJacobian(const VectorXd &x_state);
};

#endif /* TOOLS_H_ */
