#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

/**
 * @brief Kalman filter for tracking object using ladar and radar
 * 
 */
class KalmanFilter
{
public:
  // model
  Eigen::VectorXd x_; //!< state vector
  Eigen::MatrixXd P_; //!< covariance matrix

  // for prediction
  Eigen::MatrixXd F_; //!< state transition matrix
  Eigen::MatrixXd Q_; //!< process covariance matrix;

  // for kalman
  Eigen::MatrixXd H_; //!< measurement matrix
  Eigen::MatrixXd R_; //!< measurement covariance matrix

  // constant value
  Eigen::MatrixXd I; //!< identity matrix
public:
  KalmanFilter() = default;
  virtual ~KalmanFilter() = default;

  void Initialize(const Eigen::VectorXd &x, const Eigen::MatrixXd &P);

  /**
   * @brief Prediction Predicts the state and the state covariance
   * using the process model
   * @param dt Time between k and k+1 in s
   */
  void Predict(float dt);

  /**
   * @brief Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
  * @brief Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);
};

#endif /* KALMAN_FILTER_H_ */
