#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

/**
 * @brief 
 * 
 */
class FusionEKF
{
public:
  /**
   * @brief Construct a new Fusion E K F object
   * 
   */
  FusionEKF();

  /**
   * @brief Destroy the Fusion E K F object
   * 
   */
  virtual ~FusionEKF();

  /**
   * @brief Run the whole flow of the Kalman Filter from here.
   * 
   * @param measurement_pack 
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * @brief Run the whole flow of the Kalman Filter from here.
   * 
   */
  KalmanFilter ekf_;

private:
  bool is_initialized_; //!< check whether the tracking toolbox was initialized or not (first measurement)

  long long previous_timestamp_;

  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
};

#endif /* FusionEKF_H_ */
