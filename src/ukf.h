#ifndef UKF_H
#define UKF_H
#include "Eigen/Dense"
#include "measurement_package.h"
#include "tools.h"
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* previous time stamp
  long long previous_timestamp_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///*set measurement dimension for radar, radar can measure r, phi, and r_dot
  int n_z_radar_;

  ///*set measurement dimension for radar, radar can measure r, phi, and r_dot
  int n_z_laser_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  ///* for using helper functions
  Tools tools;


  /**
   * Constructor
   */
  UKF();



  /**
   * Destructor
   */
  virtual ~UKF();



  /**
  * First initialization
  */
  void InitializeByFirstMeasurement(const MeasurementPackage &measurement_pack);



  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);



  /**
  * Prediction Predicts sigma points, the state, and the state covariance
  * matrix
  * @param delta_t Time between k and k+1 in s
  */
  void Prediction(double delta_t);



  /**
  * Updates the state and the state covariance matrix using a laser measurement
  * @param meas_package The measurement at k+1
  */
  void UpdateLidar(MeasurementPackage meas_package);



  /**
  * Updates the state and the state covariance matrix using a radar measurement
  * @param meas_package The measurement at k+1
  */
  void UpdateRadar(MeasurementPackage meas_package);



  /**
  * Generates augmented sigma points for the state matrix augmented with the process noise 
  * @param Xsig_out the matrix with 2*n_aug_+1 columns representing the sigma points
  */
  void GenerateAugmentedSigmaPoints(MatrixXd* Xsig_out);



  /**
  * Predicts the transformation of sigma points
  * @param Xsig_out the matrix with 2*n_aug_+1 columns representing the sigma points
  */
  void SigmaPointPrediction(MatrixXd* Xsig_out, const MatrixXd& Xsig_aug, const double delta_t);



  /**
  * Predicts the the mean and covariance of the state based on augmented sigma points
  * @param x_out the matrix with n_x_ elements
  * @param P_out is the covariance matrix
  */
  void PredictMeanAndCovariance(VectorXd* x_pred, MatrixXd* P_pred);



  /**
  * Estimates the predicted radar measurement in ro phi ro_dot space.
  * @param z_out the matrix with n_z_ elements representing the state
  * @param S_out is the covariance matrix
  * @param Zsig_out is the matrix of sigma points in measurement space
  */
  void PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out);



  /**
  * Update the state based on the current measurement and prediction
  * @param x_out is the state vector to be updated
  * @param P_out is the state covariance matrix to be updated
  * @param z_pred is the predicted state in measurement space
  * @param z_meas is the measured state in measurement space
  * @param S is the covariance matrix of the predicted space
  * @param Zsig is the vector of sigma points in measurement space
  * @param z is the number of parameters in the measurement space - i.e. n_z_radar_ or n_z_laser_
  */
  void UpdateState(VectorXd* x_out, MatrixXd* P_out, const VectorXd& z_pred, const VectorXd& z_meas, const MatrixXd& S, const MatrixXd& Zsig, int n_z);

};

#endif /* UKF_H */
