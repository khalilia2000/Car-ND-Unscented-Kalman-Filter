#include <iostream>
#include "ukf.h"

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.2; //30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2; // 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;

  //set measurement dimension for radar, radar can measure r, phi, and r_dot
  n_z_radar_ = 3;

  //set measurement dimension for laser, laser can measure px and py
  n_z_laser_ = 2;

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  //create vector for weights
  weights_ = tools.CalculateWeights(n_aug_, lambda_);
  
  //prediction sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

}



UKF::~UKF() {}



void UKF::InitializeByFirstMeasurement(const MeasurementPackage &measurement_pack) {

  // first measurement
  x_.fill(0.0);
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    // Convert to cartesian coordinates and assign the position
    Eigen::VectorXd radar_x_ = tools.CalculateCartesianFromPolar(measurement_pack.raw_measurements_);
    x_ << radar_x_;
    //
    previous_timestamp_ = measurement_pack.timestamp_;
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

    x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0, 0;
    //
    previous_timestamp_ = measurement_pack.timestamp_;
  }

  //re-set state covariance matrix P
  P_.fill(0.0);
  P_(0, 0) = 1;
  P_(1, 1) = 1;
  P_(2, 2) = 1000;
  P_(3, 3) = 1000;
  P_(4, 4) = 1000;

  // done initializing
  is_initialized_ = true;

}



/**
* @param {MeasurementPackage} meas_package The latest measurement data of
* either radar or laser.
*/
void UKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  if (!is_initialized_) {
    //initialize using first measurement
    InitializeByFirstMeasurement(measurement_pack);
  }
  else {

    //compute the time elapsed between the current and previous measurements
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    // Predict the new state based on dt - same for both laser and radar
    Prediction(dt);

    //  Update - Use the sensor type to perform the update step - Update the state and covariance matrices.
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      if (use_radar_) {
        UpdateRadar(measurement_pack);
      }
    } else {
      if (use_laser_) {
        UpdateLidar(measurement_pack);
      }
    }

  }
}



/**
* Predicts sigma points, the state, and the state covariance matrix.
* @param {double} delta_t the change in time (in seconds) between the last
* measurement and this one.
*/
void UKF::Prediction(double delta_t) {

  //generate sigma points from state distribution
  MatrixXd Xsig_aug(n_aug_, 2 * n_aug_ + 1);
  GenerateAugmentedSigmaPoints(&Xsig_aug);

  //predict sigma points to the current timestep
  SigmaPointPrediction(&Xsig_pred_, Xsig_aug, delta_t);

  //predict mean and covariance and update x_ and P_ matrices
  PredictMeanAndCovariance(&x_, &P_);

}



/**
* Updates the state and the state covariance matrix using a laser measurement.
* @param {MeasurementPackage} meas_package
*/
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  
  // initialize z_pred and S of the measurement space
  VectorXd z_pred = x_.head(n_z_laser_);
  MatrixXd S = P_.topLeftCorner(n_z_laser_, n_z_laser_);
  MatrixXd Zsig = Xsig_pred_.topRows(n_z_laser_);

  //add measurement noise to covariance matrix S
  MatrixXd R = MatrixXd(n_z_laser_, n_z_laser_);
  R.fill(0.0);
  R(0, 0) = std_laspx_*std_laspx_;
  R(1, 1) = std_laspx_*std_laspy_;
  S += R;

  // update space
  UpdateState(&x_, &P_, z_pred, meas_package.raw_measurements_, S, Zsig, n_z_laser_);

}



/**
* Updates the state and the state covariance matrix using a radar measurement.
* @param {MeasurementPackage} meas_package
*/
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  
  // transfer the predicted sigma points to the measurement space
  VectorXd z_pred(n_z_radar_);
  MatrixXd S(n_z_radar_, n_z_radar_);
  MatrixXd Zsig(n_z_radar_, 2 * n_aug_ + 1);
  PredictRadarMeasurement(&z_pred, &S, &Zsig);

  // update state
  UpdateState(&x_, &P_, z_pred, meas_package.raw_measurements_, S, Zsig, n_z_radar_);

}



/**
* Generates augmented sigma points for the state matrix augmented with the process noise
* @param Xsig_out the matrix with 2*n_aug_+1 columns representing the sigma points
*/
void UKF::GenerateAugmentedSigmaPoints(MatrixXd* Xsig_out) {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;
  for (int i = n_x_; i<n_aug_; i++) {
    x_aug(i) = 0;
  }

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.block(0, 0, n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_*std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < P_aug.cols(); i++) {
    Xsig_aug.col(i + 1) =                 x_aug + sqrt(lambda_ + n_aug_) * A_aug.col(i);
    Xsig_aug.col(i + 1 + P_aug.cols()) =  x_aug - sqrt(lambda_ + n_aug_) * A_aug.col(i);
  }

  //write result
  *Xsig_out = Xsig_aug;

}



/**
* Predicts the transformation of sigma points
* @param Xsig_out the matrix with 2*n_aug_+1 columns representing the sigma points
*/
void UKF::SigmaPointPrediction(MatrixXd* Xsig_out, const MatrixXd& Xsig_aug, const double delta_t) {

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // time squared
  double delta_t2 = delta_t*delta_t;

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // initialize variables using previous state
    double px =       Xsig_aug(0, i);
    double py =       Xsig_aug(1, i);
    double v =        Xsig_aug(2, i);
    double yaw =      Xsig_aug(3, i);
    double yaw_dot =  Xsig_aug(4, i);
    double nu_a =     Xsig_aug(5, i);
    double nu_yaw =   Xsig_aug(6, i);
    // update next state including noise addition
    if (yaw_dot<0.001) {
      Xsig_pred(0, i) = px + v*cos(yaw)*delta_t + 0.5*delta_t2*cos(yaw)*nu_a;
      Xsig_pred(1, i) = py + v*sin(yaw)*delta_t + 0.5*delta_t2*sin(yaw)*nu_a;
    } else {
      Xsig_pred(0, i) = px + v / yaw_dot*(sin(yaw + yaw_dot*delta_t) - sin(yaw)) + 0.5*delta_t2*cos(yaw)*nu_a;
      Xsig_pred(1, i) = py + v / yaw_dot*(-cos(yaw + yaw_dot*delta_t) + cos(yaw)) + 0.5*delta_t2*sin(yaw)*nu_a;

    }
    Xsig_pred(2, i) = v + delta_t*nu_a;
    Xsig_pred(3, i) = yaw + yaw_dot*delta_t + 0.5*delta_t2*nu_yaw;
    Xsig_pred(4, i) = yaw_dot + delta_t*nu_yaw;
  }

  //write result
  *Xsig_out = Xsig_pred;

}



/**
* Predicts the the mean and covariance of the state based on augmented sigma points
* @param x_out the matrix with n_x_ elements
* @param P_out is the covariance matrix
*/
void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {

  //create vector for predicted state
  VectorXd x(n_x_);

  //create covariance matrix for prediction
  MatrixXd P(n_x_, n_x_);
      
  //predict state mean
  x.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; i++) {
    x += weights_(i)*Xsig_pred_.col(i);
  }

  //std::cout << Xsig_pred_ << std::endl;

  //predict state covariance matrix
  P.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    P += weights_(i)*(x_diff*x_diff.transpose());
  }

  //write result
  *x_out = x;
  *P_out = P;
}



/**
* Estimates the predicted radar measurement in ro phi ro_dot space. 
* @param z_out the matrix with n_z_ elements representing the state
* @param S_out is the covariance matrix
*/
void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {

  //create matrix for sigma points in measurement space
  MatrixXd Zsig(n_z_radar_, 2 * n_aug_ + 1);
  Zsig.fill(0.0);

  //mean predicted measurement
  VectorXd z_pred(n_z_radar_);
  z_pred.fill(0.0);

  //measurement covariance matrix S
  MatrixXd S(n_z_radar_, n_z_radar_);
  S.fill(0.0);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Zsig.col(i) = tools.CalculatePolarFromCartesian(Xsig_pred_.col(i));
  }

  //calculate mean predicted measurement
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred += weights_(i)*Zsig.col(i);
  }

  //calculate measurement covariance matrix S
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // state difference
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;
    //
    S += weights_(i)*(z_diff*z_diff.transpose());
  }
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_radar_, n_z_radar_);
  R.fill(0.0);
  R(0, 0) = std_radr_*std_radr_;
  R(1, 1) = std_radphi_*std_radphi_;
  R(2, 2) = std_radrd_*std_radrd_;
  S += R;

  //write result
  *z_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
}



/**
* Update the state based on the current measurement and prediction
* @param x_out is the state vector to be updated
* @param P_out is the state covariance matrix to be updated
* @param z_pred is the predicted state in measurement space
* @param z_meas is the measured state in measurement space
* @param S is the covariance matrix of the predicted space
* @param Zsig is the vector of sigma points in measurement space
*/
void UKF::UpdateState(VectorXd* x_out, MatrixXd* P_out, const VectorXd& z_pred, const VectorXd& z_meas, const MatrixXd& S, const MatrixXd& Zsig, int n_z) {


  //create vector/matrix for predicted state mean and state covariance
  //create vector/matrix for predicted state mean and state covariance
  VectorXd x = x_;
  MatrixXd P = P_;
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;
    while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

    Tc += weights_(i)*(x_diff*z_diff.transpose());
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //update state mean and covariance matrix
  VectorXd z_diff = z_meas - z_pred;
  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

  //
  x += K*z_diff;
  P -= K*S*K.transpose();

  //write result
  *x_out = x;
  *P_out = P;
  
}
