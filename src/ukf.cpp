#include <iostream>
#include "ukf.h"

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
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
  int n_x_ = 5;

  //set augmented dimension
  int n_aug_ = 7;

  //set measurement dimension for radar, radar can measure r, phi, and r_dot
  int n_z_ = 3;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  //create vector for weights
  VectorXd weights_ = VectorXd(2 * n_aug_ + 1);
   
}



UKF::~UKF() {}



/**
* @param {MeasurementPackage} meas_package The latest measurement data of
* either radar or laser.
*/
/**
TODO:

Complete this function! Make sure you switch between lidar and radar
measurements.
*/
void UKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
  *  Initialization
  ****************************************************************************/
  if (!is_initialized_) {
    /**
    * Initialize the state ukf_.x_ with the first measurement.
    * Create the covariance matrix.
    * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */

    // first measurement
    x_ = VectorXd(4);
    x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Initialize state.
      */
      // Convert to cartesian coordinates and assign the position
      Eigen::VectorXd radar_x_ = tools.CalculateCartesianFromPolar(measurement_pack.raw_measurements_);
      x_ << radar_x_;
      //
      previous_timestamp_ = measurement_pack.timestamp_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      //
      previous_timestamp_ = measurement_pack.timestamp_;
    }

    //re-set state covariance matrix P
    P_(0, 0) = 1;
    P_(1, 1) = 1;
    P_(2, 2) = 1000;
    P_(3, 3) = 1000;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /****************************************************************************************************
  *  Prediction
  *  Update the state transition matrix F according to the new elapsed time - convert time to seconds.
  *  Update the process noise covariance matrix.
  ****************************************************************************************************/


  //compute the time elapsed between the current and previous measurements
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  //update the F matrix so that the time is integrated
  //  ukf_.F_(0, 2) = dt;
  //  ukf_.F_(1, 3) = dt;

  //update the process noise covariance matrix Q
  //note noise_ax and noise_ay are already set in the constructor
  //  ukf_.Q_(0, 0) = dt_4 / 4 * noise_ax;
  //  ukf_.Q_(0, 2) = dt_3 / 2 * noise_ax;
  //  ukf_.Q_(1, 1) = dt_4 / 4 * noise_ay;
  //  ukf_.Q_(1, 3) = dt_3 / 2 * noise_ay;
  //  ukf_.Q_(2, 0) = dt_3 / 2 * noise_ax;
  //  ukf_.Q_(2, 2) = dt_2 * noise_ax;
  //  ukf_.Q_(3, 1) = dt_3 / 2 * noise_ay;
  //  ukf_.Q_(3, 3) = dt_2 * noise_ay;

  //call predict function - same for both laser and radar
  Prediction(dt);

  /********************************************************************************************************
  *  Update - Use the sensor type to perform the update step - Update the state and covariance matrices.
  ********************************************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    // Convert to cartesian coordinate and calculate Jacobian matrix Hj
    Eigen::VectorXd radar_x_ = tools.CalculateCartesianFromPolar(measurement_pack.raw_measurements_);
//    Hj_ = tools.CalculateJacobian(radar_x_);
    // Assign matrices R and Hj
//       ukf_.R_ = R_radar_;
//      ukf_.H_ = Hj_;
//    // use Extended Kalman Filter update
//       ukf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else {
    // Laser updates
    // Set measurement matrix H and measurmeent covariance matrix R
//       ukf_.H_ = H_laser_;
//       ukf_.R_ = R_laser_;
//     use regular Kalman Filter update
//       ukf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}



/**
* Predicts sigma points, the state, and the state covariance matrix.
* @param {double} delta_t the change in time (in seconds) between the last
* measurement and this one.
*/
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}



/**
* Updates the state and the state covariance matrix using a laser measurement.
* @param {MeasurementPackage} meas_package
*/
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}



/**
* Updates the state and the state covariance matrix using a radar measurement.
* @param {MeasurementPackage} meas_package
*/
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}



/**
* Generates sigma points for the state matrix
* @param Xsig_out the matrix with 2*n_x_+1 columns representing the sigma points
*/
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {

  //define spreading parameter
  lambda_ = 3 - n_x_;

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //calculate sigma points ...
  
  //set the first collumn for sigma points
  Xsig.col(0) = x_;

  // set remaining sigma points
  for (int i = 0; i<P_.cols(); i++) {
    Xsig.col(i + 1) =             x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i + 1 + P_.cols()) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }

  //write result
  *Xsig_out = Xsig;

}


/**
* Generates augmented sigma points for the state matrix augmented with the process noise
* @param Xsig_out the matrix with 2*n_aug_+1 columns representing the sigma points
*/
void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
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
void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  //create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints(&Xsig_aug);

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //time diff in sec
  double delta_t = 0.1; 
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

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  SigmaPointPrediction(&Xsig_pred);
      
  //set weights
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    if (i == 0) {
      weights_(i) = lambda_ / (lambda_ + n_aug_);
    }
    else {
      weights_(i) = 0.5 / (lambda_ + n_aug_);
    }
  }

  //predict state mean
  x.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; i++) {
    x += weights_(i)*Xsig_pred.col(i);
  }

  //predict state covariance matrix
  P.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
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
void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) {

  //define spreading parameter
  lambda_ = 3 - n_aug_;
  
  //create matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  SigmaPointPrediction(&Xsig_pred);

  //set vector for weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    double weight_ = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight_;
  }

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // initializing variables
    double px = Xsig_pred(0, i);
    double py = Xsig_pred(1, i);
    double v = Xsig_pred(2, i);
    double yaw = Xsig_pred(3, i);
    double yaw_dot = Xsig_pred(4, i);
    //
    double ro = sqrt(px*px + py*py);
    double phi = atan2(py, px);
    double ro_dot;
    if (ro>0.001) {
      ro_dot = (px*cos(yaw)*v + py*sin(yaw)*v) / ro;
    }
    else {
      ro_dot = (px*cos(yaw)*v + py*sin(yaw)*v) / 0.001;
    }
    Zsig.col(i) << ro, phi, ro_dot;
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
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R.fill(0.0);
  R(0, 0) = std_radr_*std_radr_;
  R(1, 1) = std_radphi_*std_radphi_;
  R(2, 2) = std_radrd_*std_radrd_;
  S += R;

  //write result
  *z_out = z_pred;
  *S_out = S;
}


void UKF::UpdateState(VectorXd* x_out, MatrixXd* P_out) {

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //create vector/matrix for predicted state mean and state covariance
  VectorXd x = x_;
  MatrixXd P = P_;

  //set vector for weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }

  //create matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  SigmaPointPrediction(&Xsig_pred);

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // initializing variables
    double px = Xsig_pred(0, i);
    double py = Xsig_pred(1, i);
    double v = Xsig_pred(2, i);
    double yaw = Xsig_pred(3, i);
    double yaw_dot = Xsig_pred(4, i);
    //
    double ro = sqrt(px*px + py*py);
    double phi = atan2(py, px);
    double ro_dot;
    if (ro>0.001) {
      ro_dot = (px*cos(yaw)*v + py*sin(yaw)*v) / ro;
    }
    else {
      ro_dot = (px*cos(yaw)*v + py*sin(yaw)*v) / 0.001;
    }
    Zsig.col(i) << ro, phi, ro_dot;
  }

  //create example vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  //create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z_, n_z_);
  PredictRadarMeasurement(&z_pred, &S);

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_);
  z <<
    5.9214,   //rho in m
    0.2187,   //phi in rad
    2.0062;   //rho_dot in m/s

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
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
  VectorXd z_diff = z - z_pred;
  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

  //
  x += K*z_diff;
  P -= K*S*K.transpose();

  //write result
  *x_out = x;
  *P_out = P;
}