#include "tools.h"


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()	|| estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (size_t i = 0; i < estimations.size(); ++i) {
    // initialize variables
    double px =       estimations[i](0);
    double py =       estimations[i](1);
    double v =        estimations[i](2);
    double yaw =      estimations[i](3);
    double yaw_dot =  estimations[i](4);
    // convert to px, py, px_dot and py_dot
    double px_dot = v * cos(yaw);
    double py_dot = v * sin(yaw);
    VectorXd converted_(4);
    converted_ << px, py, px_dot, py_dot;
    //
    VectorXd diff = converted_ - ground_truth[i];
    diff = diff.array()*diff.array();
    rmse += diff;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;

}



VectorXd Tools::CalculatePolarFromCartesian(const Eigen::VectorXd& x_state) {
  /**
  * Calculate polar values ro, phi and ro_dot from cartesian position, velocity, yaw and yaw_dot values
  */

  // recover cartesian values from state vector
  double px =       x_state(0);
  double py =       x_state(1);
  double v =        x_state(2);
  double yaw =      x_state(3);
  double yaw_dot =  x_state(4);
  // calculate polar values
  double ro = sqrt(px*px + py*py);
  double phi = atan2(py, px);
  double ro_dot;
  if (ro>0.001) {
    ro_dot = (px*cos(yaw)*v + py*sin(yaw)*v) / ro;
  }
  else {
    ro_dot = (px*cos(yaw)*v + py*sin(yaw)*v) / 0.001;
  }

  // define and initialize h_x
  VectorXd polar(3);
  polar << ro, phi, ro_dot;

  return polar;
}


VectorXd Tools::CalculateCartesianFromPolar(const Eigen::VectorXd& x_state) {
  /**
  * A helper method to calculate c(x), which returns the cartesian coordinates (i.e. px, py, px_dot and py_dot) from polar coordinates (i.e. ro, phi and ro_dot)
  */
  double ro;
  double phi;
  double ro_dot;
  //
  double px;
  double py;
  double px_dot;
  double py_dot;
  double v;
  double yaw;
  double yaw_dot;
  //
  ro =      x_state(0);
  phi =     x_state(1);
  ro_dot =  x_state(2);
  // calculating position and velocity
  px = ro * cos(phi);
  py = ro * sin(phi);
  v = ro_dot;
  if ((py == 0) || ((px*px / py + py) == 0)) {
    py_dot = 0;
  }
  else {
    py_dot = ro * ro_dot / (px*px / py + py);
  }
  if (py == 0) {
    px_dot = 0;
  }
  else {
    px_dot = px*py_dot / py;
  }
  yaw = atan2(py_dot, px_dot);
  yaw_dot = 0;
  //
  Eigen::VectorXd cartesian(5);
  cartesian << px, py, v, yaw, yaw_dot;

  return cartesian;
}


/**
* A helper method for calculating weights to be used for estimating mean and covariance from the sigma points
* @param n: size of the weight vector - i.e. n_aug
*/
VectorXd Tools::CalculateWeights(int n, double lambda) {
 
  //create vector for weights
  VectorXd weights(2 * n + 1);

  //set vector for weights
  double weight_0 = lambda / (lambda + n);
  weights(0) = weight_0;
  for (int i = 1; i < 2 * n + 1; i++) {
    double weight = 0.5 / (n + lambda);
    weights(i) = weight;
  }

  return weights;
}


/**
* A helper method for calculating the percentage of the elemnets in a list that are greater than a certain threshold
* @param values is a vector of double with all values in the list
* @param threshold is used for calculating the fraction larger
*/
double Tools::FractionLargerThanThreshold(const std::vector<double> &values, double threshold) {
  
  if (values.size() == 0) {
    return 0.0;
  }

  double counter = 0;
  for (size_t i = 0; i < values.size(); ++i) {
    if (values[i] > threshold) {
      counter += 1.0;
    }
  }
  return counter / values.size();

}