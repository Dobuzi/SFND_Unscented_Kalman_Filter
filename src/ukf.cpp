#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
   
   n_x_ = 5;
   n_aug_ = 7;
   lambda_ = 3 - n_aug_;

   weights_ = VectorXd(2 * n_aug_ + 1);

   is_initialized_ = false;

   Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

   time_us_ = 0;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_)
  {
    time_us_ = meas_package.timestamp_;
    double px, py, var_px, var_py;
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      px = meas_package.raw_measurements_(0);
      py = meas_package.raw_measurements_(1);
      var_px = pow(std_laspx_, 2);
      var_py = pow(std_laspy_, 2);
    }
    else
    {
      double rho = meas_package.raw_measurements_(0),
             phi = meas_package.raw_measurements_(1);
      px = rho * cos(phi);
      py = rho * sin(phi);
      var_px = std_radr_ * std_radphi_;
      var_py = std_radr_ * std_radphi_;
    }

    x_ << px, py, 0, 0, 0;
    P_ << var_px, 0, 0, 0, 0,
          0, var_py, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
    
    is_initialized_ = true;
    return;
  }

  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  Prediction(dt);
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  x_aug << x_, 0, 0;
  P_aug.fill(0);
  P_aug.topLeftCorner(n_x_, n_x_) << P_;
  P_aug.bottomRightCorner(n_aug_-n_x_, n_aug_-n_x_) << pow(std_a_, 2), 0,
                                                       0, pow(std_yawdd_, 2);
  MatrixXd A_aug = P_aug.llt().matrixL();
  MatrixXd x_aug_Mat = MatrixXd(n_aug_, n_aug_);
  
  for (int i = 0; i < x_aug_Mat.cols(); i++)
  {
    x_aug_Mat.col(i) = x_aug;
  }
  Xsig_aug << x_aug,
              x_aug_Mat + sqrt(lambda_+n_aug_) * A_aug,
              x_aug_Mat - sqrt(lambda_+n_aug_) * A_aug;

  Xsig_pred_.fill(0.0);
  MatrixXd X_del = MatrixXd(n_x_, 1);
  MatrixXd X_noise = MatrixXd(n_x_, 1);

  for (int i = 0; i < 2 * n_aug_+1; i++)
  {
    const double v = Xsig_aug(2, i),
                 yaw = Xsig_aug(3, i),
                 yawd = Xsig_aug(4, i),
                 nu_a = Xsig_aug(5, i),
                 nu_yawdd = Xsig_aug(6, i);
    
    if (yawd == 0)
    {
      X_del << v * cos(yaw) * delta_t,
               v * sin(yaw) * delta_t,
               0,
               0,
               0;
    }
    else
    {
      X_del << v / yawd * (sin(yaw+yawd*delta_t) - sin(yaw)),
               v / yawd * (-cos(yaw+yawd*delta_t) + cos(yaw)),
               0,
               yawd * delta_t,
               0;
    }
    X_noise << 0.5 * pow(delta_t, 2) * cos(yaw) * nu_a,
               0.5 * pow(delta_t, 2) * sin(yaw) * nu_a,
               delta_t * nu_a,
               0.5 * pow(delta_t, 2) * nu_yawdd,
               delta_t * nu_yawdd;

    Xsig_pred_.col(i) = Xsig_aug.col(i).head(n_x_) + X_del + X_noise;
  }

  VectorXd x_pred = VectorXd(n_x_);
  MatrixXd P_pred = MatrixXd(n_x_, n_x_);
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    if (i > 0)
    {
      weights_(i) = 1 / (2 * (lambda_ + n_aug_));
    }

    for (int j = 0; j < n_x_; j++)
    {
      x_pred(j) += weights_(i) * Xsig_pred_(j, i);
    }
  }

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    MatrixXd x_diff = Xsig_pred_.col(i) - x_pred;
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -1*M_PI) x_diff(3) += 2. * M_PI;
    P_pred += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
   
}