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
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.0;
  
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

template<typename T>
void normalizeAngle(T &angle)
{
  angle = fmod(angle + M_PI, 2.*M_PI);
  if (angle < 0)
  {
    angle += 2. * M_PI;
  }
  angle -= M_PI;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_)
  {
    double px, py, var_px, var_py, var_v = 1., var_angle = 1.;

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
      var_px    = pow(std_radr_, 2);
      var_py    = var_px;
      var_v     = pow(std_radrd_, 2);
      var_angle = pow(std_radphi_, 2);
    }
    x_ << px, py, -0.3, 0., 0.;
    P_ << var_px, 0., 0., 0., 0.,
          0., var_py, 0., 0., 0.,
          0., 0., var_v, 0., 0.,
          0., 0., 0., var_angle, 0.,
          0., 0., 0., 0., var_angle;
    
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  const float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  Prediction(dt);
  time_us_ = meas_package.timestamp_;
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    UpdateRadar(meas_package);
  }
  return;
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // Augmentation
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  x_aug << x_, 0., 0.;
  P_aug.fill(0.);
  P_aug.topLeftCorner(n_x_, n_x_) << P_;
  P_aug.bottomRightCorner(n_aug_-n_x_, n_aug_-n_x_) << pow(std_a_, 2), 0.,
                                                       0., pow(std_yawdd_, 2);
  MatrixXd A_aug = P_aug.llt().matrixL();
  MatrixXd x_aug_Mat = MatrixXd(n_aug_, n_aug_);
  
  for (int i = 0; i < n_aug_; i++)
  {
    x_aug_Mat.col(i) = x_aug;
  }
  Xsig_aug << x_aug,
              x_aug_Mat + sqrt(lambda_+n_aug_) * A_aug,
              x_aug_Mat - sqrt(lambda_+n_aug_) * A_aug;

  // Sigma Point Prediction
  Xsig_pred_.fill(0.);
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
               0.,
               0.,
               0.;
    }
    else
    {
      X_del << v / yawd * (sin(yaw+yawd*delta_t) - sin(yaw)),
               v / yawd * (-cos(yaw+yawd*delta_t) + cos(yaw)),
               0.,
               yawd * delta_t,
               0.;
    }
    X_noise << 0.5 * pow(delta_t, 2) * cos(yaw) * nu_a,
               0.5 * pow(delta_t, 2) * sin(yaw) * nu_a,
               delta_t * nu_a,
               0.5 * pow(delta_t, 2) * nu_yawdd,
               delta_t * nu_yawdd;

    Xsig_pred_.col(i) = Xsig_aug.col(i).head(n_x_) + X_del + X_noise;
  }

  // Predicted Mean and Covariance
  x_.fill(0.);
  P_.fill(0.);

  weights_(0) = lambda_ / (lambda_ + n_aug_);
  double weight = 0.5 / (lambda_ + n_aug_);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    if (i > 0)
    {
      weights_(i) = weight;
    }

    for (int j = 0; j < n_x_; j++)
    {
      x_(j) += weights_(i) * Xsig_pred_(j, i);
    }
  }
  while (x_(3) >  M_PI) x_(3) -= 2. * M_PI;
  while (x_(3) < -M_PI) x_(3) += 2. * M_PI;

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    normalizeAngle(x_diff(3));
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  
  // Set measurement error
  MatrixXd R = MatrixXd(2, 2);
  R << pow(std_laspx_, 2), 0.,
      0., pow(std_laspy_, 2);
  
  // Measurement Function
  MatrixXd H = MatrixXd(2, 5);
  H << 1., 0., 0., 0., 0.,
       0., 1., 0., 0., 0.;

  // Get Kalman Gain
  VectorXd y = meas_package.raw_measurements_ - H * x_;
  MatrixXd S = H * P_ * H.transpose() + R;
  MatrixXd K = P_ * H.transpose() * S.inverse();

  // Measurement Update
  x_ += K * y;
  normalizeAngle(x_(3));
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  P_ = (I - K * H) * P_;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  // Predict Radar Measurement
  int n_z = 3;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    double px  = Xsig_pred_(0, i),
           py  = Xsig_pred_(1, i),
           v   = Xsig_pred_(2, i),
           yaw = Xsig_pred_(3, i);

    Zsig(0, i) = sqrt(pow(px, 2) + pow(py, 2));
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = (px * cos(yaw) + py * sin(yaw)) * v / Zsig(0, i);
  }

  z_pred = (Zsig * weights_).rowwise().sum();

  MatrixXd R = MatrixXd(n_z, n_z);
  R << pow(std_radr_, 2), 0., 0.,
       0., pow(std_radphi_, 2), 0.,
       0., 0., pow(std_radrd_, 2);
  
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    normalizeAngle(z_diff(1));
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  S += R;

  // UKF Update
  VectorXd z = meas_package.raw_measurements_;
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  Tc.fill(0.);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    normalizeAngle(x_diff(3));
    normalizeAngle(z_diff(1));
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = MatrixXd(n_x_, n_z);
  K = Tc * S.inverse();

  VectorXd z_diff = z - z_pred;
  normalizeAngle(z_diff(1));
  
  x_ += K * z_diff;
  normalizeAngle(x_(3));
  P_ -= K * S * K.transpose(); 
}