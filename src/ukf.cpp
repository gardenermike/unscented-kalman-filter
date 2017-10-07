#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.setIdentity(); // an approximation; some entries tuned below
  P_(0, 0) = 0.02;
  P_(0, 0) = 0.02;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.;

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

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.setConstant(1.0 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  R_radar_ = MatrixXd(3, 3);
  R_radar_ << pow(std_radr_, 2), 0, 0,
        0, pow(std_radphi_, 2), 0,
        0, 0, pow(std_radrd_, 2);

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << pow(std_laspx_, 2), 0,
              0, pow(std_laspy_, 2);

  laser_nis_file_.open("laser_nis.csv");
  radar_nis_file_.open("radar_nis.csv");

  is_initialized_ = false;
}

UKF::~UKF() {
  laser_nis_file_.close();
  radar_nis_file_.close();
}

/*
 * Initializes the first measurement to the filter
 */
void UKF::InitializeFirstMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    std::cout << "Initializing with first measurement" << std::endl;

    time_us_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];
      float x = rho * cos(phi);
      float y = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      float v = sqrt(vx * vx + vy * vy);

      x_ << x, y, v, 0., 0.;

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //set the state with the initial location, and estimated velocity, yaw, and delta yaw
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 5., 0., 0.;
    }

    is_initialized_ = true;

    std::cout << "Initialized first measurement" << endl;
  }
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    InitializeFirstMeasurement(measurement_pack);
    return;
  }

  //std::cout << "about to make prediction" << std::endl;
  double dt = DeltaTime(measurement_pack);
  this->Prediction(dt);

  //std::cout << "about to make update" << std::endl;
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    this->UpdateLidar(measurement_pack);

  } else {
    this->UpdateRadar(measurement_pack);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  // SIGMA POINTS
  // sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  // the first sigma point is the predicted mean
  Xsig.col(0) = x_;

  // scale A based on lambda
  A *= sqrt(lambda_ + n_x_);
  for (int i = 0; i < n_x_; i++) {
    // sigma points greater than mean
    Xsig.col(i + 1) = x_ + A.col(i);

    // sigma points less than mean
    Xsig.col(i + 1 + n_x_) = x_ - A.col(i);
  }

  // AUGMENTED x
  // created augmented x vector with (zero) noise
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.setZero();
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  MatrixXd Q = MatrixXd(2, 2);
  Q << std_a_ * std_a_, 0,
       0, std_yawdd_ * std_yawdd_;
  P_aug.bottomRightCorner(2, 2) = Q;

  // augmented square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();

  //create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // the first augmented sigma point is the predicted mean
  Xsig_aug.col(0) = x_aug;

  // scale square root matrix by lambda
  A_aug *= sqrt(lambda_ + n_aug_);
  // set remaining augmented sigma points
  for (int i = 0; i < n_aug_; i++) {
      Xsig_aug.col(i + 1) = x_aug + A_aug.col(i);
      Xsig_aug.col(i + 1 + n_aug_) = x_aug - A_aug.col(i);
  }

  // PREDICT SIGMA POINTS
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd x_k = Xsig_aug.col(i);
    VectorXd x = x_k.head(n_x_);
    VectorXd x1 = VectorXd(n_x_);
    VectorXd x2 = VectorXd(n_x_);

    float v = x_k(2);
    float phi = x_k(3);
    float phi_dot = x_k(4);
    float nu_a = x_k(5);
    float nu_phi_dot_dot = x_k(6);

    if (phi_dot != 0.0) {
      x1 << (v / phi_dot) * (sin(phi + phi_dot * delta_t) - sin(phi)),
            (v / phi_dot) * (-cos(phi + phi_dot * delta_t) + cos(phi)),
            0,
            phi_dot * delta_t,
            0;

    } else {
      x1 << v * cos(phi) * delta_t,
            v * sin(phi) * delta_t,
            0,
            phi_dot * delta_t,
            0;

    }
    x2 << 0.5 * delta_t * delta_t * cos(phi) * nu_a,
          0.5 * delta_t * delta_t * sin(phi) * nu_a,
          delta_t * nu_a,
          0.5 * delta_t * delta_t * nu_phi_dot_dot,
          delta_t * nu_phi_dot_dot;

    Xsig_pred_.col(i) = x + x1 + x2;
  }

  // UPDATE x_ and P_
  // update x
  x_.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ += Xsig_pred_.col(i) * weights_(i);
  }

  // update P
  P_.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd difference = Xsig_pred_.col(i) - x_;

    // normalize angle
    while (difference(3) > M_PI) difference(3) -= 2.*M_PI;
    while (difference(3) < -M_PI) difference(3) += 2.*M_PI;

    P_ += weights_[i] * difference * difference.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} measurement_pack
 */
void UKF::UpdateLidar(const MeasurementPackage &measurement_pack) {
  int n_z = 2; // size of measurements
  VectorXd z = measurement_pack.raw_measurements_.head(n_z);

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  Zsig.setZero();
  z_pred.setZero();

  //calculate mean predicted measurement
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Zsig.col(i) = Xsig_pred_.col(i).head(2);
    z_pred += Zsig.col(i) * weights_(i);
  }

  //calculate measurement covariance matrix S
  S.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_sig = Zsig.col(i);
    VectorXd z_diff = z_sig - z_pred;

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  S += R_laser_;
  MatrixXd S_inverse = S.inverse();

  // calculate cross-correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd pred = Xsig_pred_.col(i);
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S_inverse;

  //update state mean and covariance matrix
  x_ += K * (z - z_pred);

  P_ -= K * S * K.transpose();

  // calculate Normalized Innovation Squared error
  VectorXd z_diff = z - z_pred;
  float epsilon = (z_diff.transpose() * S_inverse * z_diff)(0);
  laser_nis_file_ << epsilon << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} measurement_pack
 */
void UKF::UpdateRadar(const MeasurementPackage &measurement_pack) {
  int n_z = 3; // size of measurements
  VectorXd z = measurement_pack.raw_measurements_.head(n_z);

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  Zsig.setZero();
  z_pred.setZero();

  //transform sigma points into measurement space
  //calculate mean predicted measurement
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd sigma_point = Xsig_pred_.col(i);
    double px = sigma_point(0);
    double py = sigma_point(1);
    double v = sigma_point(2);
    double psi = sigma_point(3);
    double psi_dot = sigma_point(4);

    double rho = sqrt(pow(px, 2) + pow(py, 2));
    double phi = atan2(py, px);
    double rho_dot = ((px * cos(psi) * v) + (py * sin(psi) * v)) / rho;

    VectorXd z = VectorXd(n_z);
    z << rho, phi, rho_dot;
    Zsig.col(i) = z;

    z_pred += z * weights_(i);
  }

  //calculate measurement covariance matrix S
  S.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_sig = Zsig.col(i);
    VectorXd z_diff = z_sig - z_pred;

    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  S += R_radar_;
  MatrixXd S_inverse = S.inverse();

  // calculate cross-correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd pred = Xsig_pred_.col(i);
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // normalize angles
    while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S_inverse;

  //update state mean and covariance matrix
  x_ += K * (z - z_pred);

  P_ -= K * S * K.transpose();

  // calculate Normalized Innovation Squared error
  VectorXd z_diff = z - z_pred;
  float epsilon = (z_diff.transpose() * S_inverse * z_diff)(0);
  radar_nis_file_ << epsilon << std::endl;
}

double UKF::DeltaTime(const MeasurementPackage &measurement_pack) {
  double dt = (measurement_pack.timestamp_ - time_us_) / 1000000.0;  //dt - expressed in seconds
  time_us_ = measurement_pack.timestamp_;
  return dt;
}
