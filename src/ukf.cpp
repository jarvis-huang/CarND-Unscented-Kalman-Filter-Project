#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  
  is_initialized_ = false;

  n_x_ = 5;
  n_aug_ = n_x_ + 2;
  
  // initial state vector
  x_ = VectorXd(n_x_); // px, py, v, yaw, yawd

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 4.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.5;

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
  
  P_.fill(0.0);
  //P_ = MatrixXd::Identity(n_x_, n_x_);
     
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  x_.fill(0.0);

  // Pre-compute weights (for predicting mean and covariance from sigma points)
  double lambda = 3 - n_aug_;
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda / (lambda + n_aug_);
  weights_.tail(2*n_aug_) = VectorXd::Constant(2*n_aug_, 0.5/(lambda + n_aug_));
  
  nis_laser_file.open ("nis_laser.txt");
  nis_radar_file.open ("nis_radar.txt");
  log_file.open("log.txt");
}

UKF::~UKF() {
    nis_laser_file.close();
    nis_radar_file.close();
    log_file.close();
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_)
  {
    if (meas_package.sensor_type_==MeasurementPackage::LASER)
    {
        x_(0) = meas_package.raw_measurements_(0);
        x_(1) = meas_package.raw_measurements_(1);
        x_(2) = 0;
        x_(3) = 0;
        x_(4) = 0;
    }
    else
    {
        double r = meas_package.raw_measurements_(0);
        double phi = meas_package.raw_measurements_(1);
        double rdot = meas_package.raw_measurements_(2);
        x_(0) = r*cos(phi);
        x_(1) = r*sin(phi);
        x_(2) = rdot;
        x_(3) = 0;
        x_(4) = 0;
        
    }   
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    return;
  }
    
  double delta_t = (meas_package.timestamp_-time_us_)/1000000.0;

  if (meas_package.sensor_type_==MeasurementPackage::LASER && use_laser_)
  {
    // prediction
    Prediction(delta_t);    
    // update
    UpdateLidar(meas_package);
    time_us_ = meas_package.timestamp_;
  }
  else if (meas_package.sensor_type_==MeasurementPackage::RADAR && use_radar_)
  {
    // prediction
    Prediction(delta_t);
    // update
    UpdateRadar(meas_package);
    time_us_ = meas_package.timestamp_;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  lambda_ = 3 - n_x_;
  
  // Create sigma point matrix
  VectorXd x_aug(n_aug_); 
  x_aug.head(n_x_) = x_;
  x_aug.tail(2) << 0, 0; // augmentation
  
  MatrixXd P_aug(n_aug_,n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug.bottomRightCorner(2,2) << std_a_*std_a_, 0,
                                  0, std_yawdd_*std_yawdd_; // augmentation
    
  // Compute augmented sigma points                           
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  lambda_ = 3 - n_aug_;
  double factor = sqrt(lambda_+n_aug_);
  
  // Calculate square root of P
  MatrixXd A_aug = P_aug.llt().matrixL();

  // Compute sigma points (column vectors of Xsig)
  for (unsigned int i=1; i<=n_aug_; i++)
  {
      Xsig_aug.col(i) = x_aug + factor*A_aug.col(i-1);
      Xsig_aug.col(i+n_aug_) = x_aug - factor*A_aug.col(i-1);
  }
  
  // Predict sigma points thru CTRV motion model
  for (unsigned int i=0; i<=2*n_aug_; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  // Predicted mean
  x_.fill(0.0);
  for (unsigned int i=0; i<=2*n_aug_; i++)
  {
      x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // Predicted covariance
  P_.fill(0.0);
  for (unsigned int i=0; i<=2*n_aug_; i++)
  {
      VectorXd X_diff = Xsig_pred_.col(i) - x_;
      while (X_diff(3)> M_PI) X_diff(3)-=2.*M_PI;
      while (X_diff(3)<-M_PI) X_diff(3)+=2.*M_PI;     
      P_ += X_diff * X_diff.transpose() * weights_(i);
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:
  Using the linear Kalman filter update equations just like in the EKF 
  project should give the same results and has the advantage to 
  require less computational resources. 
  Thus, better Real Time performance.
  */
  
  int n_z = 2;
  MatrixXd Zsig(n_z, 2 * n_aug_+1);

  //transform sigma points into measurement space
  for (unsigned int i=0; i<=2 * n_aug_; i++)
  {
      Zsig(0,i) = Xsig_pred_(0,i); // px
      Zsig(1,i) = Xsig_pred_(1,i); // py
  }
  
  //calculate mean predicted measurement
  VectorXd z_pred(n_z);
  z_pred.fill(0.0);
  for (unsigned int i=0; i<=2 * n_aug_; i++)
  {
      z_pred += Zsig.col(i) * weights_(i);
  }

  //calculate measurement covariance matrix S
  MatrixXd S(n_z,n_z);
  S.fill(0.0);
  for (unsigned int i=0; i<=2 * n_aug_; i++)
  {
      VectorXd z_diff = Zsig.col(i) - z_pred;
      S += z_diff * z_diff.transpose() * weights_(i);
  }
  MatrixXd R(n_z,n_z);
  R.fill(0.0);
  R(0,0) = std_laspx_*std_laspx_;
  R(1,1) = std_laspy_*std_laspy_;
  S += R;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (unsigned int i=0; i<2*n_aug_+1; i++)
  {
      VectorXd x_diff = Xsig_pred_.col(i)-x_;
      //angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;     
      
      VectorXd z_diff = Zsig.col(i)-z_pred;
      
      Tc += weights_(i) * x_diff * z_diff.transpose();    
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();
    
  //update state mean and covariance matrix
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_diff = z-z_pred;
  x_ += K*z_diff;
  P_ -= K*S*K.transpose();

  // NIS
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  nis_laser_file << NIS_laser_ << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  int n_z = 3;
  MatrixXd Zsig(n_z, 2 * n_aug_+1);
  
  //transform sigma points into measurement space
  for (unsigned int i=0; i<=2 * n_aug_; i++)
  {
      double px = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double psi = Xsig_pred_(3,i);
      //double psid = Xsig_pred_(4,i);
          
      Zsig(0,i) = sqrt(px*px+py*py);
      Zsig(1,i) = atan2(py, px);
      Zsig(2,i) = v*(px*cos(psi)+py*sin(psi))/Zsig(0,i);
  }  
  
  //calculate mean predicted measurement
  VectorXd z_pred(n_z);
  z_pred.fill(0.0);
  for (unsigned int i=0; i<=2 * n_aug_; i++)
  {
      z_pred += Zsig.col(i) * weights_(i);
  }    
  
  //calculate measurement covariance matrix S
  MatrixXd S(n_z,n_z);
  S.fill(0.0);
  for (unsigned int i=0; i<=2 * n_aug_; i++)
  {
      VectorXd z_diff = Zsig.col(i) - z_pred;
      
      //angle normalization
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
      S += z_diff * z_diff.transpose() * weights_(i);
  }
  MatrixXd R(n_z,n_z);
  R.fill(0.0);
  R(0,0) = std_radr_*std_radr_;
  R(1,1) = std_radphi_*std_radphi_;
  R(2,2) = std_radrd_*std_radrd_;
  S += R;  
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (unsigned int i=0; i<2*n_aug_+1; i++)
  {
      VectorXd x_diff = Xsig_pred_.col(i)-x_;
      //angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;     
      
      VectorXd z_diff = Zsig.col(i)-z_pred;
      //angle normalization
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
      
      Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();
  
  //update state mean and covariance matrix
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_diff = z-z_pred;
  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
  x_ += K*z_diff;
  P_ -= K*S*K.transpose();  
  
  // NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  nis_radar_file << NIS_radar_ << std::endl;
}

void UKF::Log(const VectorXd& estimate, const VectorXd& gt)
{
    log_file << estimate(0) << " " << estimate(1) << " " << estimate(2) << " " << estimate(3);
    log_file << " " << gt(0) << " " << gt(1) << " " << gt(2) << " " << gt(3) << std::endl;
}
