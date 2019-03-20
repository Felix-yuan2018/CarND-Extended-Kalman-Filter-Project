#include "kalman_filter.h"
#include<iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;


/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_; 
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

//implement a function/method for normalization.
//This function could be verified once using unit testing.
void NormalizeAngle(double& phi){
  phi = atan2(sin(phi), cos(phi));
  }

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  ////Formulas for Kalman filter 
  //cout<< "Update Kalmanfilter: " << endl;
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
   //new estimate
  x_ += (K * y); 
  P_ -= K * H_ * P_;
	
  //x_ = x_ + (K * y);
  //long x_size = x_.size();
  //MatrixXd I = MatrixXd::Identity(x_size, x_size);
  //P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // For RADA data, we use Hj jacobian insted of H

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
 
 //Avoid division by zero
  float ro = sqrt(px*px + py*py);
  if (ro < .00001) {
    px += .001;
    py += .001;
    //ro = sqrt(px*px + py*py);
  }
  float theta = atan2(py , px);
  float ro_dot = (px*vx + py*vy) / ro;
  VectorXd z_pred(3);
  z_pred << ro, theta, ro_dot;

  VectorXd y = z - z_pred;

  // make sure that the angle is between -pi and pi,first is a solution
  //for (; y(1) < -M_PI; y(1) += 2*M_PI) {}
  //for (; y(1) > M_PI;  y(1) -= 2*M_PI) {}
  
  //second solution,call NormalizeAngle function
  NormalizeAngle(y(1));
 
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ += (K * y); 
  P_ -= K * H_ * P_;

}
