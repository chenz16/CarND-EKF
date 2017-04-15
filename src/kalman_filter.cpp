#include "kalman_filter.h"
#include <cmath>
#include <iostream>
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd z_pred=VectorXd(3);
  float px=x_(0);
  float py=x_(1);
  float vx=x_(2);
  float vy=x_(3);
  //cout<<"x_\n"<<x_<<"\n"<<endl;
  float roh_pred = sqrt(px*px + py*py);
  float psi_pred = atan(py/px);
  float dot_psi_pred = (px*vx+py*vy)/roh_pred;

  if (roh_pred < 0.01) {
    roh_pred=0.01;
    psi_pred = atan(py/roh_pred)*px/(fabs(px)+0.0001);
    dot_psi_pred = (px*vx+py*vy)/roh_pred;
  }
  z_pred << roh_pred, psi_pred, dot_psi_pred;
  //cout<<"z pred"<<z_pred<<endl;


  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //cout<<"x="<<x_<<endl;
  //cout<<"y="<<y<<"\n"<<endl;
  //cout<<"K="<<K<<endl;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
