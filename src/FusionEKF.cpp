#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools tool;

FusionEKF::FusionEKF() {

  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

	ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
			    0, 1, 0, 0,
			    0, 0, 1000, 0,
			    0, 0, 0, 1000;

	ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
    			  0, 1, 0, 1,
    			  0, 0, 1, 0,
    			  0, 0, 0, 1;

  H_laser_<< 1, 0, 0, 0,
			       0, 1, 0, 0;

}

/**
 *Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement

    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;
    //cout<<ekf_.x_<<'\n';

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
       //cout << "Sensor_Type: RADAR" << '\n'<<endl;
      float ro = measurement_pack.raw_measurements_[0];
      float psi = measurement_pack.raw_measurements_[1];
      ekf_.x_ << ro*cos(psi), ro*sin(psi), 0, 0;
      previous_timestamp_ = measurement_pack.timestamp_;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //cout << "Sensor_Type: LASER" << '\n'<<endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      previous_timestamp_ = measurement_pack.timestamp_;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    //cout<<ekf_.x_<<endl;
    return;
  }

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the process covariance matrix Q

  float noise_ax = 9;
  float noise_ay = 9;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
        0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
        dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
        0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
   // Make sure F, P, and Q is properly processed before calling ekf_.Predict()
   ekf_.Predict();

   /*****************************************************************************
    *  Update
    ****************************************************************************/
   // Renew H, R before update
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Update measurement related matrix H and R
    cout << "Sensor_Type: RADAR" << '\n'<<endl;
    ekf_.H_ = tool.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)  {
    // Update measurement related matrix H and R
    cout << "Sensor_Type: LASER" << '\n'<<endl;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " <<"\n" << ekf_.x_ << "\n"<<endl;
  cout << "P_ = " << "\n" << ekf_.P_ << endl;
}
