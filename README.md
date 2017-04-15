# Extended Kalman Filter Project Submission
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

  * cmake >= 3.5
  * make >= 4.1
  * gcc/g++ >= 5.4

## Basic Build Instructions

  1. Clone this repo.
  
  2. Make a build directory: `mkdir build && cd build`
  
  3. Compile: `cmake .. && make` 
    * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
    
  4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
   
      - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt  ../data/output1.txt`


## Go trhough project rubic 

### compile without errors 

Code compiled without errors with cmake and make.


### Accuacy 
run `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt  
../data/output1.txt` to get the first data accuacy
Accuracy - RMSE:
0.0651649
0.0605378
0.54319
0.544191

which is within the criteria:

[0.08, 0.08, 0.60, 0.60].


run `./ExtendedKF ../data/sample-laser-radar-measurement-data-2.txt  
../data/output2.txt` to get the second data accuacy

Accuracy - RMSE:
0.185496
0.190302
0.476754
0.804469

which is within the criteria:

[0.20, 0.20, .50, .85].


### Follows the Correct Algorithm

`Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
Your Kalman Filter algorithm handles the first measurements appropriately.
Your Kalman Filter algorithm first predicts then updates.
Your Kalman Filter can handle radar and lidar measurements.`

The algorithm treats the first measurement as initlization of the states (x). Normal steps of kalman filter calcuation are applied from the second measurement.  

The algorithm predicted through calling the prediction function in the instance of `ekf_` defined from the class   `KalmanFilter`;  the algorithm then updated the states and covariance matrix through calling the update functions in the instance of `ekf_` defined from the class `KalmanFilter`. The update functions are different for laser and radar data. When laser data was dected, the update function is a standard update function of Kalman filter; when radar data was dected, the update function used extended kalman filter method. In the extended kalman filter, the prediction of current measurement is calcated through non-linear measurement functions h(x); the Jacobian matrix was obtained from h(x) and used for calculating the kalman gain K.    

     
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



### Code Efficiency

Your algorithm should avoid unnecessary calculations.
