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


## Follows the Correct Algorithm

Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

Your Kalman Filter algorithm handles the first measurements appropriately.

Your Kalman Filter algorithm first predicts then updates.

Your Kalman Filter can handle radar and lidar measurements.

## Code Efficiency

Your algorithm should avoid unnecessary calculations.
