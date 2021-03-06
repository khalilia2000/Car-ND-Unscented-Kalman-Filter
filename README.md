# Unscented Kalman Filter Project 
Udacity Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Code Style

It's been tried to stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Results for dataset #1

The following images present the results obtained on the first data file. The ground truth values of Px, Py and V along with the predicted values using UKF and the measurement values are shown on the images below. NIS values for radar and laser measurements are also shown in separate plots. RMSE values of **0.0702229**, **0.0792156**, **0.604636** and **0.64632** were obtained for Px, Py, Px_dot, and Py_dot.

| Position Estimates vs. Ground Truth | Velocity Estimates vs. Ground Truth | 
|:-----------------------------------:|:-----------------------------------:|  
| <img src="./results/results1.PNG" alt="Visualization of the resutls for dataset #1"> | <img src="./results/results1-v.PNG" alt="Visualization of the resutls for dataset #1"> |

| NIS Values for Laser | NIS Values for Radar | 
|:--------------------:|:--------------------:|  
| <img src="./results/results1-v-NISL.PNG" alt="Visualization of the resutls for dataset #1"> | <img src="./results/results1-v-NISR.PNG" alt="Visualization of the resutls for dataset #1"> |




## Results for dataset #2

The following images present the results obtained on the second data file. The ground truth values of Px, Py and V along with the predicted values using UKF and the measurement values are shown on the images below. NIS values for radar and laser measurements are also shown in separate plots. RMSE values of **0.197258**, **0.189033**, **0.41664** and **0.362757** were obtained for Px, Py, Px_dot, and Py_dot.

| Position Estimates vs. Ground Truth | Velocity Estimates vs. Ground Truth | 
|:-----------------------------------:|:-----------------------------------:|  
| <img src="./results/results2.PNG" alt="Visualization of the resutls for dataset #2"> | <img src="./results/results2-v.PNG" alt="Visualization of the resutls for dataset #2"> |

| NIS Values for Laser | NIS Values for Radar | 
|:--------------------:|:--------------------:|  
| <img src="./results/results2-v-NISL.PNG" alt="Visualization of the resutls for dataset #2"> | <img src="./results/results2-v-NISR.PNG" alt="Visualization of the resutls for dataset #2"> |


## Other Considerations

- Calculation of the square root of the matrix was the tricky part. In order to avoid numerical instabilities, the suggestions provided [here](https://discussions.udacity.com/t/numerical-instability-of-the-implementation/230449) were implemented.  
- The RMSE values can be brought down further, however not without sacrifising the integrity of the NIS distributions. 
