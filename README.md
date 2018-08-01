# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
# Overview

This project drives a simulated car using modelpredictive control. The simulator may be [downloaded here](https://github.com/udacity/self-driving-car-sim/releases). 
The simulated car can steer, brake, and accelerate and has to go around a lakeside track without leaving the track. 
The MPC uses uWebSockets to communicate with the control. 
The student has to implement the MPC algorithm, but the [structure of the project is provided by Udacity](https://github.com/udacity/CarND-MPC-Project). 

The 



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


# Implementation

## Overview
The modelpredicted controller uses states, which get propagated in the future. This prediction is compared against a desired state which is a given trajectory in this example. 
A cost function and an optimizer is used to evaluate the fitness of the predicted states. By varying inputs, the optimizer chooses the best input. 
Only the first computed input is used, then a new trajectory gets computed. 

* main.cpp handles the communication with the simulator and prepares the data to be send to the mpc class, which it also plots
* MPC.cpp consists of the eval routine for determining the cost of a trajectory and the mpc class. 

## Implementation

### Cost function

The cost function consists of the crosstrack-error, the angle error, the speed error, cost for steering and acceleration and continous input for steering and acceleration.
The continous steering is the most important function because it requires a steady line at all times. The line tries to cut the corner, which is 
desirable for safe high speed driving. A highly weighted crosstrack-error deteriorates the driving. The solver chooses a sharp return to the desired lane, but accepts oscillating behaviour.
Therefore continous steering and crosstrack-error need to be balanced. Weighting the error in steering angle leads to a middle-lane driving without oscillations. 
The speed deviation is needed so the car doesn't stop. Maybe the desired speed should depend on the modeled lateral acceleration of a corner rather than being static. 
A continous speed input prevents oscillating inputs and resulting discomfort. 

```c++
	for (size_t t = 0; t < N; t++) {
		//add cost for crosstrack-error
		fg[0] += CppAD::pow(vars[cte_start  + t],2);
		// add cost for error in direction
		fg[0] += 2*CppAD::pow(vars[epsi_start + t],2);
		// add cost for speed deviation
		fg[0] += CppAD::pow(vars[v_start + t]-ref_v,2)/5;
	}
	for (size_t t = 0; t < N-1; t++) {
		// add cost for steering 
		fg[0] += CppAD::pow(vars[delta_start + t],2);
		// add cost for accelerating/braking
		fg[0] += CppAD::pow(vars[a_start     + t],2);
	}

	// Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] +=1500*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] +=  10*CppAD::pow(vars[a_start + t + 1]     - vars[a_start     + t], 2);
    }
```

If the time predicted is too long, the algorithm and the optimizer gets unstable. Increasing the number of steps is only possible in 
conjunction with a highly weighted steering continouity. The stepsize used is 0.05 s with 15 steps predicted (0.75 s prediction). 

### Model

The model consists of six states: x-coordinate, y-coordinate, angle psi, velocity v, crosstrack-error cte and error in angle epsi. 
Steering and acceleration serve as an input. They propagate via velocity and angle into the positions x and y. 
The error in position is linearized and the angle error uses the experimental factor Lf. 

´´´
    x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
	
´´´ 


### Model constraints 

The state at time t=0 is the current state. In the following code, the states are computed using the model. The inputs may be optimized
within the following boundaries. 
Within the predefined input delay, the previous state is set constant. This is a vital part to reach high speeds! 
The following input states may be anything within the simulators boundaries. 

´´´c++
  // as long as there is delay keep previous value
  for (size_t i = delta_start; i<delta_start + latency; ++i){
	  vars_lowerbound[i] = delta_hist;
	  vars_upperbound[i] = delta_hist; 
  }
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (size_t i = delta_start+latency; i < a_start; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  for (size_t i = a_start; i < a_start + latency; ++i) {
	vars_lowerbound[i] = a_hist;
	vars_upperbound[i] = a_hist;
  }
  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (size_t i = a_start+latency; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
´´´ 