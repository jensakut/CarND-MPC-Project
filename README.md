# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
# Overview

This project drives a simulated car using modelpredictive control. The simulator may be [downloaded here](https://github.com/udacity/self-driving-car-sim/releases). 
The simulated car can steer, brake, and accelerate and has to go around a lakeside track without leaving the track. 
The MPC uses uWebSockets to communicate with the control. 
The student has to implement the MPC algorithm, but the [structure of the project is provided by Udacity](https://github.com/udacity/CarND-MPC-Project). 

The result is shown in the following animation. The car is optimized for save speeding.  

![Alt Text](result.gif) 



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

The model-predicted controller uses states, which numerically calculate into the future. This prediction is compared against a desired state which, in this case, is a given trajectory and a static speed. 
A cost function and an optimizer serve to evaluate the fitness of the predicted states. By varying inputs, the optimizer approximates the best input. 
Only the first computed input is used, then new inputs and a new trajectory is computed. 

* main.cpp handles the communication with the simulator, plotting and preparatioin of the data to be send to the mpc class
* MPC.cpp consists of an eval routine for determining the cost of a trajectory and the mpc class itself. 

## Implementation

### Cost function

The cost function consists of the crosstrack-error, the angle error, the speed error, cost for the inputs as well as their continuity. 
The continous steering is the most important function, because it requires a steady line at all times. The line tries to cut the corner, which is 
desirable for safe high speed driving. A highly weighted crosstrack-error deteriorates the driving. This leads to a solver choosing a sharp return to the desired lane, but accepts oscillating behaviour.
Therefore continous steering and crosstrack-error need to be balanced. The solutionin this dilemma is adding weights to the error in steering angle. 
The speed deviation is needed so the car doesn't optimize the error by stopping. 
A continous speed input prevents oscillating inputs and resulting discomfort. 

```c++
	double cte_fac = 1; 
	double epsi_fac = 2; 
	double v_fac = 0.2; 
	double steering_fac = 1;
	double accel_fac = 1;
	double steadysteering_fac = 1500; 
	double steadyaccel_fac = 10; 
	
	for (size_t t = 0; t < N; t++) {
		//add cost for crosstrack-error
		fg[0] += cte_fac *CppAD::pow(vars[cte_start  + t],2);
		// add cost for error in direction
		fg[0] += epsi_fac*CppAD::pow(vars[epsi_start + t],2);
		// add cost for speed deviation
		fg[0] += v_fac   *CppAD::pow(vars[v_start + t]-ref_v,2);
	}
	for (size_t t = 0; t < N-1; t++) {
		// add cost for steering 
		fg[0] += steering_fac *CppAD::pow(vars[delta_start + t],2);
		// add cost for accelerating/braking
		fg[0] += accel_fac    *CppAD::pow(vars[a_start     + t],2);
	}

	// Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += steadysteering_fac*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += steadyaccel_fac*CppAD::pow(vars[a_start + t + 1]     - vars[a_start     + t], 2);
    }
```

### Stepsize and number of steps 

The stepsize should be as short as possible given the input delay. A rough stepsize leads to a rough corner approximation which deteriorates stability. 
With a fixed stepsize, the car has issues approximating the correct line at very low speeds. A stepsize of 0.04 is unstable, whereas 0.05 is stable. A stepsize of 200 ms
crashes the car because the resolution is too low. 
A polynomial approximation gains stability both from a high stepcount and a low order. Therefore, the lowest stepsize possible is used. 
The length of the prediction needs to be 8 to function properly, anything below can't approximate well enough. On the other hand, a stepcount of 30 with a resolution of 0.05 s
leads to a crash, because the polynomial approximates the current and the next corner. This leads to a complicated optimization problem which needs a lot of computational effort. 
With the stepcound of 25 and 0.05 s resolution, the car can follow the trajectory smoothly. 
Still, the additional information given by the waypoints in the distance isn't useful for driving. 
To not include distant waypoints into the cost function, a stepcount of 15 (0.75 s) gives a reasonable prediction distance for the given course. 
The shape of the corner further ahead should be incorporated in the pathplanning module. 

```c++
const int N =15;
const double dt = 0.05;
double ref_v = 65; 
```

### Model

The model consists of six states: x-coordinate, y-coordinate, angle psi, velocity v, crosstrack-error cte and error in angle epsi. 
Steering and acceleration serve as an input. They propagate via velocity and angle into the positions x and y. 
The error in position is linearized and the angle error uses the experimental factor Lf. 

```c++
    x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```


### Model constraints 

The state at time t=0 is the current state. In the following code, the states are computed using the model. The inputs may be optimized
within the following boundaries. 
The previous input is set constant within the predefined input delay. Therefore, the model propagates this delay into the future. 
The controller sets the input according to the action right after the delay. This algorithm incorporates the input delay into the model. 
This is a vital part to safely reach high speeds! 
The following input states may be anything within the simulators boundaries. 

```c++
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
```


## Next steps

The car is still wobbly after cutting a fast corner, there is a reluctance to return to the center. Less steering continuity might improve this. 

In real life, the speed should be dynamic. It should be a minimum of speed limit and cornering angle of a longer prediction into the future. Right now, a car would drive like a maniac, breaking at the last possible moment. 
While computing the waypoints themselves, a target speed should be computed. 
It would be interesting to use an optimizer to search for better cost parameters by using a representative measurement set. 

The car doesn't start stable, because the prediction uses an almost-zero speed. At zero speed, a minimum speed for the model would help. 

Once the car is off trajectory, the algorithm doesn't work anymore. Therefore, either the path planning or the control system has to be improved for these unusual states. 