# Model Predictive Control Project

Self-driving Car Nanodegree Project 5/Term 2 - Implementation of a Model Predictive Control to drive a car around a track on a simulation.

The state of the car on a specifically period of time was define by the following state: 

$$
[x, y, \psi, v, cte, e\psi]
$$

For the model, the update equations used the Global Kinematic Model which is modeled by the following equations (including the formula for the next cross track and orientation error).

$$
x_{t+1} = x_{t} + v_{t} . cos(\psi_{t}) . dt
\\y_{t+1} = y_{t} + v_{t} . sin(\psi_{t}) . dt
\\\psi_{t+1} = \psi_{t} + \dfrac{v_{t}}{L_{f}} . \delta . dt
\\v_{t+1} = v_{t} + a_{t} . dt
\\cte_{t+1} = cte_{t} - y_{t} + (v_{t} . sin(e\psi_{t}) . dt) 
\\e\psi_{t+1} = \psi_{t} - \psi des_{t} + (\dfrac{v_{t}}{L_{f}} . \delta_{t} . dt)
$$

For the acuators, there was used the steering angle and acceleration defined by the following variables:

$$
[\phi, a]
$$

And the actuators were restricted between -25 and +25 degrees for the steering angle and between -1 and +1 for the acceleration.

For the timestep length (N), I chose 12 because it wasn't to many to make the process too slow and it was a sufficient number. For the elapsed duration between timesteps (dt), I chose 0.1s (100 ms) to make easier to deal with the latency (also 100 ms).

The waypoints was preprocessed suffering a rotation + translation transform from a global coordinate to the car's coordinate. Then a 3rd degree polynomial was fitted with the waypoints.

With the latency of 100 ms, I used a dt of 100 ms and used the second pair of actuators calculated by the solve function to compensate the latency. 

---

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
