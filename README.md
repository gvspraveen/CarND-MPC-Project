# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Description of Implementation

In this repository I implemented a Model Predictive Controller (MPC) to drive car around
track in udacity provided simulator. Here, I will explain my implementation details following the guidelines
from [rubric](https://review.udacity.com/#!/rubrics/896/view)

### Model Description
In this repository I used Kinematic model described in lecture videos. Model takes into account position of the car,
heading direciton, velocity, cross track error and orientation error. Dynamic features like friction etc are not considered.

Model takes into account current state and computes actuataions (steering and throttle). At the next time step it
again takes state and actuations issued in previous state. Actuations from previous state help determine predicted
state at current step. Model progressively issues new acutaitons based on error.

Following equations describe the vehicle model mathematically.

- `x, y` => vehicle coordinates
- `psi` => heading of the car
- `v` => velocity of the car
- `cte` => cross track error
- `epsi` => orientation error
- `Lf` => Distance between center of car to the front of car
- `dt` => time delta
- `delta` => throttle/accleration

```
 x_t1 = x_t0 + v_t0 * cos(psi_t0) * dt
 y_t1 = y_t0 + v_t0 * sin(psi_t0) * dt
 psi_t1 = psi_t0 + v_t0 / Lf * delta_t0 * dt
 v_t1 = v_t0 + a_t0 * dt
 cte_t1 = f(x_t0) - y_t0 + v_t0 * sin(epsi_t0) * dt
 epsi_t1 = psi_t0 - psides_t0 + v_t0 * delta_t0 / Lf * dt

```

### Reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values
My final values chosen are `N=12` and `dt=0.05`.  I tried various combinations for N ranging from 8-14 and dt from 0.1 to 0.01. I found this combination
to be most optimal for driving the car around the track at 40-45mph speed. Lower time horizons (t=N*dt) tend to run the car off track while higher values
cause too many oscillations. Also I tuned ref velocity to be around 40-45. At higher speeds on my computer, whatever values of N and dt I chose, tend to either cause
too many oscillations of take care off the track

### Polynomial Fitting and MPC Preprocessing

Simulator already gives us way points. However, I need to transform them in to vehicle coordinate system.
This means that the position of the vehicle state provided to solver has (0,0) as coordinates and heading as 0. Equations for transformation can be simply
expressed as:

```
dx = x_waypoint - px;
dy = y_waypoint - py;
x_waypoint_vehicle_space = dx * cos(psi) + dy * sin(psi);
y_waypoint_vehicle_space = - dx * sin(psi) + dy * cos(psi);

```

### Model Predictive Control with Latency

As required by project, this code implements Model Predictive Control that handles a 100 millisecond latency.

This was achieved by taking into consideration latency while setting constraints. During constraint setting, we penalize
the difference between predicted state and actual state. Predicted state at time step t1 is computed by applying kinematic equations on state at time step t0.
However, since actuations are exectued by car with latency of 100ms, we need to be careful in selecting acceleration and throttle at time t0 for prediction purpose.
We need to instead select actuations from t0-100ms.

In case of my parameters, since `dt = 0.05` (=50ms), that is two time steps behind t0. I use this in the code, in `MCP.cpp` @line#105.



--------------------------------------------------------------------------------------------------

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
