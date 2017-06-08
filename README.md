# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Reflections

In this implementation the steering is controlled by a PID controller with the following parameters optimized for a target speed of **50 mph**: `P = 0.106, I = 0.001, D = 2.400`.

If `cte` is the cross track error, and assuming a time step `dt = 1`, the steering value becomes:

	steering(t) = - P * cte(t) - I * i_error(t) - D * (cte(t) - cte(t-1))
	
To avoid the accumulation of the integral error, `i_error`, a memory loss mechanism is implemented in this way: 
	
	i_error(t) = cte(t) + 0.95 * i_error(t-1)
	
The steering is clipped to the range `[-1, 1]` using the `tanh` function to smooth the clipping in the extremes. Also, an exponential filter is used to smooth the steering in time.

The meaning of the PID gains is the following:

* The gain P builds a term proportional to the current error. It works in the present and it's the main factor to keep the car in the road and turn in the curves. A small value will make the car to respond slowly (like having a big inertia). A value too high will make the car oscillate (and potentially go out of the road) like a leaf in the wind. 

* The integral gain, I, accounts for the past error. It allows to correct for a bias in the steering. If the P gain is not strong enough to keep the car in the center, the error will accumulate and the integral term will correct it. However, this can leads to more oscillations and overshotting if the error accumulates indefinitely. 

* The derivative gain, D, accounts for the future changes trying to cancel the rate of change of the error. It's basically a damping force that compensates the oscillations, flattening the trajectory. However, a value too high will make car unable to react in time in a curve or it will introduce to many corrections making the system unstable.

In this solution, the throttle is also controlled with a mechanism equivalent to having two P-controllers connected in serie. One direct controller proportional to the speed error (the difference bettween the current and the target speeds). And another reverse controller proportional to the `cte` and a smoothed exponential to the speed that will make the car to brake harder at higher speeds:

	throttle = a * speed_error - b * abs(cte) * exp(1.1 * abs(speed)/100  - 1),

where 
	
	speed_error = target_speed - current_speed + speed_margin	

Tuning the throttle parameters `a` and `b` allows for different driving modes. From a maniac driver (high `a`, and low `b`) to a scared novice driver pushing the brake at the minimum risk (low `a`, high `b`). Somewhere in the middle allows to play with higher speeds and smooth driving. In the current implementation these are the values chosen:

* `a = 0.2`
* `b = 0.8`

All these parameters were tuned manually with an interactive input loop that allows to see the changes in real time. Fixed the throttle parameters to a convservative driving style, and fixed the `I` gain to a small value, the biggest task was to tune the P and D gains until the trajectory seemed stable. I also tried with an implementation of the twiddle algorithm, a derivative-free optimization algorithm, see [coordinate descent](https://en.wikipedia.org/wiki/Coordinate_descent), trying to minimize the `cte` plus the error on the average speed. Howevert, the convergence to a minimum seemed too slow to be apply online. The implementation is included in the repository.

When tuning the parameters, the biggest challenge was that the PID gains are dependent on the speed. Above certain limit it's not possible to keep the speed of the car constant for the obvious reason that there are curves in the circuit. A PID with adaptive or scheduled gains is possible. However, the throttle control mechanism is enough to drive at speeds below 60 mph. To drive at higher speeds, a different configuration of parameters or a more robust controller is needed.

## Demo

[![Demo 50 MPH](video.gif)](video.mp4)

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
