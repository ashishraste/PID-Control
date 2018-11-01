# PID-Control

PID control for a simulated autonomous car.

---

## Introduction

This project is part of [Udacity's Self-Driving Car Nanodegree program](https://www.udacity.com/drive). The purpose of this project is to build
a PID controller and tune its hyper-parameters. The controller is tested on an
Udacity simulator, which provides cross-track error (CTE), speed and steering
angle data via a websocket connection. The car drives autonomously around the
simulator track by using steering and throttle commands from the PID-controller.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator: You can download the Udacity simulator [here](https://github.com/udacity/self-driving-car-sim/releases).

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Build and Run Instructions

1. Clone this repo.
2. Scripts provided in this repo could be used in the following order to build
and run the PID controller.

```bash
./clean.sh  # Cleans any existing build.
./build.sh  # Builds the PID controller.
./run.sh    # Runs the PID controller; connects to the simulator via uWebSockets.
```

## Discussion

This section discusses the effect of PID parameters and how they are tuned to
reliably drive the car autonomously.

### PID parameters

The proportional (P) component, as a starting point, has a huge influence on the
steering and throttle commands. As the name suggests, it has a proportional
effect to the cross-track error received from the simulator i.e. the more the
cross-track error the harder (inversely proportional) the P-component tries to
get back the car on to the middle of the track.

The derivative (D) component is quite important to avoid the car from
the overshooting effects of the P-component. It helps the car to smoothly
approach a lower cross-track error margin.

The integral (I) component to naked-eyes to have little direct effect on the
driving behaviour of the car, but is crucial to be tuned in order to account
for any steering-drift bias that the car may have. By tuning the I-component, we
observe some CTE-reduction around the corners.

Following videos show the behaviour of the car when one of D or I components
are missing.

* Controller without D-component:  [no_derivative.mp4](output_videos/no_derivative.mp4)
* Controller without I-component: [no-integral.mp4](output_videos/no_integral.mp4)

### Hyperparameter tuning

Twiddle algorithm is applied to tune the PID components i.e. the hyperparameters.
Before enabling Twiddle, it was required to set the hyperparameters manually to
make the car stay on the track. Below PID values for steering and throttle were
a good starting point before running Twiddle

* For Steering – P: 0.13, I: 0.0002, D: 3.0
* For Throttle – P: 0.3, I: 0, D:0.02

Twiddle is run continuously as car drives around the track; with 2000 evaluation
steps and 100 steps for the parameters to settle. In total, about 52000 steps
were allowed to run which produced a best CTE of 1130.49 with the following
parameters. Output log from Twiddle on CTE could be found here:
[twiddle-log](twiddle_log.txt).

* Optimised Steering parameters – P: 0.170041, I: 0.000268225, D: 3.8937
* Optimised Throttle parameters – P: 0.405483, I: 0, D: 0.025958;

Attached video of the car driving around the track with the above optimised
parameters: [optimised_pid.mp4](output_videos/optimised_pid.mp4).
