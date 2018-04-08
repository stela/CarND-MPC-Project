# Project Overview

This project makes use of Model Predictive Control - MPC to keep a car on the track. "Ideal" waypoints are given (in a "real world" setting this would probably be provided by some existing path planning component). A kinematic model of the car is used to predict which controls (steering + gas/break) are needed to keep the car on the planned path, with some extra constraints to keep up the speed and avoid car-sickness, besides following the provided path.

# Rubric criteria

## Your Code Should Compile

See continuous travis-ci builds: [![Build Status](https://travis-ci.org/stela/CarND-MPC-Project.svg?branch=master)](https://travis-ci.org/stela/CarND-MPC-Project)

These show that both g++ and clang on Linux can build the project besides macOS which I used for development.

## The Model

I'm using the kinematic vehicle model described in the course.

The state consist of the car's current x and y coordinates, the (forward) velocity and the direction, ψ (psi). In order to optimize the previous "base state", errors are also calculated and added to the state, namely the cross-track-error (cte) and direction error (eψ or epsi). You can see the state encoding e.g. in [MPC::Solve()](src/MPC.cpp#L164-L170).

The actuators of the car consist of steering angle and gas throttle/break. The throttle and break are merged into a single actuator, where the value -1 means full break and +1 means full throttle. They are returned by MPC::Solve() [here](src/MPC.cpp#L262-L269).

The update equations as given in the "Global Kinematic Model" chapter are:

x(t+1) = x(t) + v(t) ∗ cos(ψ(t)) ∗ dt

y(t+1) = y(t) + v(t) * sin(ψ(t)) * dt

ψ(t+1) = ψ(t) + v(t) / Lf * δ ∗ dt

v(t+1) = v(t) + a(t) * dt

These equations can be found in [MPC::Solve()](src/MPC.cpp#L122-L125).

## Timestamp Length and Elapsed Duration

Since humans are also able to drive a car and have reaction delays of about 100-200ms, I thought a computer should not need more than that either, so assigned dt = 0.1 second. As the latency which needs to be handled is 100ms which is then a single timestep, this seemed to be a convenient setting.

Since the road in this assignment only has relatively gentle curves and there's no need to react to pedestrians, traffic signs and such far ahead, I picked a relatively short timestamp length of one second, which means 10 steps of 100 ms. 

It turned out the above choices worked just fine (after a few bugs of mine were removed), the car was able to drive the complete track without trouble with those settings.

## Polynomial Fitting and MPC Preprocessing

The given waypoints, together with the car's coordinates implicitly, are transformed such that the car's x and y are both zero, and that the car is pointing at zero degrees, see [main.cpp](src/main.cpp#L101-L104). This means that the state's x, y and ψ components can simply be initialized to 0.

## MPC with Latency

The latency is taken into account by shifting the delta and a actuators one timestamp entry, see [FG_eval::operator()](src/MPC.cpp#L105-L108) in MPC.cpp. 

## Simulation - Drive a Lap

The car successfully drives more than a lap on the track. It stays farly well centered on the waypoints given. The speed could probably be increased quite a bit and still stay between the curbs, at the cost of driving a bit further off-center.


## Building the Code
See the [original README](README-original.md).
