+++
title = "About me"
description = "A about page of ..."
template = "prose.html"

url = "dla-coumentation/dla"
[extra]
lang = 'en'
math = true
mermaid = false
copy = true
comment = false
toc = true
+++

# Off-grid Diffusion Limited Aggregation on Memory and a Computation Constrained Microcontroller

As part of our [ECE 4760](https://ece4760.github.io/) final project, we created a
cyclic [Diffusion Limited Aggregation](https://en.wikipedia.org/wiki/Diffusion-limited_aggregation)
simulator that can be controlled via handmotions.

The following details the design of our simulator, the testing we performed on our system,
how our system performed, and some takeaways for the future.
and

## Background

DLA models aggregation of particles whose primary motion is [Brownian](https://en.wikipedia.org/wiki/Brownian_motion). Particles undergoing brownian motion can be modeled by moving an amount
that is described by a [normal distribution]
whose variance is proportional to time elapsed.

IMUs measure the acceleration and rotational velocity they experience
With the help of some [trigonometry and filtering](https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html#Accelerometer-vs.-Gyroscope), it is possible
to determine the orientation of an IMU.

With this in mind, we set off to build a motion-controlled DLA simulator.

## Introduction

We utilized a PICO RP 2040 to simulate brownian motion of particles along with parameterized
aggregation characteristics. In parallel, the same RP 2040 utilized an MPU6050 IMU to modify the behavior
of the particles being simulated. In particular, the IMU could modify the mean and variance of
the normal distribution used to model particles' behavior. Concretely, this meant that we could bias particles
to, on average, move in a certain direction, as well as control the simulated speed at which the particles were moving.

<!-- TODO: add a gif of DLA -->


## Design

### Hardware

Some of the following is largely taken from a previous report we have written for this
class (specifically, [lab 3](https://vanhunteradams.com/Pico/Helicopter/Helicopter.html)). It is included here for completeness. 

The heart of our hardware system is a Raspberry Pi Pico, which features the RP2040 microcontroller.
We implemented the circuitry for this lab using a breadboard, shown in Fig. TODO: add figure of breadbvoard
Similar to previous labs, our microcontroller communicates through UART to interface with a PuTTY terminal
serial interface and utilizes the Pico's PIO state machines (see [chapter 3](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)) to implement [VGA drivers](https://vanhunteradams.com/Pico/VGA/VGA.html) that allow us to visualize our simulation.
As shown in Fig.TODO: change fig number, the UART connection consists of two data data wires (RX for receiving data and TX for transmitting data) and
a ground wire to the USB-A port. The VGA connection consists of TODO: fix this and explain voltage divider
<!-- three RGB lines with $330 \Omega$ voltage dividers and two digital synchronization signals (VSYNC and HSYNC). -->

The MPU6050 is an IMU that communicates via I2C. The I2C communication protocol consists of only two lines between a controller and peripheral device. 
The SCL is the clock line, and the SDA is the data line. The MPU6050 also receives 3.3V power from the RP2040. Communication between a controller and peripheral device generally consists of 
6 steps. First, the RP2040 initiates a signal to start communicating by pulling the SDA line low while the SCL line stays high. Next, the RP2040 transmits the address of the peripheral it wants to 
communicate with, which in our case is the address of the MPU6050. The address consists of 7 bits of data. Then, the RP2040 transmits a read write bit on the 8th bit. On the 9th clock bit, the MPU6050 will pull the SDA 
line low to acknowledge it has understood the request. Afterwards, a series of data frames, each consisting of 8 data bits and 1 acknowledge bit, are transmitted, initiated from either the RP2040 or the MPU6050. 
Finally, after all data frames are complete, the RP2040 will generate a stop condition. 



TODO: Angela? Will?
Can one of you work on the voltage divider here? We also need to get rid of motor stuff https://drive.google.com/file/d/144f4phreXZ6joDq6fZQi09b3GmID1vCD/view?usp=drive_link



### Software

The software we built for our simulator can be broken into a number of components.
We will describe each component as it stands alone, and then describe how these components integrate together.

#### Random number generator

Simulating brownian motion requires drawing from a [normal distribution]. Most modern languages
have implementations of functions that do this built in to their [standard libraries](https://cplusplus.com/reference/random/normal_distribution/).

C, however, lacks such functionality.

What C does offer is a [`rand()` function](https://en.cppreference.com/w/c/numeric/random/rand) that returns a value sampled from a uniform distribution between 0 and some constant `RAND_MAX`. We use this uniform distribution to approximate
a normal distribution by drawing from `rand()` multiple times and updating a particle based on that value.
Over many time steps, the motion of a given particle will approximate a normal distribution as a result of the .





## References

TODO: Do we need a references section? ask hunter


{{ figure(src="google.png", alt="alt text", caption="This is here to show how to use images in source code") }}

[normal distribution]: https://en.wikipedia.org/wiki/Normal_distribution