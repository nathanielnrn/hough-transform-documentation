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
class (specifically, [lab 3]). It is included here for completeness. 

The heart of our hardware system is a Raspberry Pi Pico, which features the RP2040 microcontroller.
We implemented the circuitry for this lab using a breadboard, shown in Fig. TODO: add figure of breadbvoard
Similar to previous labs, our microcontroller communicates through UART to interface with a PuTTY terminal
serial interface and utilizes the Pico's PIO state machines (see [chapter 3](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)) to implement [VGA drivers](https://vanhunteradams.com/Pico/VGA/VGA.html) that allow us to visualize our simulation.
As shown in Fig.TODO: change fig number, add image, the UART connection consists of two data data wires (RX for receiving data and TX for transmitting data) and
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
Over many time steps, the motion of a given particle will approximate a normal distribution as a result of the [Central Limit Theorem](https://en.wikipedia.org/wiki/Central_limit_theorem). As our particles are updated 30 times a second, and the sampling distribution tends to normal over time,
we can get a good approximation of brownian motian with this uniform distribution.

<!-- TODO: Move this to conclusion/bugs section maybe? -->
Before settling on the approach described above, we attempted a few ways
to sample from a normal distribution for every particle, once a frame.
One approach involved summing over multiple calls to `rand()` in order to approximate
a normal distribution within a single frame. Because this involved 10s of function calls per particle,
it severely limited the speed of our due to the increased computation complexity.
Another approach involved [sampling a random bit](https://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_random/index_random.html) using the RP 2040's ring oscillator clock (ROSC).
With these random bits it is possible to generate a wide variety of distributions, including
a normal one. Furthermore, it is possible to perform many of the tasks required
to generate a normal distribution by utilizing the RP 2040's DMA channels, at little to no
cost to the CPU.

While the second approach in particular is attractive due to the minimal cpu overhead,
user testing found that the simple approach that we went with,
generating a normal distribution "over time" simulated well and was both nice
to interact with via motion controls, and behaved inline with other DLA simulations.

#### Particle State and Collision Detection

Our particles consisted of `x` and `y` coordinates stored as `short`s, `color` stored as a `char`,
and a `cyclic_counter` that tracked how long a particle was aggregated for. In order
to minimize memory usage, our `color` member variable also acted as a way to determine if a particle
was aggregated or not. Particles that were not aggregated had a color of either `1` (dimmest possible green)
or `0` (black), while particles that were aggregated had colors ranging from `2-15`, (increasingly bright shades of green). 
All particles were stored in an array.

Collision detection was implemented by with the help of fixed point precision types, with 16 decimal places, a [alpha max beta min](https://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm)
square root approximation algorithm, and our pixel backing array.

Our collision detector calculates the distance between a particle's current location
and it's new desired location using the alpha max beta min algorithm.
It then uses this distance to determine an increment, which can be thought of as a vector of unit length
in the direction of the particles updated location. With the help of this "increment vector", each pixel in between
the particle's current location and new location is checked to see if it is touching existing aggregate or not.
If a pixel is determined to
be touching our aggregate, the particle is moved to that pixel (falling short of the initial update location)
and it is mutated to be part of the collective aggregate (by changing its color to bright green).



#### Touching Aggregate Detection
Recall that our aggregate consists of various shades of green (represented as `color` values from `2-15`). A pixel is deemed to be touching aggregate if the sum of the colors of the 8 pixels surrounding it surpasses some threshold. This threshold can be tuned to change the emergent behavior of aggregation, but was often left at `15` as this often produced interesting results.
As an example, with a threshold of `15` a pixel would be deemed to be touching aggregate if it neighbored at least a single
"bright green" pixel, or at least 2 pixels with color values of `8` (theoretically, half as bright as a "bright green"), and so on.

#### Angle detection

The following is adapted from our [lab 3] report. And is included here for completeness:

Our raw MPU6050 IMU measurements were received via I2C, where specific registers
were read, corresponding with specific measurements of the IMU. While we initally planned to utilize the
IMU's raw gyroscope measurements around the x-axis and y-axis to compute rotational
deltas, we found that just using accelerometer data proved accurate enough for responsive use.
Getting rid of the gryscopic factor would reduce the computational complexity of our
simulation without affecting it's quality, so we opted to just use the accelerometer to determine our angle

Our raw accelerometer data was used to compute the angle of our lever based on
an inverse tan function. See Fig \ref{fig:angles}. The raw data from our accelerometer
was low passed, as noise in the raw data is amplified through
the inverse tan function. See the next section for more information.

Figure 1 shows how accelerometer data can be used to calculate an angle of an IMU.
In our case, we were interested in measuring rotaiton around the x and y axes.

{{ figure(src="angles.png", alt="A lever and ", caption="Figure 1: An image showing how acceleration data can be used to calculate an angle. Taken from the <a href='https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html'>course website</a>.") }}


#### Low pass
The following is adapted from  our [lab 3] report and is included for completeness:

A software [low-pass](https://vanhunteradams.com/Pico/ReactionWheel/Digital_Lowpass_Filters.html)
filter was used on our raw accelerometer data.

This software filter essentially averages our readings in relation to our current
data over multiple periods of time, smoothing out any high frequencies. We applied
low pass filters to our accelerometer data because the effects of noise would be greatly
amplified through an inverse tan function, leading to very noisy angle values.


#### Acceleration Variance Detection
The variance of the acceleration in the z axis our IMU underwent over a period of 0.66 seconds was tracked.
This was accomplished by maintaining a rolling average and storing the previous 20 z-acceleration values
obtained from our IMU and calculating the variance of our Z acceleration:
$$\text{Var}(Z) = \mathbb{E}[(Z - \mathbb{E}[Z])^2]$$

The variance calculaiton was performed naively (iterating over all values in our array), but proved fast
enough to be computed in the span of a single frame.

#### Particle Movement

Our particle movement was implemented by moving a particle in the x and y directions
an amount obtained by sampling from a uniform distribution. 
The `max_speed` variable is dependent on the variance of our z-acceleration. It describes the magnitude
of the most a particle could move in a direction. So if `max_speed` was 6, a particle could never move more than
`6` or `-6` in either direction.

The mean of this distribution depended on the angle of our IMU. Rotation along the y-axis of the IMU (pointing towards the screen) corresponded with the horizontal movement of our particles, and likewise for the x-axis and vertical movement.
The mean of our distributions moved stepwise linearly based on the ratio of the angle of our IMU with 90.
Meaning rotation around an axis of 90 degrees had a (absolute) mean of `(max_speed - 1) / 2`, while a rotation of 0 degrees (i.e the IMU was flat) had a mean of 0.

You may be asking why we set `(max_speed - 1) / 2` and not `max_speed / 2` when our IMU was fully rotated? This is because if we had set the mean to be exactly `max_speed / 2` our particles would have lost the brownian component of their motion.
For this reason we enforced the ranges of our uniform distribution to never be higher than `-1` and never be lower than `1`
(depending on the direciton of tilt). This slightly changes the mean of our distribution.

#### Simulation/Animation
30 times a second our simulation/animation [protothread](https://en.wikipedia.org/wiki/Protothread) was awoken. This thread
was responsible for polling our IMU, performing collision detection, updating particle locations and colors, and finally
writing to our pixel backing-array. While responsible for many tasks, in practice this prothread
largely consists of a a bunch of function calls in a while loop. Some of these functions
are guarded by flags that let us turn features (such as variable speed, tilt bias) on and off.


#### Serial
Part of our simulation interfaces via [UART](https://vanhunteradams.com/Protocols/UART/UART.html) to
a PuTTY terminal to allow us to turn feature flags on and off, and reset our simulation.
Serial runs on it's own protothread and utilizes non-blocking serial read and write functions,
obtained from Bruce Land's protothreads [modifications](https://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_protothreads/index_Protothreads.html). These non-blocking functions rely on the RP 2040s ability
to signal when UART can be written to. The thread responsible for reading/writing yields until a character
can be read/written via UART, and in this way only runs when needed. This allows for computaitonal threads
(such our simulator/animator) to run almost constantly.


#### Bringing it all together

After initializing all of our UART and IMU GPIOs, along with our PIO state machines, our serial and simulation/animation
threads are initialized.
Our serial thread is non-blocking, so we will focus on the simulation thread.

Our simulation thread intially polls our IMU and updates the parameters that describe our uniform distribution based on
our IMUs acceleration variance and tilt. Then, a 2 samples are drawn from our distribution and fed to our collision detector.
Our collision detector updates the location and color of our particle, based on the aggregate currently present.
The particle's old location is drawn over ("erasing" it) and the particle's new location is drawn.

The thread then yields for an amount of time 





## Testing

TODO: Write about testing



## Bugs of Note

We detail below some of the bugs we encountered while building our simulator, how we overcame them,
and any takeaways we have.

### IMU Polling Crashes
Initially, polling our IMU at a rate of 1,000 Hz via a repeating alarm timer
led to our simulation crashing shortly after startup.
It was unclear what the root cause of these issues were, as they persisted even when moving
our repeating alarm timer to a different core. Which goes against our intial suspicion that
our interrupts were interfering with in-flight writes/reads to our backing pixel array,
which may have eventually lead to the reading of garbage data which softlocked our program.

To resolve this, we moved our IMU polling trigger to occur within our simulation protothread.
This meant that no calculations/pixel updates could be interrupted, and guaranteed
the sequentiality of our polling `->` simulate step.

Moving our IMU polling trigger to our simulation protothread meant that our IMU was only polled
30 times a second, as opposed to 1,000. However, we found that this frequency still provided accurate enough readings
for our motion controls to feel responsive.


### IMU Polling Aggregation Behavior
In addition to crashes, enabling IMU polling changed the behavior of pixel aggregation, leading pixels
to aggregate seemingly at random at the borders of our simulation. This was eventually
resolved by adding better bounds logic to our touching-aggregate function. However it remains
unclear why poling the IMU changed our collision detection logic.


### Aggravating Angle Assessment

TODO: Talk about how too steep of an angle looses information 

### Restrictive Rounding
TODO: Talk about how truncating fixes gave us biased particle movement.




{{ figure(src="google.png", alt="alt text", caption="This is here to show how to use images in source code") }}

[normal distribution]: https://en.wikipedia.org/wiki/Normal_distribution
[lab 3]: https://vanhunteradams.com/Pico/Helicopter/Helicopter.html