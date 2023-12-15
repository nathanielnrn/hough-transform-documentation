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

## Project Introduction

As part of our [ECE 4760](https://ece4760.github.io/) final project, we created a
cyclic [Diffusion Limited Aggregation](https://en.wikipedia.org/wiki/Diffusion-limited_aggregation) (DLA) 
simulator that can be controlled via handmotions.

DLA is the process that simulates particles undergoing Brownian motion that form clusters and aggregate upon collisions. Our inspiration for this project comes from both the beautiful shapes that DLA creates and the natural processes that DLA models. The visual aesthetics of DLA bridge the gap between art and nature, allowing us to notice and appreciate otherwise common natural processes. DLA also has a multitude of scientific applications, such as modeling snowflakes, crystals, or chemical reactions. Further, we wanted users to be able to interact with characteristics of the DLA algorithm to observe in real-time how human movement can enhance these natural processes. 

We implemented two versions of DLA on a RP2040 microcontroller: basic off-lattice DLA, where particles stick infinitely upon aggregation, and cyclic DLA, where particles decay upon aggregation. The simulation is then displayed, real-time, on a VGA screen. The user is able to interact with the simulation using hand motions by wearing a glove with an IMU attached. Shaking and tilting their hands in various directions will cause the particles to aggregate faster or move with a bias in certain directions. To make the project accessible, we used a high-contrast color scheme for the VGA display.

The following details the design of our simulator, the testing we performed on our system,
how our system performed, and some takeaways for the future.
and

## High Level Design

DLA models aggregation of particles whose primary motion is [Brownian](https://en.wikipedia.org/wiki/Brownian_motion). Particles undergoing brownian motion can be modeled by moving an amount
that is described by a [normal distribution]
whose variance is proportional to time elapsed.

In particular, linear Brownian motion is defined (see [page 21](https://www.stat.berkeley.edu/~aldous/205B/bmbook.pdf)) as motion such that for a timestep $h$
the movement of a particle descibed by $B(t+h) - B(t)$ are normally distributed with mean 0 and standard deviation $h.$
More explicitly this increment $X$ must be obtained with probability
$$P(X=x)\frac{1} = {h \sqrt{2\pi}}e^{-\frac{1}{2}(\frac{x-0}{h})^2}.$$ 
We note that for our purposes $h$ is arbitrary, and dependent on our method of simulation and random number generation.

IMUs measure the acceleration and rotational velocity they experience
With the help of some [trigonometry and filtering](https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html#Accelerometer-vs.-Gyroscope), it is possible
to determine the orientation of an IMU.

With this in mind, we set off to build a motion-controlled DLA simulator.

On a high level, our design involves 3 blocks: 1) the RP2040 microcontroller 2) the IMU 3) the VGA display. The IMU measures the user-affected data, the VGA displays the simulation, and the MCU performs both the simulation and interfaces witht he IMU and VGA. We utilized a PICO RP 2040 to simulate brownian motion of particles along with parameterized
aggregation characteristics. The same RP 2040 utilized an MPU6050 IMU to modify the behavior
of the particles being simulated. In particular, the IMU could modify the mean and variance of
the normal distribution used to model particles' behavior. Concretely, this meant that we could bias particles
to, on average, move in a certain direction, as well as control the simulated speed at which the particles were moving. In parallel, the MCU updates the VGA display with the particle positions and colors. 

We encountered a few hardware software tradeoffs in this project. A major tradeoff we encountered involves the polling rate of the IMU. From the software side, polling the IMU for updated measurements would result in a more real-time behavior. However, polling the IMU takes a nontrivial amount of time that takes up valuable DLA computation time. Computation time became especially important when we increased the computational intensity to perform more accurate aggregate checks. We found that increasing the complexity of the algorithm and maintaining a high IMU polling rate would cause certain particles to be modified incorrectly. Thus, to execute a more accurate software design, we decreased the polling rate of the IMU. Another significant tradeoff we dealt with involved the color bitwidth for the VGA screen. We initially used 4 bit color. In our DLA algorithm, we used the color to define which stage of decay a particle was in. In order to achieve more fine-tuned decay and aggregation behavior, we considered using 8 bit color. However, we were concerned with the memory limitations, and decided to stay with 4 bit color with the loss of some software accuracy in the end. 

<!-- TODO: add a gif of DLA -->

TODO: logical structure; hardware/software tradeoffs; Discuss existing patents, copyrights, and trademarks which are relevant to your project.


## Design

### Hardware

The heart of our hardware system is a Raspberry Pi Pico, which features the RP2040 microcontroller.
We implemented the circuitry for this lab using a breadboard, shown in Fig 1.
Similar to previous labs, our microcontroller communicates through UART to interface with a PuTTY terminal
serial interface and utilizes the Pico's PIO state machines (see [chapter 3](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)) to implement [VGA drivers](https://vanhunteradams.com/Pico/VGA/VGA.html) that allow us to visualize our simulation.

{{ figure(src="breadboard.png", caption="System level diagram", width=500, height=500) }}

As shown in Fig.TODO: change fig number, add image, the UART connection consists of two data data wires (RX for receiving data and TX for transmitting data) and
a ground wire to the USB-A port. The MPU6050 is an IMU that communicates via I2C. The MPU6050 also receives 3.3V power from the RP2040.
<!-- three RGB lines with $330 \Omega$ voltage dividers and two digital synchronization signals (VSYNC and HSYNC). -->

{{ figure(src="lab4-final-dla-schematic.png", caption="System level diagram", width=500, height=500) }}

To allow for 4 bit color with a green gradient, the VGA connection utilizes a summing circuit, as seen in Fig 2. We use four resistors with the approximate form 1R, 2R, 4R, and 8R where R = 100, with limitations from the available resistors in the course lab. So, looking at the figure, if we set all four GPIOs to high, we get the brightest green, whereas if we only set the rightmost GPIO to high, we get the dimmest green.

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
Over many time steps, the motion of a given particle will approximate a normal distribution as a result of the [Central Limit Theorem](https://en.wikipedia.org/wiki/Central_limit_theorem).
As our particles are updated 30 times a second, and the sampling distribution tends to normal over time,
we can get a good approximation of brownian motian with this uniform distribution. Figure 1 shows the empircal probability distribution of displacement
of a particle over 15 timesteps (equivalent to half a second), using this "sum of uniform samples" method.
Figure 2 shows the empirical probability distribution of displacement of a particle over 15 timesteps sampling from a normal distribution at
each time step.
The distributions between each method are very similar, with our uniform distribution method being slightly biased towards the right.
We believe this is an artifact of the visualization of our as our uniform distribution has integer values, and our simulation did not
show any obvious tendency to positive values at runtime.
The takeaway from this graph is that, over time, our particles behave as if they were moving randomly with movement at each timestep being samples from a normal distribution.

{{ figure(src="nearly-uniform-distribution.png", alt="A histogram. ", caption="Figure 1: A histogram visualizing the distribution of the displacement of a particle over 15 timesteps with uniformly sampled movement. Taken from 8,000 samples. The orange line is an actual normal distribution with appropriate mean and variance.") }}

{{ figure(src="normal-sums.png", alt="A histogram. ", caption="Figure 2: A histogram visualizing the distribution of the displacement of a particle over 15 timesteps with normally sampled moved. Taken from 8,000 samples. The orange line is the same orange from Figure 1.") }}




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

{{ figure(src="angles.png", alt="A lever and acceleration vectors, along with an angle theta.", caption="Figure 1: An image showing how acceleration data can be used to calculate an angle. Taken from the <a href='https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html'>course website</a>.") }}


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

You may be asking why we set `(max_speed - 1) / 2` and not `max_speed / 2` when our IMU was fully rotated? This is because if we had set the mean to be exactly `max_speed / 2` our particles would have only ever moved in a single direction.
This makes the movement seem un-random nature and is not satisfying to interact with.
For this reason we enforced the ranges of our uniform distribution to never be higher than `-1` and never be lower than `1`
(depending on the direciton of tilt). This slightly changes the mean of our distribution.

#### Particle decay

We were interested in simualting [cyclic DLA](https://ciphrd.com/2020/07/21/cyclic-diffusion-limited-aggregation/).
To this end, our particle structs store a `cyclic_counter` member variable that counted the number of frames
a particle had been aggregated for. The more time a particle had been aggregated for, the dimmer the aggregated particle would
appear, before disappearing and returning to be a "free" particle, undergoing Brownian motion independent of the aggregate.
TODO: add gif

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
Our serial thread is non-blocking, and simply uses protothreads to output to terminal
and read in to an input buffer as needed. Some logic regarding flags is contained in this thread as well.
This thread is initialized on our second core. 

Our simulation thread is initialized on our first core. In a given frame this thread polls our IMU and updates the parameters that describe our uniform distribution based on
our IMUs acceleration variance and tilt. Then, two samples are drawn from our distribution and fed to our collision detector.
Our collision detector updates the location and color of our particle, based on the aggregate currently present (i.e. a particle
moves an amount determined by our random sampling, or collides with aggregate in the way).
The particle's old location is drawn over ("erasing" it) and the particle's new location is drawn to.

The thread then yields such that it will awaken 1/30th of a second after the current frame began. In this way we enforce a simulation and animation speed of 30 fps.




## Testing

### Hardware
TODO: Talk about voltage divider and IMU, we can say we were lucky to be familiar with IMU set up due to lab 3.


### Software

The visual nature of our graphs 

#### Angle Graphs
We used software (modified from [here](https://vanhunteradams.com/Pico/Helicopter/Display.html)) to graph the measured angle of 
our rotation around both the `x` and `y` axis. This allowed us to both examine the behavior of our angle measurements with
respect to noise and responsiveness, and qualitatively view the effects of changes we made to our measurement algorithm.
In particular, graphing proved invaluable to help us notice that changing our polling rates had down stream effects on
measured angles, and required modification to our low pass algorithm to account for these changes.

Furthermore, or graph helped us verify that our parameter changes were behaving as expected. In particular, our graph
allowed us to tell at what angle the mean of our uniform distribution shifted (which was a stepwise function). We verified
that our mean shifted at the intended angles through user testing.



#### Simulation
Having an obvious visual component to our simulation made testing for it's correctness easy.
After making changes to our algorithms,
we verified visually if our simulation behaved as expected. We compared our simulation both to other,
examples of [DLA](https://isaacshaker.github.io/DLA-Simulation/) and to previous iterations of our own work.
Examining incremental changes to our simulation algorithms helped us uncover a number of issues, described in detail below.
Among the issues we discovered was a shift in the mean of the movement of our particles from
conversion from fixed point to integer values, incorrect aggregation occurring at the
borders of our simulation, and some issues with a naive collision detection algorithm.



## Results



TODO: talk about density of particles needing tuning, maximum number of particles allowed (memory wise) (around 16k),
slow downs encountered when computaiton was too expensive
TODO: 
Any and all test data, scope traces, waveforms, etc;
speed of execution (hesitation, filcker, interactiveness, concurrency);
accuracy (numeric, music frequencies, video signal timing, etc);
how you enforced safety in the design;
usability by you and other people;


## Bugs of Note

We detail below some of the bugs we encountered while building our simulator, how we overcame them,
and any takeaways we have.

### Polling Problems
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


### Bonkers Borders
In addition to crashes, enabling IMU polling changed the behavior of pixel aggregation, leading pixels
to aggregate seemingly at random at the borders of our simulation. This was eventually
resolved by adding better bounds logic to our touching-aggregate function. However it remains
unclear why poling the IMU changed our collision detection logic.


### Aggravating Angle Assessment
During testing, we noticed that if we rotated our IMU to close to 90 degrees along the x-axis, the rotational measurements of our
y-axis would become inaccurate. The same occurred if we swapped the axes of rotation. We learned that using acceleration along axes is limited in that
at extreme angles we lose a degree of information. Consider rotating around an axis pointed in front of you (y axis pointing forward, x-axis to the left,
z-axis pointing towards the sky). Our IMU calculates
rotation around the y-axis by comparing z and x-axis accelerations. Consider that if we rotate along the x axis so that our y-axis is now pointing up
(z axis pointing towards us, x axis pointing to the left), rotation around the y-axis no longer changes the acceleration experienced in both the x-axis and z-axis, rather, both remain close to 0. This makes measurement of rotation around y very difficult and sensitive to noise.

Luckily for our project, users did not often reach rotations of 90 degrees, as doing so was quite uncomfortable with our glove. So this did
not prove an issue in practice.

**Takeaway:** It was discussed how in measuring rotation around 2 axes, we were essentially trying to model the IMU in 3 degrees of freedom.
However, we were doing this with only 2 pieces of data in each axes: z acceleration and x/y acceleration. To this end our measurement
were underdetermined, and caused the issues described above. We should aim to have fully determined systems, and if that is infeasibly be aware of the
limitations of our models


### Restrictive Rounding
When shifting the mean of our uniform distribution based on the tilt of our IMU we performed fixed point
airthmeitc for the sake of computational speed,
and then converted values to integers. Our initial system took into account the sign of our tilt,
and shifted the mean of our distribution accordingly, based on the between our current angle and 90 degrees.

Because of the way our fixed to integer conversion works, numbers were rounded differently dependent on their sign.
Namely, fixed point representations are converted to integers by right shifting `>>`. This means that integer values
are truncated, not rounded. This meant that we were taking the floor of our fixed point number.
This has an effect on the magnitude of a number agfter truncation based on its sign. Consider that `4.3` is truncated to `4`
while `-4.3` is truncated to `-5`.
This, along with the details of our initial implementation, meant that our mean was incorrectly being biased towards
negative numbers. This was apparent in our simulation as particles tending towards the top left, even when our IMU was held flat.

To fix this, we negated some fixed point numbers before truncating them and negating them again. In this way, our truncation
was "symmetric" on both positive and negative fixed point numbers.

**Takeaway:** Implementation matters! Especially when developing in C on things like microcontrollers,
we need to be very aware of the implications of various implementations, as they can have compounding downstream effects.
We were fortunate enough to figure out what was going on fairly quickly, but this may had not been obvious
if we had been less aware of different number representations and how we convert between them.




## Conclusion





## Appendix A
The group approves this report for inclusion on the course website.

"The group approves the video for inclusion on the course youtube channel."

## Appendix

### Project Video
The group approves the video for inclusion on the course youtube channel.

### Hardware

TODO: Add schematics of hardware

### Tasks
The work was largely evenly split among team members, with slight focuses on certain.
The following is a non-exhaustive list of topics focused on.

**Nathaniel:** Setup and dev environment, collision detection, voltage divider, website, and report.
**Angela:** Collision detection, voltage divider, aggregation algorithm.
**William:** Random number generation, moition-random-distribution effects, serial.

### Code

Code can be found [here](../code).


{{ webm(src="basic.webm", caption="This is here to show how to use images in source code", width=500) }}

[normal distribution]: https://en.wikipedia.org/wiki/Normal_distribution
[lab 3]: https://vanhunteradams.com/Pico/Helicopter/Helicopter.html
