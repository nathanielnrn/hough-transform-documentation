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
simulator that can be controlled via hand motions.

DLA is the process that simulates particles undergoing Brownian motion that form clusters and aggregate upon collisions. Our inspiration for this project comes from both the beautiful shapes that DLA creates and the natural processes that DLA models. The visual aesthetics of DLA bridge the gap between art and nature, allowing us to notice and appreciate otherwise common natural processes. DLA also has a multitude of scientific applications, such as modeling snowflakes, crystals, or chemical reactions. Further, we wanted users to be able to interact with characteristics of the DLA algorithm to observe in real-time how human movement can enhance these natural processes. 

We implemented two versions of DLA on a RP2040 microcontroller: basic off-lattice DLA, where particles stick infinitely upon aggregation, and cyclic DLA, where particles decay upon aggregation. The simulation is then displayed, real-time, on a VGA screen. The user is able to interact with the simulation using hand motions by wearing a glove with an IMU attached. Shaking and tilting their hands in various directions will cause the particles to aggregate faster or move with a bias in certain directions. To make the project accessible, we used a high-contrast color scheme for the VGA display.

{{ webm(src="basic.webm", caption="Basic DLA", width=500) }}

The following details the design of our simulator, the testing we performed on our system,
how our system performed, and some takeaways for the future.

## High Level Design

#### Rationale and Background Math
This project aims to show how human movement can be used to enhance dense particle models. DLA models aggregate particles whose primary motion is [Brownian](https://en.wikipedia.org/wiki/Brownian_motion). Particles undergoing brownian motion can be modeled by moving an amount
that is described by a [normal distribution]
whose variance is proportional to time elapsed.

In particular, linear Brownian motion is defined (see [page 21](https://www.stat.berkeley.edu/~aldous/205B/bmbook.pdf)) as motion such that for a timestep $h$
the movement of a particle described by $B(t+h) - B(t)$ are normally distributed with mean 0 and standard deviation $h.$
More explicitly this increment $X$ must be obtained with probability
$$P(X=x)\frac{1} = {h \sqrt{2\pi}}e^{-\frac{1}{2}(\frac{x-0}{h})^2}.$$ 
We note that for our purposes $h$ is arbitrary, and dependent on our method of simulation and random number generation.

IMUs measure the acceleration and rotational velocity they experience
With the help of some [trigonometry and filtering](https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html#Accelerometer-vs.-Gyroscope), it is possible
to determine the orientation of an IMU.

With this in mind, we set off to build a motion-controlled DLA simulator.

On a high level, our design involves 3 blocks: 1) the RP2040 microcontroller 2) the IMU 3) the VGA display. The IMU measures the user-affected data, the VGA displays the simulation, and the MCU performs both the simulation and interfaces with the IMU and VGA. We utilized a PICO RP 2040 to simulate brownian motion of particles along with parameterized
aggregation characteristics. The same RP 2040 utilized an MPU6050 IMU to modify the behavior
of the particles being simulated. In particular, the IMU could modify the mean and variance of
the normal distribution used to model particles' behavior. Concretely, this meant that we could bias particles
to, on average, move in a certain direction, as well as control the simulated speed at which the particles were moving. In parallel, the MCU updates the VGA display with the particle positions and colors. 

#### Hardware/Software Tradeoffs

We encountered a few hardware software tradeoffs in this project. A major tradeoff we encountered involves the polling rate of the IMU. From the software side, polling the IMU for updated measurements would result in a more real-time behavior. However, polling the IMU takes a nontrivial amount of time that takes up valuable DLA computation time. Computation time became especially important when we increased the computational intensity to perform more accurate aggregate checks. We found that increasing the complexity of the algorithm and maintaining a high IMU polling rate would cause certain particles to be modified incorrectly. Thus, to execute a more accurate software design, we decreased the polling rate of the IMU. Another significant tradeoff we dealt with involved the color bitwidth for the VGA screen. We initially used 4 bit color. In our DLA algorithm, we used the color to define which stage of decay a particle was in. In order to achieve more fine-tuned decay and aggregation behavior, we considered using 8 bit color. However, we were concerned with the memory limitations, and decided to stay with 4 bit color with the loss of some software accuracy in the end.


#### Existing Patents, Copyrights, and Trademarks
Regarding patents, copyrights, or trademarks, nothing is applicable to this project. We properly source all online coding resources, and this project has expanded upon an existing DLA implementation.

## Design

### Hardware

The heart of our hardware system is a Raspberry Pi Pico, which features the RP2040 microcontroller.
We implemented the circuitry for this lab using a breadboard, shown in Fig 1.
Similar to previous labs, our microcontroller communicates through UART to interface with a PuTTY terminal
serial interface and utilizes the Pico's PIO state machines (see [chapter 3](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)) to implement [VGA drivers](https://vanhunteradams.com/Pico/VGA/VGA.html) that allow us to visualize our simulation.

{{ figure(src="breadboard.png", caption="System level diagram", width=500, height=500) }}

As for the software-hardware system, the figure below visualizes how we integrated the VGA display, RP2040, IMU (MPU 6050), and Serial interface (USB-A and PuTTY terminal) together.

{{ figure(src="lab4-final-dla-schematic.png", caption="System level diagram", width=500, height=500) }}

To allow for 4 bit color with a green gradient, the VGA connection utilizes a summing circuit, as seen in the Figure above. We use four resistors with the approximate form 1R, 2R, 4R, and 8R where R = 100, with limitations from the available resistors in the course lab. So, looking at the figure, if we set all four GPIOs to high, we get the brightest green, whereas if we only set the rightmost GPIO to high, we get the dimmest green.

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
we can get a good approximation of brownian motion with this uniform distribution. Figure 1 shows the empirical probability distribution of displacement
of a particle over 15 timesteps (equivalent to half a second), using this "sum of uniform samples" method.
Figure 2 shows the empirical probability distribution of displacement of a particle over 15 timesteps sampling from a normal distribution at
each time step.
The distributions between each method are very similar, with our uniform distribution method being slightly biased towards the right.
We believe this is an artifact of the visualization of our as our uniform distribution has integer values, and our simulation did not
show any obvious tendency to positive values at runtime.
The takeaway from this graph is that, over time, our particles behave as if they were moving randomly with movement at each timestep being samples from a normal distribution.

{{ figure(src="nearly-uniform-distribution.png", alt="A histogram. ", caption="Figure 1: A histogram visualizing the distribution of the displacement of a particle over 15 timesteps with uniformly sampled movement. Taken from 8,000 samples. The orange line is an actual normal distribution with appropriate mean and variance.") }}

{{ figure(src="normal-sums.png", alt="A histogram. ", caption="Figure 2: A histogram visualizing the distribution of the displacement of a particle over 15 timesteps with normally sampled moved. Taken from 8,000 samples. The orange line is the same orange from Figure 1.") }}


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
were read, corresponding with specific measurements of the IMU. While we initially planned to utilize the
IMU's raw gyroscope measurements around the x-axis and y-axis to compute rotational
deltas, we found that just using accelerometer data proved accurate enough for responsive use.
Getting rid of the gyroscopic factor would reduce the computational complexity of our
simulation without affecting it's quality, so we opted to just use the accelerometer to determine our angle

Our raw accelerometer data was used to compute the angle of our lever based on
an inverse tan function. See Fig \ref{fig:angles}. The raw data from our accelerometer
was low passed, as noise in the raw data is amplified through
the inverse tan function. See the next section for more information.

Figure 1 shows how accelerometer data can be used to calculate an angle of an IMU.
In our case, we were interested in measuring rotation around the x and y axes.

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

The variance calculation was performed naively (iterating over all values in our array), but proved fast
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

We were interested in simulating [cyclic DLA](https://ciphrd.com/2020/07/21/cyclic-diffusion-limited-aggregation/).
To this end, our particle structs store a `cyclic_counter` member variable that counted the number of frames
a particle had been aggregated for. The more time a particle had been aggregated for, the dimmer the aggregated particle would
appear, before disappearing and returning to be a "free" particle, undergoing Brownian motion independent of the aggregate. This feature is displayed when explaining cyclic factor results.

#### Simulation/Animation
30 times a second our simulation/animation [protothread](https://en.wikipedia.org/wiki/Protothread) was awoken. This thread
was responsible for polling our IMU, performing collision detection, updating particle locations and colors, and finally
writing to our pixel backing-array. While responsible for many tasks, in practice this protothread
largely consists of a a bunch of function calls in a while loop. Some of these functions
are guarded by flags that let us turn features (such as variable speed, tilt bias) on and off.


#### Serial
Part of our simulation interfaces via [UART](https://vanhunteradams.com/Protocols/UART/UART.html) to
a PuTTY terminal to allow us to turn feature flags on and off, and reset our simulation.
Serial runs on it's own protothread and utilizes non-blocking serial read and write functions,
obtained from Bruce Land's protothreads [modifications](https://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_protothreads/index_Protothreads.html). These non-blocking functions rely on the RP 2040s ability
to signal when UART can be written to. The thread responsible for reading/writing yields until a character
can be read/written via UART, and in this way only runs when needed. This allows for computational threads
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

We used the VGA display and the IMU, both of which we are familiar with. Like usual, we had to ensure that the VGA connector was wired correctly and that the receiving monitor was functional. If the receiving monitor did not properly display our program, we knew that either the wiring was incorrect or the monitor had to be swapped out. Luckily, the IMU was already integrated from lab 3, making it easy for us to leverage its acceleration and angle capabilities.

When testing for user input, we wired up the UART connection to input keyboard statements using PuTTY. With this wired up, we could manipulate desired features. We were confident with the UART wiring given that the program always responded to our inputs.

We did have some difficulty implementing the 4 bit green gradient as it required some additional circuits knowledge. Fortunately, the required summing circuit utilized resistors available in lab and the circuitry was not complicated. That being said, we were able to verify this 4 bit green gradient by using the VGA display and determining whether particles were properly decaying from the brightest green to the dimmest green. 

### Software
Like previous labs, we utilized serial's print capabilities to allow us to examine the values of variables in real time. Beyond that, a large amount of testing was done by looking at our IMU acceleration and complementary angle graphs to determine whether the particle motion was correct. For the features that did not utilize the IMU, their effectiveness could be verified by looking at the program and comparing it to the expected result.

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

We visually tested the clusters and formations of each unique factor to guarantee the simulation would maintain uniform random motion. First we had to generate a basic DLA using no factors. Once that was complete, we tested the following factors: tilt, speed, cyclic, visibility, seed location, and reset. Because our main priority was resembling Brownian motion, we kept the particle count at around 7000-8000. This particle count was dense enough to fill our screen with clusters without risking a crash due to a computational overload.

#### Basic DLA
The basic DLA motion consists of a min speed = -2 and a max speed = 2. In order to ensure that particles resembled Brownian motion, we used the random number generator mentioned above. As shown below, the clustering motion branches out evenly from the center, indicating that the motion is truly Brownian. This motion occurs when the serial interface is either reset or no factors are activated.

{{ webm(src="basic.webm", caption="Basic DLA", width=500) }}


#### Tilt Factor
The tilt factor is hand-activated, and it is responsible for creating a slight bias in any cardinal direction (North, South, East, West) and any ordinal direction (Northeast, Southeast, Southwest, Northwest). Because a bias is introduced, we do not expect the motion influenced by the tilt factor to be Brownian. As shown below, the particles are first provided a bias to the right and then a bias to the left. A common characteristic of the tilt factor is the tendency for particles to aggregate in bursts, which is the result of introducing multiple biases on the same seed. This factor is activated through serial.

{{ webm(src="tilt.webm", caption="Hand-activated Tilt", width=500) }}

#### Speed Factor
The speed factor is hand-activated, and it is responsible for increasing the min speed and max speed bounds based on the variance of the z-acceleration. In order words, the movement of the particles will increase depending on how fast the IMU-glove is shaken. This feature was responsible for the conception of the collision detection. Before the collision detection, particles with increased speeds would cluster incorrectly, skipping over aggregate particles rather colliding with them. Now, the particles are capable of reaching min and max speeds of -10 and 10, respectively, while maintaining Brownian motion. As shown below, the particles are first moving very quickly as a response to the real-time IMU-glove movement. Then, the IMU-glove movement is stopped and the particles begin slowing down, showing how the particles movement respond to the dynamic hand motion. This factor is activated serial.

{{ webm(src="speed.webm", caption="Hand-activated Speed", width=500) }}

#### Cyclic Factor
The cyclic factor is responsible for decaying aggregate particles and randomly respawning them, allowing for a simulation with an infinite runtime. Like the other factors, it is activated through serial, but an additional integer is provided to alter the decay time. Every aggregate particle is provided an internal decay counter, and the serial integer input is responsible for hastening the rate at which the internal decay counter activates a decay function. In the past, once the particles would all aggregate, the program would have to restart. As shown below, the particles decay at a rapid rate, creating a creeping aggregate motion throughout the screen.

{{ webm(src="cyclic.webm", caption="Cyclic DLA Motion", width=500) }}

#### Visibility Factor
The visibility factor is responsible for hiding all moving particles, only highlighting the aggregate particles. The result is a cleaner clustering motion as the thousands of moving particles are not shown. As shown below, the basic DLA motion is highlighted without any background movement.

{{ webm(src="visibility.webm", caption="No Background Visibility", width=500) }}


#### Seed Location
The seed location factor is responsible for adding an aggregate particle seed in a custom location. This custom location is determined in serial based on an input x and y coordinate. The custom seed is a permanent aggregate particle, allowing for more unique patterns. It was important to ensure that the custom seed would not disrupt the Brownian aggregate formations. However, as shown below, both seeds have Brownian aggregate formations without disrupting each other.

{{ webm(src="short2seeds.webm", caption="Custom Seed Location", width=500) }}

#### Reset
The reset serial command is responsible for either committing a hard or soft reset. A hard reset is capable of reducing the program back to a basic DLA movement. All factors and parameters are reset, including custom seed locations. This is done when many factors were activated at once and the user wants a fresh start. The soft reset is capable of decaying all particle formations and randomly spawning them. This is done when exploring specific combinations of factors and wanting to see all the unique formations that can appear. Shown below is a hard reset.

{{ webm(src="reset.webm", caption="Resetting Particles", width=500) }}

#### Density of Particles
Before we decided to run the program with 7000-8000 particles, we had troubles with the density of particle aggregation. Starting off, we only ran several hundred particles. As a result, the particles would finish aggregating very quickly, and the clusters would not be large enough to properly represent Brownian motion. We continued to increase the particle count, but as we continued beyond 7000-8000 particles, we noticed that the particle aggregations were densely population the screen. Due to an excess number of particles and a space limitation, dense clusters began to form in specific locations. Eventually, we decided that 7000-8000 particles was optimal for both creating Brownian clusters and filling up a significant portion of the screen.

#### Maximum of Particles
While we were capable of smoothly running 7000-8000 particles, program is capable of running 16000 particles based on memory limitations. However, as the particle count increases, there were noticeable reductions in frame rate. This reduction is especially noticeable depending on which factors are activated. Because tilt, speed, and cyclic are computationally intensive, these factors are the computational bottleneck. Additionally, as mentioned above, running the maximum number of particles would also significantly increase the particle density, straying the program away from Brownian clusters.

#### Overall
Because our program is an expansion upon lab 2 and not hardware intensive, we did not have to perform much hardware debugging. Instead, we had to perform software debugging, graphing the IMU outputs like in lab 3 and visually checking the functionality of our particle aggregation and features. We focused on implementing features rather than optimizing the particle count and frame rate, so we do not have any results relating to speed or numerical accuracy. 

#### Safety
There are on significant safety concerns as our project mainly focuses on viewing simulations through the VGA. We ensured that the IMU safely captures hand motions using a customized glove. The human being used will not matter as the IMU is attached to the glove. The hand movements are completely safe as they are dependent on changing the angle and acceleration of the IMU. For the user, this results in tilting their hand and waving it back and forth, both of which are safe movements.

#### Usability
The program is extremely usable due to the IMU-glove functionality being simple. The only requirement is being able to wear the IMU-glove and create movement. Because our program does not integrate sound, the hearing impaired will still be able to properly engage with it. We created a simple color scheme with either bright green or black, allowing the visually impaired to properly engage with the display.

## Bugs of Note

We detail below some of the bugs we encountered while building our simulator, how we overcame them,
and any takeaways we have.

### Polling Problems
Initially, polling our IMU at a rate of 1,000 Hz via a repeating alarm timer
led to our simulation crashing shortly after startup.
It was unclear what the root cause of these issues were, as they persisted even when moving
our repeating alarm timer to a different core. Which goes against our initial suspicion that
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
unclear why polling the IMU changed our collision detection logic.


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
arithmetic for the sake of computational speed,
and then converted values to integers. Our initial system took into account the sign of our tilt,
and shifted the mean of our distribution accordingly, based on the between our current angle and 90 degrees.

Because of the way our fixed to integer conversion works, numbers were rounded differently dependent on their sign.
Namely, fixed point representations are converted to integers by right shifting `>>`. This means that integer values
are truncated, not rounded. This meant that we were taking the floor of our fixed point number.
This has an effect on the magnitude of a number after truncation based on its sign. Consider that `4.3` is truncated to `4`
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

#### Improvements and Extensions
If we were to attempt this lab again, it would be interesting to continue adding features that simulate more natural patterns. We were able to generate snowflake-like structures as well as subtle rippling motion, but we were unable to recreate patterns such as a water droplet's wave. 

For example, we could implement a "draw seed" feature that would allow a custom seed to manually be drawn on the screen. This feature would require two potentiometers, adjustable voltage dividers that can be used to implement custom particle movement. The current custom seed feature is only capable of adding a single seed at a time. However, this "draw seed" feature would be capable of drawing multiple seeds at once, perhaps leading to more complex cluster formations. 

Additionally, there are methods of optimizing the DLA algorithm that lead to a more accurate collision algorithm, such as creating a variable step size. If we had more time and a stronger mathematics background, it would be nice to learn how to utilize concepts such as hyper-matrices to increase collision accuracy. While the current collision algorithm works fairly well, it would be interesting to see how our formations alter when using an optimized collision algorithm.

For future extensions beyond adding more DLA features, we could expand our program to highlighting other particle models as well. Though the DLA patterns are interesting, there are many patterns that cannot be replicated using DLA. The Laplacian Growth model could be used to simulate crystal growth and electrodeposition. Other mathematical models could be used to simulate fungal growth or bacteria colonies. It would be worth comparing how all the models respond to adding the existing speed, tilt, and cyclic features as well.

#### Final Thoughts
After completing the previous labs, we were able to properly accumulate all of our knowledge into our final project. We were able to manipulate our IMU output so that it dynamically calculated speed and tilt. We ran our functions on two separate cores to maximize functionality. We created a serial interface that could interact with the IMU. While the results did not completely meet our expectations, we completed all the functionality we originally planned in the given timespan. As mentioned before, there are many ways for us to improve and extend our program. 

On a personal note, it was especially exciting to see our program in practice. Seeing our project begin as an abstract and then turn into life was particularly rewarding, especially because there were many challenges along the way. The program often created patterns that were unexpected, leaving us mesmerized as we fiddled with the features ourselves.

As mentioned in the previous section it would be interesting (although possibly difficult) to use a more complicated collision method that could potentially lead to more life-like formations. Due to the limited hardware, this program has a greater emphasis on software implementation, meaning that there is much more room for cheats and optimizations than we think.

#### Standards
Regarding standards, this project is specialized, meaning that it does not entirely align with a single standard. However, IEEE 1872, “Ontologies for Robotics and Automation,” is the most relevant as we integrate the IMU and hand motions and define the relationship between our DLA program with said hand motions. That being said, our project conforms to the applicable standard because our report contains language and formal engineering vocabulary that adheres to this standard. We do not hastily define or make-up any phrase or definition relating to the hardware and software technologies that we are using.

#### Acknowledgements
We do not use any third-party music clips or images, so we do not have to include any licensing agreements. Because we based our implementation off of code from several public online demo sources, we have properly cited our sources below. While we used these sources as motivation, we created our own DLA design that utilizes a customized collision algorithm. Since the DLA is a common algorithm, we do not have any patent opportunities for our project. No NDA was required as all our hardware was provided in lab. Additionally, because we do not use any RF transmitters or have any legal considerations regarding the FDA, motor vehicle codes, and similar regulations, this project will be recreatable without the user ensuring that their setup passes specific certifications.

## Appendix A

#### Project Website
The group approves this report for inclusion on the course website.

#### Project Video
The group approves the video for inclusion on the course youtube channel.

## Appendix

### Hardware

{{ figure(src="lab4-final-dla-schematic.png", caption="System level diagram", width=500, height=500) }}

### Tasks
The work was largely evenly split among team members, with slight focuses on certain.
The following is a non-exhaustive list of topics focused on.

**Nathaniel:** Setup and dev environment, collision detection, voltage divider, website, and report.

**Angela:** Collision detection, voltage divider, aggregation algorithm, website, and report.

**William:** Random number generation, motion-random-distribution effects, serial, website, and report.

### References
http://formandcode.com/code-examples/simulate-dla

https://isaacshaker.github.io/DLA-Simulation/

https://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_random/index_random.html

https://ciphrd.com/2020/07/21/cyclic-diffusion-limited-aggregation/

### Code

Code can be found [here](../code).

[normal distribution]: https://en.wikipedia.org/wiki/Normal_distribution
[lab 3]: https://vanhunteradams.com/Pico/Helicopter/Helicopter.html