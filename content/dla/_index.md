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
that is described by a [normal distribution](https://en.wikipedia.org/wiki/Normal_distribution)
whose variance is proportional to time elapsed.

IMUs measure the acceleration it is undergoing, as well as any rotational velocity it
experiences. With the help of some [trigonometry and filtering](https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html#Accelerometer-vs.-Gyroscope), it is possible
to determine the orientation of an IMU.

With this in mind, we set offa motion-controlled to build a DLA simulator



{{ figure(src="google.png", alt="alt text", caption="test caption text") }}

