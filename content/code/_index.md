
+++
title = "DLA - Code"
description = "Code listing for our DLA project."
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

See the [documentation](../dla) of this code.

```C

/**
 * nrn25, ayc62, wty5
 *
 * This produces DLA using an RP2040. Some DLA parameters are controllable via
 * an IMU
 *
 * HARDWARE CONNECTIONS
 *  - (White) GPIO 16 ---> VGA Hsync
 *  - (Black) GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - (Brown board 18) RP2040 GND ---> VGA GND
 *
 *
 *  - 3.3V    ---> IMU Vin
 *  - (Grey board 2) RP2040 GND ---> IMU GND
 *  - (Purple board 11) GPIO 8 ---> MPU6050 SDA
 *  - (White board 12)  GPIO 9 ---> MPU6050 SCL
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 *  - GPIO 28 - used for ISR mapping, used for oscilliscope.
 *
 *
 *
 * ANGLE ORIENTATIONS
 *
 * X to the right
 * Vertical is around X axis
 *
 * Y is forward
 * Horizontal is around Y
 */

// Include the VGA grahics library
#include "vga_graphics.h"
// Include standard libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// Include Pico libraries
#include "pico/divider.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"
// Include hardware libraries
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/pll.h"

// Include protothreads
#include "pt_cornell_rp2040_v1.h"
// Include IMU libraries
#include "mpu6050.h"

// === the fixed point macros ========================================

// NOTE: we are actually using fix16, see mpu6050.h
typedef signed int fix15;

// Wall detection
#define hitBottom(b) (b > int2fix15(380))
#define hitTop(b) (b < int2fix15(100))
#define hitLeft(a) (a < int2fix15(100))
#define hitRight(a) (a > int2fix15(540))

// uS per frame
#define FRAME_RATE 33000
#define LED_PIN 25

#define ABS_ARENA_LEFT 30
#define ABS_ARENA_TOP 20
#define ABS_ARENA_BOT 220
#define ABS_ARENA_RIGHT 319
#define ARENA_HEIGHT (ABS_ARENA_BOT - ABS_ARENA_TOP)
#define ARENA_WIDTH (ABS_ARENA_RIGHT - ABS_ARENA_LEFT)

#define NINETY_FIX 5898240

// the color of freely moving particles, initially dim green
char active_color = 1;

// #define PARTICLE_COUNT 1000
#define PARTICLE_COUNT 8000

#define SPEED 2

int max_speed = SPEED;
int min_speed = -SPEED;
int max_x_speed = SPEED;
int min_x_speed = -SPEED;
int max_y_speed = SPEED;
int min_y_speed = -SPEED;

fix15 acceleration_arr[20] = {0};
int acceleration_counter = 0;
fix15 acceleration_sum = 0;
fix15 acceleration_variance;
fix15 FIX20 = int2fix15(20);

// features
bool tilt_feature = 0;
bool speed_feature = 0;
bool cyclic_feature = 1;
bool reset_feature = 1;
bool seed_feature = 0;

int seed_x = 0;
int seed_y = 0;
int old_seed_x = 0;
int old_seed_y = 0;

// boid structure
struct particle {
  short x;
  short y;
  char color;
  short cyclic_counter;  // tracks "age" of aggregated particle
};

bool aggregate[ARENA_HEIGHT][ARENA_WIDTH];
struct particle particles[PARTICLE_COUNT];

// fps and timing info
static int spare_time_0;
static int spare_time_acc_0 = 0;
static int avg_spare_time_0 = 0;
static int frame_count_0 = 0;
static int grad_incr = 1;

volatile float filtered_ay = 0;
volatile float filtered_az = 0;
volatile float filtered_ax = 0;
fix15 vertical_accel_angle;
fix15 horizontal_accel_angle;
fix15 vertical_complementary_angle;
fix15 horizontal_complementary_angle;
// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

#define AVG_SPARE_TIME_FRAME_COUNT 512

int current_fps = 30;
static int rand_idx = 0;
static int rand_arr[4] = {3, 4, -5, -6};

// assume this is only used for brownian motion. SHOULD CHANGE TO NORMAL
int uniform_rand(int min, int max) {
  return (rand() % (max - min + 1)) + min;
}

inline void bound_particle(struct particle *particle) {
  // return;
  if (particle->x < ABS_ARENA_LEFT + 1)
    particle->x = ABS_ARENA_LEFT + 1;
  else if (particle->x > ABS_ARENA_RIGHT - 1)
    particle->x = ABS_ARENA_RIGHT - 1;

  if (particle->y < ABS_ARENA_TOP + 1)
    particle->y = ABS_ARENA_TOP + 1;
  else if (particle->y > ABS_ARENA_BOT - 1)
    particle->y = ABS_ARENA_BOT - 1;
}

void drawInfo() {
  setCursor(5, 10);

  setTextSize(1);
  setTextColor2(RED, BLACK);

  char string[200];
  sprintf(string, "Var: %f", fix2float15(acceleration_variance));
  writeString(string);

  setCursor(5, 20);

  setTextSize(1);
  setTextColor2(RED, BLACK);

  sprintf(string, "sum: %f", fix2float15(acceleration_sum));
  writeString(string);

  setCursor(5, 30);

  setTextSize(1);
  setTextColor2(RED, BLACK);

  sprintf(string, "maxy: %d miny: %d", max_y_speed, min_y_speed);
  writeString(string);
}

// this reads from the IMU!
bool update_angles() {
  gpio_put(LED_PIN, !gpio_get(LED_PIN));
  gpio_put(28, 1);  // for testing of ISR length

  // // Read the IMU
  // // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
  // // If you want these values in floating point, call fix2float15() on
  // // the raw measurements.

  filtered_ax = filtered_ax + ((int)((float)acceleration[0] - filtered_ax) >> 2);
  filtered_ay = filtered_ay + ((int)((float)acceleration[1] - filtered_ay) >> 2);
  filtered_az = filtered_az + ((int)((float)acceleration[2] - filtered_az) >> 2);

  // // 0 is hanging down!!!
  // // this returns -180 to 180
  // note this is z over y
  vertical_accel_angle = multfix15(float2fix15(atan2(filtered_ay, filtered_az)), oneeightyoverpi);
  horizontal_accel_angle = multfix15(float2fix15(atan2(-filtered_ax, filtered_az)), oneeightyoverpi);

  vertical_complementary_angle = vertical_accel_angle;
  horizontal_complementary_angle = horizontal_accel_angle;

  gpio_put(28, 0);  // for testing of ISR length

  return true;
}

// Create a particle
void spawnParticle(struct particle *particle) {
  // Start in random place
  particle->x = (short)(uniform_rand(ABS_ARENA_LEFT, ABS_ARENA_RIGHT));
  particle->y = (short)(uniform_rand(ABS_ARENA_TOP, ABS_ARENA_BOT));
  particle->cyclic_counter = 0;

  bound_particle(particle);

  particle->color = active_color;
}

bool in_bound(short x, short y) {
  return ((x > ABS_ARENA_LEFT) && (x < ABS_ARENA_RIGHT) && (y > ABS_ARENA_TOP) && (y < ABS_ARENA_BOT));
}

inline bool is_aggregated(struct particle *particle) {
  //>0 cause BLACK is 0 and 2 lsb are green (see vga_graphics.h).
  return particle->color != 1 && particle->color > 0;
}

// instead of simple yes/no, we will aggregate if sum of neighbors is bigger than some threshold
// threshold needs to be 3 if we only have 1 starting seed
// can be 6 if we have 2 seeds next to each other, etc.
char aggregation_threshold = 15;
bool touching_aggregate(short x, short y) {
  if (!in_bound(x, y)) return false;

  char sum = 0;
  char curr_color = 0;

  curr_color = getPixel(x - 1, y);
  if (curr_color != active_color)
    sum += curr_color;
  curr_color = getPixel(x + 1, y);
  if (curr_color != active_color)
    sum += curr_color;
  curr_color = getPixel(x, y - 1);
  if (curr_color != active_color)
    sum += curr_color;
  curr_color = getPixel(x, y + 1);
  if (curr_color != active_color)
    sum += curr_color;
  curr_color = getPixel(x - 1, y - 1);
  if (curr_color != active_color)
    sum += curr_color;
  curr_color = getPixel(x + 1, y - 1);
  if (curr_color != active_color)
    sum += curr_color;
  curr_color = getPixel(x - 1, y + 1);
  if (curr_color != active_color)
    sum += curr_color;
  curr_color = getPixel(x + 1, y + 1);
  if (curr_color != active_color)
    sum += curr_color;

  return sum >= aggregation_threshold;
}

void move_crystal(struct particle *particle, short dx, short dy) {
  fix15 fix_dx = short2fix15(dx);
  fix15 fix_dy = short2fix15(dy);
  fix15 abs_dx = absfix15(fix_dx);
  fix15 abs_dy = absfix15(fix_dy);

  // alpha = 1, beta = 1/4
  fix15 len = ((abs_dx > abs_dy) ? abs_dx : abs_dy) + (((abs_dx > abs_dy) ? abs_dy : abs_dx) >> 2);

  len = (len & (0xFFFF0000));  // mask off decimal bits

  fix15 lil_dx = (len != 0) ? divfix(fix_dx, len) : 0;
  fix15 lil_dy = (len != 0) ? divfix(fix_dy, len) : 0;

  fix15 particle_x = short2fix15(particle->x);
  fix15 particle_y = short2fix15(particle->y);

  short short_particle_x;
  short short_particle_y;

  for (short i = 0; i < fix2short15(len); i++) {
    // for (short i = 0; i < 8; i++) {
    short_particle_x = fix2short15(particle_x);
    short_particle_y = fix2short15(particle_y);
    if (touching_aggregate(short_particle_x, short_particle_y) && getPixel(particle->x, particle->y) == BLACK) {
      particle->x = short_particle_x;
      particle->y = short_particle_y;
      particle->color = WHITE;
      particle->cyclic_counter = 0;
      return;
    }
    particle_x += lil_dx;
    particle_y += lil_dy;
  }

  short_particle_x = fix2short15(particle_x);
  short_particle_y = fix2short15(particle_y);
  particle->x = short_particle_x;
  particle->y = short_particle_y;
}

void calculate_tilt_parameters() {
  fix15 vertical_ratio = divfix(vertical_complementary_angle, NINETY_FIX);
  fix15 horizontal_ratio = divfix(horizontal_complementary_angle, NINETY_FIX);
  if (vertical_ratio < 0) {  // GO UP
    max_y_speed = max_speed + fix2int15(multfix15(int2fix15(max_speed), vertical_ratio));
    if (max_speed == 1) min_y_speed = min_y_speed + fix2int15(multfix15(int2fix15(max_speed), vertical_ratio));
  } else {  // GO DOWN
    min_y_speed = min_speed - fix2int15(multfix15(int2fix15(max_speed), -vertical_ratio));
    if (max_speed == 1) max_y_speed = max_y_speed - fix2int15(multfix15(int2fix15(max_speed), -vertical_ratio));
  }

  if (horizontal_ratio > 0) {  // GO RIGHT
    min_x_speed = min_speed - fix2int15(multfix15(int2fix15(max_speed), -horizontal_ratio));
    if (max_speed == 1) max_x_speed = max_x_speed - fix2int15(multfix15(int2fix15(max_speed), -horizontal_ratio));
  } else {  // GO LEFT
    max_x_speed = max_speed + fix2int15(multfix15(int2fix15(max_speed), horizontal_ratio));
    if (max_speed == 1) min_x_speed = min_x_speed + fix2int15(multfix15(int2fix15(max_speed), horizontal_ratio));
  }

  // cap everything
  if (min_y_speed > -1) min_y_speed = -1;
  if (min_x_speed > -1) min_x_speed = -1;
  if (max_y_speed < 1) max_y_speed = 1;
  if (max_x_speed < 1) max_x_speed = 1;
}

void set_speed_parameters() {
  fix15 variance = 0;
  fix15 avg = 0;
  fix15 sum = 0;

  avg = divfix(acceleration_sum, FIX20);
  for (int i = 0; i < 20; i++) {
    fix15 diff = (acceleration_arr[i] - avg);
    sum += multfix15(diff, diff);
  }
  // multiply by 16 to get a good scaling, this caps around 6 or 7 empirically.
  variance = divfix(sum, FIX20) << 4;

  acceleration_variance = variance;
  int acceleration_variance_int = fix2int15(acceleration_variance);
  if (acceleration_variance_int > 0) {
    min_speed = -acceleration_variance_int;
    max_speed = acceleration_variance_int;
  } else {
    min_speed = -1;
    max_speed = 1;
  }
}

// responsible for updating cyclic counter and spawning new particle on decay.
void decay(struct particle *particle) {
  if (is_aggregated(particle)) {
    particle->cyclic_counter += 1;
    // tune here
    if (particle->cyclic_counter % grad_incr == 0 && particle->color > 2) {
      particle->color -= 1;
    }
    if (particle->cyclic_counter >= grad_incr * 16) {
      // get rid of i.e respawn
      drawPixel(particle->x, particle->y, BLACK);
      spawnParticle(particle);
    }

    // }
  }
}

// Detect wallstrikes, update velocity and position
void resetPixel(struct particle *particle) {
  drawPixel(particle->x, particle->y, BLACK);
  spawnParticle(particle);
}

// Detect wallstrikes, update velocity and position
void updatePosAndVel(struct particle *particle) {
  // needs to be both for when restart/transition visibility
  if (particle->color == 1 || particle->color == BLACK) {
    particle->color = active_color;

    short dx = (short)uniform_rand(min_x_speed, max_x_speed);
    short dy = (short)(uniform_rand(min_y_speed, max_y_speed));

    move_crystal(particle, dx, dy);

    bound_particle(particle);

  } else if (cyclic_feature && is_aggregated(particle)) {
    decay(particle);
  }
}

// Draw the boundaries
void drawArena() {
  drawVLine(ABS_ARENA_LEFT, ABS_ARENA_TOP, ARENA_HEIGHT, BLACK);   // Left Line
  drawVLine(ABS_ARENA_RIGHT, ABS_ARENA_TOP, ARENA_HEIGHT, BLACK);  // Right Line
  drawHLine(ABS_ARENA_LEFT, ABS_ARENA_TOP, ARENA_WIDTH, BLACK);    // Top Line
  drawHLine(ABS_ARENA_LEFT, ABS_ARENA_BOT, ARENA_WIDTH, BLACK);    // Bottom Line
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD(protothread_serial(struct pt *pt)) {
  PT_BEGIN(pt);
  // stores user input

  // wait for 0.1 sec
  PT_YIELD_usec(1000000);
  // announce the threader version
  sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
  // non-blocking write
  serial_write;

  while (1) {
    char user_input[16];
    int value_x;
    int value_y;

    // print prompt
    sprintf(pt_serial_out_buffer, "input a command:");
    // non-blocking write
    serial_write;
    // spawn a thread to do the non-blocking serial read
    serial_read;
    // convert input string to number
    sscanf(pt_serial_in_buffer, "%s %d %d", user_input, &value_x, &value_y);

    // tilt feature
    if (user_input[0] == 't') {
      tilt_feature = !tilt_feature;
    }

    // speed feature
    else if (user_input[0] == 's') {
      if (value_x == 0) {
        speed_feature = !speed_feature;
      } else {
        min_speed = -(value_x);
        max_speed = value_x;
        speed_feature = 0;
      }
    }

    else if (user_input[0] == 'c') {
      if (value_x == 0) {
        cyclic_feature = 0;
        grad_incr = 0;
      } else {
        cyclic_feature = 1;
        grad_incr = value_x;
      }
    }

    else if (user_input[0] == 'v') {
      if (active_color == 1) {
        active_color = BLACK;
      } else if (active_color == BLACK) {
        active_color = 1;
      }
    }

    else if (user_input[0] == 'R') {
      reset_feature = 1;
      cyclic_feature = 0;
      grad_incr = 0;
      speed_feature = 0;
      min_speed = -1;
      max_speed = 1;
      tilt_feature = 0;
      active_color = 1;
      aggregation_threshold = 15;
      seed_feature = 0;
    }

    else if (user_input[0] == 'r') {
      reset_feature = 1;
    }

    else if (user_input[0] == 'l') {
      if (value_x > ABS_ARENA_LEFT && value_x < (ABS_ARENA_RIGHT - 1) && value_y < (ABS_ARENA_BOT - 1) && value_y > ABS_ARENA_TOP) {
        seed_feature = 1;
        old_seed_x = seed_x;
        old_seed_y = seed_y;
        seed_x = value_x;
        seed_y = value_y;
      }
    }

    else if (user_input[0] == 'p') {
      // print prompt
      sprintf(pt_serial_out_buffer, "speed: %d tilt: %d cyclic: %d aggregate: %d ",
              max_speed, tilt_feature, grad_incr, aggregation_threshold);
      // non-blocking write
      serial_write;
    }

    else if (user_input[0] == 'a') {
      if (value_x > 1) {
        aggregation_threshold = value_x;
      }
    }

    // seed in the middle and very responsive. Can tilt all to one side and
    // then "launch" from that side to get interesting effects
    else if (user_input[0] == '1') {
      min_speed = -5;
      max_speed = 5;
      tilt_feature = 1;
      grad_incr = 1;
      aggregation_threshold = 15;

      seed_feature = 0;
    }

    // tilt and speed both on.
    else if (user_input[0] == '2') {
      speed_feature = 1;
      tilt_feature = 1;
    }

    // sets seed to corner, leave tilting and can begin to "pulse"
    else if (user_input[0] == '3') {
      min_speed = -1;
      max_speed = 1;
      tilt_feature = 1;
      grad_incr = 2;
      aggregation_threshold = 15;

      seed_feature = 1;
      old_seed_x = seed_x;
      old_seed_y = seed_y;
      seed_x = 32;
      seed_y = 218;
    }
  }
  PT_END(pt);
}  // timer thread

void update_accel() {
  // acceleration logic
  fix15 abs_z_accel = abs(acceleration[2]);
  acceleration_sum += abs_z_accel - acceleration_arr[acceleration_counter];
  acceleration_arr[acceleration_counter] = abs_z_accel;

  acceleration_counter += 1;
  if (acceleration_counter > 19) {
    acceleration_counter = 0;
  }
}

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt)) {
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time;

  //  We will start drawing at column 81
  static int xcoord = 40;
  // Rescale the measurements for display
  static float NewRange = 75.;  // (looks nice on VGA)
  // Control rate of drawing
  static int throttle;

  // Draw the static aspects of the display
  setTextSize(1);
  setTextColor(WHITE);

  // Spawn a particle
  for (int i = 0; i < PARTICLE_COUNT; ++i) {
    spawnParticle(&particles[i]);
  }

  while (1) {
    mpu6050_read_raw(acceleration, gyro);
    update_angles();
    update_accel();

    // Measure time at start of thread
    begin_time = time_us_32();
    if (!seed_feature) {
      drawPixel(ABS_ARENA_LEFT + ARENA_WIDTH / 2, ABS_ARENA_TOP + ARENA_HEIGHT / 2, WHITE);
    }

    else {
      drawPixel(ABS_ARENA_LEFT + ARENA_WIDTH / 2, ABS_ARENA_TOP + ARENA_HEIGHT / 2, BLACK);
      drawPixel(old_seed_x, old_seed_y, BLACK);
      drawPixel(old_seed_x + 1, old_seed_y, BLACK);
      drawPixel(old_seed_x, old_seed_y + 1, BLACK);
      drawPixel(old_seed_x + 1, old_seed_y + 1, BLACK);

      drawPixel(seed_x, seed_y, WHITE);
      drawPixel(seed_x + 1, seed_y, WHITE);
      drawPixel(seed_x, seed_y + 1, WHITE);
      drawPixel(seed_x + 1, seed_y + 1, WHITE);
    }

    if (reset_feature) {
      for (int i = 0; i < PARTICLE_COUNT; ++i)
        resetPixel(&particles[i]);
      reset_feature = 0;
    }

    min_x_speed = min_speed;
    min_y_speed = min_speed;
    max_x_speed = max_speed;
    max_y_speed = max_speed;
    if (tilt_feature) {
      calculate_tilt_parameters();
    }

    if (speed_feature) {
      set_speed_parameters();
    }

    // Measure time at start of thread
    begin_time = time_us_32();

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      // draw particle if valid

      drawPixel(particles[i].x, particles[i].y, BLACK);
      // update particle's position and velocity
      updatePosAndVel(&particles[i]);
      // draw the particle at its new position
      drawPixel(particles[i].x, particles[i].y, particles[i].color);
    }

    // draw the boundaries
    drawArena();
    // drawInfo();

    // delay in accordance with frame rate
    spare_time_0 = FRAME_RATE - (time_us_32() - begin_time);
    spare_time_acc_0 += spare_time_0;
    frame_count_0 += 1;

    if (frame_count_0 >= AVG_SPARE_TIME_FRAME_COUNT) {
      avg_spare_time_0 = spare_time_acc_0 / AVG_SPARE_TIME_FRAME_COUNT;
      frame_count_0 = 0;
      spare_time_acc_0 = 0;
    }

    // calc current fps
    if (spare_time_0 >= 0) {
      current_fps = 30;
    } else {
      if (FRAME_RATE - spare_time_0 != 0) {
        current_fps = 1000000 / (FRAME_RATE - spare_time_0);
      }
    }

    // yield for necessary amount of time
    PT_YIELD_usec(spare_time_0);
  }
  PT_END(pt);
}  // animation thread

void core1_entry() {
  pt_add_thread(protothread_serial);
  pt_schedule_start;
}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main() {
  // screw it lets overclock
  // set_sys_clock_khz(250000, true);

  // initialize stio
  stdio_init_all();

  // initialize VGA
  initVGA();
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_init(28);
  gpio_set_dir(28, GPIO_OUT);

  i2c_init(I2C_CHAN, I2C_BAUD_RATE);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);

  // MPU6050 initialization
  mpu6050_reset();

  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(&core1_entry);

  // add threads
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start;
}

```
