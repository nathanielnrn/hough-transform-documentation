
+++
title = "Hough Transform - Code"
description = "Code listing for our Hough Transform project."
template = "prose.html"

url = "hough-transform-documentation/code"
[extra]
lang = 'en'
math = true
mermaid = false
copy = true
comment = false
toc = true
+++

See the [documentation](../hough) of this code.

What follows is an abridged version of our Verilog code (generated segments removed)
and our C code. The full repo is available upon [request](mailto:nathanielnrn99@gmail.com).

# C++
## Hough and Line Drawer

```cpp
// By downloading, copying, installing or using the software you agree to this license.
// If you do not agree to this license, do not download, install,
// copy or use the software.


//                           License Agreement
//                For Open Source Computer Vision Library
//                        (3-clause BSD License)

// Copyright (C) 2000-2020, Intel Corporation, all rights reserved.
// Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
// Copyright (C) 2009-2016, NVIDIA Corporation, all rights reserved.
// Copyright (C) 2010-2013, Advanced Micro Devices, Inc., all rights reserved.
// Copyright (C) 2015-2016, OpenCV Foundation, all rights reserved.
// Copyright (C) 2015-2016, Itseez Inc., all rights reserved.
// Copyright (C) 2019-2020, Xperience AI, all rights reserved.
// Third party copyrights are property of their respective owners.

// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:

//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.

//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.

//   * Neither the names of the copyright holders nor the names of the contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.

// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall copyright holders or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
///////////////////////////////////////
/// 640x480 version!
/// test VGA with hardware video input copy to VGA
///////////////////////////////////////
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <iterator>
#include <vector>

#include "address_map_arm_brl4.h"

#define PI 3.14159
#define assert(b)                      \
  if (!b) {                            \
    printf("assertion does not hold"); \
    exit(3);                           \
  }

enum { XY_SHIFT = 16,
       XY_ONE = 1 << XY_SHIFT,
       DRAWING_STORAGE_BLOCK = (1 << 12) - 256 };

/*
Here image is an input raster;
step is it's step; size characterizes it's ROI;
rho and theta are discretization steps (in pixels and radians correspondingly).
threshold is the minimum number of pixels in the feature for it
to be a candidate for line. lines is the output
array of (rho, theta) pairs. linesMax is the buffer size (number of pairs).
Functions return the actual number of found lines.
*/
struct LinePolar {
  float rho;
  float angle;
};

struct Point {
  int x;
  int y;
};

struct Rect {
  int x;
  int y;
  int width;
  int height;
};

struct hough_cmp_gt {
  hough_cmp_gt(const int* _aux) : aux(_aux) {}
  inline bool operator()(int l1, int l2) const {
    return aux[l1] > aux[l2] || (aux[l1] == aux[l2] && l1 < l2);
  }
  const int* aux;
};

static inline int
computeNumangle(double min_theta, double max_theta, double theta_step) {
  int numangle = floor((max_theta - min_theta) / theta_step) + 1;
  // If the distance between the first angle and the last angle is
  // approximately equal to pi, then the last angle will be removed
  // in order to prevent a line to be detected twice.
  if (numangle > 1 && fabs(PI - (numangle - 1) * theta_step) < theta_step / 2)
    --numangle;
  return numangle;
}

static void
createTrigTable(int numangle, double min_theta, double theta_step,
                float irho, float* tabSin, float* tabCos) {
  float ang = static_cast<float>(min_theta);
  for (int n = 0; n < numangle; ang += (float)theta_step, n++) {
    tabSin[n] = (float)(sin((double)ang) * irho);
    tabCos[n] = (float)(cos((double)ang) * irho);
  }
}

static void
findLocalMaximums(int numrho, int numangle, int threshold,
                  const int* accum, std::vector<int>& sort_buf) {
  for (int r = 0; r < numrho; r++)
    for (int n = 0; n < numangle; n++) {
      int base = (n + 1) * (numrho + 2) + r + 1;
      if (accum[base] > threshold &&
          accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
          accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2])
        sort_buf.push_back(base);
    }
}

// lines should be of size linesMax
static void
HoughLinesStandard(std::vector<LinePolar>& lines,
                   float rho, float theta,
                   int threshold, int linesMax,
                   double min_theta, double max_theta, int* accum) {
  // unsigned char* img = (unsigned char*)src;

  int i, j;
  float irho = 1 / rho;

  assert(linesMax > 0);

  // const unsigned char* image = img;
  int step = 320; //Size in bytes of a row of image
  int width = 320;
  int height = 240;

  int max_rho = (width + height) / 2;
  int min_rho = -max_rho;

  if (max_theta < min_theta) {
    printf("max_theta must be greater than min_theta");
    exit(2);
  };

  int numangle = computeNumangle(min_theta, max_theta, theta);
  int numrho = round(((max_rho - min_rho) + 1) / rho);


  // ALSO rho is x axis and y is theta, not other way around
  // for(int i = 0; i <(180 + 2) * (1121 + 2); ++i ){
  //   accum[i] = 0;
  // }

  std::vector<int> _sort_buf;

  struct timeval t1, t2;
  double elapsedTime;

  gettimeofday(&t1, NULL);


  gettimeofday(&t2, NULL);
  elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
  elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
  // printf("Fill Accumulator: %f\n", elapsedTime);


  
  // stage 2. find local maximums
  gettimeofday(&t1, NULL);

  findLocalMaximums(numrho, numangle, threshold, accum, _sort_buf);

  gettimeofday(&t2, NULL);
  elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
  elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
  // printf("Find Local Maximum: %f\n", elapsedTime);



  // stage 3. sort the detected lines by accumulator value
  gettimeofday(&t1, NULL);

  std::sort(_sort_buf.begin(), _sort_buf.end(), hough_cmp_gt(accum));

  gettimeofday(&t2, NULL);
  elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
  elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
  // printf("Sorting: %f\n", elapsedTime);



  // stage 4. store the first min(total,linesMax) lines to the output buffer

  gettimeofday(&t1, NULL);

  linesMax = std::min(linesMax, (int)_sort_buf.size());
  double scale = 1. / (numrho + 2);

  lines.resize(linesMax);
  // Mat _lines = lines.getMat();
  for (i = 0; i < linesMax; i++) {
    LinePolar line;
    int idx = _sort_buf[i];
    int n = floor(idx * scale) - 1;
    int r = idx - (n + 1) * (numrho + 2) - 1;
    line.rho = (r - (numrho - 1) * 0.5f) * rho;
    line.angle = static_cast<float>(min_theta) + n * theta;
    lines[i] = line;
  }

  gettimeofday(&t2, NULL);
  elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
  elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
  // printf("Lines Stuff: %f\n", elapsedTime);

}

class LineIterator {
 public:
  /** @brief Initializes iterator object for the given line and image.

  The returned iterator can be used to traverse all pixels on a line that
  connects the given two points.
  The line will be clipped on the image boundaries.

  @param img Underlying image.p
  @param pt1 First endpoint of the line.
  @param pt2 The other endpoint of the line.
  @param connectivity Pixel connectivity of the iterator. Valid values are 4 (iterator can move
  up, down, left and right) and 8 (iterator can also move diagonally).
  @param leftToRight If true, the line is traversed from the leftmost endpoint to the rightmost
  endpoint. Otherwise, the line is traversed from \p pt1 to \p pt2.
  */
  LineIterator(const char* img, Point pt1, Point pt2,
               int connectivity = 8, bool leftToRight = false) {
    Rect rect;
    rect.x = 0;
    rect.y = 0;
    rect.width = 320;
    rect.height = 240;
    init(img, rect, pt1, pt2, connectivity, leftToRight);
    ptmode = false;
  }

  void init(const char* img, Rect boundingAreaRect, Point pt1, Point pt2, int connectivity, bool leftToRight);

  /** @brief Returns pointer to the current pixel.
   */

  /** @brief Moves iterator to the next pixel on the line.

  This is the prefix version (++it).
  */

  /** @brief Moves iterator to the next pixel on the line.

  This is the postfix version (it++).
  */

  /** @brief Returns coordinates of the current pixel.
   */
  Point pos() const;

  char* ptr;
  const char* ptr0;
  int step, elemSize;
  int err, count;
  int minusDelta, plusDelta;
  int minusStep, plusStep;
  int minusShift, plusShift;
  Point p;
  bool ptmode;

  inline char* operator*() {
    return ptmode ? 0 : ptr;
  }

  inline LineIterator& operator++() {
    int mask = err < 0 ? -1 : 0;
    err += minusDelta + (plusDelta & mask);
    if (!ptmode) {
      ptr += minusStep + (plusStep & mask);
    } else {
      p.x += minusShift + (plusShift & mask);
      p.y += minusStep + (plusStep & mask);
    }
    return *this;
  }

  inline LineIterator operator++(int) {
    LineIterator it = *this;
    ++(*this);
    return it;
  }
};

void LineIterator::init(const char* img, Rect rect, Point pt1_, Point pt2_, int connectivity, bool leftToRight) {
  count = -1;
  p = Point{0, 0};
  ptr0 = ptr = 0;
  step = elemSize = 0;

  Point pt1 = pt1_;
  Point pt2 = pt2_;

  int delta_x = 1, delta_y = 1;
  int dx = pt2.x - pt1.x;
  int dy = pt2.y - pt1.y;

  if (dx < 0) {
    if (leftToRight) {
      dx = -dx;
      dy = -dy;
      pt1 = pt2;
    } else {
      dx = -dx;
      delta_x = -1;
    }
  }

  if (dy < 0) {
    dy = -dy;
    delta_y = -1;
  }

  bool vert = dy > dx;
  if (vert) {
    std::swap(dx, dy);
    std::swap(delta_x, delta_y);
  }

  if (connectivity == 8) {
    err = dx - (dy + dy);
    plusDelta = dx + dx;
    minusDelta = -(dy + dy);
    minusShift = delta_x;
    plusShift = 0;
    minusStep = 0;
    plusStep = delta_y;
    count = dx + 1;
  } else /* connectivity == 4 */
  {
    err = 0;
    plusDelta = (dx + dx) + (dy + dy);
    minusDelta = -(dy + dy);
    minusShift = delta_x;
    plusShift = -delta_x;
    minusStep = 0;
    plusStep = delta_y;
    count = dx + dy + 1;
  }

  if (vert) {
    std::swap(plusStep, plusShift);
    std::swap(minusStep, minusShift);
  }

  p = pt1;
  if (!ptmode) {
    ptr0 = img;
    step = (int)320;  //(size occurpied by row)
    elemSize = 1;
    ptr = (char*)ptr0 + (size_t)p.y * step + (size_t)p.x * elemSize;
    plusStep = plusStep * step + plusShift * elemSize;
    minusStep = minusStep * step + minusShift * elemSize;
  }
}

static void
Line(char* img, Point pt1, Point pt2, const char color, int connectivity = 8) {
  if (connectivity == 0)
    connectivity = 8;
  else if (connectivity == 1) {
    connectivity = 4;
  }

  LineIterator iterator(img, pt1, pt2, connectivity, true);
  int i, count = iterator.count;
  int pix_size = 1;

  for (i = 0; i < count; i++, ++iterator) {
    char* ptr = *iterator;
    *ptr = color;
  }
}

static void ThickLine(char* img, Point p0, Point p1, const char color,
                      int thickness, int line_type, int flags, int shift) {
  static const double INV_XY_ONE = 1. / XY_ONE;
  

  //TODO: Are we sure these are ints?
  p0.x <<= XY_SHIFT - shift;
  p0.y <<= XY_SHIFT - shift;
  p1.x <<= XY_SHIFT - shift;
  p1.y <<= XY_SHIFT - shift;

  p0.x = (p0.x + (XY_ONE >> 1)) >> XY_SHIFT;
  p0.y = (p0.y + (XY_ONE >> 1)) >> XY_SHIFT;
  p1.x = (p1.x + (XY_ONE >> 1)) >> XY_SHIFT;
  p1.y = (p1.y + (XY_ONE >> 1)) >> XY_SHIFT;
  Line(img, p0, p1, color, line_type);
}





void line(char* _img, Point pt1, Point pt2, const char color = (7 << 5),
          int thickness = 1, int line_type = 8, int shift = 0) {
  char* img = _img;

  line_type = 8;

  assert(0 < thickness && thickness <= 8);
  assert(0 <= shift && shift <= XY_SHIFT);

  ThickLine(img, pt1, pt2, color, thickness, line_type, 3, shift);
}
```

## Main Program
```cpp
///////////////////////////////////////
/// 640x480 version!
/// test VGA with hardware video input copy to VGA
///////////////////////////////////////
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h>
#include <sys/time.h> 
#include <vector>
#include <math.h>
#include "address_map_arm_brl4.h"
#include "hough.cpp"

#define PI 3.14159

// #define HW_REGS_BASE 0xff200000
// #define HW_REGS_SPAN 0x00006000


/* function prototypes */
void VGA_text (int, int, char *);
void VGA_text_clear();
void VGA_box (int, int, int, int, short);
void VGA_line(int, int, int, int, short) ;
void VGA_disc (int, int, int, short);
int  VGA_read_pixel(int, int) ;
int  video_in_read_pixel(int, int);
void draw_delay(void) ;


// the light weight buss base
void *h2p_lw_virtual_base;
volatile unsigned int *h2p_lw_video_in_control_addr=NULL;
volatile unsigned int *h2p_lw_video_in_resolution_addr=NULL;

volatile unsigned int *h2p_lw_video_edge_control_addr=NULL;

// pixel buffer from
volatile unsigned int * vga_pixel_ptr = NULL ;
void *vga_pixel_virtual_base;

// video input buffer
volatile unsigned int * video_in_ptr = NULL ;
void *video_in_virtual_base;

void *sram_master_base;

//hps copy sram pointer from the first copy of camera in that hps reads from
volatile char * hps_copy_sram2_ptr = NULL;
//hps copy sram pointer to 3rd sram that hps writes into
volatile char * lines_ptr = NULL;

// character buffer
// volatile unsigned int * vga_char_ptr = NULL ;
// void *vga_char_virtual_base;

// /dev/mem file id
int fd;

// shared memory 
key_t mem_key=0xf0;
int shared_mem_id; 
int *shared_ptr;
int shared_time;
int shared_note;
char shared_str[64];

  
volatile signed int *color_in_ptr; 
volatile signed int *color_out_ptr; 
volatile signed int *color_wr_addr_ptr; 
volatile signed int *color_wr_en_ptr;
volatile signed int *color_in_addr_ptr; 

volatile bool *wait_accum_ptr;

// measure time
struct timeval t1, t2;
double elapsedTime;
struct timespec delay_time;

bool check(Point pt){
  return (pt.x >= 0) && (pt.x <= 340) && (pt.y <= 240) && (pt.y >= 0);
}
void coords(double m, double b, int*x0, int*x1, int*y0, int*y1){
  
  Point pt1, pt2, pt3, pt4;

  pt1.x = round((240-b)/m);
  pt1.y = 240;

  pt2.x = round(-b/m);
  pt2.y = 0;

  pt3.x = 320;
  pt3.y = round(m*320 + b);

  pt4.x = 0;
  pt4.y = round(b);

  bool v1, v2, v3, v4;
  v1 = check(pt1);
  v2 = check(pt2);
  v3 = check(pt3);
  v4 = check(pt4);

  // Forgive me lord.
  if(v1){
    if(v2){
      *x0 = pt1.x;
      *y0 = pt1.y;
      *x1 = pt2.x;
      *y1 = pt2.y;
    }else{
      if(v3){
      *x0 = pt1.x;
      *y0 = pt1.y;
      *x1 = pt3.x;
      *y1 = pt3.y;
      }else{
        if(v4){
          *x0 = pt1.x;
          *y0 = pt1.y;
          *x1 = pt4.x;
          *y1 = pt4.y;
        }else{
          *x0 = 0;
          *y0 = 0;
          *x1 = 0;
          *y1 = 0;
          // printf("WARNING: Invalid Line \n");	
        }
      }
    }
  }else{
    if(v2){
      if(v3){
          *x0 = pt2.x;
          *y0 = pt2.y;
          *x1 = pt3.x;
          *y1 = pt3.y;
      }else{
        if(v4){
          *x0 = pt2.x;
          *y0 = pt2.y;
          *x1 = pt4.x;
          *y1 = pt4.y;
        }else{
          *x0 = 0;
          *y0 = 0;
          *x1 = 0;
          *y1 = 0;
          // printf("WARNING: Invalid Line \n");						
        }
      }
    }else{
      if(v3){
        if(v4){
          *x0 = pt3.x;
          *y0 = pt3.y;
          *x1 = pt4.x;
          *y1 = pt4.y;
        }else{
          *x0 = 0;
          *y0 = 0;
          *x1 = 0;
          *y1 = 0;
          // printf("WARNING: Invalid Line \n");
        }
      }else{
        *x0 = 0;
        *y0 = 0;
        *x1 = 0;
        *y1 = 0;
        // printf("WARNING: Invalid Line \n");
      }
    }
  }

}



int main(void)
{
  delay_time.tv_nsec = 10 ;
  delay_time.tv_sec = 0 ;

  
  // === get FPGA addresses ==================
    // Open /dev/mem
  if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) 	{
    printf( "ERROR: could not open \"/dev/mem\"...\n" );
    return( 1 );
  }

  h2p_lw_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );	
  if( h2p_lw_virtual_base == MAP_FAILED ) {
    printf( "ERROR: mmap1() failed...\n" );
    close( fd );
    return(1);
  }
    h2p_lw_video_in_control_addr=(volatile unsigned int *)(h2p_lw_virtual_base+VIDEO_IN_BASE+0x0c);
  h2p_lw_video_in_resolution_addr=(volatile unsigned int *)(h2p_lw_virtual_base+VIDEO_IN_BASE+0x08);
  *(h2p_lw_video_in_control_addr) = 0x04 ; // turn on video capture
  *(h2p_lw_video_in_resolution_addr) = 0x00f00140 ;  // high 240 low 320
  h2p_lw_video_edge_control_addr=(volatile unsigned int *)(h2p_lw_virtual_base+VIDEO_IN_BASE+0x10);
  *h2p_lw_video_edge_control_addr = 0x01 ; // 1 means edges
  // *h2p_lw_video_edge_control_addr = 0x00 ; // 1 means edges

  
  // === get video input =======================
  // on-chip sRAM
  video_in_virtual_base = mmap( NULL, FPGA_ONCHIP_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_ONCHIP_BASE); 
  if( video_in_virtual_base == MAP_FAILED ) {
    printf( "ERROR: mmap3() failed...\n" );
    close( fd );
    return(1);
  }
  // format the pointer
  video_in_ptr =(unsigned int *)(video_in_virtual_base);
// 0x00040000
  // for srams using not lw
  sram_master_base = mmap( NULL, 0x00040000, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, 0xc0000000); 
  if( sram_master_base == MAP_FAILED ) {
    printf( "ERROR: mmap3() failed...\n" );
    close( fd );
    return(1);
  }
  // format the pointer
  // sram_master_ptr =(unsigned int *)(sram_master_base);
  lines_ptr = (char *)(sram_master_base + 0x20000); // Lines 
  hps_copy_sram2_ptr = (char *)(sram_master_base); // Hough Space

  wait_accum_ptr = (volatile bool *)(sram_master_base+0x40000);


  // ===========================================

  /* create a message to be displayed on the VGA 
          and LCD displays */
  char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
  char text_bottom_row[40] = "Cornell ece5760\0";
  char num_string[20], time_string[50] ;
  
  // a pixel from the video
  int pixel_color;
  // video input index
  int i,j;

  // HoughLinesStandard parameters

  std::vector<LinePolar> lines(5);

  float rho;
  rho = 1.0;

  float theta;
  theta = PI/180.0;

  int threshold;
  threshold = 1;

  int linesMax;
  linesMax = lines.size();

  double min_theta;
  min_theta = 0.0;

  double max_theta;
  max_theta = PI;

  char *color;

  char sw_m10k[76800];

  char black[76800];
  // Init Black
  for(int i = 0; i < 76800; i++){
    black[i] = 0;
  }


  int accum[(179 + 2) * (561 + 2)];
  int rho_count = 561;
  int rho_n;
  int theta_n;
  char* accum_val;

  *wait_accum_ptr = 0;	

  while(1) 
  {

    // Init Black
    for(int i = 0; i < 76800; i++){
      black[i] = 0;
    }

    // Time full Computation
    gettimeofday(&t1, NULL);

    // Get edge-detected video copy from FPGA sram
    //ALSO rho is x axis and y is theta, not other way around
  
    for(theta_n = 0; theta_n < 180; ++theta_n){
      for(rho_n = 0; rho_n < rho_count; ++ rho_n ){
        accum_val = (char*)(hps_copy_sram2_ptr + ((theta_n + 1) * (rho_count + 2)) + (rho_n + 1));
        accum[((theta_n + 1) * (rho_count + 2)) + (	rho_n + 1)] = *accum_val;
      }
      // printf("\n");
    }

    HoughLinesStandard(lines, rho, theta, threshold, linesMax, min_theta, max_theta, accum );
    
    //zero after houghlinesstandard call
    *wait_accum_ptr = 1;
    for(theta_n = 0; theta_n < 180; ++theta_n){
      for(rho_n = 0; rho_n < rho_count; ++ rho_n ){
        *(hps_copy_sram2_ptr + ((theta_n + 1) * (rho_count + 2)) + (rho_n + 1)) = 0;
      }
    }
    *wait_accum_ptr = 0;


    
    // Draw the lines calculated in lines vector from above
    for( size_t i = 0; i < lines.size(); i++ ) {
      float rho_out = lines[i].rho, theta = lines[i].angle;
      Point pt1, pt2;
      double a = cos(theta), b = sin(theta);


      double m = -(a/b);
      double y_int = rho_out/b; 
      
      coords(m, y_int, &pt1.x, &pt2.x, &pt1.y, &pt2.y);
      line(black, pt1, pt2);
      }

    int count = 0;
    for(j = 0; j < 240; ++j){ //theta
      for(i = 0; i < 320; ++i){ //rho
        *(lines_ptr + (i) + (j*320)) = black[i+(j*320)];
      }
    }
    
    // stop timer
     gettimeofday(&t2, NULL);
     elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
     elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
    
  } // end while(1)
} // end main

```

# Verilog
## Accumulator Module

```v
//Takes in an x and y, rho and theta information (tbd what exactly)
// and outputs (or puts into a memory directly)
module accumulator #(
    parameter WIDTH = 10
  )
  (
    input reset, 
    input clk, 

    input [7:0]  THETA_COUNT,
    input [10:0] RHO_COUNT,
    input [8:0]  x_in,
    input [8:0]  y_in,

    input  [9:0]  prev_rho_theta_read_in, //for our pipeline, we expect this to be the data of the previous rho_out_read and theta_out_read

    //these correspond to memory locations in our accumulator grid.
    //log2(1121 * 180) = 18 bits to address
    output reg [17:0]  curr_addr_out, //up to 180, maybe more, so 8 bits gives us 256 max
    output reg [9:0]   accumulator_write_out_data, //assumed we don't go above 400 ever, so 
    output reg         mem_write_en,
    
    output wire   done
  );


  ///////////////////////////////////////
  /// STATUS SIGNALS & STATE VARIABLES
  ///////////////////////////////////////
  reg [17:0] curr_rho; // Current rho we're on. Set as state machine output.
  reg  [7:0]  curr_theta; //in degrees/radians, need to check which
  reg  [7:0]  curr_theta_n; // i.e 0-180
  wire [17:0]  theta_incr;


  //TODO: Figure out representation of theta to assign this based on THETA_COUNT
  assign theta_incr = 18'd0;

  reg [1:0] accumulator_state;



  ///////////////////////////////////////
  /// DATAPATH
  ///////////////////////////////////////

//TODO: figure out representation of all sin tables
wire [17:0] sin_out;
wire [17:0] cos_out;

reg  [7:0] trig_table_in;

sine_table sin(clk, trig_table_in, sin_out);
//TODO: make a cos_table
cosine_table cos(clk, trig_table_in, cos_out);

reg [17:0] x_in_18;
reg [17:0] y_in_18;
wire [17:0] horz_rho_comp;
wire [17:0] vert_rho_comp;

signed_mult sin_mult(
  .a(y_in_18),
  .b(sin_out),
  .out(horz_rho_comp)
);

signed_mult cos_mult(
  .a(x_in_18),
  .b(cos_out),
  .out(vert_rho_comp)
);

///////////////////////////////////////
  /// STATE MACHINE
  //////////////////////////////////////


  //Idea is that we go through THETA_COUNT cycles then we are done

  localparam STATE_INIT = 2'd0,
             STATE_TRAVERSE_READ = 2'd1,
             STATE_TRAVERSE_WRITE = 2'd2,
             STATE_DONE= 2'd3;

  assign done = accumulator_state == STATE_DONE;

  // STATE UPDATES
  always@(posedge clk) begin 
      if(reset) begin
          accumulator_state <= STATE_INIT;
      end else begin
          case (accumulator_state)
              STATE_INIT: begin // TODO: Update to accomodate updating the MK10 data
                  accumulator_state <= STATE_TRAVERSE_READ;
              end
              STATE_TRAVERSE_READ: begin 
                //TODO: Is this ok? May need to doa  signed check
                accumulator_state <= STATE_TRAVERSE_WRITE;
              end
              STATE_TRAVERSE_WRITE: begin
                if(curr_theta_n >= THETA_COUNT) begin
                  accumulator_state <= STATE_DONE;
                end else begin
                  accumulator_state <= STATE_TRAVERSE_READ;
                end
              end
              STATE_DONE: begin
                accumulator_state <= STATE_DONE;
              end
              default:
                accumulator_state <= accumulator_state;
          endcase
      end
  end


 // CONTROL SIGNALS REGS
  always@(posedge clk) begin
      case (accumulator_state) 
        STATE_INIT: begin
          curr_theta_n <= 8'b0;
          curr_theta <= 8'b0;
          trig_table_in <= 8'b0;

          x_in_18 <= x_in << 7;
          y_in_18 <= y_in << 7;

        end
        STATE_TRAVERSE_READ: begin
          // curr_theta <= theta_incr; //should be equal to curr_theta_n * theta_incr
          trig_table_in <= trig_table_in + 8'd1;
          curr_theta_n <= curr_theta_n;
          x_in_18 <= x_in_18;
          y_in_18 <= y_in_18;

        end
        STATE_TRAVERSE_WRITE: begin
          trig_table_in <= trig_table_in;
          curr_theta_n <= curr_theta_n + 8'b1;
          x_in_18 <= x_in_18;
          y_in_18 <= y_in_18;

        end
        STATE_DONE: begin
        end
      endcase
  end

  // CONTROL SIGNALS CONTINUOUS
  always@(*) begin
    //increments accumulator, always set, Rely on mem_write_en to write or not

    //these 2 lines are the address translator. See line 171 hough.cpp
    //Depending on format of sin need to round to a whole number Lines 159 and 168 in hough.cpp
    //curr_rho should be though of as an int

    // 3 is functionally incorrect. 2 works better than 3
    // 1 is just 2 but with no truncation warnings

    // THIS IS 1
    curr_rho = ((horz_rho_comp + vert_rho_comp) >> 7) + (({7'd0, RHO_COUNT} - 18'd1) >> 1);
    curr_addr_out = ((({10'd0, curr_theta_n} + 18'd1) * ({7'd0, RHO_COUNT} + 18'd2)) + (curr_rho) + 18'd1);

    // THIS IS 2
    // curr_rho = ((horz_rho_comp + vert_rho_comp) >> 7) + ((RHO_COUNT - 11'd1) >> 1);
    // curr_addr_out = (((curr_theta_n + 8'd1) * (RHO_COUNT + 11'd2)) + (curr_rho) + 1);

    // THIS IS 3
    // curr_rho = ((horz_rho_comp + vert_rho_comp) >> 7) + ((RHO_COUNT - 11'd1));
    // curr_addr_out = (((curr_theta_n + 8'd1) * (RHO_COUNT + 11'd2)) + (curr_rho >> 1) + 1);
      case (accumulator_state) 
        STATE_INIT: begin
          mem_write_en = 1'b0;
          accumulator_write_out_data = 10'dx; 
        end
        STATE_TRAVERSE_READ: begin
          mem_write_en = 1'b0;
          accumulator_write_out_data = 10'dx; 
        end
        STATE_TRAVERSE_WRITE: begin
          //just got last read
          accumulator_write_out_data = prev_rho_theta_read_in + 10'b1; 
          mem_write_en = 1'b1;
        end
        STATE_DONE: begin
          mem_write_en = 1'b0;
          accumulator_write_out_data = 10'dx; 
        end
      endcase
  end


endmodule








//////////////////////////////////////////////////
////////////	Sin Wave ROM Table	//////////////
//////////////////////////////////////////////////
// produces a 2's comp, 16-bit, approximation
// of a sine wave, given an input phase (address)

//This takes in a degree between 0-180 and outputs
//a sin value in 1.8 fixed point (unsigned)
module sine_table (clock, address, sine);
input clock;
input [7:0] address;
output [17:0] sine;
reg signed [17:0] sine;
always@(posedge clock)
begin
    case(address)
8'd0: sine = 18'h0;
8'd1: sine = 18'h2;
8'd2: sine = 18'h4;
8'd3: sine = 18'h7;
8'd4: sine = 18'h9;
8'd5: sine = 18'hb;
8'd6: sine = 18'hd;
8'd7: sine = 18'h10;
8'd8: sine = 18'h12;
8'd9: sine = 18'h14;
8'd10: sine = 18'h16;
8'd11: sine = 18'h18;
8'd12: sine = 18'h1b;
8'd13: sine = 18'h1d;
8'd14: sine = 18'h1f;
8'd15: sine = 18'h21;
8'd16: sine = 18'h23;
8'd17: sine = 18'h25;
8'd18: sine = 18'h28;
8'd19: sine = 18'h2a;
8'd20: sine = 18'h2c;
8'd21: sine = 18'h2e;
8'd22: sine = 18'h30;
8'd23: sine = 18'h32;
8'd24: sine = 18'h34;
8'd25: sine = 18'h36;
8'd26: sine = 18'h38;
8'd27: sine = 18'h3a;
8'd28: sine = 18'h3c;
8'd29: sine = 18'h3e;
8'd30: sine = 18'h40;
8'd31: sine = 18'h42;
8'd32: sine = 18'h44;
8'd33: sine = 18'h46;
8'd34: sine = 18'h48;
8'd35: sine = 18'h49;
8'd36: sine = 18'h4b;
8'd37: sine = 18'h4d;
8'd38: sine = 18'h4f;
8'd39: sine = 18'h51;
8'd40: sine = 18'h52;
8'd41: sine = 18'h54;
8'd42: sine = 18'h56;
8'd43: sine = 18'h57;
8'd44: sine = 18'h59;
8'd45: sine = 18'h5b;
8'd46: sine = 18'h5c;
8'd47: sine = 18'h5e;
8'd48: sine = 18'h5f;
8'd49: sine = 18'h61;
8'd50: sine = 18'h62;
8'd51: sine = 18'h63;
8'd52: sine = 18'h65;
8'd53: sine = 18'h66;
8'd54: sine = 18'h68;
8'd55: sine = 18'h69;
8'd56: sine = 18'h6a;
8'd57: sine = 18'h6b;
8'd58: sine = 18'h6d;
8'd59: sine = 18'h6e;
8'd60: sine = 18'h6f;
8'd61: sine = 18'h70;
8'd62: sine = 18'h71;
8'd63: sine = 18'h72;
8'd64: sine = 18'h73;
8'd65: sine = 18'h74;
8'd66: sine = 18'h75;
8'd67: sine = 18'h76;
8'd68: sine = 18'h77;
8'd69: sine = 18'h77;
8'd70: sine = 18'h78;
8'd71: sine = 18'h79;
8'd72: sine = 18'h7a;
8'd73: sine = 18'h7a;
8'd74: sine = 18'h7b;
8'd75: sine = 18'h7c;
8'd76: sine = 18'h7c;
8'd77: sine = 18'h7d;
8'd78: sine = 18'h7d;
8'd79: sine = 18'h7e;
8'd80: sine = 18'h7e;
8'd81: sine = 18'h7e;
8'd82: sine = 18'h7f;
8'd83: sine = 18'h7f;
8'd84: sine = 18'h7f;
8'd85: sine = 18'h80;
8'd86: sine = 18'h80;
8'd87: sine = 18'h80;
8'd88: sine = 18'h80;
8'd89: sine = 18'h80;
8'd90: sine = 18'h80;
8'd91: sine = 18'h80;
8'd92: sine = 18'h80;
8'd93: sine = 18'h80;
8'd94: sine = 18'h80;
8'd95: sine = 18'h80;
8'd96: sine = 18'h7f;
8'd97: sine = 18'h7f;
8'd98: sine = 18'h7f;
8'd99: sine = 18'h7e;
8'd100: sine = 18'h7e;
8'd101: sine = 18'h7e;
8'd102: sine = 18'h7d;
8'd103: sine = 18'h7d;
8'd104: sine = 18'h7c;
8'd105: sine = 18'h7c;
8'd106: sine = 18'h7b;
8'd107: sine = 18'h7a;
8'd108: sine = 18'h7a;
8'd109: sine = 18'h79;
8'd110: sine = 18'h78;
8'd111: sine = 18'h77;
8'd112: sine = 18'h77;
8'd113: sine = 18'h76;
8'd114: sine = 18'h75;
8'd115: sine = 18'h74;
8'd116: sine = 18'h73;
8'd117: sine = 18'h72;
8'd118: sine = 18'h71;
8'd119: sine = 18'h70;
8'd120: sine = 18'h6f;
8'd121: sine = 18'h6e;
8'd122: sine = 18'h6d;
8'd123: sine = 18'h6b;
8'd124: sine = 18'h6a;
8'd125: sine = 18'h69;
8'd126: sine = 18'h68;
8'd127: sine = 18'h66;
8'd128: sine = 18'h65;
8'd129: sine = 18'h63;
8'd130: sine = 18'h62;
8'd131: sine = 18'h61;
8'd132: sine = 18'h5f;
8'd133: sine = 18'h5e;
8'd134: sine = 18'h5c;
8'd135: sine = 18'h5b;
8'd136: sine = 18'h59;
8'd137: sine = 18'h57;
8'd138: sine = 18'h56;
8'd139: sine = 18'h54;
8'd140: sine = 18'h52;
8'd141: sine = 18'h51;
8'd142: sine = 18'h4f;
8'd143: sine = 18'h4d;
8'd144: sine = 18'h4b;
8'd145: sine = 18'h49;
8'd146: sine = 18'h48;
8'd147: sine = 18'h46;
8'd148: sine = 18'h44;
8'd149: sine = 18'h42;
8'd150: sine = 18'h40;
8'd151: sine = 18'h3e;
8'd152: sine = 18'h3c;
8'd153: sine = 18'h3a;
8'd154: sine = 18'h38;
8'd155: sine = 18'h36;
8'd156: sine = 18'h34;
8'd157: sine = 18'h32;
8'd158: sine = 18'h30;
8'd159: sine = 18'h2e;
8'd160: sine = 18'h2c;
8'd161: sine = 18'h2a;
8'd162: sine = 18'h28;
8'd163: sine = 18'h25;
8'd164: sine = 18'h23;
8'd165: sine = 18'h21;
8'd166: sine = 18'h1f;
8'd167: sine = 18'h1d;
8'd168: sine = 18'h1b;
8'd169: sine = 18'h18;
8'd170: sine = 18'h16;
8'd171: sine = 18'h14;
8'd172: sine = 18'h12;
8'd173: sine = 18'h10;
8'd174: sine = 18'hd;
8'd175: sine = 18'hb;
8'd176: sine = 18'h9;
8'd177: sine = 18'h7;
8'd178: sine = 18'h4;
8'd179: sine = 18'h2;
  endcase
end
endmodule



module cosine_table (clock, address, cosine);
input clock;
input [7:0] address;
output [17:0] cosine;
reg signed [17:0] cosine;
always@(posedge clock)
begin
    case(address)
 8'd0: cosine = 18'h80;
8'd1: cosine = 18'h80;
8'd2: cosine = 18'h80;
8'd3: cosine = 18'h80;
8'd4: cosine = 18'h80;
8'd5: cosine = 18'h80;
8'd6: cosine = 18'h7f;
8'd7: cosine = 18'h7f;
8'd8: cosine = 18'h7f;
8'd9: cosine = 18'h7e;
8'd10: cosine = 18'h7e;
8'd11: cosine = 18'h7e;
8'd12: cosine = 18'h7d;
8'd13: cosine = 18'h7d;
8'd14: cosine = 18'h7c;
8'd15: cosine = 18'h7c;
8'd16: cosine = 18'h7b;
8'd17: cosine = 18'h7a;
8'd18: cosine = 18'h7a;
8'd19: cosine = 18'h79;
8'd20: cosine = 18'h78;
8'd21: cosine = 18'h77;
8'd22: cosine = 18'h77;
8'd23: cosine = 18'h76;
8'd24: cosine = 18'h75;
8'd25: cosine = 18'h74;
8'd26: cosine = 18'h73;
8'd27: cosine = 18'h72;
8'd28: cosine = 18'h71;
8'd29: cosine = 18'h70;
8'd30: cosine = 18'h6f;
8'd31: cosine = 18'h6e;
8'd32: cosine = 18'h6d;
8'd33: cosine = 18'h6b;
8'd34: cosine = 18'h6a;
8'd35: cosine = 18'h69;
8'd36: cosine = 18'h68;
8'd37: cosine = 18'h66;
8'd38: cosine = 18'h65;
8'd39: cosine = 18'h63;
8'd40: cosine = 18'h62;
8'd41: cosine = 18'h61;
8'd42: cosine = 18'h5f;
8'd43: cosine = 18'h5e;
8'd44: cosine = 18'h5c;
8'd45: cosine = 18'h5b;
8'd46: cosine = 18'h59;
8'd47: cosine = 18'h57;
8'd48: cosine = 18'h56;
8'd49: cosine = 18'h54;
8'd50: cosine = 18'h52;
8'd51: cosine = 18'h51;
8'd52: cosine = 18'h4f;
8'd53: cosine = 18'h4d;
8'd54: cosine = 18'h4b;
8'd55: cosine = 18'h49;
8'd56: cosine = 18'h48;
8'd57: cosine = 18'h46;
8'd58: cosine = 18'h44;
8'd59: cosine = 18'h42;
8'd60: cosine = 18'h40;
8'd61: cosine = 18'h3e;
8'd62: cosine = 18'h3c;
8'd63: cosine = 18'h3a;
8'd64: cosine = 18'h38;
8'd65: cosine = 18'h36;
8'd66: cosine = 18'h34;
8'd67: cosine = 18'h32;
8'd68: cosine = 18'h30;
8'd69: cosine = 18'h2e;
8'd70: cosine = 18'h2c;
8'd71: cosine = 18'h2a;
8'd72: cosine = 18'h28;
8'd73: cosine = 18'h25;
8'd74: cosine = 18'h23;
8'd75: cosine = 18'h21;
8'd76: cosine = 18'h1f;
8'd77: cosine = 18'h1d;
8'd78: cosine = 18'h1b;
8'd79: cosine = 18'h18;
8'd80: cosine = 18'h16;
8'd81: cosine = 18'h14;
8'd82: cosine = 18'h12;
8'd83: cosine = 18'h10;
8'd84: cosine = 18'hd;
8'd85: cosine = 18'hb;
8'd86: cosine = 18'h9;
8'd87: cosine = 18'h7;
8'd88: cosine = 18'h4;
8'd89: cosine = 18'h2;
8'd90: cosine = 18'h0;
8'd91: cosine = 18'h3fffe;
8'd92: cosine = 18'h3fffc;
8'd93: cosine = 18'h3fff9;
8'd94: cosine = 18'h3fff7;
8'd95: cosine = 18'h3fff5;
8'd96: cosine = 18'h3fff3;
8'd97: cosine = 18'h3fff0;
8'd98: cosine = 18'h3ffee;
8'd99: cosine = 18'h3ffec;
8'd100: cosine = 18'h3ffea;
8'd101: cosine = 18'h3ffe8;
8'd102: cosine = 18'h3ffe5;
8'd103: cosine = 18'h3ffe3;
8'd104: cosine = 18'h3ffe1;
8'd105: cosine = 18'h3ffdf;
8'd106: cosine = 18'h3ffdd;
8'd107: cosine = 18'h3ffdb;
8'd108: cosine = 18'h3ffd8;
8'd109: cosine = 18'h3ffd6;
8'd110: cosine = 18'h3ffd4;
8'd111: cosine = 18'h3ffd2;
8'd112: cosine = 18'h3ffd0;
8'd113: cosine = 18'h3ffce;
8'd114: cosine = 18'h3ffcc;
8'd115: cosine = 18'h3ffca;
8'd116: cosine = 18'h3ffc8;
8'd117: cosine = 18'h3ffc6;
8'd118: cosine = 18'h3ffc4;
8'd119: cosine = 18'h3ffc2;
8'd120: cosine = 18'h3ffc0;
8'd121: cosine = 18'h3ffbe;
8'd122: cosine = 18'h3ffbc;
8'd123: cosine = 18'h3ffba;
8'd124: cosine = 18'h3ffb8;
8'd125: cosine = 18'h3ffb7;
8'd126: cosine = 18'h3ffb5;
8'd127: cosine = 18'h3ffb3;
8'd128: cosine = 18'h3ffb1;
8'd129: cosine = 18'h3ffaf;
8'd130: cosine = 18'h3ffae;
8'd131: cosine = 18'h3ffac;
8'd132: cosine = 18'h3ffaa;
8'd133: cosine = 18'h3ffa9;
8'd134: cosine = 18'h3ffa7;
8'd135: cosine = 18'h3ffa5;
8'd136: cosine = 18'h3ffa4;
8'd137: cosine = 18'h3ffa2;
8'd138: cosine = 18'h3ffa1;
8'd139: cosine = 18'h3ff9f;
8'd140: cosine = 18'h3ff9e;
8'd141: cosine = 18'h3ff9d;
8'd142: cosine = 18'h3ff9b;
8'd143: cosine = 18'h3ff9a;
8'd144: cosine = 18'h3ff98;
8'd145: cosine = 18'h3ff97;
8'd146: cosine = 18'h3ff96;
8'd147: cosine = 18'h3ff95;
8'd148: cosine = 18'h3ff93;
8'd149: cosine = 18'h3ff92;
8'd150: cosine = 18'h3ff91;
8'd151: cosine = 18'h3ff90;
8'd152: cosine = 18'h3ff8f;
8'd153: cosine = 18'h3ff8e;
8'd154: cosine = 18'h3ff8d;
8'd155: cosine = 18'h3ff8c;
8'd156: cosine = 18'h3ff8b;
8'd157: cosine = 18'h3ff8a;
8'd158: cosine = 18'h3ff89;
8'd159: cosine = 18'h3ff89;
8'd160: cosine = 18'h3ff88;
8'd161: cosine = 18'h3ff87;
8'd162: cosine = 18'h3ff86;
8'd163: cosine = 18'h3ff86;
8'd164: cosine = 18'h3ff85;
8'd165: cosine = 18'h3ff84;
8'd166: cosine = 18'h3ff84;
8'd167: cosine = 18'h3ff83;
8'd168: cosine = 18'h3ff83;
8'd169: cosine = 18'h3ff82;
8'd170: cosine = 18'h3ff82;
8'd171: cosine = 18'h3ff82;
8'd172: cosine = 18'h3ff81;
8'd173: cosine = 18'h3ff81;
8'd174: cosine = 18'h3ff81;
8'd175: cosine = 18'h3ff80;
8'd176: cosine = 18'h3ff80;
8'd177: cosine = 18'h3ff80;
8'd178: cosine = 18'h3ff80;
8'd179: cosine = 18'h3ff80;
  endcase
end
endmodule


module signed_mult (out, a, b);
  output 	signed  [17:0]	out;
  input 	signed	[17:0] 	a;
  input 	signed	[17:0] 	b;
  // intermediate full bit length
  wire 	signed	[35:0]	mult_out;
  assign mult_out = a * b;
  // select bits for 11.7 fixed point
  //in our mult we have 14 fractional bits and 22 integer bits
  // the decimal lands between 13 and 14 (0-indexed).
  assign out = {mult_out[35], mult_out[23:7]};
endmodule
```

## Dispatcher Module

```v
module not_zero 
  (
    
    input clk,
    input reset,
    input [7:0] color,
    input acc_done,

    output [8:0] sram_x_out,
    output [8:0] sram_y_out,
    output go,
    output [8:0] acc_x_out,
    output [8:0] acc_y_out,

    input wait_sig
    
  );

  wire valid_pixel;
  assign valid_pixel = |color;

  //=======================================================
  //  FSM
  //=======================================================

  reg [8:0] prev_x_cood;
  reg [8:0] prev_y_cood;
  reg [8:0] curr_x_cood;
  reg [8:0] curr_y_cood;
  reg [16:0] prev_address;
  reg [16:0] curr_address;
  reg go_reg;

  localparam INIT = 2'd0,
             READY = 2'd1,
             WAITING = 2'd2;
          

  reg [1:0] zero_state;

  assign go = go_reg;

  always@(posedge clk) begin 
      if(reset) begin
        zero_state <= INIT;
      end 
      else begin
        case (zero_state)

          INIT: begin
            
            go_reg <= 0;
            zero_state <= READY;

          end 
          READY: begin 

            if (valid_pixel && !wait_sig) begin
              zero_state <= WAITING;
              go_reg <= 1;
            end
            else begin
              zero_state <= READY;
              go_reg <= 0;
            end

          end
          WAITING: begin

            go_reg <= 0;

            if (acc_done) begin 
              zero_state <= READY;
            end

          end
          
          default:
            zero_state <= zero_state;
        endcase
      end
  end

  // assign sram_address =  {8'b0,curr_x_cood} + ({8'b0,curr_y_cood} * 320) ;

  assign acc_x_out = prev_x_cood;
  assign acc_y_out = prev_y_cood;
  assign sram_x_out = curr_x_cood;
  assign sram_y_out = curr_y_cood;

  always @(posedge clk) begin 

    

    case (zero_state) 

      INIT: begin

        curr_x_cood <= 9'd0;
        curr_y_cood <= 9'd0;

        prev_x_cood <= 0 ;
        prev_y_cood <= 0 ;
      end

      READY: begin

        prev_x_cood <= curr_x_cood;
        prev_y_cood <= curr_y_cood;
        if (wait_sig) begin
          curr_x_cood <= curr_x_cood;
          curr_y_cood <= curr_y_cood;
        end else begin
          if (curr_x_cood >= 9'd319) begin
            curr_x_cood <= 9'd0 ;
            if (curr_y_cood >= 9'd239) 
              curr_y_cood <= 9'd0 ;
            else 
              curr_y_cood <= curr_y_cood + 9'd1 ;
          end
          else 
            curr_x_cood <= curr_x_cood + 9'd1 ;

        end
      end
      WAITING: begin

        curr_x_cood <= curr_x_cood;
        curr_y_cood <= curr_y_cood;

        prev_x_cood <= prev_x_cood ;
        prev_y_cood <= prev_y_cood ;
        
      end

    endcase

  end

endmodule
```

## Toplevel Module

```v
// Generated code omitted...


//=======================================================
// Bus controller for AVALON bus-master
//=======================================================
reg [31:0] timer ;
reg [3:0] state ;

// PLLs
wire vga_pll_lock;
wire vga_pll;
wire M10k_pll_locked;
wire M10k_pll;

// Video In M10k
reg [7:0] video_in_read_data;

// VGA M10k that holds pixels directly from camera in
wire [16:0] sram1_read_addr;
wire [16:0] sram1_write_addr;
wire [9:0] vga_next_x;
wire [9:0] vga_next_y;

// pixel address is
reg [9:0] vga_x_cood, vga_y_cood, video_in_x_cood, video_in_y_cood ;
reg [7:0] current_pixel_color1, current_pixel_color2 ;
// compute address

assign sram1_read_addr  =  ({7'b0, vga_next_x}) + (({7'b0,vga_next_y})*17'd320) ;


//traverse over x and y coordinates
always @(posedge CLOCK_50) begin //M10k_pll

  // reset state machine and read/write controls
  if (~KEY[0]) begin
    state <= 0 ;
    video_in_x_cood <= 0 ;
    video_in_y_cood <= 0 ;

    timer <= 0;
  end
  else begin
    timer <= timer + 1;
  end
  
  // and put in a small delay to aviod bus hogging
  // timer delay can be set to 2**n-1, so 3, 7, 15, 31
  // bigger numbers mean slower frame update to VGA
  if (state==0 && SW[0] && (timer & 30)==0 ) begin //

    if (video_in_x_cood >= 10'd319) begin
      video_in_x_cood <= 10'd0 ;
      if (video_in_y_cood >= 10'd239) 
        video_in_y_cood <= 10'd0 ;
      else 
        video_in_y_cood <= video_in_y_cood + 10'd1 ;
    end
    else 
      video_in_x_cood <= video_in_x_cood + 10'd1 ;
  end
  
end // always @(posedge state_clock)



//=======================================================
//  HPS Data M10k (M10k_3) Stuff
//=======================================================

reg [16:0] hps_copy_sram_read_address;



  localparam STATE_ACC_INIT = 2'd0,
             STATE_ACC_TRAVERSE = 2'd1,
             STATE_ACC_DISPLAY= 2'd2; //loop through reading only
reg [1:0] acc_state;

reg [8:0] acc_sram_x_cood;
reg [8:0] acc_sram_y_cood;


//Acc test state transitions
always @(posedge CLOCK_50) begin
  if (~KEY[0]) begin
    acc_state <= STATE_ACC_INIT;
  end else begin
    case (acc_state)
      STATE_ACC_INIT: begin
        acc_state <= STATE_ACC_TRAVERSE;
      end
      STATE_ACC_TRAVERSE: begin
        // acc_state <= (acc_done && acc_x_coord >= 9'd319) ? STATE_ACC_DISPLAY : STATE_ACC_TRAVERSE;
        acc_state <= (acc_done) ? STATE_ACC_DISPLAY : STATE_ACC_TRAVERSE;

      end
      STATE_ACC_DISPLAY: begin
        acc_state <= STATE_ACC_DISPLAY;
      end
    endcase
  end
end

assign acc_sram_read_addr = {8'b0, acc_sram_x_cood}  + ({8'b0,acc_sram_y_cood} * 17'd320) ;

 //acc test state reg updates
always @(posedge CLOCK_50) begin
  case (acc_state)
    STATE_ACC_INIT: begin
      acc_sram_x_cood <= 0;
      acc_sram_y_cood <= 0;
      // acc_x_coord <= 9'd0;
      // acc_y_coord <= 9'd50;
      // acc_reset <= 1'b0;
    end
    STATE_ACC_TRAVERSE: begin
      acc_sram_x_cood <= acc_sram_x_cood;
      acc_sram_y_cood <= acc_sram_y_cood;
      // acc_reset <= acc_done; //TODO: Make sure this isnt staying high for 2 cycles
      // acc_x_coord <= acc_done ? acc_x_coord + 9'd1 : acc_x_coord;
      // acc_y_coord <= 9'd50;
    end
    STATE_ACC_DISPLAY: begin
      if (acc_sram_x_cood >= 9'd319) begin
        acc_sram_x_cood <= 9'd0 ;
        if (acc_sram_y_cood >= 9'd239) 
          acc_sram_y_cood <= 9'd0 ;
        else 
          acc_sram_y_cood <= acc_sram_y_cood + 9'd1 ;
      end
      else 
        acc_sram_x_cood <= acc_sram_x_cood + 9'd1 ;
      // acc_reset <= 1'b0;
      // acc_x_coord <= acc_x_coord;
      // acc_y_coord <= acc_y_coord;
    end
  endcase
end

reg [7:0] force_color;



//=======================================================
//  VIDEO M10K Stuff
//=======================================================
M10K_76800_8 video_m10k( //8 bit colors with 240 rows * 320 pixels/row
  .clk(CLOCK_50),
  .read_address(src_copy_read_addr),
    .out(not_zero_color),
  .we(1'b1),
  .write_address(sram1_read_addr), //this sounds backwards but fine, connects Qsys camera-in sram to verilog module
    .in(no_top_line)
);

wire [7:0] no_top_line;
assign no_top_line = (vga_next_y == 8 || vga_next_y == 9 || vga_next_y == 10) ? 8'b000_000_00 : video_in_read_data;

//=======================================================
//  NOT ZERO Stuff
//=======================================================
wire [17:0] acc_address;

wire [8:0] sram_x_in;
wire [8:0] sram_y_in;

wire [16:0] src_copy_read_addr;

assign src_copy_read_addr = {8'b0,sram_x_in} + ({8'b0,sram_y_in} * 17'd320) ;

wire [7:0] not_zero_color;

wire wait_dispatch;

assign LEDR[3] = wait_dispatch;

not_zero dispatch
  (
    .clk(CLOCK_50),
    .reset(~KEY[0]),
    .color(not_zero_color),
    .acc_done(acc_done),
    .go(acc_reset),
  .acc_x_out(acc_x_in),
  .acc_y_out(acc_y_in),
  .sram_x_out(sram_x_in),
  .sram_y_out(sram_y_in),
  .wait_sig(wait_dispatch)
  );

//=======================================================
//  VGA Driver Stuff
//=======================================================

// Val/rdy protocol

wire acc_reset;
wire [7:0] theta_count;
assign theta_count = 8'd179;
wire [10:0] rho_count;
assign rho_count = 11'd561;
wire [8:0] acc_x_in;
wire [8:0] acc_y_in;
wire [9:0]  acc_write_out_data;
wire acc_mem_write_en;
wire acc_done;

wire [16:0] acc_sram_read_addr;

accumulator acc(
  .reset(acc_reset),
  .clk(CLOCK_50),

  .THETA_COUNT(theta_count),
  .RHO_COUNT(rho_count),

  .x_in(acc_x_in),
  .y_in(acc_y_in),
  .prev_rho_theta_read_in(acc_read_data),
  .curr_addr_out(acc_address),
  .accumulator_write_out_data(acc_write_out_data),
  .mem_write_en(acc_mem_write_en),
  .done(acc_done)
);


wire [9:0] acc_read_data;
wire [7:0] lines_read_data;
// reg [7:0] second_sram_out;

wire my_clk;
assign my_clk = SW[5] ? CLOCK_50 : vga_pll;

reg [7:0] vga_data;
// assign vga_data = (vga_next_x <= 320 && vga_next_y <= 240) ? m10k_out : 8'b000_000_00;
always @(*) begin

  if (vga_next_x <= 320 && vga_next_y <= 240) begin // top left
    vga_data = SW[6] ? 8'b111111_11 : (SW[8] ? (video_in_read_data) : (video_in_read_data + lines_read_data));
  end

  else if (vga_next_x >= 320 && vga_next_y <= 240) begin // top right
      vga_data = SW[7] ?  lines_read_data : (video_in_read_data + lines_read_data);
  end


  else begin
    vga_data = 8'b000_000_00;
  end
  
end

reg vga_reset;

always @(posedge my_clk)begin
  if (~KEY[0]) begin
    vga_reset <= 1;
  end else begin
    vga_reset <= 0;
  end
end
          
vga_driver DUT   (	.clock(my_clk), 
          .reset(vga_reset),
          .color_in(SW[4] ? 8'b000_111_00 : vga_data),	// Pixel color (8-bit) from memory
          // .color_in(8'b000_111_00),	// Pixel color (8-bit) from memory
          .next_x(vga_next_x),		// This (and next_y) used to specify memory read address
          .next_y(vga_next_y),		// This (and next_x) used to specify memory read address
          .hsync(VGA_HS),
          .vsync(VGA_VS),
          .red(VGA_R),
          .green(VGA_G),
          .blue(VGA_B),
          .sync(VGA_SYNC_N),
          .clk(VGA_CLK),
          .blank(VGA_BLANK_N)
);

//=======================================================
//  Structural coding
//=======================================================

Computer_System The_System (

  // Video In M10k SRAM 1
  .onchip_sram_s2_address          (sram1_read_addr),
  .onchip_sram_s2_chipselect       (1'b1),
  .onchip_sram_s2_clken            (1'b1),
  .onchip_sram_s2_write            (1'b0),
  .onchip_sram_s2_readdata         (video_in_read_data),
  .onchip_sram_s2_writedata        (),

    //HOUGH SPACE SRAM
  //2nd sram is what we want to output in the upper right corner of the vga
  .hps_to_onchip_sram_s2_address    (acc_address), 
  .hps_to_onchip_sram_s2_write      (acc_mem_write_en),
  .hps_to_onchip_sram_s2_readdata   (acc_read_data),  
  .hps_to_onchip_sram_s2_writedata  (acc_write_out_data),   
  .hps_to_onchip_sram_s2_chipselect (1'b1),    
  .hps_to_onchip_sram_s2_clken      (1'b1),


     // Just lines (ie black[])
  .onchip_camera_in_to_onchip_copy_0_s2_address     (sram1_read_addr),
  .onchip_camera_in_to_onchip_copy_0_s2_chipselect  (1'b1),    
  .onchip_camera_in_to_onchip_copy_0_s2_clken       (1'b1),
  .onchip_camera_in_to_onchip_copy_0_s2_write       (1'b0),
  .onchip_camera_in_to_onchip_copy_0_s2_readdata    (lines_read_data),  
  .onchip_camera_in_to_onchip_copy_0_s2_writedata   (),   

  .wait_signal_dispatcher_export(wait_dispatch),

  // PLLs
  .vga_pll_locked_export			(vga_pll_lock),           //       vga_pio_locked.export
  .vga_pll_outclk0_clk				(vga_pll),              //      vga_pio_outclk0.clk
  // .m10k_pll_locked_export			(M10k_pll_locked),          //      m10k_pll_locked.export
  // .m10k_pll_outclk0_clk			(M10k_pll),            //     m10k_pll_outclk0.clk

  // HPS data pio ports

// More generated code omitted.

endmodule

module vga_driver (
    input wire clock,     // 25 MHz
    input wire reset,     // Active high
    input [7:0] color_in, // Pixel color data (RRRGGGBB)
    output [9:0] next_x,  // x-coordinate of NEXT pixel that will be drawn
    output [9:0] next_y,  // y-coordinate of NEXT pixel that will be drawn
    output wire hsync,    // HSYNC (to VGA connector)
    output wire vsync,    // VSYNC (to VGA connctor)
    output [7:0] red,     // RED (to resistor DAC VGA connector)
    output [7:0] green,   // GREEN (to resistor DAC to VGA connector)
    output [7:0] blue,    // BLUE (to resistor DAC to VGA connector)
    output sync,          // SYNC to VGA connector
    output clk,           // CLK to VGA connector
    output blank          // BLANK to VGA connector
);

    // Horizontal parameters (measured in clock cycles)
    parameter [9:0] H_ACTIVE  =  10'd_639 ;
    parameter [9:0] H_FRONT   =  10'd_15 ;
    parameter [9:0] H_PULSE   =  10'd_95 ;
    parameter [9:0] H_BACK    =  10'd_47 ;

    // Vertical parameters (measured in lines)
    parameter [9:0] V_ACTIVE   =  10'd_479 ;
    parameter [9:0] V_FRONT    =  10'd_9 ;
    parameter [9:0] V_PULSE =  10'd_1 ;
    parameter [9:0] V_BACK  =  10'd_32 ;

    // Parameters for readability
    parameter   LOW     = 1'b_0 ;
    parameter   HIGH    = 1'b_1 ;

    // States (more readable)
    parameter   [7:0]    H_ACTIVE_STATE    = 8'd_0 ;
    parameter   [7:0]   H_FRONT_STATE     = 8'd_1 ;
    parameter   [7:0]   H_PULSE_STATE   = 8'd_2 ;
    parameter   [7:0]   H_BACK_STATE     = 8'd_3 ;

    parameter   [7:0]    V_ACTIVE_STATE    = 8'd_0 ;
    parameter   [7:0]   V_FRONT_STATE    = 8'd_1 ;
    parameter   [7:0]   V_PULSE_STATE   = 8'd_2 ;
    parameter   [7:0]   V_BACK_STATE     = 8'd_3 ;

    // Clocked registers
    reg              hysnc_reg ;
    reg              vsync_reg ;
    reg     [7:0]   red_reg ;
    reg     [7:0]   green_reg ;
    reg     [7:0]   blue_reg ;
    reg              line_done ;

    // Control registers
    reg     [9:0]   h_counter ;
    reg     [9:0]   v_counter ;

    reg     [7:0]    h_state ;
    reg     [7:0]    v_state ;

    // State machine
    always@(posedge clock) begin
        // At reset . . .
        if (reset) begin
            // Zero the counters
            h_counter   <= 10'd_0 ;
            v_counter   <= 10'd_0 ;
            // States to ACTIVE
            h_state     <= H_ACTIVE_STATE  ;
            v_state     <= V_ACTIVE_STATE  ;
            // Deassert line done
            line_done   <= LOW ;
        end
        else begin
            //////////////////////////////////////////////////////////////////////////
            ///////////////////////// HORIZONTAL /////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////
            if (h_state == H_ACTIVE_STATE) begin
                // Iterate horizontal counter, zero at end of ACTIVE mode
                h_counter <= (h_counter==H_ACTIVE)?10'd_0:(h_counter + 10'd_1) ;
                // Set hsync
                hysnc_reg <= HIGH ;
                // Deassert line done
                line_done <= LOW ;
                // State transition
                h_state <= (h_counter == H_ACTIVE)?H_FRONT_STATE:H_ACTIVE_STATE ;
            end
            if (h_state == H_FRONT_STATE) begin
                // Iterate horizontal counter, zero at end of H_FRONT mode
                h_counter <= (h_counter==H_FRONT)?10'd_0:(h_counter + 10'd_1) ;
                // Set hsync
                hysnc_reg <= HIGH ;
                // State transition
                h_state <= (h_counter == H_FRONT)?H_PULSE_STATE:H_FRONT_STATE ;
            end
            if (h_state == H_PULSE_STATE) begin
                // Iterate horizontal counter, zero at end of H_PULSE mode
                h_counter <= (h_counter==H_PULSE)?10'd_0:(h_counter + 10'd_1) ;
                // Clear hsync
                hysnc_reg <= LOW ;
                // State transition
                h_state <= (h_counter == H_PULSE)?H_BACK_STATE:H_PULSE_STATE ;
            end
            if (h_state == H_BACK_STATE) begin
                // Iterate horizontal counter, zero at end of H_BACK mode
                h_counter <= (h_counter==H_BACK)?10'd_0:(h_counter + 10'd_1) ;
                // Set hsync
                hysnc_reg <= HIGH ;
                // State transition
                h_state <= (h_counter == H_BACK)?H_ACTIVE_STATE:H_BACK_STATE ;
                // Signal line complete at state transition (offset by 1 for synchronous state transition)
                line_done <= (h_counter == (H_BACK-1))?HIGH:LOW ;
            end
            //////////////////////////////////////////////////////////////////////////
            ///////////////////////// VERTICAL ///////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////
            if (v_state == V_ACTIVE_STATE) begin
                // increment vertical counter at end of line, zero on state transition
                v_counter<=(line_done==HIGH)?((v_counter==V_ACTIVE)?10'd_0:(v_counter+10'd_1)):v_counter ;
                // set vsync in active mode
                vsync_reg <= HIGH ;
                // state transition - only on end of lines
                v_state<=(line_done==HIGH)?((v_counter==V_ACTIVE)?V_FRONT_STATE:V_ACTIVE_STATE):V_ACTIVE_STATE ;
            end
            if (v_state == V_FRONT_STATE) begin
                // increment vertical counter at end of line, zero on state transition
                v_counter<=(line_done==HIGH)?((v_counter==V_FRONT)?10'd_0:(v_counter + 10'd_1)):v_counter ;
                // set vsync in front porch
                vsync_reg <= HIGH ;
                // state transition
                v_state<=(line_done==HIGH)?((v_counter==V_FRONT)?V_PULSE_STATE:V_FRONT_STATE):V_FRONT_STATE;
            end
            if (v_state == V_PULSE_STATE) begin
                // increment vertical counter at end of line, zero on state transition
                v_counter<=(line_done==HIGH)?((v_counter==V_PULSE)?10'd_0:(v_counter + 10'd_1)):v_counter ;
                // clear vsync in pulse
                vsync_reg <= LOW ;
                // state transition
                v_state<=(line_done==HIGH)?((v_counter==V_PULSE)?V_BACK_STATE:V_PULSE_STATE):V_PULSE_STATE;
            end
            if (v_state == V_BACK_STATE) begin
                // increment vertical counter at end of line, zero on state transition
                v_counter<=(line_done==HIGH)?((v_counter==V_BACK)?10'd_0:(v_counter + 10'd_1)):v_counter ;
                // set vsync in back porch
                vsync_reg <= HIGH ;
                // state transition
                v_state<=(line_done==HIGH)?((v_counter==V_BACK)?V_ACTIVE_STATE:V_BACK_STATE):V_BACK_STATE ;
            end

            //////////////////////////////////////////////////////////////////////////
            //////////////////////////////// COLOR OUT ///////////////////////////////
            //////////////////////////////////////////////////////////////////////////
            // Assign colors if in active mode
            red_reg<=(h_state==H_ACTIVE_STATE)?((v_state==V_ACTIVE_STATE)?{color_in[7:5],5'd_0}:8'd_0):8'd_0 ;
            green_reg<=(h_state==H_ACTIVE_STATE)?((v_state==V_ACTIVE_STATE)?{color_in[4:2],5'd_0}:8'd_0):8'd_0 ;
            blue_reg<=(h_state==H_ACTIVE_STATE)?((v_state==V_ACTIVE_STATE)?{color_in[1:0],6'd_0}:8'd_0):8'd_0 ;

        end
    end
    // Assign output values - to VGA connector
    assign hsync = hysnc_reg ;
    assign vsync = vsync_reg ;
    assign red = red_reg ;
    assign green = green_reg ;
    assign blue = blue_reg ;
    assign clk = clock ;
    assign sync = 1'b_0 ;
    assign blank = hysnc_reg & vsync_reg ;
    // The x/y coordinates that should be available on the NEXT cycle
    assign next_x = (h_state==H_ACTIVE_STATE)?h_counter:10'd_0 ;
    assign next_y = (v_state==V_ACTIVE_STATE)?v_counter:10'd_0 ;

endmodule


module M10K_76800_8( //8 bit colors with 240 rows * 320 pixels/row
    output reg [7:0] out,
    input [7:0] in,
    input [16:0] write_address, read_address,
    input we, clk
);
   // force M10K ram style
    reg [7:0] mem [76800:0]  /* synthesis ramstyle = "no_rw_check, M10K" */;

    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= in;
        end
        out <= mem[read_address]; // q doesn't get d in this clock cycle
    end
endmodule

```