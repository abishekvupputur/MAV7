/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif
double green1=0;
double green2=0;
double green3=0;
double blue1=0;
double blue2=0;
double blue3=0;
double black1=0;
double black2=0;
double black3=0;
double red1=0;
double red2=0;
double red3=0;
double white1=0;
double white2=0;
double white3=0;
// Orange color settings
uint8_t y_min = 50;
uint8_t y_max = 255;
uint8_t u_min = 0;
uint8_t u_max = 130;
uint8_t v_min = 150; 
uint8_t v_max = 255;

// Green color settings
//uint8_t y_min = 50;
//uint8_t y_max = 255;
//uint8_t u_min = 0;
//uint8_t u_max = 130;
//uint8_t v_min = 0; 
//uint8_t v_max = 130;


// Filter Settings
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

// define global variables

uint32_t col1;
uint32_t col2;
uint32_t col3;
uint32_t col4;
uint32_t col5;

uint32_t floor1;
uint32_t floor2;
uint32_t floor3;
uint32_t floor4;
uint32_t floor5;

double cnt[15] = { 0.0 };

struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};

struct color_object_t global_filters[2];

// Function
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;

  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;

  // Filter and find centroid
uint32_t count = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = 1;
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img);
struct image_t *object_detector1(struct image_t *img)
{
  return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img);
struct image_t *object_detector2(struct image_t *img)
{
  return object_detector(img, 2);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2);
#endif
}

/*
 * find_object_centroid
 * @param img - input image to process
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param draw - whether or not to draw on image
 * @return number of pixels found
 */

// Piggybacking on this function that counts green pixels to also count orange pixels per column
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t total_y = 0;
  uint32_t total_x1 = 0;
  uint32_t total_x2 = 0;
  uint32_t total_x3 = 0;

  uint32_t col_1 = 0;
  uint32_t col_2 = 0;
  uint32_t col_3 = 0;
  uint32_t col_4 = 0;
  uint32_t col_5 = 0;

  uint32_t floor_1 = 0;
  uint32_t floor_2 = 0;
  uint32_t floor_3 = 0;
  uint32_t floor_4 = 0;
  uint32_t floor_5 = 0;

  double cnt[15] = { 0.0 };
  uint32_t dummy = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  // Go through all the pixels
  for (uint16_t y = 0.2*(img->h); y < 0.9*(img->h); y++) {
    //total_y++;
    //total_x = 0;
    for (uint16_t x = 0; x < 0.2*(img->w); x ++) {
      //total_x++;
    	total_x1++;
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
	
	if ((*yp > 0 ) && (*yp <= 0.1) &&
           (*up > -0.5 ) && (*up <= 0 ) &&
           (*vp > -0.5 ) && (*vp <= 0 )) 
{
green1++;
}

else if ((*yp > 0 ) && (*yp <= 0.1) &&
           (*up > -0.5 ) && (*up <= -0.2 ) &&
           (*vp > 0 ) && (*vp <= 0.5 )) 
{
blue1++;
}
else if ((*yp > 0 ) && (*yp <= 0.1) &&
           (*up > 0 ) && (*up <= 0.1 ) &&
           (*vp > -0.5 ) && (*vp <= 0 )) 
{
black1++;
}
else if ((*yp > 0 ) && (*yp <= 0.1) &&
           (*up > -0.2 ) && (*up <= 0.1 ) &&
           (*vp > 0 ) && (*vp <= 0.5 )) 
{
blue1++;
}
else if ((*yp > 0 ) && (*yp <=0.1) &&
           (*up > 0.1 ) && (*up <= 0.3 ) &&
           (*vp > -0.5 ) && (*vp <= 0.2 )) 
{
red1++;
}

else if ((*yp > 0 ) && (*yp <= 0.1) &&
           (*up > 0.1 ) && (*up <= 0.3 ) &&
           (*vp > 0.2) && (*vp <= 0.5 )) 
{
blue1++;
}

else if ((*yp > 0 ) && (*yp <= 0.1) &&
           (*up > 0.3 ) && (*up <= 0.5 ) &&
           (*vp > -0.5 ) && (*vp <= 0.5 )) 
{
red1++;
}

else if ((*yp > 0.1 ) && (*yp <= 0.3) &&
           (*up > -0.5 ) && (*up <= 0 ) &&
           (*vp > -0.5 ) && (*vp <= 0 )) 
{
green1++;
}
else if ((*yp > 0.1 ) && (*yp <= 0.3) &&
           (*up > -0.5 ) && (*up <= 0 ) &&
           (*vp > 0 ) && (*vp <= 0.5 )) 
{
blue1++;
}
else if ((*yp > 0.1 ) && (*yp <= 0.3) &&
           (*up > 0 ) && (*up <= 0.5 ) &&
           (*vp > -0.5 ) && (*vp <= 0.5 )) 
{
red1++;
}
else if ((*yp > 0.3 ) && (*yp <= 0.5) &&
           (*up > -0.5 ) && (*up <= 0 ) &&
           (*vp >-0.5 ) && (*vp <= 0 )) 
{
green1++;
}
else if ((*yp > 0.3 ) && (*yp <= 0.5) &&
           (*up > -0.5 ) && (*up <= 0 ) &&
           (*vp > 0 ) && (*vp <= 0.5 )) 
{
blue1++;
}
else if ((*yp > 0.3 ) && (*yp <= 0.5) &&
           (*up > 0 ) && (*up <= 0.5 ) &&
           (*vp > -0.5 ) && (*vp <= 0.5 )) 
{
red1++;
}

else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
           (*up > -0.5 ) && (*up <= -0.1 ) &&
           (*vp > -0.5 ) && (*vp <= 0 )) 
{
green1++;
}

else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
           (*up > -0.5 ) && (*up <= -0.1 ) &&
           (*vp > 0 ) && (*vp <= 0.5 )) 
{
blue1++;
}


else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
           (*up > -0.1 ) && (*up <= 0.1 ) &&
           (*vp > -0.5 ) && (*vp <= -0.1 )) 
{
green1++;
}

else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
           (*up > -0.1 ) && (*up <= 0.1 ) &&
           (*vp > -0.1 ) && (*vp <= 0.1 )) 
{
white1++;
}

else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
           (*up > -0.1 ) && (*up <= 0.1 ) &&
           (*vp > 0.1 ) && (*vp <= 0.5 )) 
{
blue1++;
}

else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
           (*up > 0.1 ) && (*up <= 0.5 ) &&
           (*vp > -0.5 ) && (*vp <= 0.5 )) 
{
red1++;
}

else if ((*yp > 0.75 ) && (*yp <=1) &&
           (*up > -0.5 ) && (*up <= 0.2 ) &&
           (*vp > -0.5 ) && (*vp <= -0.2 )) 
{
green1++;
}

else if ((*yp > 0.75 ) && (*yp <=1) &&
           (*up > -0.5 ) && (*up <= 0.2 ) &&
           (*vp > -0.2 ) && (*vp <= 0.5 )) 
{
white1++;
}

else if ((*yp > 0.75 ) && (*yp <=1) &&
           (*up > 0.2 ) && (*up <= 0.5 ) &&
           (*vp > -0.5 ) && (*vp <= -0.3 )) 
{
red1++;
}

else if ((*yp > 0.75 ) && (*yp <=1) &&
           (*up > 0.2 ) && (*up <= 0.5 ) &&
           (*vp > -0.3 ) && (*vp <= 0 )) 
{
white1++;
}

else if ((*yp > 0.75 ) && (*yp <=1) &&
           (*up > 0.2 ) && (*up <= 0.5 ) &&
           (*vp > 0 ) && (*vp <= 0.5 )) 
{
red1++;
}
    }

    for (uint16_t x = 0.2*(img->w); x < 0.8*(img->w); x ++) {
    	total_x2++;
         // Check if the color is inside the specified values
         uint8_t *yp, *up, *vp;
         if (x % 2 == 0) {
           // Even x
           up = &buffer[y * 2 * img->w + 2 * x];      // U
           yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
           vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
           //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
         } else {
           // Uneven x
           up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
           //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
           vp = &buffer[y * 2 * img->w + 2 * x];      // V
           yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
         }

   	if ((*yp > 0 ) && (*yp <= 0.1) &&
              (*up > -0.5 ) && (*up <= 0 ) &&
              (*vp > -0.5 ) && (*vp <= 0 ))
   {
   green2++;
   }

   else if ((*yp > 0 ) && (*yp <= 0.1) &&
              (*up > -0.5 ) && (*up <= -0.2 ) &&
              (*vp > 0 ) && (*vp <= 0.5 ))
   {
   blue2++;
   }
   else if ((*yp > 0 ) && (*yp <= 0.1) &&
              (*up > 0 ) && (*up <= 0.1 ) &&
              (*vp > -0.5 ) && (*vp <= 0 ))
   {
   black2++;
   }
   else if ((*yp > 0 ) && (*yp <= 0.1) &&
              (*up > -0.2 ) && (*up <= 0.1 ) &&
              (*vp > 0 ) && (*vp <= 0.5 ))
   {
   blue2++;
   }
   else if ((*yp > 0 ) && (*yp <=0.1) &&
              (*up > 0.1 ) && (*up <= 0.3 ) &&
              (*vp > -0.5 ) && (*vp <= 0.2 ))
   {
   red2++;
   }

   else if ((*yp > 0 ) && (*yp <= 0.1) &&
              (*up > 0.1 ) && (*up <= 0.3 ) &&
              (*vp > 0.2) && (*vp <= 0.5 ))
   {
   blue2++;
   }

   else if ((*yp > 0 ) && (*yp <= 0.1) &&
              (*up > 0.3 ) && (*up <= 0.5 ) &&
              (*vp > -0.5 ) && (*vp <= 0.5 ))
   {
   red2++;
   }

   else if ((*yp > 0.1 ) && (*yp <= 0.3) &&
              (*up > -0.5 ) && (*up <= 0 ) &&
              (*vp > -0.5 ) && (*vp <= 0 ))
   {
   green2++;
   }
   else if ((*yp > 0.1 ) && (*yp <= 0.3) &&
              (*up > -0.5 ) && (*up <= 0 ) &&
              (*vp > 0 ) && (*vp <= 0.5 ))
   {
   blue2++;
   }
   else if ((*yp > 0.1 ) && (*yp <= 0.3) &&
              (*up > 0 ) && (*up <= 0.5 ) &&
              (*vp > -0.5 ) && (*vp <= 0.5 ))
   {
   red2++;
   }
   else if ((*yp > 0.3 ) && (*yp <= 0.5) &&
              (*up > -0.5 ) && (*up <= 0 ) &&
              (*vp >-0.5 ) && (*vp <= 0 ))
   {
   green2++;
   }
   else if ((*yp > 0.3 ) && (*yp <= 0.5) &&
              (*up > -0.5 ) && (*up <= 0 ) &&
              (*vp > 0 ) && (*vp <= 0.5 ))
   {
   blue2++;
   }
   else if ((*yp > 0.3 ) && (*yp <= 0.5) &&
              (*up > 0 ) && (*up <= 0.5 ) &&
              (*vp > -0.5 ) && (*vp <= 0.5 ))
   {
   red2++;
   }

   else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
              (*up > -0.5 ) && (*up <= -0.1 ) &&
              (*vp > -0.5 ) && (*vp <= 0 ))
   {
   green2++;
   }

   else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
              (*up > -0.5 ) && (*up <= -0.1 ) &&
              (*vp > 0 ) && (*vp <= 0.5 ))
   {
   blue2++;
   }


   else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
              (*up > -0.1 ) && (*up <= 0.1 ) &&
              (*vp > -0.5 ) && (*vp <= -0.1 ))
   {
   green2++;
   }

   else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
              (*up > -0.1 ) && (*up <= 0.1 ) &&
              (*vp > -0.1 ) && (*vp <= 0.1 ))
   {
   white2++;
   }

   else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
              (*up > -0.1 ) && (*up <= 0.1 ) &&
              (*vp > 0.1 ) && (*vp <= 0.5 ))
   {
   blue2++;
   }

   else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
              (*up > 0.1 ) && (*up <= 0.5 ) &&
              (*vp > -0.5 ) && (*vp <= 0.5 ))
   {
   red2++;
   }

   else if ((*yp > 0.75 ) && (*yp <=1) &&
              (*up > -0.5 ) && (*up <= 0.2 ) &&
              (*vp > -0.5 ) && (*vp <= -0.2 ))
   {
   green2++;
   }

   else if ((*yp > 0.75 ) && (*yp <=1) &&
              (*up > -0.5 ) && (*up <= 0.2 ) &&
              (*vp > -0.2 ) && (*vp <= 0.5 ))
   {
   white2++;
   }

   else if ((*yp > 0.75 ) && (*yp <=1) &&
              (*up > 0.2 ) && (*up <= 0.5 ) &&
              (*vp > -0.5 ) && (*vp <= -0.3 ))
   {
   red2++;
   }

   else if ((*yp > 0.75 ) && (*yp <=1) &&
              (*up > 0.2 ) && (*up <= 0.5 ) &&
              (*vp > -0.3 ) && (*vp <= 0 ))
   {
   white2++;
   }

   else if ((*yp > 0.75 ) && (*yp <=1) &&
              (*up > 0.2 ) && (*up <= 0.5 ) &&
              (*vp > 0 ) && (*vp <= 0.5 ))
   {
   red2++;
   }
       }


    for (uint16_t x = 0.8*(img->w); x < (img->w); x ++) {
    	total_x3++;
           // Check if the color is inside the specified values
           uint8_t *yp, *up, *vp;
           if (x % 2 == 0) {
             // Even x
             up = &buffer[y * 2 * img->w + 2 * x];      // U
             yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
             vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
             //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
           } else {
             // Uneven x
             up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
             //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
             vp = &buffer[y * 2 * img->w + 2 * x];      // V
             yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
           }

     	if ((*yp > 0 ) && (*yp <= 0.1) &&
                (*up > -0.5 ) && (*up <= 0 ) &&
                (*vp > -0.5 ) && (*vp <= 0 ))
     {
     green3++;
     }

     else if ((*yp > 0 ) && (*yp <= 0.1) &&
                (*up > -0.5 ) && (*up <= -0.2 ) &&
                (*vp > 0 ) && (*vp <= 0.5 ))
     {
     blue3++;
     }
     else if ((*yp > 0 ) && (*yp <= 0.1) &&
                (*up > 0 ) && (*up <= 0.1 ) &&
                (*vp > -0.5 ) && (*vp <= 0 ))
     {
     black3++;
     }
     else if ((*yp > 0 ) && (*yp <= 0.1) &&
                (*up > -0.2 ) && (*up <= 0.1 ) &&
                (*vp > 0 ) && (*vp <= 0.5 ))
     {
     blue3++;
     }
     else if ((*yp > 0 ) && (*yp <=0.1) &&
                (*up > 0.1 ) && (*up <= 0.3 ) &&
                (*vp > -0.5 ) && (*vp <= 0.2 ))
     {
     red3++;
     }

     else if ((*yp > 0 ) && (*yp <= 0.1) &&
                (*up > 0.1 ) && (*up <= 0.3 ) &&
                (*vp > 0.2) && (*vp <= 0.5 ))
     {
     blue3++;
     }

     else if ((*yp > 0 ) && (*yp <= 0.1) &&
                (*up > 0.3 ) && (*up <= 0.5 ) &&
                (*vp > -0.5 ) && (*vp <= 0.5 ))
     {
     red3++;
     }

     else if ((*yp > 0.1 ) && (*yp <= 0.3) &&
                (*up > -0.5 ) && (*up <= 0 ) &&
                (*vp > -0.5 ) && (*vp <= 0 ))
     {
     green3++;
     }
     else if ((*yp > 0.1 ) && (*yp <= 0.3) &&
                (*up > -0.5 ) && (*up <= 0 ) &&
                (*vp > 0 ) && (*vp <= 0.5 ))
     {
     blue3++;
     }
     else if ((*yp > 0.1 ) && (*yp <= 0.3) &&
                (*up > 0 ) && (*up <= 0.5 ) &&
                (*vp > -0.5 ) && (*vp <= 0.5 ))
     {
     red3++;
     }
     else if ((*yp > 0.3 ) && (*yp <= 0.5) &&
                (*up > -0.5 ) && (*up <= 0 ) &&
                (*vp >-0.5 ) && (*vp <= 0 ))
     {
     green3++;
     }
     else if ((*yp > 0.3 ) && (*yp <= 0.5) &&
                (*up > -0.5 ) && (*up <= 0 ) &&
                (*vp > 0 ) && (*vp <= 0.5 ))
     {
     blue3++;
     }
     else if ((*yp > 0.3 ) && (*yp <= 0.5) &&
                (*up > 0 ) && (*up <= 0.5 ) &&
                (*vp > -0.5 ) && (*vp <= 0.5 ))
     {
     red3++;
     }

     else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
                (*up > -0.5 ) && (*up <= -0.1 ) &&
                (*vp > -0.5 ) && (*vp <= 0 ))
     {
     green3++;
     }

     else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
                (*up > -0.5 ) && (*up <= -0.1 ) &&
                (*vp > 0 ) && (*vp <= 0.5 ))
     {
     blue3++;
     }


     else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
                (*up > -0.1 ) && (*up <= 0.1 ) &&
                (*vp > -0.5 ) && (*vp <= -0.1 ))
     {
     green3++;
     }

     else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
                (*up > -0.1 ) && (*up <= 0.1 ) &&
                (*vp > -0.1 ) && (*vp <= 0.1 ))
     {
     white3++;
     }

     else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
                (*up > -0.1 ) && (*up <= 0.1 ) &&
                (*vp > 0.1 ) && (*vp <= 0.5 ))
     {
     blue3++;
     }

     else if ((*yp > 0.5 ) && (*yp <= 0.75) &&
                (*up > 0.1 ) && (*up <= 0.5 ) &&
                (*vp > -0.5 ) && (*vp <= 0.5 ))
     {
     red3++;
     }

     else if ((*yp > 0.75 ) && (*yp <=1) &&
                (*up > -0.5 ) && (*up <= 0.2 ) &&
                (*vp > -0.5 ) && (*vp <= -0.2 ))
     {
     green3++;
     }

     else if ((*yp > 0.75 ) && (*yp <=1) &&
                (*up > -0.5 ) && (*up <= 0.2 ) &&
                (*vp > -0.2 ) && (*vp <= 0.5 ))
     {
     white3++;
     }

     else if ((*yp > 0.75 ) && (*yp <=1) &&
                (*up > 0.2 ) && (*up <= 0.5 ) &&
                (*vp > -0.5 ) && (*vp <= -0.3 ))
     {
     red3++;
     }

     else if ((*yp > 0.75 ) && (*yp <=1) &&
                (*up > 0.2 ) && (*up <= 0.5 ) &&
                (*vp > -0.3 ) && (*vp <= 0 ))
     {
     white3++;
     }

     else if ((*yp > 0.75 ) && (*yp <=1) &&
                (*up > 0.2 ) && (*up <= 0.5 ) &&
                (*vp > 0 ) && (*vp <= 0.5 ))
     {
     red3++;
     }
         }
 }
	//cnt[0]=white1/total_x1;
	cnt[0] = 2.5;
	cnt[1]=blue1/total_x1;
	cnt[2]=green1/total_x1;
	cnt[3]=red1/total_x1;
	cnt[4]=black1/total_x1;

	cnt[5]=white2/total_x2;
	cnt[6]=blue2/total_x2;
	cnt[7]=green2/total_x2;
	cnt[8]=red2/total_x2;
	cnt[9]=black2/total_x2;

	cnt[10]=white3/total_x3;
	cnt[11]=blue3/total_x3;
	cnt[12]=green3/total_x3;
	cnt[13]=red3/total_x3;
	cnt[14]=black3/total_x3;
  return dummy;

}

void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
    
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        0, 0, local_filters[1].color_count, 1);
    AbiSendMsgORANGE_COLOR_COLUMNS(COLOR_COLUMNS_COUNTED_ID, cnt[0], cnt[1],cnt[2],cnt[3],cnt[4],cnt[5],cnt[6],cnt[7],cnt[8],cnt[9],cnt[10],cnt[11],cnt[12],cnt[13],cnt[14]);
    local_filters[1].updated = false;
  }
}
