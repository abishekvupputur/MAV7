/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(float col1,float col2,float col3,float col4,float col5, int flag);
int sign(int x);
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};

// define settings
float oag_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.

double cnt1_count=0;
double cnt2_count=0;
double cnt3_count=0;
double cnt4_count=0;
double cnt5_count=0;
double cnt6_count=0;
double cnt7_count=0;
double cnt8_count=0;
double cnt9_count=0;
double cnt10_count=0;
double cnt11_count=0;
double cnt12_count=0;
double cnt13_count=0;
double cnt14_count=0;
double cnt15_count=0;

const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}

#ifndef COLUMNS_COUNT_ID
#error This module requires the columns counter ID
#endif
static abi_event orange_color_columns_ev;
static void orange_color_columns_cb(uint8_t __attribute__((unused)) sender_id,
                                   double cnt_1,double cnt_2,double cnt_3,double cnt_4,double cnt_5,
								   double cnt_6,double cnt_7,double cnt_8,double cnt_9,double cnt_10,
								   double cnt_11,double cnt_12,double cnt_13,double cnt_14,double cnt_15)
{
	cnt1_count=cnt_1;
	cnt2_count=cnt_2;
	cnt3_count=cnt_3;
	cnt4_count=cnt_4;
	cnt5_count=cnt_5;
	cnt6_count=cnt_6;
	cnt7_count=cnt_7;
	cnt8_count=cnt_8;
	cnt9_count=cnt_9;
	cnt10_count=cnt_10;
	cnt11_count=cnt_11;
	cnt12_count=cnt_12;
	cnt13_count=cnt_13;
	cnt14_count=cnt_14;
	cnt15_count=cnt_15;

}



/*
 * Initialisation function
 */
void orange_avoider_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance(0,0,0,0,0,0);

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
  AbiBindMsgORANGE_COLOR_COLUMNS(COLUMNS_COUNT_ID, &orange_color_columns_ev, orange_color_columns_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
//int ease_factor=[1,2,3,2,1];
void orange_avoider_guided_periodic(void)
{
	  VERBOSE_PRINT(" Fuckkk me    ;;%d  \n", cnt1_count);

  return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
int sign(int x) {
    if (x>0)
    {
    	return 1;	/* code */
    }
    else
    	return -1;
}
uint8_t chooseRandomIncrementAvoidance(float col1,float col2,float col3,float col4,float col5, int flag)
{
  // Randomly choose CW or CCW avoiding direction
	float col[5]={col1,col2,col3,col4,col5};
	float alpha=0.05;
	float ease[5] = {2,1,0,1,2};
	float minimum =0;
	int c=0;
	float sum[5]={0,0,0,0,0};
	int size=5;
	int location =0;
	int view_angle=90; //degree
	for (c = 1; c < size; c++)
    {
        sum[c]=col[c]+alpha*ease[c];
    }
	for (c = 1; c < size; c++)
    {
        if (sum[c] < minimum)
        {
           minimum = sum[c];
           location = c+1;
        }
    }
    location=location-3;
	VERBOSE_PRINT("");
     VERBOSE_PRINT("GO TO: %f\n",location);
   	VERBOSE_PRINT("");
  if (location*location == 1) {
    avoidance_heading_direction = -1.f*sign(location);
    oag_heading_rate = RadOfDeg(view_angle/4);
   // VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  } else if(location*location ==4)
   {
    avoidance_heading_direction = -1.f*sign(location);
    oag_heading_rate = RadOfDeg(view_angle/2);
   // VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  }
  else
  {	
  		avoidance_heading_direction = -1.f;
    	oag_heading_rate = RadOfDeg(19);
    //	VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  	
  }
  return false;
}
