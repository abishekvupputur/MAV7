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
#include "firmwares/rotorcraft/navigation.h"
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

uint8_t chooseRandomIncrementAvoidance(float col[5], int speed, float green_col[5]);
bool InsideObstacleZone(float _x, float _y) ;

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
float oag_max_speed = 0.65f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.

uint32_t column1_count = 0;
uint32_t column2_count = 0;
uint32_t column3_count = 0;
uint32_t column4_count = 0;
uint32_t column5_count = 0;

uint32_t floor1_count = 0;
uint32_t floor2_count = 0;
uint32_t floor3_count = 0;
uint32_t floor4_count = 0;
uint32_t floor5_count = 0;

float end_clearence= 1.0f; // Distance to clear the ends of the arena

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
                                    uint32_t column_1, uint32_t column_2,
                                    uint32_t column_3, uint32_t column_4,
			            uint32_t column_5, uint32_t floor_1,
				    uint32_t floor_2, uint32_t floor_3,
				    uint32_t floor_4, uint32_t floor_5)
{
  column1_count = column_1;
  column2_count = column_2;
  column3_count = column_3;
  column4_count = column_4;
  column5_count = column_5;

  floor1_count = floor_1;
  floor2_count = floor_2;
  floor3_count = floor_3;
  floor4_count = floor_4;
  floor5_count = floor_5;
}



/*
 * Initialisation function
 */
void orange_avoider_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));
  float zeros[5]={0,0,0,0,0}; //Intensity of Orange, green on screen anytime.
  chooseRandomIncrementAvoidance(zeros,0,zeros);

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
  AbiBindMsgORANGE_COLOR_COLUMNS(COLUMNS_COUNT_ID, &orange_color_columns_ev, orange_color_columns_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_avoider_guided_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  uint32_t column_size = front_camera.output_size.w * front_camera.output_size.h / 5;

  float column1_orange_fraction = (float)column1_count / column_size;
  float column2_orange_fraction = (float)column2_count / column_size;
  float column3_orange_fraction = (float)column3_count / column_size;
  float column4_orange_fraction = (float)column4_count / column_size;
  float column5_orange_fraction = (float)column5_count / column_size;

  float column1_green_fraction = (float)floor1_count / (column_size * 2);
  float column2_green_fraction = (float)floor2_count / (column_size * 2);
  float column3_green_fraction = (float)floor3_count / (column_size * 2);
  float column4_green_fraction = (float)floor4_count / (column_size * 2);
  float column5_green_fraction = (float)floor5_count / (column_size * 2);

  VERBOSE_PRINT("\n");
  VERBOSE_PRINT("Fraction of orange in column 1: %f\n", column1_orange_fraction);
  VERBOSE_PRINT("Fraction of orange in column 2: %f\n", column2_orange_fraction);
  VERBOSE_PRINT("Fraction of orange in column 3: %f\n", column3_orange_fraction);
  VERBOSE_PRINT("Fraction of orange in column 4: %f\n", column4_orange_fraction);
  VERBOSE_PRINT("Fraction of orange in column 5: %f\n", column5_orange_fraction);
  VERBOSE_PRINT("\n");
  VERBOSE_PRINT("Fraction of green in bottom half of column 1: %f\n", column1_green_fraction);
  VERBOSE_PRINT("Fraction of green in bottom half of column 2: %f\n", column2_green_fraction);
  VERBOSE_PRINT("Fraction of green in bottom half of column 3: %f\n", column3_green_fraction);
  VERBOSE_PRINT("Fraction of green in bottom half of column 4: %f\n", column4_green_fraction);
  VERBOSE_PRINT("Fraction of green in bottom half of column 5: %f\n", column5_green_fraction);
  VERBOSE_PRINT("\n");
  float col[5]={column1_orange_fraction, column2_orange_fraction, column3_orange_fraction, column4_orange_fraction, column5_orange_fraction};
  float green_col[5]={column1_green_fraction, column2_green_fraction, column3_green_fraction, column4_green_fraction, column5_green_fraction};
  // update our safe confidence using color threshold

  //Obstacle in centre of image is not tolerated and obstacles in the immediate sides are tolerated a little.
  if(column3_orange_fraction < 0.5 || column2_orange_fraction < 0.6 || column4_orange_fraction < 0.6){ 
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float speed_sp = fminf(oag_max_speed, 0.3f * obstacle_free_confidence);
  float new_coor_x=0;
  float new_coor_y=0;
  float heading;
  heading = stateGetNedToBodyEulers_f()->psi;
  new_coor_x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (end_clearence));
  new_coor_y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (end_clearence));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", end_clearence,  
                POS_FLOAT_OF_BFP(new_coor_x), POS_FLOAT_OF_BFP(new_coor_y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  switch (navigation_state){
    case SAFE:
     	if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        guidance_h_set_guided_body_vel(speed_sp, 0);
      }

      /*if (!InsideObstacleZone(POS_FLOAT_OF_BFP(new_coor_x), POS_FLOAT_OF_BFP(new_coor_y))){
      	navigation_state= OUT_OF_BOUNDS;
      	VERBOSE_PRINT("OUTSIDE ARENA \n");
      	VERBOSE_PRINT("new_coor_x: %f\n",new_coor_x);
      	VERBOSE_PRINT("new_coor_y: %f\n",new_coor_y);
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        guidance_h_set_guided_body_vel(speed_sp, 0);
      }*/
      
      break;
    case OBSTACLE_FOUND:
      // stop
      //guidance_h_set_guided_body_vel(0, 0);
      VERBOSE_PRINT("OBSTACLE IN COLUMN 3");
      // randomly select new search direction
      chooseRandomIncrementAvoidance(col,speed_sp,green_col);

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      guidance_h_set_guided_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2){
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_guided_body_vel(0, 0);
      chooseRandomIncrementAvoidance(col,speed_sp,green_col);

      // start turn back into arena

      guidance_h_set_guided_heading_rate(avoidance_heading_direction * RadOfDeg(53));

      navigation_state = REENTER_ARENA;

      break;
    case REENTER_ARENA:
      // force floor center to opposite side of turn to head back into arena
      /*if (InsideObstacleZone(POS_FLOAT_OF_BFP(new_coor_x), POS_FLOAT_OF_BFP(new_coor_y))&& green_col[3]>0.1 ){
        // return to heading mode
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SAFE;
      }*/
    	if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SAFE;
      }
      break;
    default:
      break;
  }
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
uint8_t chooseRandomIncrementAvoidance(float col[5], int speed, float green_col[5])
{
  // Randomly choose CW or CCW avoiding direction
	float alpha=0.01;
	float ease[5] = {2,1,0,1,2};
	float minimum =0;
	int c=0;
	float sum[5]={0,0,0,0,0};
	int size=5;
	int location =0;
	int view_angle=180; //degree
	float gamma=0.07;
	for (c = 1; c < size; c++)
    {
        sum[c]=col[c]+ (alpha*ease[c]*speed/oag_max_speed) + (gamma*(1-green_col[c]));
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
    oag_heading_rate = RadOfDeg(view_angle/8);
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  } else if(location*location ==4)
   {
    avoidance_heading_direction = -1.f*sign(location);
    oag_heading_rate = RadOfDeg(view_angle/4);
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  }
  else
  {	
  		if (rand() % 2 == 0) {
   		avoidance_heading_direction = 1.f;
    	} 
  		else {
    	avoidance_heading_direction = -1.f;
    	}
  		oag_heading_rate = RadOfDeg(87);
    	VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  	
  }
  return false;
}


bool InsideObstacleZone(float _x, float _y) {
  if (_y <= 0.6) {
    if (_y <= -1.1) {
      if (_y <= -4.3) {
        return FALSE;
      } else {
        float dy = _y - -1.1;
        return (-2.7+dy*-0.644444<= _x && _x <= 4.2+dy*1.498442);
      }
    } else {
      float dy = _y - 0.6;
      return (-3.8+dy*-0.644444<= _x && _x <= 3.0+dy*-0.672878);
    }
  } else {
    if (_y <= 3.7) {
      float dy = _y - 3.7;
      return (1.0+dy*1.537217<= _x && _x <= 1.0+dy*-0.672878);
    } else {
      return FALSE;
    }
  }
}
