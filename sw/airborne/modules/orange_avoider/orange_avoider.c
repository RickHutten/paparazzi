/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "firmwares/rotorcraft/navigation.h"

#include "boards/bebop/mt9f002.h"
#include "generated/flight_plan.h"
#include "modules/computer_vision/colorfilter.h"
#include "modules/orange_avoider/orange_avoider.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif


struct mt9f002_t camera;
uint8_t safeToGoForwards        = false;
int tresholdColorCount          = 0.05 * 124800; // 520 x 240 = 124.800 total pixels
float incrementForAvoidance;
uint16_t trajectoryConfidence   = 1;
float maxDistance               = 2.25;

/*
 * Initialization function, initialize camera object
 */
void orange_avoider_init()
{
	// TODO: Initialize camera
//	mt9f002_init(&camera);

	// Initialize random values
	srand(time(NULL));
	chooseRandomIncrementAvoidance();
}

/*
 * Function that checks it is safe to move forwards,
 * and then moves a waypoint forward or changes the heading
 */
void orange_avoider_periodic() {
	// Set crop of image based on the drones pitch
	setImageCrop();

	// Create moving average of boundary[] array
	createSmoothedBoundary();

	// Calculate if we can move forwards
	int canGoForwards = getCanGoForwards();

	// Get maximum value of boundary
	int max = getBoundaryMaxVal();

	// Get x position of the maximum value
	int pos_x = getBoundaryMaxPosX(max);

	// If its safe to go forwards
	if(canGoForwards){
		int moveDistance = 1.5;
		if (pos_x < 20 && boundary_smoothed[24] < 240) {
			// Highest boundary is slightly to the left, move forwards to the left
			moveWaypointForwardAngle(WP_GOAL, moveDistance, -2);
		} else if (pos_x > 29 && boundary_smoothed[24] < 240) {
			// Highest boundary is slightly to the right, move forwards to the right
			moveWaypointForwardAngle(WP_GOAL, moveDistance, 2);
		} else {
			// Highest boundary is straight ahead, go straight
			moveWaypointForwardAngle(WP_GOAL, moveDistance, 0);
		}
		// Set heading of the drone towards the waypoint
		nav_set_heading_towards_waypoint(WP_GOAL);
	} else {
		// Not safe to go forwards, stop and turn to the right 10 degrees
		waypoint_set_here_2d(WP_GOAL);
		increase_nav_heading(&nav_heading, 10);
	}

	printf("Max val: %d    Pos: %d    CanGoForwards: %d\n", max, pos_x, canGoForwards);

//  safeToGo = MAX_POINT_VALUE > THRESHOLD;
//  float moveDistance = fmin(maxDistance, 0.05 * trajectoryConfidence);
//  img_width = 52;
//  right_lim = img_width/2 + 3;
//  left_lim = img_width/2 - 2;
//  if(safeToGo){
//      if (MAX_POINT >= left_lim && MAX_POINT_LOC <= right_lim){
//    	  moveWaypointForward(WP_GOAL, moveDistance);
//    	  moveWaypointForward(WP_TRAJECTORY, 1.25 * moveDistance);
//    	  nav_set_heading_towards_waypoint(WP_GOAL);
//    	  chooseRandomIncrementAvoidance();
//    	  trajectoryConfidence += 1;
//      }
//      else if(MAX_POINT_LOC < left_lim){
//    	  waypoint_set_here_2d(WP_GOAL);
// 	      waypoint_set_here_2d(WP_TRAJECTORY);
//    	  increase_nav_heading(&nav_heading, incrementForAvoidance);
//    	  if(trajectoryConfidence > 5){
//    	            trajectoryConfidence -= 4;
//    	        }
//    	        else{
//    	            trajectoryConfidence = 1;
//    	        }
//      }
//      else if(MAX_POINT_LOC > right_lim){
//    	  waypoint_set_here_2d(WP_GOAL);
//    	  waypoint_set_here_2d(WP_TRAJECTORY);
//    	  increase_nav_heading(&nav_heading, -incrementForAvoidance);
//    	  if(trajectoryConfidence > 5){
//    	            trajectoryConfidence -= 4;
//    	        }
//    	        else{
//    	            trajectoryConfidence = 1;
//    	        }
//      }
//
//  }
//  else{
//      waypoint_set_here_2d(WP_GOAL);
//      waypoint_set_here_2d(WP_TRAJECTORY);
//      increase_nav_heading(&nav_heading, incrementForAvoidance);
//      if(trajectoryConfidence > 5){
//          trajectoryConfidence -= 4;
//      }
//      else{
//          trajectoryConfidence = 1;
//      }
//  }
	return;
}

/*
 * Sets the crop of the image depending on the theta angle of the drone,
 * so the done sees the same area no matter the pitch.
 */
void setImageCrop() {
	// Set crop of image
	struct FloatEulers* my_euler_angles = stateGetNedToBodyEulers_f();
	float theta = my_euler_angles->theta;
	printf("Theta: %f   ", theta);
	// TODO: Calculate the offset at specific angles theta
	(&camera)->offset_x = 0;
	mt9f002_update_resolution(&camera);
}

/*
 * Calculates whether there is something directly in front of the drone
 */
int getCanGoForwards() {
	int threshold = 40;  // Center boundary should all be higher than this
	for (int i = 16; i < 33; i ++) {
		if (boundary_smoothed[i] <= threshold) {
			return 0;
		}
	}
	return 1;
}

/*
 * Get the maximum value of the boundary
 */
int getBoundaryMaxVal() {
	int max = 0;
	for (int i = 0; i < 48; i ++) {
		if (boundary_smoothed[i] > max) {
			max = boundary_smoothed[i];
		}
	}
	return max;
}

/*
 * Get the index of the item that is returned by getBoundaryMaxVal()
 */
int getBoundaryMaxPosX(int max) {
	for (int i = 0; i < 48; i ++) {
		if (boundary_smoothed[i] == max) {
			return i;
		}
	}
	return -1; // This is never supposed to happen
}

/*
 * Smooth boundary array in order to calculate the desired heading
 */
void createSmoothedBoundary() {
	for (int i = 2; i < 50; i ++) {
		boundary_smoothed[i-2] = 0.2 * (boundary[i-2] + boundary[i-1] + boundary[i] + boundary[i+1] + boundary[i+2]);
	}
	return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees)
{
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL( incrementDegrees / 180.0 * M_PI);
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
  *heading = newHeading;
//  VERBOSE_PRINT("Increasing heading to %f\n", ANGLE_FLOAT_OF_BFP(*heading) * 180 / M_PI);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters, int angle)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi) + (3.141592*angle)/180.0);
  float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi) + (3.141592*angle)/180.0);
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
//  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y), ANGLE_FLOAT_OF_BFP(eulerAngles->psi)*180/M_PI);
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
//  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters, 0);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Moves waypoint at distanceMeters ahead of the drone with a specific angle
 */
void moveWaypointForwardAngle(uint8_t waypoint, float distanceMeters, int angle)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters, angle);
  moveWaypoint(waypoint, &new_coor);
  return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance()
{
  // Randomly choose CW or CCW avoiding direction
  int r = rand() % 2;
  if (r == 0) {
    incrementForAvoidance = 10.0;
//    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  } else {
    incrementForAvoidance = -10.0;
//    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  }
  return false;
}

