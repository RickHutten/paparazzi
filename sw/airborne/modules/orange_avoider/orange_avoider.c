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

#include "subsystems/datalink/datalink.h"
#include "subsystems/electrical.h"
#include "subsystems/radio_contrraol.h"
#include "subsystems/ahrs.h"


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
float incrementForAvoidance		= 10;
uint16_t trajectoryConfidence   = 1;
float maxDistance               = 2.25;
char prevCanGoForwards = 1;

/*
 * Initialization function, initialize camera object
 */
void orange_avoider_init() {
	printf("Init");

	// Initialize random values
	srand(time(NULL));
}

/*
 * Function that checks it is safe to move forwards,
 * and then moves a waypoint forward or changes the heading
 */
void orange_avoider_periodic() {

	// Change the threshold depending on the pitch of the drone
	setThreshold();

	// Create moving average of the boundary[] array
	createSmoothedBoundary();

	// Calculate if we can move forwards
	char canGoForwards = getCanGoForwards();

	// Get maximum value of boundary
	int max = getBoundaryMaxVal();

	// Get x position of the maximum value
	int pos_x = getBoundaryMaxPosX(max);

	// If its safe to go forwards
	if(canGoForwards){
		// Get the move distance
		float moveDistance = getMoveDistance();

		printf("Move: %f ", moveDistance);

		if (pos_x < 20 && boundary_smoothed_2[24] < 230) {
			// Highest boundary is slightly to the left, move forwards to the left
			moveWaypointForwardAngle(WP_GOAL, moveDistance, -5);
			printf("GoLeft      ");
		} else if (pos_x > 29 && boundary_smoothed_2[24] < 230) {
			// Highest boundary is slightly to the right, move forwards to the right
			moveWaypointForwardAngle(WP_GOAL, moveDistance, 5);
			printf("GoRight     ");
		} else {
			// Highest boundary is straight ahead, go straight
			moveWaypointForwardAngle(WP_GOAL, moveDistance, 0);
			printf("GoStraight  ");
		}
		// Set heading of the drone towards the waypoint
		nav_set_heading_towards_waypoint(WP_GOAL);
	} else {
		if (prevCanGoForwards) {
			// First time we can't go forwards, set the waypoint once
			waypoint_set_here_2d(WP_GOAL);
		}
		// Turn to the right
		printf("Move: stop  Avoidance: %f  ", incrementForAvoidance);
		increase_nav_heading(&nav_heading, incrementForAvoidance);
	}
	// Save the previous value of can go forwards
	prevCanGoForwards = canGoForwards;

	printf("Pos: (%f, %f) Heading: %f\n", getPositionX(), getPositionY(), getHeading());
	return;
}

float getMoveDistance() {
	// Get the moveDistance (speed) according to the camera
	float moveDistance =  2.0 - 1.8 * (1.0 - getBoundaryMinVal()/240.);

	// Get the heading of the drone
	float heading = getHeading();

	// If the drone is close to the optitrack boundary and the heading is
	// pointing to the outside, reduce the speed of the drone
	if (getPositionX() < 20 && (heading < - 1.57 || heading > 1.57)) {
		moveDistance /= 2.0;
		return moveDistance;  // Slightly quicker to return here
	}
	if (getPositionX() > 80 && (heading > - 1.57 && heading < 1.57)) {
		moveDistance /= 2.0;
		return moveDistance;  // Slightly quicker to return here
	}
	if (getPositionY() < 20 && heading > 0) {
		moveDistance /= 2.0;
		return moveDistance;  // Slightly quicker to return here
	}
	if (getPositionY() > 80 && heading < 0){
		moveDistance /= 2.0;
	}
	return moveDistance;
}

/*
 * Get the heading of the drone in radians from -pi to pi
 * Heading is zero in positive x direction (simplified coordinate system)
 */
float getHeading() {
	struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
	float angle = ANGLE_FLOAT_OF_BFP(eulerAngles->psi) - 1.02;  // 1.57 - 0.55 = 1.02 offset
	if (angle < -3.1415) {  // Make sure the the range is from -pi to pi
		angle += 6.2830;
	}
	return angle;
}

/*
 * Get the x position of the drone in a simplified coordinate system
 */
float getPositionX() {
	int x = stateGetPositionEnu_i()->x;
	int y = stateGetPositionEnu_i()->y;
	float posx = cosf(-0.55)*x - sinf(-0.55)*y;  // Rotate coordinate system
	posx += 435;  // Translate coordinate system
	posx /= 18.54;  // Scale coordinate system
	return posx;
}

/*
 * Get the y position of the drone in a simplified coordinate system
 */
float getPositionY() {
	int x = stateGetPositionEnu_i()->x;
	int y = stateGetPositionEnu_i()->y;
	float posy = cosf(-0.55)*y + sinf(-0.55)*x;  // Rotate coordinate system
	posy += 613;  // Translate coordinate system
	posy /= 14.15;  // Scale coordinate system
	return posy;
}

/*
 * Sets the threshold depending on the theta angle of the drone,
 * so it is independent of the pitch
 */
void setThreshold() {
	// Set crop of image
	struct FloatEulers* my_euler_angles = stateGetNedToBodyEulers_f();
	float theta = my_euler_angles->theta;
	threshold = 40 - theta * 300;
}

/*
 * Calculates whether there is something directly in front of the drone
 */
char getCanGoForwards() {
    // Center boundary should all be higher than this
	for (int i = 12; i < 37; i ++) {
		if (boundary_smoothed_2[i] <= threshold) {
			printf("F: color  ");
			// Set random turn direction (-10 or +10)
			chooseRandomIncrementAvoidance();
			return 0;
		}
	}

	// Check if were close to the optitrack boundary
	float heading = getHeading();

	// Drone is below y = 10  (danger zone)
	if (getPositionY() < 10 && heading > 0) {
		printf("F: y 20   ");
		// Choose best avoidance direction
		if (heading < 0.79) {
			incrementForAvoidance = -15;
		} else if (heading > 2.36) {
			incrementForAvoidance = 15;
		} else {
			chooseRandomIncrementAvoidance();
		}
		return 0;
	}

	// Drone is above y = 90  (danger zone)
	if (getPositionY() > 90 && heading < 0){
		printf("F: y 90   ");
		// Choose best avoidance direction
		if (heading > -0.79) {
			incrementForAvoidance = 15;
		} else if (heading < -2.36) {
			incrementForAvoidance = -15;
		} else {
			chooseRandomIncrementAvoidance();
		}
		return 0;
	}

	// Drone is below x = 10 (danger zone)
	if (getPositionX() < 10 && (heading < - 1.57 || heading > 1.57)) {
		printf("F: x 10   ");
		// Choose best avoidance direction
		if (heading < -2.36) {
			incrementForAvoidance = 15;
		} else if (heading > 2.36) {
			incrementForAvoidance = -15;
		} else {
			chooseRandomIncrementAvoidance();
		}
		return 0;
	}

	// Drone is above x = 90 (danger zone)
	if (getPositionX() > 90 && (heading > - 1.57 && heading < 1.57)) {
		printf("F: x 90   ");
		// Choose best avoidance direction
		if (heading < -0.79) {
			incrementForAvoidance = -15;
		} else if (heading > 0.79) {
			incrementForAvoidance = 15;
		} else {
			chooseRandomIncrementAvoidance();
		}
		return 0;
	}

	// Otherwise you can go forwards
	printf("F: yes    ");
	return 1;
}

/*
 * Get the maximum value of the boundary
 */
int getBoundaryMaxVal() {
	int max = 0;
	for (int i = 0; i < 48; i ++) {
		if (boundary_smoothed_2[i] > max) {
			max = boundary_smoothed_2[i];
		}
	}
	return max;
}

/*
 * Get the minimum value of the boundary in the center region
 */
int getBoundaryMinVal() {
	int min = 240;
	for (int i = 16; i < 33; i ++) {
		if (boundary_smoothed_2[i] < min) {
			min = boundary_smoothed_2[i];
		}
	}
	return min;
}

/*
 * Get the index of the item that is returned by getBoundaryMaxVal()
 */
int getBoundaryMaxPosX(int max) {
	for (int i = 0; i < 48; i ++) {
		if (boundary_smoothed_2[i] == max) {
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
//		boundary_smoothed[i-2] = 0.2 * (boundary[i-2] + boundary[i-1] + boundary[i] + boundary[i+1] + boundary[i+2]);
		boundary_smoothed_2[i-1] = (boundary[i-1] + boundary[i] + boundary[i+1]) / 3;
	}
	return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees) {
	struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
	int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL( incrementDegrees / 180.0 * M_PI);
	// Check if your turn made it go out of bounds...
	INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
	*heading = newHeading;
	return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters, int angle) {
	struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
	struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
	// Calculate the sine and cosine of the heading the drone is keeping
	float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi) + (3.141593*angle)/180.0);
	float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi) + (3.141593*angle)/180.0);
	// Now determine where to place the waypoint you want to go to
	new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor) {
	waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
	return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters) {
	struct EnuCoor_i new_coor;
	calculateForwards(&new_coor, distanceMeters, 0);
	moveWaypoint(waypoint, &new_coor);
	return false;
}

/*
 * Moves waypoint at distanceMeters ahead of the drone with a specific angle
 */
void moveWaypointForwardAngle(uint8_t waypoint, float distanceMeters, int angle) {
	struct EnuCoor_i new_coor;
	calculateForwards(&new_coor, distanceMeters, angle);
	moveWaypoint(waypoint, &new_coor);
	return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance() {
	// Only set the first time the drone can not go forwards
	if (!prevCanGoForwards) {
		// If this is not the first time, don't change incrementForAvoidance
		return false;
	}
	// Randomly choose CW or CCW avoiding direction
	int r = rand() % 2;
	if (r == 0) {
		incrementForAvoidance = 10.0;
	} else {
		incrementForAvoidance = -10.0;
	}
	return false;
}

