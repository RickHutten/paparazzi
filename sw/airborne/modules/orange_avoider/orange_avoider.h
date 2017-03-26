/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H
#include <inttypes.h>
#include "state.h"

extern uint8_t safeToGoForwards;
extern float incrementForAvoidance;
extern uint16_t trajectoryConfidence;

extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

float getMoveDistance(void);
void setThreshold(void);
char getCanGoForwards(void);
int getBoundaryMaxVal(void);
int getBoundaryMaxPosX(int);
void createSmoothedBoundary(void);
void moveWaypointForwardAngle(uint8_t, float, int);
float getHeading(void);
float getPositionX(void);
float getPositionY(void);

uint8_t calculateForwards(struct EnuCoor_i *, float, int);
extern uint8_t moveWaypointForward(uint8_t, float);
extern uint8_t moveWaypoint(uint8_t, struct EnuCoor_i *);
extern uint8_t increase_nav_heading(int32_t *, float);
extern uint8_t chooseRandomIncrementAvoidance(void);

#endif
