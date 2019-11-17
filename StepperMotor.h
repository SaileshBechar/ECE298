/*
 * StepperMotor.h
 *
 *  Created on: Nov 12, 2019
 *      Author: Mike
 */

#ifndef CODESHELLV8_3_STEPPERMOTOR_H_
#define CODESHELLV8_3_STEPPERMOTOR_H_

typedef enum {UP, RIGHT, DOWN, LEFT} Direction;
typedef enum {CW, CCW} RotationalDirection;
typedef enum {X, Y} Axis;

typedef struct {
    Direction lastDir;
    int curPos;
    int stepRatio;
} StepperMotor;

void move(Axis, Direction, int);
void forwardStep(Axis);
void backwardStep(Axis);
void takeUpBacklash(Axis, Direction);
void pointToPoint(int, int);


#endif /* CODESHELLV8_3_STEPPERMOTOR_H_ */
