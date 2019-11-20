/*
 * StepperMotor.c
 *
 *  Created on: Nov 12, 2019
 *      Author: Mike
 */

#include "StepperMotor.h"
#include "main.h"
#include "driverlib/driverlib.h"

/*
 * Moves the specified axis in the specified direction the specified number of steps
 * Axis axis: the axis you want to move
 * Direction direction: the direction you want to move
 * int steps: the number of steps you want to move
 * return: void when the move is complete
 */


const int xBacklashSteps = 0;
const int yBacklashSteps = 0;

void move(Axis axis, Direction direction, int steps){
    // Return if incorrect axis, direction combo
    if((axis == X && direction == UP) ||
       (axis == X && direction == DOWN) ||
       (axis == Y && direction == RIGHT) ||
       (axis == Y && direction == LEFT)){
        return;
    }

    // Take up backlash if needed
    if((axis == X && direction != xMotor.lastDir) ||
       (axis == Y && direction != yMotor.lastDir)){
        takeUpBacklash(axis, direction);
    }

    // Move the desired amount in the desired direction
    if(direction == UP || direction == RIGHT){
        int i;
        for(i = steps; i > 0; i--){
            forwardStep(axis);
        }
    }
    else if (direction == DOWN || direction == LEFT){
        int i;
        for(i = steps; i > 0; i--){
            backwardStep(axis);
        }
    }

}

/*
 * Moves specified axis one step forward or right
 * Axis axis: the axis you want to move
 * return: void when the step is complete
 */
void forwardStep(Axis axis){
    int phase = 1;
    int i = 20;

    // Return if hall effect sensor is triggered in the direction of movement
    if((GPIO_getInputPinValue(HE_X2_PORT, HE_X2_PIN) == 0 && axis == X)||
       (GPIO_getInputPinValue(HE_Y1_PORT, HE_Y1_PIN) == 0 && axis == Y)){
        return;
    }

    if(axis == X){
        GPIO_setOutputHighOnPin(STEPPER_EnX_ENY_PORT, STEPPER_EnX_ENY_PIN);
        xMotor.curPos++;
        xMotorCounter++;
    }else if(axis == Y){
        GPIO_setOutputLowOnPin(STEPPER_EnX_ENY_PORT, STEPPER_EnX_ENY_PIN);
        yMotor.curPos++;
        yMotorCounter++;
    }

    if (xMotorCounter == stepsPerMM ) {
        current_coordinate.x++;
        xMotorCounter = 0;
    }
    if (yMotorCounter == stepsPerMM ) {
       current_coordinate.y++;
       yMotorCounter = 0;
   }

    displayCoordinates(current_coordinate);

    while(phase <= 4){
        switch(phase){
            case 1:
                GPIO_setOutputHighOnPin(STEPPER_A_PORT, STEPPER_A_PIN);
                GPIO_setOutputHighOnPin(STEPPER_B_PORT, STEPPER_B_PIN);
                GPIO_setOutputLowOnPin(STEPPER_C_PORT, STEPPER_C_PIN);
                GPIO_setOutputLowOnPin(STEPPER_D_PORT, STEPPER_D_PIN);
                break;
            case 2:
                GPIO_setOutputLowOnPin(STEPPER_A_PORT, STEPPER_A_PIN);
                GPIO_setOutputHighOnPin(STEPPER_B_PORT, STEPPER_B_PIN);
                GPIO_setOutputHighOnPin(STEPPER_C_PORT, STEPPER_C_PIN);
                GPIO_setOutputLowOnPin(STEPPER_D_PORT, STEPPER_D_PIN);
                break;
            case 3:
                GPIO_setOutputLowOnPin(STEPPER_A_PORT, STEPPER_A_PIN);
                GPIO_setOutputLowOnPin(STEPPER_B_PORT, STEPPER_B_PIN);
                GPIO_setOutputHighOnPin(STEPPER_C_PORT, STEPPER_C_PIN);
                GPIO_setOutputHighOnPin(STEPPER_D_PORT, STEPPER_D_PIN);
                break;
            case 4:
                GPIO_setOutputHighOnPin(STEPPER_A_PORT, STEPPER_A_PIN);
                GPIO_setOutputLowOnPin(STEPPER_B_PORT, STEPPER_B_PIN);
                GPIO_setOutputLowOnPin(STEPPER_C_PORT, STEPPER_C_PIN);
                GPIO_setOutputHighOnPin(STEPPER_D_PORT, STEPPER_D_PIN);
                break;
        }

        if(i > 0){
            i--;
        } else {
            phase++;
            i = 20;
        }
    }

    //GPIO_setOutputLowOnPin(STEPPER_A_PORT, STEPPER_A_PIN);
    //GPIO_setOutputLowOnPin(STEPPER_B_PORT, STEPPER_B_PIN);
    //GPIO_setOutputLowOnPin(STEPPER_C_PORT, STEPPER_C_PIN);
    //GPIO_setOutputLowOnPin(STEPPER_D_PORT, STEPPER_D_PIN);
}

/*
 * Moves specified axis one step backward or left
 * Axis axis: the axis you want to move
 * return: void when the step is complete
 */
void backwardStep(Axis axis){
    int phase = 1;
    int i = 20;

    // Return if hall effect sensor is triggered in the direction of movement
    if((GPIO_getInputPinValue(HE_X1_PORT, HE_X1_PIN) == 0 && axis == X)||
       (GPIO_getInputPinValue(HE_Y2_PORT, HE_Y2_PIN) == 0 && axis == Y)){
        return;
    }

    if(axis == X){
        GPIO_setOutputHighOnPin(STEPPER_EnX_ENY_PORT, STEPPER_EnX_ENY_PIN);
        xMotor.curPos--;
        xMotorCounter++;
    }else if(axis == Y){
        GPIO_setOutputLowOnPin(STEPPER_EnX_ENY_PORT, STEPPER_EnX_ENY_PIN);
        yMotor.curPos--;
        yMotorCounter++;
    }

    if (xMotorCounter == stepsPerMM ) {
        current_coordinate.x--;
        xMotorCounter = 0;
    }
    if (yMotorCounter == stepsPerMM ) {
       current_coordinate.y--;
       yMotorCounter = 0;
   }
    displayCoordinates(current_coordinate);

    while(phase <= 4){
        switch(phase){
            case 1:
                GPIO_setOutputHighOnPin(STEPPER_A_PORT, STEPPER_A_PIN);
                GPIO_setOutputLowOnPin(STEPPER_B_PORT, STEPPER_B_PIN);
                GPIO_setOutputLowOnPin(STEPPER_C_PORT, STEPPER_C_PIN);
                GPIO_setOutputHighOnPin(STEPPER_D_PORT, STEPPER_D_PIN);
                break;
            case 2:
                GPIO_setOutputLowOnPin(STEPPER_A_PORT, STEPPER_A_PIN);
                GPIO_setOutputLowOnPin(STEPPER_B_PORT, STEPPER_B_PIN);
                GPIO_setOutputHighOnPin(STEPPER_C_PORT, STEPPER_C_PIN);
                GPIO_setOutputHighOnPin(STEPPER_D_PORT, STEPPER_D_PIN);
                break;
            case 3:

                GPIO_setOutputLowOnPin(STEPPER_A_PORT, STEPPER_A_PIN);
                GPIO_setOutputHighOnPin(STEPPER_B_PORT, STEPPER_B_PIN);
                GPIO_setOutputHighOnPin(STEPPER_C_PORT, STEPPER_C_PIN);
                GPIO_setOutputLowOnPin(STEPPER_D_PORT, STEPPER_D_PIN);
                break;
            case 4:
                GPIO_setOutputHighOnPin(STEPPER_A_PORT, STEPPER_A_PIN);
                GPIO_setOutputHighOnPin(STEPPER_B_PORT, STEPPER_B_PIN);
                GPIO_setOutputLowOnPin(STEPPER_C_PORT, STEPPER_C_PIN);
                GPIO_setOutputLowOnPin(STEPPER_D_PORT, STEPPER_D_PIN);
                break;
        }

        if(i > 0){
            i--;
        } else {
            phase++;
            i = 20;
        }
    }

    //GPIO_setOutputLowOnPin(STEPPER_A_PORT, STEPPER_A_PIN);
    //GPIO_setOutputLowOnPin(STEPPER_B_PORT, STEPPER_B_PIN);
    //GPIO_setOutputLowOnPin(STEPPER_C_PORT, STEPPER_C_PIN);
    //GPIO_setOutputLowOnPin(STEPPER_D_PORT, STEPPER_D_PIN);
}

/*
 * Takes up the backlash on the specified axis in the specified direction
 * Axis axis: the axis to take up backlash on
 * Direction direction: the direction to take up backlash
 * return: void when the backlash is taken up
 */
void takeUpBacklash(Axis axis, Direction direction){
    if(axis == X){
        xMotor.lastDir = direction;
        move(axis, direction, xBacklashSteps);
    }
    else if(axis == Y){
        yMotor.lastDir = direction;
        move(axis, direction, yBacklashSteps);
    }
}

void pointToPoint(int xDestination, int yDestination){
    int xSteps;
    int ySteps;
    Direction xDir;
    Direction yDir;
    int stepRatio;
    int xLeftoverSteps;
    int yLeftoverSteps;
    int iterations;

    // Convert mm destination to steps
    xSteps = (xDestination * stepsPerMM ) - xMotor.curPos;
    ySteps = (yDestination * stepsPerMM ) - yMotor.curPos;

    // Setup directions
    xDir = RIGHT;
    yDir = UP;

    if(xSteps < 0){
        xDir = LEFT;
        xSteps *= -1;
    }
    if(ySteps < 0){
        yDir = DOWN;
        ySteps *= -1;
    }

    // Determine step ratio, x:1 or 1:y, and leftover steps
    if(xSteps > ySteps){
        stepRatio = xSteps/ySteps;
        xLeftoverSteps = xSteps % stepRatio;
        yLeftoverSteps = 0;
        iterations = ySteps;

        xMotor.stepRatio = stepRatio;
        yMotor.stepRatio = 1;
    }else{
        stepRatio = ySteps/xSteps;
        xLeftoverSteps = 0;
        yLeftoverSteps = ySteps % stepRatio;
        iterations = xSteps;

        xMotor.stepRatio = 1;
        yMotor.stepRatio = stepRatio;
    }

    // Move
    int i;
    for(i = iterations; i > 0; i--){
        move(X, xDir, xMotor.stepRatio);
        move(Y, yDir, yMotor.stepRatio);
    }
    move(X, xDir, xLeftoverSteps);
    move(Y, yDir, yLeftoverSteps);
}
