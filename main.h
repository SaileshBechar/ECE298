#ifndef MAIN_H_
#define MAIN_H_

#include "driverlib/driverlib.h"
#include "StepperMotor.h"

//Stepper Ports/Pins
#define STEPPER_A_PORT  GPIO_PORT_P5
#define STEPPER_A_PIN   GPIO_PIN2
#define STEPPER_B_PORT  GPIO_PORT_P5
#define STEPPER_B_PIN   GPIO_PIN3
#define STEPPER_C_PORT  GPIO_PORT_P1
#define STEPPER_C_PIN   GPIO_PIN3
#define STEPPER_D_PORT  GPIO_PORT_P1
#define STEPPER_D_PIN   GPIO_PIN4
#define STEPPER_EnX_ENY_PORT  GPIO_PORT_P1
#define STEPPER_EnX_ENY_PIN   GPIO_PIN5

//Hall Effect Ports/Pins
#define HE_X1_PORT  GPIO_PORT_P8    //Left
#define HE_X1_PIN   GPIO_PIN1
#define HE_X2_PORT  GPIO_PORT_P2    //Right
#define HE_X2_PIN   GPIO_PIN7
#define HE_Y1_PORT  GPIO_PORT_P8    //Up
#define HE_Y1_PIN   GPIO_PIN0
#define HE_Y2_PORT  GPIO_PORT_P2    //Down
#define HE_Y2_PIN   GPIO_PIN5

//PWM
#define TIMER_A_PERIOD  20000 //T = 1/f = (TIMER_A_PERIOD * 1 us)
#define HIGH_COUNT      2000  //Number of cycles signal is high (Duty Cycle = HIGH_COUNT / TIMER_A_PERIOD)

//Output pin to buzzer
#define PWM_PORT        GPIO_PORT_P1
#define PWM_PIN         GPIO_PIN7
//LaunchPad LED1 - note unavailable if UART is used
#define LED1_PORT       GPIO_PORT_P1
#define LED1_PIN        GPIO_PIN0
//LaunchPad LED2
#define LED2_PORT       GPIO_PORT_P4
#define LED2_PIN        GPIO_PIN0
//LaunchPad Pushbutton Switch 1
#define SW1_PORT        GPIO_PORT_P1
#define SW1_PIN         GPIO_PIN2
//LaunchPad Pushbutton Switch 2
#define SW2_PORT        GPIO_PORT_P2
#define SW2_PIN         GPIO_PIN6
//Input to ADC - in this case input A9 maps to pin P8.1
#define ADC_IN_PORT     GPIO_PORT_P8
#define ADC_IN_PIN      GPIO_PIN1
#define ADC_IN_CHANNEL  ADC_INPUT_A9

int stepsPerMM;

typedef struct buffer {
    int x;
    int y;
} COORDINATE;

COORDINATE current_coordinate;

void waitForButtonRelease(uint8_t, uint16_t, int);
void runStepper();
void displayUART();
void displayText(char *);
void handleUART();
int inputToCoor(char* input, int num_bytes_to_copy);
int insertCoordinate(int num, char pos);
int removeCoordinate(COORDINATE* coordinate);
void displayCoordinates(COORDINATE curr_coordinate);
void jog();
int fetch_coordinate(COORDINATE* curr_pos);

void Init_GPIO(void);
void Init_Clock(void);
void Init_UART(void);
void Init_PWM(void);
void Init_ADC(void);

Timer_A_outputPWMParam param; //Timer configuration data structure for PWM

#endif /* MAIN_H_ */
