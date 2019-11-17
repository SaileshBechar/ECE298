#include "main.h"
#include "string.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include "Board.h"
#include "StepperMotor.h"
#include <stdlib.h>
#include <stdbool.h>

/*
 * Matyszczuk, Michael
 * Bechar, Sailesh
 * Code for XY-Plotter for ECE 298: Instrumentation and Prototyping Labratory
 */

#define MAX_INPUT_SIZE 10

char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result

volatile int8_t changeMode = 1;
volatile char coor_recieved;
char temp_buf[6] = "000000";
volatile unsigned char curr_buf_pos = 0;
volatile int8_t Stepper_EnA_ENB = 0;

bool print_toggle = false;

typedef struct buffer {
    int x;
    int y;
} COORDINATE;

COORDINATE input_coordinates[MAX_INPUT_SIZE];
unsigned char first_element_pos = 0;
unsigned char last_element_pos = 0;
int coordinate_size = 0;

void handleUART();
int inputToCoor(char* input, int num_bytes_to_copy);
int insertCoordinate(int num, char pos);
int removeCoordinate(COORDINATE* coordinate);
void printCoordinates();

void main(void)
{
    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_PWM();     //Sets up a PWM output
    //Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display

    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();

    displayScrollText("ECE 298");

    //Initialize Variables
    volatile unsigned int i;
    enum Mode {STEPPER, UART, NONE};
    enum Mode mode = STEPPER;

    //Stepper Motor
    GPIO_setAsOutputPin(STEPPER_A_PORT, STEPPER_A_PIN);
    GPIO_setAsOutputPin(STEPPER_B_PORT, STEPPER_B_PIN);
    GPIO_setAsOutputPin(STEPPER_C_PORT, STEPPER_C_PIN);
    GPIO_setAsOutputPin(STEPPER_D_PORT, STEPPER_D_PIN);
    GPIO_setAsOutputPin(STEPPER_EnX_ENY_PORT, STEPPER_EnX_ENY_PIN);

    //HallEffect
    GPIO_setAsInputPinWithPullUpResistor(HE_X1_PORT, HE_X1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(HE_X2_PORT, HE_X2_PIN);
    GPIO_setAsInputPinWithPullUpResistor(HE_Y1_PORT, HE_Y1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(HE_Y2_PORT, HE_Y2_PIN);

    clearLCD();

    while(1){

        /*while(1){
            move(X, RIGHT, 500);
            GPIO_setOutputHighOnPin(LED2_PORT, LED2_PIN);
            move(Y, UP, 500);
            GPIO_setOutputLowOnPin(LED2_PORT, LED2_PIN);
            move(X, LEFT, 500);
            GPIO_setOutputHighOnPin(LED2_PORT, LED2_PIN);
            move(Y, DOWN, 500);
            GPIO_setOutputLowOnPin(LED2_PORT, LED2_PIN);
            pointToPoint(0, 20);
            GPIO_setOutputHighOnPin(LED2_PORT, LED2_PIN);
            pointToPoint(0, -20);
            GPIO_setOutputLowOnPin(LED2_PORT, LED2_PIN);
            int i;
            for(i = 100; i > 0; i--){
                move(X, RIGHT, 1);
                GPIO_setOutputHighOnPin(LED2_PORT, LED2_PIN);
                move(Y, UP, 4);
                GPIO_setOutputLowOnPin(LED2_PORT, LED2_PIN);
            }
            for(i = 100; i > 0; i--){
                move(X, LEFT, 1);
                GPIO_setOutputHighOnPin(LED2_PORT, LED2_PIN);
                move(Y, DOWN, 4);
                GPIO_setOutputLowOnPin(LED2_PORT, LED2_PIN);
            }
        }*/
        // Changes the mode and displays on LCD
        if(changeMode){
            changeMode = 0;

            switch(mode){
                case STEPPER:
                    clearLCD();
                    displayText("STEP");
                    runStepper();
                    mode = UART;
                    break;
                case UART:
                    clearLCD();
                    displayUART();
                    mode = STEPPER;
                    break;
                default:
                    break;

            }
        }
    }
}

/*
 * Polls an input pin, returns when the value changes.
 * uint8_t port: port
 * uint16_t pin: pin
 * int currentState: the state of the pin when the function was entered
 * return: void when the pin state != currentState
 */
void waitForButtonRelease(uint8_t port, uint16_t pin, int currentState){
    while(GPIO_getInputPinValue(port, pin) == currentState){
    }
}

/*
 * Run through all of the coordinates that have been entered and move to them one by one.
 * return: void when all coordinates have been moved to
 */
void runStepper(){
    COORDINATE destination;

    int i;
    for(i = coordinate_size; i>0; i--){
        removeCoordinate(&destination);
        pointToPoint(destination.x, destination.y);
    }
    return;
}

/*
 * Displays text on the LCD
 * char *msg: the text that you want to display
 * return: void after displaying the text
 */
void displayText(char *msg){
    int length = strlen(msg);
    int i;
    char buffer[6] = "      ";
    for (i=0; i<6; i++){
        buffer[i] = ' ';
    }
    for (i=0; i<length; i++){
        buffer[i] = msg[i];
    }

    showChar(buffer[0], pos1);
    showChar(buffer[1], pos2);
    showChar(buffer[2], pos3);
    showChar(buffer[3], pos4);
    showChar(buffer[4], pos5);
    showChar(buffer[5], pos6);
}

/*
 * Displays the last 6 digits entered into the UART on the LCD
 * return: void after displaying the text
 */
void displayUART() {
    while (1) {
        coor_recieved = 0;
        displayText(temp_buf);
        handleUART();
        printCoordinates();
        if(changeMode == 1){
            break;
        }
    }
}

void handleUART() {
    if ((coor_recieved != 0 && coor_recieved >= '0' && coor_recieved <= '9') || coor_recieved == 13 || coor_recieved == 8) {

        if (coor_recieved == 13) {
            int x = inputToCoor(temp_buf, curr_buf_pos);
            int y = inputToCoor(&temp_buf[3], curr_buf_pos-3);
            if (insertCoordinate(x, 'X') && insertCoordinate(y, 'Y')) {
                curr_buf_pos = 0;
                memset(temp_buf, '0', 6);
            }
        }

        if (curr_buf_pos <= 5 && coor_recieved >= '0' && coor_recieved <= '9') {
            temp_buf[curr_buf_pos] = coor_recieved;
            curr_buf_pos++;
        }

        if (coor_recieved == 8) {
            curr_buf_pos--;
            temp_buf[curr_buf_pos] = '0';
        }

    }
}

int inputToCoor(char* input, int num_bytes_to_copy) {
    char three_digit_char[4];
    memset(three_digit_char, 0, 4);
    if (num_bytes_to_copy <= 0) {
        return 0;
    }
    if (num_bytes_to_copy > 3) {
        num_bytes_to_copy = 3;
    }
    memcpy(three_digit_char, input, num_bytes_to_copy);
    return atoi(three_digit_char);
}

/*removes top element and writes it to coordinate
 * return value is false if unsuccessful
 */
int removeCoordinate(COORDINATE* coordinate) {
    if (coordinate_size <= 0) {
        return -1;
    }
    *coordinate = input_coordinates[first_element_pos];

    if (first_element_pos == MAX_INPUT_SIZE){
        first_element_pos = 0;
    } else {
        first_element_pos++;
    }
    coordinate_size--;
    return 1;
}

/* inserts element to last position in queue
 * returns false if full
 */
int insertCoordinate(int num, char pos) {
    if (coordinate_size > MAX_INPUT_SIZE) {
        return -1;
    }
    if (pos == 'X') {
        input_coordinates[last_element_pos].x = num;
    }
    else if (pos == 'Y'){
    input_coordinates[last_element_pos].y = num;
        if (last_element_pos == MAX_INPUT_SIZE) {
            last_element_pos = 0;
        } else {
            last_element_pos++;
        }
        coordinate_size++;
    }

    return 1;
}

void printCoordinates() {
    while (coor_recieved == 'P') {
        char temp_coord[6];
        int temp_int;
        COORDINATE curr_coordinate;
        int returnval = 0;

        if (print_toggle == true) {
            returnval = removeCoordinate(&curr_coordinate);
            print_toggle = false;
        }

        if (returnval) {
            temp_int = curr_coordinate.x;
            temp_coord[2] = temp_int % 10 + '0';
            temp_int /= 10;
            temp_coord[1] = temp_int % 10 + '0';
            temp_int /= 10;
            temp_coord[0] = temp_int % 10 + '0';

            temp_int = curr_coordinate.y;
            temp_coord[5] = temp_int % 10 + '0';
            temp_int /= 10;
            temp_coord[4] = temp_int % 10 + '0';
            temp_int /= 10;
            temp_coord[3] = temp_int % 10 + '0';

            displayText(temp_coord);
        }
    }
}



/*************************************/
//Initialization
void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        if (EUSCI_A_UART_receiveData(EUSCI_A0_BASE) == 'N') {
           changeMode = 1;
        }
        else if (EUSCI_A_UART_receiveData(EUSCI_A0_BASE) == 'P') {
            coor_recieved = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
            print_toggle = true;
        }
        else {
            coor_recieved = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
        }
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    GPIO_setOutputHighOnPin(LED2_PORT, LED2_PIN);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}
