//*****************************************************************************
//
// MSP432 main.c - 2-DOF linkage PAM control
//
// Lucas Tiziani
// 20 October 2017
//
//****************************************************************************

/* Program outline
 *
 *    - 5x ADC for pressure transducers (minimum)
 *    - 8x ADC for optical sensors
 *    - 6x external interrupts (3 per encoder)
 *    - 4x PWM signals
 *    - 1x timer for sensing loop
 *    - 1x timer for control loop
 *    - UART for communication with Python
 *
 */

//TODO: append f to floats

#include "header.h"

void PORT3_IRQHandler(void);
extern "C" void TA1_0_IRQHandler(void);
extern "C" void TA2_0_IRQHandler(void);

void main(void) { //!!! seems to be working now, just need to fix control settings. filter turned off
    MAP_WDT_A_holdTimer(); // hold the watchdog timer (stop from running)
    MAP_Interrupt_disableMaster(); // disable interrupts
    MAP_FPU_enableModule();

    configClocks();
    configPins();
    configTimers();

    Analog_Config();
    Enc_Config();

    configUart();
//    configInterrupts();
    startTimers();

    MAP_Interrupt_enableMaster(); // enable interrupts
    initEncoder(); // initialize encoders
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    int flag_step = 0;
    int ii;
    for(ii=0; ii < NUM_PRES_SENSOR; ii++){
        ctrl_params[ii][0] =0.1;
        ctrl_params[ii][1] =0.03;
        ctrl_params[ii][2] =0.30;
        ctrl_params[ii][3] =50.0;
    }
    pres_options[0] = 0;
    pres_options[1]=100.;
    pres_options[2]=200.;
    pres_options[3]=300.;

    pres_des[1]=1;
    int pres_index = 0;
    for (int ii = 1; ii < NUM_PRES_SENSOR; ii++) {
        prev_error[ii] = -1;
    }
    while(1) {
        if (sensorFlag) {
            sensorUpdate(); // get sensor data
            sensorFlag = 0; // clear sensor flag

        }
        if (controlFlag) {
            controlUpdate(); // update control
            controlFlag = 0; // clear control flag
            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);
            if (sendDataCount == 1) {
                sendData(); // send sensor data
                sendDataCount = 0; // reset sensor flag
            }
            //sendDataCount++; // increment sensor flag
        }
        if (updateValuesFlag) {
            //printf("update \n");
            updateValues(); // update pressure setpoint values
            updateValuesFlag = 0; // clear values update flag
        }
        if (MAP_GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0) {
            pres_des[1] = 0;
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); // turn on green LED}
        }
        else{
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); // turn off green LED
        }
        if (MAP_GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == 0) {
            flag_step = 1;
        }
        else if (flag_step > 0){
            flag_step = 0;
            pres_index +=1;
            if (pres_index ==4){
                pres_index = 0;
            }
            pres_des[1] = pres_options[pres_index];
        }
    }


}

