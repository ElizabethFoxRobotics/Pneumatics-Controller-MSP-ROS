#include "header.h"

// PID pressure control: maintain desired pressure setpoints
void controlUpdate(void) {
    float err;
    static float errInt[NUM_PRES_SENSOR] = {0};
    float u;
    float duty;

    float pgain = 0.1;//.01;
    float dgain = 0.0;
    float igain = 0.0001;
    float offset = 6; // !!! consider adding d parameter

    // loop over pressure control inputs
    int i;
    for (i = 1; i < NUM_PRES_SENSOR; i++) { //0 is tank sensor which has no control


        err = presFilt[i][0] - pres_des[i]; // (psi) calculate pressure error
        errInt[i] = errInt[i] + err; // (psi-time) integrate pressure error


        // reset integral windup
        if (errInt[i]*igain > 1) {
            errInt[i] = 1;///ctrl_params[i][1];
        }
        /*else if (errInt[i]*igain < -1) {
            errInt[i] = -1;//ctrl_params[i][1];
        }*/

        // calculate raw input
        u = 23- ctrl_params[i][0]*err;//(ctrl_params[i][2]*pres_des[i] + 23) - ctrl_params[i][0]*err; // - ctrl_params[i][1]*errInt[i] ; // input: (ff + deadzone offset) + prop + int
        duty = u;

        //printf("duty %f err %f \n", duty, err);
        // impose saturation limits
        if (duty > 100.0) {
            duty = 100.0;
        }
        else if (duty < 20.0) {
            duty = 0.0;
        }

        // if desired pressure is zero or already at desired pressure, just set duty cycle to zero
        if (pres_des[i] == 0 || pres_des[i] < presFilt[i][0]) {
            duty = 0.0;
            errInt[i] = 0; // reset integral
        }

        // !!! also test on nonleaking/unkinked pam
        if (err < 0){
            duty =  5;//offset- pgain*err -dgain*fabs(err-prev_error[i]) -igain*errInt[i];
            if (err > -10){
                duty = 0;
            }
            else if (duty > 100){
                duty = 100;
            }
        }
        else{
            duty =-4.5;//offset - pgain*err +dgain*fabs(err-prev_error[i]);
            errInt[i] = 0;
            if (err < 10){
                duty = 0;
            }

        }


        prev_error[i] = (prev_error[i]+err)/2.;

        /*
        if (ctrlcount > 400){ //4
            duty = 10;
            ctrlcount = 0;
            sendDataCount = 1;
        }
        else{
            ctrlcount+=1;
            duty = 0;
            sendDataCount = 1;
        }*/
        sendDataCount = 1;
        //printf("pressure:  %f     duty %f \n", presFilt[1][0], duty);
        // update PWM timer duty cycle
        if (i == 1) {
            uart_tx[3] = err; // !!! something wrong with reported error
            uart_tx[4] = duty;

            //xprintf("uart_tx %f %f %f %f %f \n",uart_tx[0],uart_tx[1],uart_tx[2],uart_tx[3],uart_tx[4]);
            if (duty > 0){ //inflate
                TIMER_A0->CCR[1] = (int)((duty/100.0)*PWM_PERIOD);
                TIMER_A0->CCR[2] = ((0/100.0)*PWM_PERIOD);
            }
            else{
                //printf("Vent");
                TIMER_A0->CCR[1] = 0;
                TIMER_A0->CCR[2] = ((-duty/100.0)*PWM_PERIOD);  //!!! venting acting weird seems to vent more after duty0
                //def vents more at higher pressures
            }
            //TIMER_A0->CCR[2] = (int)((duty/100.0)*PWM_PERIOD);
            //TIMER_A0->CCR[3] = (int)((duty/100.0)*PWM_PERIOD);
        }
        else{
            TIMER_A0->CCR[i] = 0;//(int)((duty/100.0)*PWM_PERIOD);
        }
    }




    //    // pump control
    //    if (presFilt[4][0] > 14) { // 10 for free actuation
    //        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN6 | GPIO_PIN7);// disable pumps
    //    }
    //    else if (presFilt[4][0] < 12) { // 8 for free actuation
    //        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN6 | GPIO_PIN7); // enable pumps
    //    }
}


// control update timer interrupt routine
extern "C" void TA2_0_IRQHandler(void) {
    MAP_Interrupt_disableMaster(); // disable interrupts
    controlFlag = 1; // set flag to get update system control
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0); // clear interrupt flag
    MAP_Interrupt_enableMaster(); // enable interrupts
}

