#include "header.h"
#include <vector>
#include <array>
#include <algorithm>
#include <iterator>
#include "byte_stuff.hpp"


// send data over UART
void sendData(void) { //TODO: restructure function to take number of floats, etc.
    // set up data structures !!! seems to be exporting mcu_rx (type 9, size 15) but no others; numbers e-38
    int n_float_tx = 5; //15; // number of floats to transmit
    int n8_tx = n_float_tx*4; // number of data bytes to transmit
    uint8_t arr8_unstuff_tx[252]; //[n8_tx]; // array for unstuffed data
    uint8_t arr8_stuff_tx[256]; // array for stuffed data


    // separate, stuff, and send data
    SeparateArr(uart_tx, n_float_tx, arr8_unstuff_tx);  // separate floats into bytes
    StuffArr(arr8_unstuff_tx, n8_tx, arr8_stuff_tx); // create stuffed byte packet

    uint8_t arr_un_rx[252];
    static float arr_f[63];

    UnstuffArr(arr8_stuff_tx, 13, arr_un_rx);
    ConcatArr(arr_un_rx, 13-2,arr_f);

    //printf("stuff %u %u %u %u %u %u %u %u %u %u %u %u %u %u \n",arr8_stuff_tx[0], arr8_stuff_tx[1],arr8_stuff_tx[2],arr8_stuff_tx[3],arr8_stuff_tx[4],arr8_stuff_tx[5],arr8_stuff_tx[6],arr8_stuff_tx[7],arr8_stuff_tx[8],arr8_stuff_tx[9],arr8_stuff_tx[10],arr8_stuff_tx[11],arr8_stuff_tx[12],arr8_stuff_tx[13]);
    //printf("unstuff %u %u %u %u %u %u %u %u %u %u %u %u %u %u \n",arr8_unstuff_tx[0], arr8_unstuff_tx[1],arr8_unstuff_tx[2],arr8_unstuff_tx[3],arr8_unstuff_tx[4],arr8_unstuff_tx[5],arr8_unstuff_tx[6],arr8_unstuff_tx[7],arr8_unstuff_tx[8],arr8_unstuff_tx[9],arr8_unstuff_tx[10],arr8_unstuff_tx[11],arr8_unstuff_tx[12],arr8_unstuff_tx[13]);

    //printf("arr_f %f %f %f \n",arr_f[0], arr_f[1],arr_f[2]);
    for (int i = 0; i < (n8_tx+2); i++) { // loop over data bytes + 2 stuff bytes
        MAP_UART_transmitData(EUSCI_A0_BASE, arr8_stuff_tx[i]); // transmit byte


        while (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
                != EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG); // wait for transmission completion
    }
}


// receive data over UART: eUSCI A module interrupt routine
void EUSCIA0_IRQHandler(void) {
    MAP_Interrupt_disableMaster(); // disable all interrupts

    static uint8_t arr8_stuff_rx[256]; // array for stuffed data
    static uint8_t arr8_unstuff_rx[252]; // array for unstuffed data
    static float arr_float_cat_rx[63]; // array for concatenated data
    static uint8_t buf_rx; // byte receive buffer
    static int n8_rx = 0; // number of bytes received in packet

    if (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
            == EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG) { // if receive interrupt flag

        buf_rx = MAP_UART_receiveData(EUSCI_A0_BASE); // access received data by reading RXBUF register; flag automatically cleared
        if (buf_rx == 0) { // if 0 received
            arr8_stuff_rx[n8_rx] = buf_rx; // add byte to array
            n8_rx++; // increment number of bytes in packet

            UnstuffArr(arr8_stuff_rx, n8_rx, arr8_unstuff_rx); // unstuff data in packet (auto clears unstuff vector)
            ConcatArr(arr8_unstuff_rx, n8_rx-2, arr_float_cat_rx); // concatenate bytes into uint16_ts (auto clears cat vector)
            int n_float_rx = (n8_rx-2)/4; // number of floats received
            std::copy(arr_float_cat_rx, arr_float_cat_rx + n_float_rx, uart_rx); // add received data into global array
            n8_rx = 0; // reset packet length
            updateValuesFlag = 1; // set flag to update values
        }
        else {
            arr8_stuff_rx[n8_rx] = buf_rx; // add byte to array
            n8_rx++; // increment number of bytes in packet
        }
    }

    MAP_Interrupt_enableMaster(); // enable all interrupts
}


// update values based on received UART data
void updateValues(void) { // TODO: change function name?
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);

    switch ((int)uart_rx[0] & 255) { // LSbyte indicates message type
        volatile float *start;

        case 0: { // update pressure setpoints
            start = uart_rx+1;
            std::copy(start, start+NUM_PRES_SENSOR, pres_des);
            break;
        }
        case 1: { // update pressure controller gains
            const int ctrl_idx = ((int)uart_rx[0]) >> 8 & 255; // 2nd LSbyte indicates index of pressure controller
            start = uart_rx+1;
            std::copy(start, start+4, ctrl_params[ctrl_idx]);
            break;
        }
        case 2: { // update length estimation coefficients TODO
            break;
        }
        case 3: { // update force estimation coefficients TODO
            break;
        }
    }
}
