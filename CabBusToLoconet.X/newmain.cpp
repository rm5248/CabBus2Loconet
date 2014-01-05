/* 
 * File:   newmain.cpp
 * Author: Rob
 *
 * Created on December 7, 2013, 10:10 AM
 */

#include <cstdlib>
#include <p32xxxx.h>
#include <plib.h>
#include <string.h>

#include "LocoNet.h"
#include "Delay.h"
#include "config.h"

/*
 * 
 */
int main(int argc, char** argv) {
    unsigned int on;
    unsigned int pos;

    PORTBbits.RB5 = 0; //clear bit
    TRISBbits.TRISB5 = 0; //set as output

    // C function, C++ is being picky about the enums here
    doUART1Config();

    lnMsg message;
    memset(&message, 0x55, 16);

    on = 0;

    while (1) {
        pos = 0;
        DelayMs(100);
        if (on) {
            PORTBbits.RB5 = 1;
        } else {
            PORTBbits.RB5 = 0;
        }
        on = !on;

        while (pos < 16) {
            while (!UARTTransmitterIsReady(UART1))
                ;

            UARTSendDataByte(UART1, message.data[pos]);

            while (!UARTTransmissionHasCompleted(UART1))
            ;

            pos++;
        }

        while (!UARTTransmissionHasCompleted(UART1))
            ;
    }

    return 0;
}

