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
#include "CabBus.h"

struct LoconetData{
    uint8_t slot;
    uint8_t speed; // direction is the top bit
};

/*
 * 
 */
int main(int, char**) {
    unsigned int on;
    struct Cab* currentCab;
    LocoNetClass lnClass;
    lnMsg* incomingMessage;

    struct Cab* MY_CAB = NULL;

    currentCab = NULL;

    PORTBbits.RB5 = 0; //clear bit
    TRISBbits.TRISB5 = 0; //set as output

    // C function, C++ is being picky about the enums here
    // This is configuring our hardware for loconet.
    doUART1Config();

    // Init our cab
    initCabs();

    // Init loconet
    lnClass.init();

    // we can turn on interrupts now
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();

    on = 0;

    while (1) {
        if (on) {
            PORTBbits.RB5 = 1;
        } else {
            PORTBbits.RB5 = 0;
        }
        on = !on;

        currentCab = pingNextCab();
        if (currentCab != NULL) {
            // We got data back from the cab!
            //printf( "cab number is %d", currentCab->number );
            MY_CAB = currentCab;

            //Let's process this data and see if we need to send anything out
            //onto loconet
        }

        //UARTSendDataByte(UART1, 0x55);

        do {
            incomingMessage = lnClass.receive();
            if (incomingMessage != NULL) {
                if (incomingMessage->data[ 0 ] == OPC_LOCO_SPD) {
                    uint8_t speed = incomingMessage->lsp.spd;
                    uint8_t slot = incomingMessage->lsp.slot;

                    if (MY_CAB != NULL)
                        setCabSpeed(MY_CAB, speed);
                    //          printf("");
                }
            }
        } while (incomingMessage != NULL);

        DelayUs(100); //wait .1mS until next ping
    }

    return 0;
}

