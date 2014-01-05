#include <string.h>


#include "Delay.h"

#include "CabBus.h"
#include "RS485Comm.h"

//Contains the data for all the cabs
static struct Cab allCabs[ 64 ];

static unsigned char currentCabAddr;
static unsigned char outputBuffer[ 10 ];

static const unsigned char NO_KEY = 0x7D;
static const unsigned char REPEAT_SCREEN = 0x7E;

void initCabs() {
    unsigned char x;

    for (x = 0; x < 64; x++) {
        memset(&allCabs[ x ], 0, sizeof ( struct Cab));
        allCabs[ x ].number = x;
    }

    currentCabAddr = 0;
}

struct Cab* pingNextCab() {
    currentCabAddr++;
    if (currentCabAddr == 1) currentCabAddr++; //don't ping address 1
    if (currentCabAddr == 2) currentCabAddr++; //don't ping address 2 either

    if (currentCabAddr == 64) currentCabAddr = 0; //only up to 63 cabs

    //Go and ping the next address
    outputBuffer[ 0 ] = 0x80 | currentCabAddr;

    SendDataBuffer(UART2, outputBuffer, 1);

    //Delay to make sure that we get a response back
    DelayMs( 2 );

    if (!U2STAbits.RIDLE || U2STAbits.URXDA) {
        //we have a response back from a cab
        unsigned char firstByte;
        unsigned char secondByte;
        unsigned int loopTimes = 0;

        firstByte = UARTGetDataByte(UART2);
        while (!UARTReceivedDataIsAvailable(UART2)) {
            loopTimes++;
            if (loopTimes > 1000) {
                //Clear all bytes
                U2STAbits.OERR = 0; //clear the overrun bit, this will reset it
                return;
            }
        }

        if (loopTimes < 1000) {
            //The data is good
            secondByte = UARTGetDataByte(UART2);

            if (firstByte != NO_KEY) {
                //process this key
            }

            //second byte contains the current speed

            //we got a response, now we need to go and send it out on loconet
            return &allCabs[ currentCabAddr ];

            //            cabNumber = currentCabAddr;
            //
            //            if (!sentFirstScreen) {
            //                //Send the screen data
            //                outputBuffer[ 0 ] = 0xC0;
            //                outputBuffer[ 1 ] = 'H';
            //                outputBuffer[ 2 ] = 'I';
            //                outputBuffer[ 3 ] = ' ';
            //                outputBuffer[ 4 ] = 'R';
            //                outputBuffer[ 5 ] = 'I';
            //                outputBuffer[ 6 ] = 'T';
            //                outputBuffer[ 7 ] = 'M';
            //                outputBuffer[ 8 ] = 'R';
            //                SendDataBuffer(UART2, outputBuffer, 9);
            //                sentFirstScreen = 1;
            //            } else {
            //
            //                outputBuffer[ 0 ] = 0xC0 | 0x01;
            //                outputBuffer[ 1 ] = 'C';
            //                outputBuffer[ 2 ] = '!';
            //                outputBuffer[ 3 ] = ' ';
            //                outputBuffer[ 4 ] = ' ';
            //                outputBuffer[ 5 ] = ' ';
            //                outputBuffer[ 6 ] = ' ';
            //                outputBuffer[ 7 ] = ' ';
            //                outputBuffer[ 8 ] = ' ';
            //                SendDataBuffer(UART2, outputBuffer, 9);
            //            }
            //       }
        }
    }

    return NULL; //unable to ping this cab
}