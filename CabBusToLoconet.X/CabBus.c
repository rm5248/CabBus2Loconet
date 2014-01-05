#include <string.h>


#include "Delay.h"

#include "CabBus.h"
#include "RS485Comm.h"
#include "config.h"

//Contains the data for all the cabs
static struct Cab allCabs[ 64 ];

static unsigned char currentCabAddr;
static unsigned char outputBuffer[ 10 ];

static const unsigned char COMMAND_STATION_START_BYTE = 0xC0;
static const unsigned char NO_KEY = 0x7D;
static const unsigned char REPEAT_SCREEN = 0x7E;
static const unsigned char TOP_LEFT_LCD = 0x00;
static const unsigned char TOP_RIGHT_LCD = 0x01;
static const unsigned char BOTTOM_LEFT_LCD = 0x02;
static const unsigned char BOTTOM_RIGHT_LCD = 0x03;

static char simp_atoi( int number ){
    switch( number ){
        case 9: return '9';
        case 8: return '8';
        case 7: return '7';
        case 6: return '6';
        case 5: return '5';
        case 4: return '4';
        case 3: return '3';
        case 2: return '2';
        case 1: return '1';
        case 0: return '0';
    }
    
    return '-';
}

void initCabs() {
    unsigned char x;

    for (x = 0; x < 64; x++) {
        memset(&allCabs[ x ], 0, sizeof ( struct Cab));
        allCabs[ x ].number = x;
        // Set defaults for our cabs; will also set the screens to be dirty
        setCabLocoNumber(&allCabs[ x ], 44);
        setCabSpeed(&allCabs[ x ], 12);
        setCabTime(&allCabs[ x ], 5, 55, 1);
        setCabFunctions(&allCabs[ x ], 1, 1);
    }

    currentCabAddr = 0;

    //Config UART2
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);
    UARTSetDataRate(UART2, GetPeripheralClock(), 9600);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    //Set up output and input pins for UART2
    TRISBbits.TRISB8 = 1; //RB8 = RX
    TRISBbits.TRISB9 = 0; //RB9 = TX
    U2RXRbits.U2RXR = 4; //RPB8 = U2RX
    RPB9Rbits.RPB9R = 2; //RPB9 = U2TX

    //set up our flow control for the RS485 transceiver
    PORTBbits.RB5 = 0; //clear bit
    TRISBbits.TRISB5 = 0; //set as output
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
    DelayMs(2);

    if (!U2STAbits.RIDLE || U2STAbits.URXDA) {
        //we have a response back from a cab
        unsigned char knobByte;
        unsigned char keyByte;
        unsigned int loopTimes = 0;

        knobByte = UARTGetDataByte(UART2);
        while (!UARTReceivedDataIsAvailable(UART2)) {
            loopTimes++;
            if (loopTimes > 1000) {
                //Clear all bytes
                U2STAbits.OERR = 0; //clear the overrun bit, this will reset it
                return NULL;
            }
        }

        if (loopTimes < 1000) {
            //The data is good
            keyByte = UARTGetDataByte(UART2);

            if (keyByte != NO_KEY) {
                int key = keyByte + 10;
                if (keyByte == REPEAT_SCREEN) {
                    // set all screens to be dirty
                    allCabs[ currentCabAddr ].dirty_screens = 0x0F;
                }
            }

            //setCabSpeed(allCabs[ currentCabAddr ], knobByte );

           /* if (allCabs[ currentCabAddr ].ping_times < 20) {
                //pretend that we haven't seen this cab
                allCabs[ currentCabAddr ].ping_times++;
                return NULL;
            }*/

            /*/if (knobByte != NO_KEY) {
                //process this key
                int key = knobByte + 10;

                if( knobByte == REPEAT_SCREEN ){
                    // set all screens to be dirty
                    allCabs[ currentCabAddr ].dirty_screens = 0x0F;
                }
            }*/

            if (allCabs[ currentCabAddr ].dirty_screens) {
                outputBuffer[ 0 ] = COMMAND_STATION_START_BYTE;
                // send out the first dirty screen!
                if (allCabs[ currentCabAddr ].dirty_screens & (0x01 << 3)) {
                    memcpy(&(outputBuffer[1]), allCabs[ currentCabAddr ].topLeft, 8);
                    outputBuffer[ 0 ] |= TOP_LEFT_LCD;
                    allCabs[ currentCabAddr ].dirty_screens &= (~(0x01 << 3)); //clear the dirty bit
                } else if (allCabs[ currentCabAddr ].dirty_screens & (0x01 << 2)) {
                    memcpy(&(outputBuffer[1]), allCabs[ currentCabAddr ].bottomLeft, 8);
                    outputBuffer[ 0 ] |= BOTTOM_LEFT_LCD;
                    allCabs[ currentCabAddr ].dirty_screens &= (~(0x01 << 2)); //clear the dirty bit
                } else if (allCabs[ currentCabAddr ].dirty_screens & (0x01 << 1)) {
                    memcpy(&(outputBuffer[1]), allCabs[ currentCabAddr ].topRight, 8);
                    outputBuffer[ 0 ] |= TOP_RIGHT_LCD;
                    allCabs[ currentCabAddr ].dirty_screens &= (~(0x01 << 1)); //clear the dirty bit
                } else if (allCabs[ currentCabAddr ].dirty_screens & (0x01)) {
                    memcpy(&(outputBuffer[1]), allCabs[ currentCabAddr ].bottomRight, 8);
                    outputBuffer[ 0 ] |= BOTTOM_RIGHT_LCD;
                    allCabs[ currentCabAddr ].dirty_screens &= (~(0x01)); //clear the dirty bit
                }

                //Delay, the Power Cab is pretty slow it seems
                DelayMs( 2 );
                SendDataBuffer(UART2, outputBuffer, 9);
            }

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
    } else {
        //No ping was received from this cab, set his stuff to be ignored
//        allCabs[ currentCabAddr ].ping_times = 0;
        // set all screens to be dirty
        allCabs[ currentCabAddr ].dirty_screens = 0x0F;
    }

    return NULL; //unable to ping this cab
}

void setCabLocoNumber(struct Cab* cab, int number) {
    //first off, quick sanity check here
    if( number > 9999 ){
        number = 9999;
    }
    
    if (cab->loco_number != number) {
        cab->loco_number = number;
        char tempBuffer[ 9 ];
        snprintf( tempBuffer, 9, "LOC:%3d", number );
        memcpy( cab->topLeft, tempBuffer, 8 );
        cab->topLeft[ 7 ] = ' ';
        cab->dirty_screens |= (0x01 << 3);
    }
}

void setCabSpeed(struct Cab* cab, char speed ) {
    const char* FWD = "FWD";
    const char* REV = "REV";
    if (cab->speed != speed) {
        cab->speed = speed;
        //need a temp buffer, sprintf will put a NULL at the end, we
        //only want 8 bytes
        char tempBuffer[ 9 ];
        snprintf( tempBuffer, 9, "%s:%3d", speed > 0 ? FWD : REV, speed );
        memcpy( cab->bottomLeft, tempBuffer, 8 );
        cab->bottomLeft[ 7 ] = ' ';
        cab->dirty_screens |= (0x01 << 2);
    }
}

void setCabTime(struct Cab* cab, char hour, char minute, char am) {
    const char* AM = "AM";
    const char* PM = "PM";

    char tempBuffer[ 9 ];
    snprintf( tempBuffer, 9, "%2d:%02d %s", hour, minute, am ? AM : PM );
    memcpy( cab->topRight, tempBuffer, 8 );
    cab->dirty_screens |= (0x01 << 1);
}

void setCabFunctions(struct Cab* cab, char functionNum, char on) {
    unsigned char x;
    if( on ){
        cab->functions |= ( 0x01 << functionNum );
    }else{
        cab->functions &= ~( 0x01 << functionNum );
    }

    for( x = 0; x < 8; x++ ){
        if( cab->functions & ( 0x01 << x ) ){
            cab->bottomRight[ x ] = simp_atoi( x );
            if( x == 0 ){
                cab->bottomRight[ x ] = 'L';
            }
        }else{
            cab->bottomRight[ x ] = '-';
        }
    }

    cab->dirty_screens |= (0x01);
}