#include <string.h>
#include <inttypes.h>
#include <stdio.h>

#include "CabBus.h"

//Contains the data for all the cabs
static struct Cab allCabs[ 64 ];

static uint8_t currentCabAddr;
static uint8_t outputBuffer[ 10 ];

static const unsigned char COMMAND_STATION_START_BYTE = 0xC0;
static const unsigned char NO_KEY = 0x7D;
static const unsigned char REPEAT_SCREEN = 0x7E;
static const unsigned char TOP_LEFT_LCD = 0x00;
static const unsigned char TOP_RIGHT_LCD = 0x01;
static const unsigned char BOTTOM_LEFT_LCD = 0x02;
static const unsigned char BOTTOM_RIGHT_LCD = 0x03;
static const unsigned char STEP_FASTER = 0x4A;
static const unsigned char STEP_SLOWER = 0x4B;

// functions to assist us in communications
static cab_delay_fn delayFunction;
static cab_write_fn writeFunction;

// storage for incoming bytes
volatile uint8_t firstByte;
volatile uint8_t secondByte;
volatile uint8_t byteStatus;

//
// Local functions
//

static char simp_atoi( int number ) {
    switch( number ) {
    case 9:
        return '9';
    case 8:
        return '8';
    case 7:
        return '7';
    case 6:
        return '6';
    case 5:
        return '5';
    case 4:
        return '4';
    case 3:
        return '3';
    case 2:
        return '2';
    case 1:
        return '1';
    case 0:
        return '0';
    }

    return '-';
}

//
// Cabbus functions
//

void cabbus_init( cab_delay_fn inDelay, cab_write_fn inWrite ) {
    unsigned char x;

    delayFunction = inDelay;
    writeFunction = inWrite;

    for (x = 0; x < 64; x++) {
        memset(&allCabs[ x ], 0, sizeof ( struct Cab));
        allCabs[ x ].number = x;
        // Set defaults for our cabs; will also set the screens to be dirty
        cabbus_set_loco_number(&allCabs[ x ], 44);
        cabbus_set_loco_speed(&allCabs[ x ], 12);
        cabbus_set_time(&allCabs[ x ], 5, 55, 1);
        cabbus_set_functions(&allCabs[ x ], 1, 1);
        cabbus_set_direction( &allCabs[ x ], FORWARD );
    }

    currentCabAddr = 0;
}

struct Cab* cabbus_ping_next() {
    currentCabAddr++;
    if (currentCabAddr == 1) currentCabAddr++; //don't ping address 1
    if (currentCabAddr == 2) currentCabAddr++; //don't ping address 2 either

    if (currentCabAddr == 64) currentCabAddr = 0; //only up to 63 cabs

    //Go and ping the next address
    outputBuffer[ 0 ] = 0x80 | currentCabAddr;

    writeFunction( outputBuffer, 1 );

    //Delay to make sure that we get a response back
    delayFunction( 2 );

    if ( byteStatus & 0x01 ) {
        //we have a response back from a cab
        unsigned char knobByte;
        unsigned char keyByte;
        unsigned int loopTimes = 0;

        loopTimes = 0;
        while ( !(byteStatus & 0x02) ) {
            ++loopTimes;
            if (loopTimes > 1000) {
                byteStatus = 0x00;
                return NULL;
            }
        }
        knobByte = secondByte;
        keyByte = firstByte;
        byteStatus = 0x00;

        if (keyByte != NO_KEY) {
            int key = keyByte + 10;
            if (keyByte == REPEAT_SCREEN) {
                // set all screens to be dirty
                allCabs[ currentCabAddr ].dirty_screens = 0x0F;
            }
        }

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
            delayFunction( 2 );
            writeFunction( outputBuffer, 9 );
        }

        //we got a response, now we need to go and send it out on loconet
        return &allCabs[ currentCabAddr ];
    }

    return NULL; //unable to ping this cab
}

void cabbus_set_loco_number(struct Cab* cab, int number) {
    //first off, quick sanity check here
    if( number > 9999 ) {
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

void cabbus_set_loco_speed(struct Cab* cab, char speed ) {
    const char* FWD = "FWD";
    const char* REV = "REV";

    speed = speed & 0x7F;

    if (cab->speed != speed) {
        cab->speed = speed;
        //need a temp buffer, sprintf will put a NULL at the end, we
        //only want 8 bytes
        char tempBuffer[ 9 ];
        snprintf( tempBuffer, 9, "%s:%3d", speed & 0x80 ? FWD : REV, speed );
        memcpy( cab->bottomLeft, tempBuffer, 8 );
        cab->bottomLeft[ 7 ] = ' ';
        cab->dirty_screens |= (0x01 << 2);
    }
}

void cabbus_set_time(struct Cab* cab, char hour, char minute, char am) {
    const char* AM = "AM";
    const char* PM = "PM";

    char tempBuffer[ 9 ];
    snprintf( tempBuffer, 9, "%2d:%02d %s", hour, minute, am ? AM : PM );
    memcpy( cab->topRight, tempBuffer, 8 );
    cab->dirty_screens |= (0x01 << 1);
}

void cabbus_set_functions(struct Cab* cab, char functionNum, char on) {
    unsigned char x;
    if( on ) {
        cab->functions |= ( 0x01 << functionNum );
    } else {
        cab->functions &= ~( 0x01 << functionNum );
    }

    for( x = 0; x < 8; x++ ) {
        if( cab->functions & ( 0x01 << x ) ) {
            cab->bottomRight[ x ] = simp_atoi( x );
            if( x == 0 ) {
                cab->bottomRight[ x ] = 'L';
            }
        } else {
            cab->bottomRight[ x ] = '-';
        }
    }

    cab->dirty_screens |= (0x01);
}

void cabbus_set_direction( struct Cab* cab, enum Direction direction ) {
    if( direction == FORWARD ) {
        cab->speed |= 0x80;
    } else {
        cab->speed &= ~( 0x80 );
    }

    //force an update
    cabbus_set_loco_speed( cab, cab->speed );
}

void cabbus_incoming_byte( uint8_t byte ) {
    if( byteStatus == 0x00 ) {
        //no bytes, put in first byte
        firstByte = byte;
        byteStatus = 0x01;
    } else if( byteStatus == 0x01 ) {
        secondByte = byte;
        byteStatus = 0x03;
    } else {
        byteStatus = 0x00;
    }
}
