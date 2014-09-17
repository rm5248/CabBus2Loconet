#include <string.h>
#include <inttypes.h>
#include <stdio.h>

#include "CabBus.h"
#include "Bitset.h"

#define COMMAND_STATION_START_BYTE  0xC0
#define REPEAT_SCREEN  0x7E
#define TOP_LEFT_LCD  0x00
#define TOP_RIGHT_LCD 0x01
#define BOTTOM_LEFT_LCD 0x02
#define BOTTOM_RIGHT_LCD  0x03

//key definitions
#define NO_KEY  0x7D
#define ENTER	0x40
#define STEP_FASTER  0x4A
#define STEP_SLOWER  0x4B
#define SELECT_LOCO  0x48
#define KEY_0	0x50
#define KEY_1	0x51
#define KEY_2	0x52
#define KEY_3	0x53
#define KEY_4	0x54
#define KEY_5	0x55
#define KEY_6	0x56
#define KEY_7	0x57
#define KEY_8	0x58
#define KEY_9	0x59

#define CAB_GET_ASK_QUESTION(cab) CHECK_BIT(cab->dirty_screens, 5)
#define CAB_SET_ASK_QUESTION(cab) SET_BIT(cab->dirty_screens, 5)
#define CAB_CLEAR_ASK_QUESTION(cab) CLEAR_BIT(cab->dirty_screens, 5)
#define CAB_GET_SELECTING_LOCO(cab) CHECK_BIT(cab->dirty_screens, 6)
#define CAB_SET_SELECTING_LOCO(cab) SET_BIT(cab->dirty_screens, 6)
#define CAB_CLEAR_SELECTING_LOCO(cab) CLEAR_BIT(cab->dirty_screens, 6)
//Macros for getting/setting dirty state of screens
#define CAB_SET_TOPLEFT_DIRTY(cab) SET_BIT(cab->dirty_screens, 3)
#define CAB_SET_BOTTOMLEFT_DIRTY(cab) SET_BIT(cab->dirty_screens, 2)
#define CAB_SET_TOPRIGHT_DIRTY(cab) SET_BIT(cab->dirty_screens, 1)
#define CAB_SET_BOTTOMRIGHT_DIRTY(cab) SET_BIT(cab->dirty_screens, 0)
#define CAB_GET_TOPLEFT_DIRTY(cab) CHECK_BIT(cab->dirty_screens, 3)
#define CAB_GET_BOTTOMLEFT_DIRTY(cab) CHECK_BIT(cab->dirty_screens, 2)
#define CAB_GET_TOPRIGHT_DIRTY(cab) CHECK_BIT(cab->dirty_screens, 1)
#define CAB_GET_BOTTOMRIGHT_DIRTY(cab) CHECK_BIT(cab->dirty_screens, 0)
//Clearing dirty bits
#define CAB_SET_TOPLEFT_CLEAN(cab) CLEAR_BIT(cab->dirty_screens, 3)
#define CAB_SET_BOTTOMLEFT_CLEAN(cab) CLEAR_BIT(cab->dirty_screens, 2)
#define CAB_SET_TOPRIGHT_CLEAN(cab) CLEAR_BIT(cab->dirty_screens, 1)
#define CAB_SET_BOTTOMRIGHT_CLEAN(cab) CLEAR_BIT(cab->dirty_screens, 0)
//check if we have dirty screens
#define CAB_HAS_DIRTY_SCREENS(cab) (cab->dirty_screens & 0x0F)

static const char* FWD = "FWD";
static const char* REV = "REV";

//
// Local Structs
//
struct Cab {
    //the number of this cab, 0-64
    uint8_t number;
    //the current speed of this cab(0-127 speed steps).  Top bit = direction
    uint8_t speed;
    //the current locomotive number that we are controlling
    uint16_t loco_number;
    //bitfields representing the functions that we are using(displayed on the cab)
    uint8_t functions;
    //lower 4 bits correspond to the dirtyness of the screens
    //upper 4 bits = flags
    // 0x01 << 5 = asking a question
    uint8_t dirty_screens;
    char topLeft[8];
    char topRight[8];
    char bottomLeft[8];
    char bottomRight[8];
    //the latest command from the cab
    struct cab_command command;
    uint32_t last_ping;
    //any other data that you want to associate with this cab.
    void* user_data;
};

//
// Local variables
//

//Contains the data for all the cabs
static struct Cab allCabs[ 64 ];
static uint8_t currentCabAddr;
static uint8_t outputBuffer[ 10 ];

// functions to assist us in communications
static cab_delay_fn delayFunction;
static cab_write_fn writeFunction;
static cab_incoming_data incomingFunction;

// storage for incoming bytes
volatile uint8_t firstByte;
volatile uint8_t secondByte;
volatile uint8_t byteStatus;

// The current ping#, if we did not ping this cab the last time
// around set the screens to be dirty
static uint32_t pingNum;

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

void cab_reset( struct Cab* cab ){
    int x;

    char tempBuffer[ 9 ];
    snprintf( tempBuffer, 9, "%s:%3d", cab->speed & 0x80 ? FWD : REV, cab->speed & 0x7F );
    memcpy( cab->bottomLeft, tempBuffer, 8 );
    cab->bottomLeft[ 7 ] = ' ';

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

    CAB_SET_BOTTOMLEFT_DIRTY(cab);
    CAB_SET_BOTTOMRIGHT_DIRTY(cab);
}

//
// Cabbus functions
//

void cabbus_init( cab_delay_fn inDelay, cab_write_fn inWrite, cab_incoming_data inData ) {
    unsigned char x;

    delayFunction = inDelay;
    writeFunction = inWrite;
    incomingFunction = inData;

    for (x = 0; x < 64; x++) {
        memset(&allCabs[ x ], 0, sizeof ( struct Cab));
        allCabs[ x ].number = x;
        // Set defaults for our cabs; will also set the screens to be dirty
        cabbus_set_loco_number(&allCabs[ x ], 44);
        cabbus_set_loco_speed(&allCabs[ x ], 0);
        cabbus_set_time(&allCabs[ x ], 5, 55, 1);
        cabbus_set_functions(&allCabs[ x ], 1, 1);
        cabbus_set_direction( &allCabs[ x ], FORWARD );
    }

    currentCabAddr = 0;
    pingNum = 0;
}

struct Cab* cabbus_ping_next() {
    static struct Cab* current;

    currentCabAddr++;
    if (currentCabAddr == 1) currentCabAddr++; //don't ping address 1
    if (currentCabAddr == 2) currentCabAddr++; //don't ping address 2 either

    if (currentCabAddr == 64){
        currentCabAddr = 0; //only up to 63 cab
        pingNum++;
    }

    //Go and ping the next address
    outputBuffer[ 0 ] = 0x80 | currentCabAddr;

    writeFunction( outputBuffer, 1 );

    //Delay to make sure that we get a response back
    delayFunction( 3 );

    //wait while we have data to get
    //while( incomingFunction() );

    if( pingNum - allCabs[ currentCabAddr ].last_ping > 1 ){
        //we haven't seen this guy, set all screens to dirty
        allCabs[ currentCabAddr ].dirty_screens |= 0x0F;
    }

    if ( byteStatus & 0x01 ) {
        //we have a response back from a cab
        unsigned char knobByte;
        unsigned char keyByte;
        unsigned int loopTimes = 0;

        current = &allCabs[ currentCabAddr ];
        current->command.command = CAB_CMD_NONE;

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

        current->last_ping = pingNum;

        if (keyByte != NO_KEY) {
            int key = keyByte + 10;
            if (keyByte == REPEAT_SCREEN) {
                // set all screens to be dirty
                allCabs[ currentCabAddr ].dirty_screens = 0x0F;
            }else if( keyByte == SELECT_LOCO ){
                //send the message 'select loco:' to the cab

                memcpy( current->bottomLeft, "SELECT LOCO:    ", 16 );
                CAB_SET_BOTTOMLEFT_DIRTY(current);
                CAB_SET_BOTTOMRIGHT_DIRTY(current);
                CAB_SET_SELECTING_LOCO( current );
            }else if( keyByte == ENTER ){
                //reset all screens
                cab_reset( current );
                if( CAB_GET_SELECTING_LOCO( current ) ){
                    current->command.command = CAB_CMD_SEL_LOCO;
                    CAB_CLEAR_SELECTING_LOCO(current);
                }
            }else if( keyByte == KEY_0 ){
                if( CAB_GET_ASK_QUESTION( current ) ){
                    cab_reset( current );
                    current->command.command = CAB_CMD_RESPONSE;
                    current->command.response.response = 0;
                }
 
            }else if( keyByte == KEY_1 ){
                if( CAB_GET_ASK_QUESTION( current ) ){
                    cab_reset( current );
                    current->command.command = CAB_CMD_RESPONSE;
                    current->command.response.response = 1;
                }
            }else if( keyByte == STEP_FASTER ){
                current->command.command = CAB_CMD_SPEED;
                current->command.speed.speed = current->speed + 1;
            }else if( keyByte == STEP_SLOWER ){
                current->command.command = CAB_CMD_SPEED;
                current->command.speed.speed = current->speed - 1;
            }
        }

        if ( CAB_HAS_DIRTY_SCREENS(current) ) {
            outputBuffer[ 0 ] = COMMAND_STATION_START_BYTE;
            // send out the first dirty screen!
            if (CAB_GET_TOPLEFT_DIRTY(current)) {
                memcpy(&(outputBuffer[1]), current->topLeft, 8);
                outputBuffer[ 0 ] |= TOP_LEFT_LCD;
                CAB_SET_TOPLEFT_CLEAN(current);
            } else if (CAB_GET_BOTTOMLEFT_DIRTY(current)) {
                memcpy(&(outputBuffer[1]), current->bottomLeft, 8);
                outputBuffer[ 0 ] |= BOTTOM_LEFT_LCD;
                CAB_SET_BOTTOMLEFT_CLEAN(current);
            } else if (CAB_GET_TOPRIGHT_DIRTY(current)) {
                memcpy(&(outputBuffer[1]), current->topRight, 8);
                outputBuffer[ 0 ] |= TOP_RIGHT_LCD;
                CAB_SET_TOPRIGHT_CLEAN(current);
            } else if (CAB_GET_BOTTOMRIGHT_DIRTY(current)) {
                memcpy(&(outputBuffer[1]), current->bottomRight, 8);
                outputBuffer[ 0 ] |= BOTTOM_RIGHT_LCD;
                CAB_SET_BOTTOMRIGHT_CLEAN(current);
            }

            //Delay, the Power Cab is pretty slow it seems
            delayFunction( 2 );
            writeFunction( outputBuffer, 9 );
        }

        //we got a response, tell the master that this cab exists
        return current;
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
        CAB_SET_TOPLEFT_DIRTY(cab);
    }
}

void cabbus_set_loco_speed(struct Cab* cab, char speed ) {
    uint8_t userSpeed = speed & 0x7F;

    cab->speed = speed;
    //need a temp buffer, sprintf will put a NULL at the end, we
    //only want 8 bytes
    char tempBuffer[ 9 ];
    snprintf( tempBuffer, 9, "%s:%3d", speed & 0x80 ? FWD : REV, userSpeed );
    memcpy( cab->bottomLeft, tempBuffer, 8 );
    cab->bottomLeft[ 7 ] = ' ';
    CAB_SET_BOTTOMLEFT_DIRTY(cab);
}

void cabbus_set_time(struct Cab* cab, char hour, char minute, char am) {
    const char* AM = "AM";
    const char* PM = "PM";

    char tempBuffer[ 9 ];
    snprintf( tempBuffer, 9, "%2d:%02d %s", hour, minute, am ? AM : PM );
    memcpy( cab->topRight, tempBuffer, 8 );
    CAB_SET_TOPRIGHT_DIRTY(cab);
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

    CAB_SET_BOTTOMLEFT_DIRTY(cab);
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

uint16_t cabbus_get_loco_number( struct Cab* cab ){
    return cab->loco_number;
}

struct cab_command* cabbus_get_command( struct Cab* cab ){
    return &(cab->command);
}

void cabbus_ask_question( struct Cab* cab, const char* message ){
    if( strlen( message ) > 16 ){
        return;
    }

    CAB_SET_ASK_QUESTION( cab );

    char tempBuffer[ 17 ];
    memset( tempBuffer, ' ', 16 );
    snprintf( tempBuffer, 17, "%s", message );
    memcpy( cab->bottomLeft, tempBuffer, 16 );
    CAB_SET_BOTTOMLEFT_DIRTY(cab);
    CAB_SET_BOTTOMRIGHT_DIRTY(cab);
}

uint8_t cabbus_get_cab_number( struct Cab* cab ){
    return cab->number;
}

void cabbus_user_message( struct Cab* cab, const char* message){
    if( strlen( message ) > 16 ){
        return;
    }

    char tempBuffer[ 17 ];
    memset( tempBuffer, ' ', 16 );
    snprintf( tempBuffer, 17, "%s", message );
    memcpy( cab->bottomLeft, tempBuffer, 16 );
    CAB_SET_BOTTOMLEFT_DIRTY(cab);
    CAB_SET_BOTTOMRIGHT_DIRTY(cab);
}

void cabbus_set_user_data( struct Cab* cab, void* data ){
    cab->user_data = data;
}

void* cabbus_get_user_data( struct Cab* cab ){
    return cab->user_data;
}
