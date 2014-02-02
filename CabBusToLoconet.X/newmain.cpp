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

// The states that the cab's metadata must go through
// The cab must go through these states in order
static const uint8_t META_STATE_REQ_LOCO = 0; // We have requested a loco
static const uint8_t META_STATE_GOT_SLOT_DATA = 1; // We have gotten the slot data for this
static const uint8_t META_STATE_NULL_MOVE = 2; // Executed the null move to get this loco
static const uint8_t META_STATE_WRITE_SLOT_DATA = 3; // The slot data was written
static const uint8_t META_STATE_ACK_SLOT_WRITE = 4; // We have ack'ed the slot

struct CabMetaData{
    uint8_t slot;  // what loconet slot is this using?
    uint8_t speed; // direction is the top bit
    uint8_t loconetState; //what state are we currently in for getting an address?
};

struct Cab* digitraxSlots[128]; // 128 digitrax slots

/*
 * 
 */
int main(int, char**) {
    unsigned int on;
    struct Cab* currentCab;
    struct Cab* locoSelectingCab;
    struct CabMetaData* currentCabData;
    struct CabMetaData* locoSelectingCabData;
    LocoNetClass lnClass;
    lnMsg* incomingMessage;
    uint8_t selectingLoco;

    //TODO remove this
    struct CabMetaData newData;
    //struct Cab* MY_CAB;

    memset( digitraxSlots, 0, sizeof( struct Cab* ) * 128 );

    selectingLoco = 0;

    PORTBbits.RB5 = 0; //clear bit
    TRISBbits.TRISB5 = 0; //set as output

    PORTAbits.RA0 = 0;
    TRISAbits.TRISA0 = 0;

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

    //OpenTimer1( T1_ON, SYS_FREQ / 6800 ); // 1.2 ms

    //PORTAbits.RA0 = 1;

    on = 0;
   //while( 1 ){
     //   UARTSendDataByte( UART1, 0x55 );
       // DelayMs( 10 );
    //}

    while (1) {
        if (on) {
            PORTBbits.RB5 = 1;
        } else {
            PORTBbits.RB5 = 0;
        }
        on = !on;

        currentCab = NULL;
        currentCabData = NULL;
        
        currentCab = pingNextCab();
        if (currentCab != NULL) {
            // We got data back from the cab!
            //printf( "cab number is %d", currentCab->number );
            currentCabData = (CabMetaData*)(currentCab->user_data);
            if( currentCabData == NULL ){
                currentCab->user_data = &newData;
                currentCabData = &newData;
                newData.loconetState = 0;
                newData.slot = 0;
                newData.speed = 0;
            }

            //Let's process this data and see if we need to send anything out
            //onto loconet
            if( currentCab->bbb == 1 ){
                //GO FASTER
                lnClass.send( OPC_LOCO_SPD, currentCabData->slot, ( currentCab->speed & 0x7F ) + 1 );
            }else if( currentCab->bbb == 2 ){
                //GO SLOWER
                lnClass.send( OPC_LOCO_SPD, currentCabData->slot, ( currentCab->speed & 0x7F ) - 1 );
            }else if( currentCab->bbb == 3 ){
                short locoNum = 301;
                //Request a LOCO
                //NOTE: for some reason, the first byte uses the upper 6 bits for
                //the address
                lnClass.send( OPC_LOCO_ADR, ((locoNum & 0xFF00 )>> 7), locoNum & 0x00FF );

                currentCabData->loconetState = META_STATE_REQ_LOCO;
                currentCabData->slot = 0;
                currentCabData->speed = 0;
                selectingLoco = 1;

                locoSelectingCab = currentCab;
                locoSelectingCabData = currentCabData;
            }

            currentCab->bbb = 0;
        }

        //UARTSendDataByte(UART1, 0x55);

        do {
            incomingMessage = lnClass.receive();
            if (incomingMessage != NULL) {
                if (incomingMessage->data[ 0 ] == OPC_LOCO_SPD) {
                    struct Cab* cabToUpdate = digitraxSlots[ incomingMessage->lsp.slot ];
                    if( cabToUpdate != NULL ){
                        setCabSpeed( cabToUpdate, incomingMessage->lsp.spd );
                    }
                }else if( incomingMessage->data[ 0 ] == OPC_LOCO_DIRF ){
                    struct Cab* cabToUpdate = digitraxSlots[ incomingMessage->lsp.slot ];
                    if( cabToUpdate != NULL ){
                        enum Direction dir;

                        if( incomingMessage->ldf.dirf & ( 0x01 << 5 ) ){
                            dir = FORWARD;
                        }else{
                            dir = REVERSE;
                        }
                        
                        setCabDirection( cabToUpdate, dir );
                        setCabFunctions( cabToUpdate, 0, incomingMessage->ldf.dirf & ( 0x01 << 4 ) );
                        setCabFunctions( cabToUpdate, 1, incomingMessage->ldf.dirf & ( 0x01 ) );
                        setCabFunctions( cabToUpdate, 2, incomingMessage->ldf.dirf & ( 0x01 << 1 ) );
                        setCabFunctions( cabToUpdate, 3, incomingMessage->ldf.dirf & ( 0x01 << 2 ) );
                        setCabFunctions( cabToUpdate, 4, incomingMessage->ldf.dirf & ( 0x01 << 3 ) );
                    }
                }else if( incomingMessage->data[ 0 ] == OPC_SL_RD_DATA ){
                    if( selectingLoco ){
                    if( locoSelectingCab != NULL &&
                            locoSelectingCabData->loconetState == META_STATE_REQ_LOCO ){
                        uint8_t activeStatus;

                        activeStatus = incomingMessage->sd.stat;
                        activeStatus &= ( 0x03 << 4 );
                        activeStatus = activeStatus >> 4;
                        if( activeStatus != 3 ){
                            //this is COMMON, IDLE or NEW
                            lnClass.send( OPC_MOVE_SLOTS, incomingMessage->sd.slot, incomingMessage->sd.slot );
                        
                            locoSelectingCabData->loconetState = META_STATE_NULL_MOVE;
                            locoSelectingCabData->slot = incomingMessage->sd.slot;
                            digitraxSlots[ locoSelectingCabData->slot ] = locoSelectingCab;
                        }else{
                            memcpy( locoSelectingCab->bottomLeft, "ERR: NT 3", 8 );
                            locoSelectingCab->dirty_screens |= (0x01 << 2);
                        }
                    }
                    }
                }else if( incomingMessage->data[ 0 ] == OPC_WR_SL_DATA ){
                    if( selectingLoco ){
                    if( locoSelectingCabData != NULL &&
                            locoSelectingCabData->loconetState == META_STATE_NULL_MOVE ){
                        lnClass.send( OPC_LONG_ACK, OPC_WR_SL_DATA & OPC_MASK, OPC_MASK );
                        locoSelectingCabData->loconetState = META_STATE_ACK_SLOT_WRITE;

                        selectingLoco = 0;
                        locoSelectingCab = NULL;
                        locoSelectingCabData = NULL;
                    }
                    }
                }
            }
        } while (incomingMessage != NULL);

        DelayUs(100); //wait .1mS until next ping
    }

    return 0;
}

