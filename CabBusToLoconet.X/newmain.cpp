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

/*
 * 
 */
int main(int, char**) {
    unsigned int on;
    struct Cab* currentCab;

    PORTBbits.RB5 = 0; //clear bit
    TRISBbits.TRISB5 = 0; //set as output

    // C function, C++ is being picky about the enums here
    doUART1Config();

    // Init our cab
    initCabs();

    on = 0;

    while (1) {
        if (on) {
            PORTBbits.RB5 = 1;
        } else {
            PORTBbits.RB5 = 0;
        }
        on = !on;

        currentCab = pingNextCab();
        if( currentCab != NULL ){
            // We got data back from the cab!
            //printf( "cab number is %d", currentCab->number );
        }
        
        DelayUs(100); //wait .1mS until next ping
    }

    return 0;
}

