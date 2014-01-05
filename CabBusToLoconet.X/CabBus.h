/* 
 * File:   CabBus.h
 * Author: Rob
 *
 * Created on November 29, 2013, 3:58 PM
 */

#ifndef CABBUS_H
#define	CABBUS_H

#ifdef	__cplusplus
extern "C" {
#endif

    struct Cab{
        //since number is always less than 64, the upper most bit
        //determines if we need to echo data out onto Loconet
        unsigned char number;
        //the current speed of this cab(0-128 speed steps)
        char speed;
        //the current locomotive number that we are controlling
        unsigned int loco_number;
        //bitfields representing the functions that we are using(displayed on the cab)
        unsigned char functions;
        //lower 4 bits correspond to the dirtyness of the screens
        unsigned char dirty_screens;
        char topLeft[8];
        char topRight[8];
        char bottomLeft[8];
        char bottomRight[8];
    };

    /**
     * Called to initialize all the cabs.
     */
    void initCabs();

    /**
     * Ping the next cab on the bus
     *
     * @return The cab, if some data needs to be sent out
     */
    struct Cab* pingNextCab();

    /**
     * Set the cab locomotive number
     * @param number
     */
    void setCabLocoNumber( struct Cab*, int number );

    /**
     * Set the cab speed.
     */
    void setCabSpeed( struct Cab*, char speed );

    /**
     * Set the time displayed on the cab
     * @param
     * @param hour
     * @param minute
     * @param am
     */
    void setCabTime( struct Cab*, char hour, char minute, char am );

    /**
     * Set a function of the cab, it being either on or off.
     */
    void setCabFunctions( struct Cab*, char functionNum, char on );

#ifdef	__cplusplus
}
#endif

#endif	/* CABBUS_H */

