/*
 * File:   CabBus.h
 * Author: Rob
 *
 * Created on November 29, 2013, 3:58 PM
 */

#ifndef CABBUS_H
#define	CABBUS_H

#include <inttypes.h>

#ifdef	__cplusplus
extern "C" {
#endif

    /**
     * This function is called to delay a given number of milliseconds.
     *
     * @param delayMs The number of milliseconds to delay.
     */
    typedef void (*cab_delay_fn)( uint32_t delayMs );

    /**
     * This function is called to actually write data out to the bus.
     *
     * @param data The data to write out
     * @param len The length of the data to write out(in bytes)
     */
    typedef void (*cab_write_fn)( void* data, uint8_t len );

    struct Cab {
        //since number is always less than 64, the upper most bit
        //determines if we need to echo data out onto Loconet
        unsigned char number;
        //the current speed of this cab(0-127 speed steps).  Top bit = direction
        unsigned char speed;
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
        //any other data that you want to associate with this cab.
        void* user_data;
    };

    enum Direction {
        FORWARD,
        REVERSE
    };

    /**
     * Called to initialize all the cabs.
     */
    void cabbus_init( cab_delay_fn inDelay, cab_write_fn inWrite );

    /**
     * Ping the next cab on the bus
     *
     * @return The cab, if some data needs to be sent out
     */
    struct Cab* cabbus_ping_next();

    /**
     * Set the cab locomotive number
     * @param number
     */
    void cabbus_set_loco_number( struct Cab*, int number );

    /**
     * Set the cab speed.
     */
    void cabbus_set_loco_speed( struct Cab*, char speed );

    /**
     * Set the time displayed on the cab
     * @param
     * @param hour
     * @param minute
     * @param am
     */
    void cabbus_set_time( struct Cab*, char hour, char minute, char am );

    /**
     * Set a function of the cab, it being either on or off.
     */
    void cabbus_set_functions( struct Cab*, char functionNum, char on );

    /**
     * Set the direction on this cab.
     */
    void cabbus_set_direction( struct Cab*, enum Direction direction );

    /**
     * Call this when a byte comes in on the bus
     */
    void cabbus_incoming_byte( uint8_t byte );

#ifdef	__cplusplus
}
#endif

#endif	/* CABBUS_H */

