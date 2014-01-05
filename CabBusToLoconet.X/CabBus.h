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

#ifdef	__cplusplus
}
#endif

#endif	/* CABBUS_H */

