#include "Delay.h"

#include <p32xxxx.h>
#include <plib.h>

#include "config.h"

//*****************************************************************************
// DelayUs creates a delay of given microseconds using the Core Timer
//
// 1 million micro seconds in a second!

void DelayUs(unsigned int delay) {
    unsigned int int_status;
    while (delay--) {
        int_status = INTDisableInterrupts();
        OpenCoreTimer(SYS_FREQ / 2000000);
        INTRestoreInterrupts(int_status);
        mCTClearIntFlag();
        while (!mCTGetIntFlag());
    }
    mCTClearIntFlag();
}

//*****************************************************************************
// DelayMs creates a delay of given miliseconds using the Core Timer
//

void DelayMs(unsigned short delay) {
    unsigned int int_status;
    while (delay--) {
        int_status = INTDisableInterrupts();
        OpenCoreTimer(SYS_FREQ / 2000);
        INTRestoreInterrupts(int_status);
        mCTClearIntFlag();
        while (!mCTGetIntFlag());
    }
    mCTClearIntFlag();
}
