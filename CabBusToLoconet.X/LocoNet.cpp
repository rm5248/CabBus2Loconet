/****************************************************************************
 * 	Copyright (C) 2009 Alex Shepherd
 *
 * 	Portions Copyright (C) Digitrax Inc.
 *
 * 	This library is free software; you can redistribute it and/or
 * 	modify it under the terms of the GNU Lesser General Public
 * 	License as published by the Free Software Foundation; either
 * 	version 2.1 of the License, or (at your option) any later version.
 *
 * 	This library is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * 	Lesser General Public License for more details.
 *
 * 	You should have received a copy of the GNU Lesser General Public
 * 	License along with this library; if not, write to the Free Software
 * 	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************
 *
 * 	IMPORTANT:
 *
 * 	Some of the message formats used in this code are Copyright Digitrax, Inc.
 * 	and are used with permission as part of the EmbeddedLocoNet project. That
 * 	permission does not extend to uses in other software products. If you wish
 * 	to use this code, algorithm or these message formats outside of
 * 	EmbeddedLocoNet, please contact Digitrax Inc, for specific permission.
 *
 * 	Note: The sale any LocoNet device hardware (including bare PCB's) that
 * 	uses this or any other LocoNet software, requires testing and certification
 * 	by Digitrax Inc. and will be subject to a licensing agreement.
 *
 * 	Please contact Digitrax Inc. for details.
 *
 *****************************************************************************
 * 	DESCRIPTION
 * 	This module provides functions that manage the sending and receiving of LocoNet packets.
 *
 * 	As bytes are received from the LocoNet, they are stored in a circular
 * 	buffer and after a valid packet has been received it can be read out.
 *
 * 	When packets are sent successfully, they are also appended to the Receive
 * 	circular buffer so they can be handled like they had been received from
 * 	another device.
 *
 * 	Statistics are maintained for both the send and receiving of packets.
 *
 * 	Any invalid packets that are received are discarded and the stats are
 * 	updated approproately.
 *
 *****************************************************************************/
#include <p32xxxx.h>
#include <plib.h>

#include "LocoNet.h"
#include "ln_uart.h"
#include "ln_config.h"
#include "ln_buf.h"
#include "config.h"

#define LN_ST_IDLE            0   // net is free for anyone to start transmission
#define LN_ST_CD_BACKOFF      1   // timer interrupt is counting backoff bits
#define LN_ST_TX_COLLISION    2   // just sending break after creating a collision
#define LN_ST_TX              3   // transmitting a packet
#define LN_ST_RX              4   // receiving bytes

#define LN_COLLISION_TICKS 15
#define LN_TX_RETRIES_MAX  25

volatile uint8_t lnState;
volatile uint8_t lnLastTransmit;
LnBuf * lnRxBuffer;

extern "C" {

    static int on;

    void __ISR(_TIMER_1_VECTOR, ipl3) IntTimer1Handler(void) {
        // The CD backoff time should have ended now
        lnState = LN_ST_IDLE;
        // turn the timer off
        OpenTimer1( T1_OFF, 0 );
        // clear the interrupt
        INTClearFlag( INT_T1 );
        PORTAbits.RA0 = on;
        on = !on;
    }

}

// UART 1 is handling the LocoNet communications
extern "C" {

    void __ISR(_UART_1_VECTOR, ipl2) IntUart1Handler(void) {
        // Is this an RX interrupt?
        if (INTGetFlag(INT_U1RX)) {
            while (UARTReceivedDataIsAvailable(UART1)) {
                uint8_t data = (UARTGetData(UART1)).data8bit;

                if (lnState == LN_ST_RX) {
                    //put the byte in the buffer
                    addByteLnBuf(lnRxBuffer, data);
                    //OpenTimer1( T1_OFF, 0 );
                    OpenTimer1( T1_ON, SYS_FREQ / 6800 ); // need to wait at least 1.2 ms for CD BACKOFF
                } else if (lnState == LN_ST_TX) {
                    if (data != lnLastTransmit) {
                        //the data that we read was NOT the same as what we just
                        //transmittied.  Collision!
                        lnState = LN_ST_TX_COLLISION;
                        OpenTimer1( T1_ON, SYS_FREQ / 6800 ); // need to wait at least 1.2 ms for CD BACKOFF
                    }
                } else if (lnState == LN_ST_IDLE) {
                    // Go to the RX state, add the byte to the buffer
                    lnState = LN_ST_RX;
                    addByteLnBuf(lnRxBuffer, data );
                    //OpenTimer1( T1_OFF, 0 );
                } else if( lnState == LN_ST_TX_COLLISION ){
                    OpenTimer1( T1_ON, SYS_FREQ / 2000 ); // assuming that it should fire after 4 ms
                } else {
                    //panic??
                    INTDisableInterrupts();
                    while (1);
                }
            }
            // Clear the RX interrupt Flag
            INTClearFlag(INT_U1RX);
        }

        // We don't care about TX interrupt
        if (INTGetFlag(INT_U1TX)) {
            INTClearFlag(INT_U1TX);
        }

        if( INTGetFlag(INT_U1E) ){
            //re-set the UART
            int stat = UARTGetLineStatus( UART1 );
            if( stat & ( UART_TRANSMITTER_EMPTY | UART_TRANSMITTER_NOT_FULL | UART_DATA_READY ) ){
                //clear the error
                INTClearFlag( INT_U1E );
            }else if( stat & UART_RECEIVER_IDLE ){
                lnState = LN_ST_CD_BACKOFF;
                INTClearFlag( INT_U1E );

                // Start timer 1
                //OpenTimer1( T1_ON, SYS_FREQ / 4000 ); // assuming that it should fire after 2 ms
                
            }else{
                while( 1 ); //kernel panic
            }
        }
    }
}

void initLocoNetHardware(LnBuf *RxBuffer) {
    lnRxBuffer = RxBuffer;
}

LN_STATUS sendLocoNetPacketTry(lnMsg *TxData, unsigned char ucPrioDelay) {
    uint8_t CheckSum;
    uint8_t CheckLength;
    uint8_t lnTxLength;
    uint8_t lnTxIndex;
    uint8_t lnTxSuccess;

    lnTxLength = getLnMsgSize(TxData);

    // First calculate the checksum as it may not have been done
    CheckLength = lnTxLength - 1;
    CheckSum = 0xFF;

    for (lnTxIndex = 0; lnTxIndex < CheckLength; lnTxIndex++) {
        CheckSum ^= TxData->data[ lnTxIndex ];
    }

    TxData->data[ CheckLength ] = CheckSum;

    // clip maximum prio delay
    if (ucPrioDelay > LN_BACKOFF_MAX) {
        ucPrioDelay = LN_BACKOFF_MAX;
    }
    // if priority delay was waited now, declare net as free for this try
    if (lnState == LN_ST_CD_BACKOFF) {
        //if (lnBitCount >= ucPrioDelay) { // Likely we don't want to wait as long as
        //lnState = LN_ST_IDLE; // the timer ISR waits its maximum delay.
        //}
        return LN_CD_BACKOFF;
    }

    if (lnState != LN_ST_IDLE) {
        return LN_NETWORK_BUSY; // neither idle nor backoff -> busy
    }

    if (UARTReceivedDataIsAvailable(UART1)) {
        return LN_NETWORK_BUSY;
    }

    lnTxIndex = 0;
    lnTxSuccess = 1;

    // Set the State to Transmit
    lnState = LN_ST_TX;

    //send the data out
    unsigned char count = 0;
    while (lnTxLength--) {
        while (!UARTTransmitterIsReady(UART1));
        lnLastTransmit = TxData->data[ count++ ];
        if (lnState != LN_ST_TX) {
            //some sort of error
            break;
        }

        UARTSendDataByte(UART1, lnLastTransmit);
        //NOTE: send bytes out one byte at a time, need to detect collisions
        //on the bus, so we don't fill up the UART buffer
        while (!UARTTransmissionHasCompleted(UART1));
        if (lnState != LN_ST_TX) {
            //some sort of error
            break;
        }
    }

    if (lnTxSuccess) {
        while (!UARTTransmissionHasCompleted(UART1));
        //lnRxBuffer->Stats.TxPackets++;
        OpenTimer1( T1_ON, SYS_FREQ / 6800 ); // need to wait at least 1.2 ms for CD BACKOFF
        return LN_DONE;
    }
    if (lnState == LN_ST_TX_COLLISION) {
        return LN_COLLISION;
    }
    return LN_UNKNOWN_ERROR; // everything else is an error
}

LocoNetClass::LocoNetClass() {
}

void LocoNetClass::init() {
    initLnBuf(&LnBuffer);
    initLocoNetHardware(&LnBuffer);
    lnState = LN_ST_IDLE;
}

lnMsg* LocoNetClass::receive() {
    return recvLnMsg(&LnBuffer);
}

LN_STATUS LocoNetClass::send(lnMsg *pPacket) {
    unsigned char ucTry;
    unsigned char ucPrioDelay = LN_BACKOFF_INITIAL;
    LN_STATUS enReturn;
    unsigned char ucWaitForEnterBackoff;

    for (ucTry = 0; ucTry < LN_TX_RETRIES_MAX; ucTry++) {

        // wait previous traffic and than prio delay and than try tx
        ucWaitForEnterBackoff = 1; // don't want to abort do/while loop before
        do // we did not see the backoff state once
        {
            enReturn = sendLocoNetPacketTry(pPacket, ucPrioDelay);

            if (enReturn == LN_DONE) // success?
                return LN_DONE;

            if (enReturn == LN_PRIO_BACKOFF)
                ucWaitForEnterBackoff = 0; // now entered backoff -> next state != LN_BACKOFF is worth incrementing the try counter
        } while ((enReturn == LN_CD_BACKOFF) || // waiting CD backoff
                (enReturn == LN_PRIO_BACKOFF) || // waiting master+prio backoff
                ((enReturn == LN_NETWORK_BUSY) && ucWaitForEnterBackoff)); // or within any traffic unfinished
        // failed -> next try going to higher prio = smaller prio delay
        if (ucPrioDelay > LN_BACKOFF_MIN)
            ucPrioDelay--;
    }
    LnBuffer.Stats.TxErrors++;
    return LN_RETRY_ERROR;
}

LN_STATUS LocoNetClass::send(lnMsg *pPacket, uint8_t ucPrioDelay) {
    return sendLocoNetPacketTry(pPacket, ucPrioDelay);
}

LN_STATUS LocoNetClass::send(uint8_t OpCode, uint8_t Data1, uint8_t Data2) {
    lnMsg SendPacket;

    SendPacket.data[ 0 ] = OpCode;
    SendPacket.data[ 1 ] = Data1;
    SendPacket.data[ 2 ] = Data2;

    return send(&SendPacket);
}

LN_STATUS LocoNetClass::send(uint8_t OpCode, uint8_t Data1, uint8_t Data2, uint8_t PrioDelay) {
    lnMsg SendPacket;

    SendPacket.data[ 0 ] = OpCode;
    SendPacket.data[ 1 ] = Data1;
    SendPacket.data[ 2 ] = Data2;

    return sendLocoNetPacketTry(&SendPacket, PrioDelay);
}

uint8_t LocoNetClass::processSwitchSensorMessage(lnMsg *LnPacket) {
    uint16_t Address;
    uint8_t Direction;
    uint8_t Output;
    uint8_t ConsumedFlag = 1;

    Address = (LnPacket->srq.sw1 | ((LnPacket->srq.sw2 & 0x0F) << 7));
    if (LnPacket->sr.command != OPC_INPUT_REP)
        Address++;

    switch (LnPacket->sr.command) {
        case OPC_INPUT_REP:
            Address <<= 1;
            Address += (LnPacket->ir.in2 & OPC_INPUT_REP_SW) ? 2 : 1;

            if (notifySensor)
                notifySensor(Address, LnPacket->ir.in2 & OPC_INPUT_REP_HI);
            break;

        case OPC_SW_REQ:
            if (notifySwitchRequest)
                notifySwitchRequest(Address, LnPacket->srq.sw2 & OPC_SW_REQ_OUT, LnPacket->srq.sw2 & OPC_SW_REQ_DIR);
            break;

        case OPC_SW_REP:
            if (notifySwitchReport)
                notifySwitchReport(Address, LnPacket->srp.sn2 & OPC_SW_REP_HI, LnPacket->srp.sn2 & OPC_SW_REP_SW);
            break;

        case OPC_SW_STATE:
            Direction = LnPacket->srq.sw2 & OPC_SW_REQ_DIR;
            Output = LnPacket->srq.sw2 & OPC_SW_REQ_OUT;

            if (notifySwitchState)
                notifySwitchState(Address, Output, Direction);
            break;

        case OPC_SW_ACK:
            break;

        case OPC_LONG_ACK:
            if (LnPacket->lack.opcode == (OPC_SW_STATE & 0x7F)) {
                Direction = LnPacket->lack.ack1 & 0x01;
            } else
                ConsumedFlag = 0;
            break;

        default:
            ConsumedFlag = 0;
    }

    return ConsumedFlag;
}

void LocoNetClass::requestSwitch(uint16_t Address, uint8_t Output, uint8_t Direction) {
    uint8_t AddrH = (--Address >> 7) & 0x0F;
    uint8_t AddrL = Address & 0x7F;

    if (Output)
        AddrH |= OPC_SW_REQ_OUT;

    if (Direction)
        AddrH |= OPC_SW_REQ_DIR;

    send(OPC_SW_REQ, AddrL, AddrH);
}

void LocoNetClass::reportSwitch(uint16_t Address) {
    Address -= 1;
    send(OPC_SW_STATE, (Address & 0x7F), ((Address >> 7) & 0x0F));
}

//LocoNetClass LocoNet = LocoNetClass();

