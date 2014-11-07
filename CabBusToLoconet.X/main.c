/*
 * File:   main.c
 * Author: Rob
 *
 * Created on May 4, 2013, 9:01 PM
 */

#include "Delay.h"


#include <stdio.h>
#include <stdlib.h>

#include <p32xxxx.h>
#include <plib.h>

#include "config.h"
#include "../CabBusCommunications/CabBus.h"
#include "../LocoNetCommunications/loconet_buffer.h"

#pragma config FWDTEN   = OFF       // Watchdog Timer
#pragma config JTAGEN = OFF         //disable JTAG
#pragma config FNOSC = FRCPLL   // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2 // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20 // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2 // Divide After PLL (now 40 MHz)

//state machine to tell us what part of the selection to do next
#define SELECTING_LOCO_STATE_NONE 0
#define SELECTING_LOCO_STATE_REQUEST 1
#define SELECTING_LOCO_STATE_NULL_MOVE 2

struct loconetMetaData{
    uint8_t slot;
    uint8_t selectState;
    uint16_t selectingLocoAddr;
};
struct Cab* myCab;

struct loconetMetaData meta;


uint8_t on;

/**
 * Setup UART1 for Loconet
 */
static void doUART1Config() {
    //Config UART1
    //UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY | UART_INVERT_RECEIVE_POLARITY );
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY | UART_INVERT_TRANSMIT_POLARITY);
    UARTSetFifoMode(UART1, /*UART_INTERRUPT_ON_TX_NOT_FULL | */ UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    //                                           115200
    UARTSetDataRate(UART1, GetPeripheralClock(), 16660);
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART RX Interrupt
    INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
    // Configure UART error interrupt
    //INTEnable(INT_SOURCE_UART_ERROR(UART1), INT_ENABLED);

    INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART1), INT_SUB_PRIORITY_LEVEL_0);


    //set up the output and input pins for UART1
    //TRISBbits.TRISB7 = 0; //RB7 = TX
    //TRISBbits.TRISB6 = 1; //RB6 = RX
    //RPB7Rbits.RPB7R = 1; //RPB7 = U1TX
    //U1RXRbits.U1RXR = 1; //RPB6 = U1RX

    //TRISBbits.TRISB13 = 1;
    //U1RXRbits.U1RXR = 3; //RPB13 = U1RX
}

static void doUART2Config(){
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY );
    UARTSetFifoMode(UART2, /*UART_INTERRUPT_ON_TX_NOT_FULL | */ UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2 );
    UARTSetDataRate(UART2, GetPeripheralClock(), 9600);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
    // Configure UART error interrupt
    //INTEnable(INT_SOURCE_UART_ERROR(UART2), INT_ENABLED);

    INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART2), INT_SUB_PRIORITY_LEVEL_0);
}

/**
 * Setup timer1 to be used for loconet timing
 */
static void setupTimer1(){
    INTSetVectorPriority(INT_VECTOR_TIMER(TMR1), INT_PRIORITY_LEVEL_3);
    INTSetVectorSubPriority(INT_VECTOR_TIMER(TMR1), INT_SUB_PRIORITY_LEVEL_0);
}

/**
 * Start the timer for loconet.
 *
 * @param time
 */
static void startTimer(uint32_t microseconds) {
    // Configure Timer 1 using PBCLK as input
    //NOTE: I don't know why this gets multiplied by 5, but i checked it
    //with the scope and it works, so....
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, microseconds * 5 );

    INTEnable(INT_SOURCE_TIMER(TMR1), INT_ENABLED);
}

static void cabbusDelay( uint32_t milliseconds ){
    DelayMs( milliseconds );
}

static void writeCabbusBytes( void* data, uint8_t len ){
   uint8_t* buffer = data;
   PORTBbits.RB5 = 1;

    while (len) {
        while (!UARTTransmitterIsReady(UART2))
            ;

        UARTSendDataByte(UART2, *buffer);

        buffer++;
        len--;
    }

    while (!UARTTransmissionHasCompleted(UART2))
        ;

    PORTBbits.RB5 = 0;  
}

/**
 * Write out a byte to loconet
 *
 * @param byte
 */
static void writeLoconetByte(uint8_t byte) {
    while (!UARTTransmitterIsReady(UART1))
        ;

    UARTSendDataByte(UART1, byte);

    while (!UARTTransmissionHasCompleted(UART1))
        ;
}

static void doSomething(){}

static uint32_t cabIncoming(){
    return !(UARTGetLineStatus( UART2 ) & UART_RECEIVER_IDLE);
}
/*
 *
 */
int main(int argc, char** argv) {
    struct Cab* current;
    struct cab_command* cmd;
    Ln_Message lnMessage;
    Ln_Message outgoingMessage;

    //first, set up our output/input pins
    //RB5 = CAB CTS line
    PORTBbits.RB5 = 0; //clear bit
    TRISBbits.TRISB5 = 0; //set as output
    //RB6 = LN_RX
    TRISBbits.TRISB6 = 1;
    //RB7 = LN_TX
    TRISBbits.TRISB7 = 0;
    //RB8 = CAB_RX
    TRISBbits.TRISB8 = 1;
    //RB9 = CAB_TX
    TRISBbits.TRISB9 = 0;
    //cab power = RA0.  Turn it on.
    TRISAbits.TRISA0 = 0;
    PORTAbits.RA0 = 1;

    //Now let's setup our remappable pins
    RPB7Rbits.RPB7R = 1; //RPB7 = U1TX
    U1RXRbits.U1RXR = 1; //RPB6 = U1RX
    U2RXRbits.U2RXR = 4; //RPB8 = U2RX
    RPB9Rbits.RPB9R = 2; //RPB9 = U2TX

    // config UART1 for Loconet
    doUART1Config();

    // config UART2 for CabBus, initialize memory
    doUART2Config();

    setupTimer1();

    //init cabbus memory
    cabbus_init( cabbusDelay, writeCabbusBytes, cabIncoming );

    // Init loconet
    ln_init(startTimer, writeLoconetByte, 200);

    // we can turn on interrupts now
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();

    //First purge our rx buffer
    U2STAbits.OERR = 0; //clear the overrun bit, this will reset it

//    int a = 0xaa;
//    while( 1 ){
//        writeCabbusBytes( &a, sizeof( a ) );
//    }
    meta.selectState = SELECTING_LOCO_STATE_NONE;

    while (1) {
        current = cabbus_ping_next();
        if( current != NULL ){
            //process any commands from this cab
            cmd = cabbus_get_command( current );
            if( cmd->command == CAB_CMD_SEL_LOCO ){
                int select = 301;
                outgoingMessage.addr.locoAddrLo = select & 0x7F;
                outgoingMessage.addr.locoAddrHi = (select & ~0x7F) >> 7;
                outgoingMessage.opcode = LN_OPC_LOCO_ADDR;
                meta.selectState = SELECTING_LOCO_STATE_REQUEST;
                meta.selectingLocoAddr = select;

                myCab = current;

                cabbus_set_user_data( current, &meta );

                ln_write_message( &outgoingMessage );
            }else if( cmd->command == CAB_CMD_RESPONSE ){
                if( cmd->response.response == 1 ){
                    cabbus_user_message( current, "YES" );
                }else{
                    cabbus_user_message( current, "NO" );
                }
            }else if( cmd->command == CAB_CMD_SPEED ){
                outgoingMessage.opcode = LN_OPC_LOCO_SPEED;
                outgoingMessage.speed.slot = meta.slot;
                outgoingMessage.speed.speed = cmd->speed.speed;

                ln_write_message( &outgoingMessage );
            }

        }

        while( ln_read_message( &lnMessage ) > 0 ){
            //process the loconet message
            if( lnMessage.opcode == LN_OPC_SLOT_READ_DATA ){
                int addr = lnMessage.rdSlotData.addr1 | (lnMessage.rdSlotData.addr2 << 7);
                if( addr == meta.selectingLocoAddr && meta.selectState == SELECTING_LOCO_STATE_REQUEST ){
                    //perform a NULL MOVE
                    outgoingMessage.opcode = LN_OPC_MOVE_SLOT;
                    outgoingMessage.moveSlot.source = lnMessage.rdSlotData.slot;
                    outgoingMessage.moveSlot.slot = lnMessage.rdSlotData.slot;
                    meta.selectState = SELECTING_LOCO_STATE_NULL_MOVE;
                    meta.slot = lnMessage.rdSlotData.slot;

                    ln_write_message( &outgoingMessage );
                }else if(addr == meta.selectingLocoAddr &&
                        meta.selectState == SELECTING_LOCO_STATE_NULL_MOVE ){
                    //we're done
                    meta.selectState = SELECTING_LOCO_STATE_NONE;
                    cabbus_user_message( myCab, "GOOD SELECTION");
                }
            }else if( lnMessage.opcode == LN_OPC_LONG_ACK ){
                if( (lnMessage.ack.ack & 0x7F) == LN_OPC_MOVE_SLOT ){
                    if( lnMessage.ack.ack == 0 ){
                        cabbus_ask_question( myCab, "STEAL?" );
                    }
                }
            }else if( lnMessage.opcode == LN_OPC_LOCO_SPEED ){
                if( lnMessage.speed.slot == meta.slot ){
                    cabbus_set_loco_speed( myCab, lnMessage.speed.speed );
                }
            }
        }

        DelayUs(100); //wait .1mS until next ping

        //writeLoconetByte( 0xAA );
    }


    return (EXIT_SUCCESS);
}


//void __ISR(_UART_1_VECTOR, ipl2) IntUart1Handler(void) {
//    uint8_t byte;
//
//    if( INTGetFlag( INT_SOURCE_UART_ERROR( UART1)) ){
//        printf("");
//    }
//
//    // Is this an RX interrupt?
//    if (INTGetFlag(INT_SOURCE_UART_RX(UART1))) {
//        byte = (UARTGetData(UART2)).data8bit;
//        ln_incoming_byte( byte );
//        // Clear the RX interrupt Flag
//        INTClearFlag(INT_SOURCE_UART_RX(UART1));
//    }
//
//    // We don't care about TX interrupt
//    if (INTGetFlag(INT_SOURCE_UART_TX(UART1))) {
//        INTClearFlag(INT_SOURCE_UART_TX(UART1));
//    }
//}

void __ISR(_UART_1_VECTOR, ipl2) IntUart1AHandler(void)
{
	unsigned int intStatus = INTDisableInterrupts();
	//unsigned char RXD = 0;

	 // *****************************************************
	 //	THIS CH IS KNOWN AS UART1A (UART1) and CONNECTED via AN
	 //	FT4262 FTDI USB-UART CHIP to PROCESSOR 1
	 // *****************************************************
	if (INTGetFlag(INT_U1RX)) {
		// Is this an RX interrupt? ... Read the Buffer first!
		while(U1STAbits.URXDA){
			//RXD = UARTGetDataByte(UART1);
                        //_UART1_receive_complete(((char)UARTGetDataByte(UART1)));
                    ln_incoming_byte( UARTGetDataByte(UART1) );
		}
		// Clear Flags
		INTClearFlag(INT_U1RX);
	} else if(INTGetFlag(INT_U1TX)){
		// Clear Flags
		INTClearFlag(INT_U1TX);
		// Disable this interrupt... re enable for transfers!
		INTEnable(INT_U1TX, INT_DISABLED);
		//	Current DMA Transfer has completed.. we could cue up another transfer or do that in a Timer routine
                //_UART1_transmit_complete();
                /*
		U1TXb.iCMDS--;							// Decrement the counter
		U1TXb.startPos++;						// A transfer has finished from U1TXb
												// Increment the start position for next usse
		if(U1TXb.startPos >= DMAcmdBuffer){
			U1TXb.startPos = 0;
		}
                */
	} else {
		// Added clarity to errors on 09/16/2012
		if(U1STAbits.PERR){
			U1STAbits.PERR = 0;
		}
		if(U1STAbits.FERR){
			U1STAbits.FERR = 0;
		}
		if(U1STAbits.OERR){
			U1STAbits.OERR = 0;
		}

		while(U1STAbits.URXDA){
			UARTGetDataByte(UART1);
		}

		INTClearFlag(INT_U1E);
		INTClearFlag(INT_U1);
		INTClearFlag(INT_U1TX);
		INTClearFlag(INT_U1RX);
	}

	// RESTORE VECTOR INTERRUPTS
	INTRestoreInterrupts(intStatus);
}

void __ISR(_UART_2_VECTOR, ipl2) IntUart2Handler(void)
{
	unsigned int intStatus = INTDisableInterrupts();
        uint8_t byte;

	if (INTGetFlag(INT_U2RX)) {
		// Is this an RX interrupt? ... Read the Buffer first!
		while(U2STAbits.URXDA){
			//RXD = UARTGetDataByte(UART1);
                        //_UART2_receive_complete(((char)UARTGetDataByte(UART2)));
                    byte = UARTGetDataByte(UART2);
                    if( byte ) cabbus_incoming_byte( byte );
		}
		// Clear Flags
		INTClearFlag(INT_U2RX);
	} else if(INTGetFlag(INT_U2TX)){
		// Clear Flags
		INTClearFlag(INT_U2TX);
		// Disable this interrupt... re enable for transfers!
		INTEnable(INT_U2TX, INT_DISABLED);
		//	Current DMA Transfer has completed.. we could cue up another transfer or do that in a Timer routine
                //_UART2_transmit_complete();
                /*
		U2TXb.iCMDS--;							// Decrement the counter
		U2TXb.startPos++;						// A transfer has finished from U2TXb
												// Increment the start position for next usse
		if(U2TXb.startPos >= DMAcmdBuffer){
			U2TXb.startPos = 0;
		}
                */
	} else {
		// Added clarity to errors on 09/16/2012
		if(U2STAbits.PERR){
			U2STAbits.PERR = 0;
		}
		if(U2STAbits.FERR){
			U2STAbits.FERR = 0;
		}
		if(U2STAbits.OERR){
			U2STAbits.OERR = 0;
		}

		while(U2STAbits.URXDA){
			UARTGetDataByte(UART2);
		}

		INTClearFlag(INT_U2E);
		INTClearFlag(INT_U2);
		INTClearFlag(INT_U2TX);
		INTClearFlag(INT_U2RX);
	}

	// RESTORE VECTOR INTERRUPTS
	INTRestoreInterrupts(intStatus);
}
//// UART 2 interrupt handler, set at priority level 2
//void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void) {
//    unsigned char byte;
//    uint32_t intStatus = INTDisableInterrupts();
//
//    if( INTGetFlag( INT_SOURCE_UART_ERROR(UART2) ) ){
//        int stat = UARTGetLineStatus( UART2 );
//        if( stat & UART_FRAMING_ERROR ){
//            U2STAbits.FERR = 0; //clear the error
//        }
//
//        if( stat & UART_OVERRUN_ERROR ){
//            U2STAbits.OERR = 0;
//        }
//
//        if( stat & UART_PARITY_ERROR ){
//            U2STAbits.PERR = 0;
//        }
//
//        stat = UARTGetLineStatus( UART2 );
//    }
//
//    // Is this an RX interrupt?
//    if (INTGetFlag(INT_SOURCE_UART_RX(UART2))) {
//        //we seem to have crosstalk on the
//        byte = (UARTGetData(UART2)).data8bit;
//        cabbus_incoming_byte( byte );
//
//        // Clear the RX interrupt Flag
//        INTClearFlag(INT_SOURCE_UART_RX(UART2));
//    }
//
//    // We don't care about TX interrupt
//    if (INTGetFlag(INT_SOURCE_UART_TX(UART2))) {
//        INTClearFlag(INT_SOURCE_UART_TX(UART2));
//    }
//
//    INTRestoreInterrupts( intStatus );
//}

// Configure the Timer 1 interrupt handler
void __ISR(_TIMER_1_VECTOR, ipl3) Timer1Handler(void)
{
    // Clear the interrupt flag
    INTClearFlag(INT_T1);

    ln_timer_fired();

    //turn off the interrupt
    INTEnable(INT_SOURCE_TIMER(TMR1), INT_DISABLED);
}