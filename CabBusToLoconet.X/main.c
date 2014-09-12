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

//The cab number
char cabNumber;

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
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, microseconds * 10 );

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

/*
 *
 */
int main(int argc, char** argv) {
    struct Cab* current;
    struct cab_command* cmd;
    Ln_Message lnMessage;

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

    //init cabbus memory
    cabbus_init( cabbusDelay, writeCabbusBytes );

    // Init loconet
    ln_init(startTimer, writeLoconetByte, 200);

    // we can turn on interrupts now
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();

    //First purge our rx buffer
    U2STAbits.OERR = 0; //clear the overrun bit, this will reset it

    int a = 0xaa;
    while( 1 ){
        writeCabbusBytes( &a, 4 );
    }


    while (1) {
        current = cabbus_ping_next();
        if( current != NULL ){
            printf("");
            //process any commands from this cab
            cmd = cabbus_get_command( current );
            if( cmd->command == CAB_CMD_SEL_LOCO ){
                printf( "" );
            }
        }

        while( ln_read_message( &lnMessage ) > 0 ){
            //process the loconet message
        }

        DelayUs(100); //wait .1mS until next ping
        //Nop();
        //Nop();
    }


    return (EXIT_SUCCESS);
}


void __ISR(_UART_1_VECTOR, ipl2) IntUart1Handler(void) {
    uint8_t byte;
    
    // Is this an RX interrupt?
    if (INTGetFlag(INT_SOURCE_UART_RX(UART1))) {
        byte = (UARTGetData(UART2)).data8bit;
        ln_incoming_byte( byte );
        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART1));
    }

    // We don't care about TX interrupt
    if (INTGetFlag(INT_SOURCE_UART_TX(UART1))) {
        INTClearFlag(INT_SOURCE_UART_TX(UART1));
    }
}

// UART 2 interrupt handler, set at priority level 2

void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void) {
    unsigned char byte;

    // Is this an RX interrupt?
    if (INTGetFlag(INT_SOURCE_UART_RX(UART2))) {
        if( !PORTBbits.RB5 ){
            //we seem to have crosstalk on the
            byte = (UARTGetData(UART2)).data8bit;
            cabbus_incoming_byte( byte );
        }
        
        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART2));
    }

    // We don't care about TX interrupt
    if (INTGetFlag(INT_SOURCE_UART_TX(UART2))) {
        INTClearFlag(INT_SOURCE_UART_TX(UART2));
    }
}

// Configure the Timer 1 interrupt handler
void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void)
{
    // Clear the interrupt flag
    INTClearFlag(INT_T1);

    ln_timer_fired();
}