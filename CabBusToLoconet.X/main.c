/* 
 * File:   main.c
 * Author: Rob
 *
 * Created on May 4, 2013, 9:01 PM
 */

#include <stdio.h>
#include <stdlib.h>

#include <p32xxxx.h>
#include <plib.h>

#include "config.h"

#pragma config FWDTEN   = OFF       // Watchdog Timer
#pragma config JTAGEN = OFF         //disable JTAG
#pragma config FNOSC = FRCPLL   // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2 // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20 // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2 // Divide After PLL (now 40 MHz)

//The cab number
char cabNumber;

void doUART1Config(){
    //Config UART1
    //UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY | UART_INVERT_RECEIVE_POLARITY );
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY );
    UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    //                                           115200
    UARTSetDataRate(UART1, GetPeripheralClock(), 16660);
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART RX Interrupt
    INTEnable(INT_SOURCE_UART_RX( UART1 ), INT_ENABLED);
    // Configure UART error interrupt
    INTEnable( INT_SOURCE_UART_ERROR( UART1 ), INT_ENABLED );
    
    INTSetVectorPriority(INT_VECTOR_UART( UART1 ), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART( UART1 ), INT_SUB_PRIORITY_LEVEL_0);
    

    //set up the output and input pins for UART1
    TRISBbits.TRISB7 = 0;  //RB7 = TX
    TRISBbits.TRISB6 = 1; //RB6 = RX
    RPB7Rbits.RPB7R = 1; //RPB7 = U1TX
    U1RXRbits.U1RXR = 1; //RPB6 = U1RX

    //TRISBbits.TRISB13 = 1;
    //U1RXRbits.U1RXR = 3; //RPB13 = U1RX

    // set up Timer1.  This lets us transition from the CD BACKOFF loconet state.
    INTSetVectorPriority( INT_VECTOR_TIMER( TMR1 ), INT_PRIORITY_LEVEL_3 );
    INTSetVectorSubPriority( INT_VECTOR_TIMER( TMR1 ), INT_SUB_PRIORITY_LEVEL_0 );

    INTEnable( INT_T1 );
}

/*
 * 
 */
int mainold(int argc, char** argv) {
    unsigned char outputBuffer[ 10 ];
    char currentCabAddr = -1;
    char sentFirstScreen = 0;

    PORTAbits.RA0 = 0; //clear bit
    TRISAbits.TRISA0 = 0; //set as output
    PORTBbits.RB5 = 0; //clear bit
    TRISBbits.TRISB5 = 0; //set as output

    //Config UART1
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART1, GetPeripheralClock(), 9600);
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    //Config UART2
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);
    UARTSetDataRate(UART2, GetPeripheralClock(), 9600);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    //set up the output and input pins for UART1
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB6 = 1;
    RPB7Rbits.RPB7R = 1; //RPB7 = U1TX
    U1RXRbits.U1RXR = 1; //RPB6 = U1RX

    //Set up output and input pins for UART2
    TRISBbits.TRISB8 = 1; //RB8 = RX
    TRISBbits.TRISB9 = 0; //RB9 = TX
    U2RXRbits.U2RXR = 4; //RPB8 = U2RX
    RPB9Rbits.RPB9R = 2; //RPB9 = U2TX

    //Enable Interrupts
    // Configure UART RX Interrupt
    //INTEnable(INT_SOURCE_UART_RX( UART1 ), INT_ENABLED);
    //INTSetVectorPriority(INT_VECTOR_UART( UART1 ), INT_PRIORITY_LEVEL_2);
    //INTSetVectorSubPriority(INT_VECTOR_UART( UART1 ), INT_SUB_PRIORITY_LEVEL_0);

    //INTEnable(INT_SOURCE_UART_RX( UART2 ), INT_ENABLED);
    //INTSetVectorPriority(INT_VECTOR_UART( UART2 ), INT_PRIORITY_LEVEL_2);
    //INTSetVectorSubPriority(INT_VECTOR_UART( UART2 ), INT_SUB_PRIORITY_LEVEL_0);

    // Enable multi-vector interrupts
    //INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();

    //First purge our rx buffer
    U2STAbits.OERR = 0; //clear the overrun bit, this will reset it

    while (1) {
        currentCabAddr++;
        if (currentCabAddr == 1) currentCabAddr++; //don't ping address 1
        if (currentCabAddr == 2) currentCabAddr++; //don't ping address 2 either

        if (currentCabAddr == 64) currentCabAddr = 0; //only up to 63 cabs
        /*
        if( address >= 0 ){
            char toSend[ 100 ];

            sprintf( toSend, "CAB Address %d\n", address );
            SendDataBuffer( UART1, toSend, strlen( toSend ) );
            address = -1;
        }
         */

        //Go and ping the next address
        outputBuffer[ 0 ] = 0x80 | currentCabAddr;

        SendDataBuffer(UART2, outputBuffer, 1);
        //if( currentCabAddr == 52 ) DelayMs( 1000 );

        {
            char buffer[ 100 ];
            sprintf(buffer, "ping %d\n", currentCabAddr);
            //SendDataBuffer( UART1, buffer, strlen( buffer ) );
        }

        //NOTE: this delay must be this high, otherwise we go around our loop
        //again, and the cab number is wrong.  TODO: get a scope and check
        //this out, because it shouldn't be happening
        DelayMs(2); //cabs must respond withing 900 uS to a ping

        if (!U2STAbits.RIDLE || U2STAbits.URXDA) {
            //if( UARTReceivedDataIsAvailable( UART2 ) ){
            unsigned char firstByte;
            unsigned char secondByte;
            int loopTimes = 0;

            char buffer[ 100 ];
            sprintf(buffer, "Response from %d\n", currentCabAddr);
            SendDataBuffer(UART1, buffer, strlen(buffer));

            firstByte = UARTGetDataByte(UART2);
            while (!UARTReceivedDataIsAvailable(UART2)) {
                loopTimes++;
                if (loopTimes > 1000) {
                    //Clear all bytes
                    U2STAbits.OERR = 0; //clear the overrun bit, this will reset it
                    break;
                }
            }

            if (loopTimes < 1000) {
                //The data is good
                secondByte = UARTGetDataByte(UART2);

                cabNumber = currentCabAddr;

                if (!sentFirstScreen) {
                    //Send the screen data
                    outputBuffer[ 0 ] = 0xC0;
                    outputBuffer[ 1 ] = 'H';
                    outputBuffer[ 2 ] = 'I';
                    outputBuffer[ 3 ] = ' ';
                    outputBuffer[ 4 ] = 'R';
                    outputBuffer[ 5 ] = 'I';
                    outputBuffer[ 6 ] = 'T';
                    outputBuffer[ 7 ] = 'M';
                    outputBuffer[ 8 ] = 'R';
                    SendDataBuffer(UART2, outputBuffer, 9);
                    sentFirstScreen = 1;
                } else {

                    outputBuffer[ 0 ] = 0xC0 | 0x01;
                    outputBuffer[ 1 ] = 'C';
                    outputBuffer[ 2 ] = '!';
                    outputBuffer[ 3 ] = ' ';
                    outputBuffer[ 4 ] = ' ';
                    outputBuffer[ 5 ] = ' ';
                    outputBuffer[ 6 ] = ' ';
                    outputBuffer[ 7 ] = ' ';
                    outputBuffer[ 8 ] = ' ';
                    SendDataBuffer(UART2, outputBuffer, 9);
                }
            }
        }

        DelayUs(100); //wait .1mS until next ping
        //Nop();
        //Nop();


    }


    return (EXIT_SUCCESS);
}

/*
void __ISR(_UART_1_VECTOR, ipl2) IntUart1Handler(void) {
    // Is this an RX interrupt?
    if (INTGetFlag(INT_SOURCE_UART_RX(UART1))) {
        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART1));

    }

    // We don't care about TX interrupt
    if (INTGetFlag(INT_SOURCE_UART_TX(UART1))) {
        INTClearFlag(INT_SOURCE_UART_TX(UART1));
    }
}
 * */

// UART 2 interrupt handler, set at priority level 2

void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void) {
    unsigned char byte;

    // Is this an RX interrupt?
    if (INTGetFlag(INT_SOURCE_UART_RX(UART2))) {
        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART2));

        //get the byte of data
        byte = (UARTGetData(UART2)).data8bit;

        {
            //char buffer[ 100 ];
            //sprintf( buffer, "GOT RESPONSE CAB %d DATA 0x%X\n", currentCabAddr, byte );
            //SendDataBuffer(UART1, buffer, strlen(buffer));
        }

        //A ping in bits: 10XXXXXX
        //if( ( byte & 0xC0 ) == 0x80 ){
        //address = byte & 0x3F; //should get ping address from this
        //}
    }

    // We don't care about TX interrupt
    if (INTGetFlag(INT_SOURCE_UART_TX(UART2))) {
        INTClearFlag(INT_SOURCE_UART_TX(UART2));
    }
}
