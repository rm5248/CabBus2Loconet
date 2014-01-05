/* 
 * File:   ln_uart.h
 * Author: Rob
 *
 * Created on December 8, 2013, 11:06 AM
 */

#ifndef LN_UART_H
#define	LN_UART_H

#ifdef	__cplusplus
extern "C" {
#endif

void initLocoNetHardware( LnBuf *RxBuffer );
LN_STATUS sendLocoNetPacketTry(lnMsg *TxData, unsigned char ucPrioDelay);


#ifdef	__cplusplus
}
#endif

#endif	/* LN_UART_H */

