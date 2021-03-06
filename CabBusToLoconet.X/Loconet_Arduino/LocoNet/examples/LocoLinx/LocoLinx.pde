#include <LocoNet.h>

// LocoNet PC Interface Demo

// Flow Control to prevent the PC sending more data than the LocoNet can handle
// is provided via the RS232 Clear-To-Send (CTS) pin. When the Serial receive
// buffer is less than RX_BUF_LOW then the 

#define  RX_CTS_PIN   9
#define  RX_BUF_LOW   32 
#define  RX_BUF_HIGH  96

static   LnBuf        LnTxBuffer ;
static   lnMsg        *LnPacket;

void setup()
{
    // First initialize the LocoNet interface
  LocoNet.init();
  
    // Configure the serial port for 57600 baud
  Serial.begin(57600);
  
    // Initialize a LocoNet packet buffer to buffer bytes from the PC 
  initLnBuf(&LnTxBuffer) ;
  
    // Configure the CTS pin for hardware flow control to control 
    // the serial data flow from the PC to the LocoNet 
  pinMode(RX_CTS_PIN,OUTPUT);
  digitalWrite(RX_CTS_PIN,LOW);
}

void loop()
{  
    // Check for any received LocoNet packets
  LnPacket = LocoNet.receive() ;
  if( LnPacket )
  {
      // Get the length of the received packet
    uint8_t Length = getLnMsgSize( LnPacket ) ;

      // Send the received packet out byte by byte to the PC
    for( uint8_t Index = 0; Index < Length; Index++ )
      Serial.print(LnPacket->data[ Index ], BYTE);
  }

    // Check to see if there are any bytes from the PC
  if(int charWaiting = Serial.available())
  {
      // If the number of bytes waiting is less than RX_BUF_LOW enable CTS
    if( charWaiting < RX_BUF_LOW )
      digitalWrite(RX_CTS_PIN,LOW);

      // If the number of bytes waiting is more than RX_BUF_HIGH disable CTS
    else if( charWaiting > RX_BUF_HIGH )
      digitalWrite(RX_CTS_PIN,HIGH);

      // Read the byte
    uint8_t inByte = Serial.read() & 0xFF;
    
      // Add it to the buffer
    addByteLnBuf( &LnTxBuffer, inByte ) ;
    
      // Check to see if we have received a complete packet yet
    LnPacket = recvLnMsg( &LnTxBuffer ) ;
    if(LnPacket )
        // Send the received packet from the PC to the LocoNet
      LocoNet.send( LnPacket ) ;
  }
  else
      // Enable CTS anyway
    digitalWrite(RX_CTS_PIN,LOW);
}
