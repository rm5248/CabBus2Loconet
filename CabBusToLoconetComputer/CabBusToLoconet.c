#include <stdio.h>
#include <time.h>
#include <signal.h>
#include <string.h>
#include <errno.h>

#include "CabBus.h"
#include "loconet_buffer.h"

//
// Local Variables
//
static volatile int loconet_fd;
static timer_t loconet_timer;

static volatile int cabbus_fd;

//
// Local Functions
//

//Loconet Functions

/**
 * Given a byte of data, print out the direction of the locomotive,
 * plus the functions that are on( F0-F4 )
 */
static void printDirectionAndFunctions( uint8_t byte ){
	printf( "  Direction: %s\n", byte & 0x20 ? "REV" : "FWD"  );
	printf( "  Functions: " );
	//F0 is in the upper byte; the lower byte contains F1-F4
	if( byte & 0x10 ) printf( "F0 " );
	if( byte & 0x01 ) printf( "F1 " );
	if( byte & 0x02 ) printf( "F2 " );
	if( byte & 0x04 ) printf( "F3 " );
	if( byte & 0x08 ) printf( "F4 " );
	printf( "\n" );
}

/**
 * Given a byte of data, print out the status.  this is the SLOT_STATUS1 byte.
 */
static void printSlotStatus( uint8_t stat ){
	uint8_t decoderType = stat & 0x07;
	uint8_t slotStatus = ( stat & ( 0x03 << 4 ) ) >> 4;
	uint8_t consistStatus = ( stat & ( 0x03 << 6 ) ) >> 6;

	if( decoderType == 0x00 ){
		printf( "  28 step/ 3byte packet regular mode\n" );
	}else if( decoderType == 0x01 ){
		printf( "  28 step/ trinary packets\n" );
	}else if( decoderType == 0x02 ){
		printf( "  14 step mode\n" );
	}else if( decoderType == 0x03 ){
		printf( "  128 mode packets\n" );
	}else if( decoderType == 0x04 ){
		printf( "  28 step, advanced DCC consisting\n" );
	}else if( decoderType == 0x07 ){
		printf( "  128 step, advanced DCC consisting\n" );
	}else{
		printf( "  Unkown DCC decoder type\n" );
	}

	printf( "  Slot status: " );
	if( slotStatus == 0x00 ){
		printf( "FREE\n" );
	}else if( slotStatus == 0x01 ){
		printf( "COMMON\n" );
	}else if( slotStatus == 0x02 ){
		printf( "IDLE\n" );
	}else if( slotStatus == 0x03 ){
		printf( "IN_USE\n" );
	}

	printf( "  Consist status: " );
	if( consistStatus == 0x00 ){
		printf( "FREE, no consist\n" );
	}else if( consistStatus == 0x01 ){
		printf( "logical consist sub-member\n" );
	}else if( consistStatus == 0x02 ){
		printf( "logical consist top\n" );
	}else if( consistStatus == 0x03 ){
		printf( "logical mid consist\n" );
	}
}

/**
 * Print out the status of the track, as given by the trk byte
 */
static void printTrackStatus( uint8_t trk ){
	if( trk & 0x01 ){
		printf( "  DCC Power ON, Global power UP\n" );
	}else{
		printf( "  DCC Power OFF\n" );
	}

	if( trk & 0x02 ){
		printf( "  Track PAUSED, Broadcast ESTOP\n" );
	}

	if( trk & 0x04 ){
		printf( "  Master has LocoNet 1.1\n" );
	}else{
		printf( "  Master is DT200\n" );
	}

	if( trk & 0x08 ){
		printf( "  Programming Track Busy\n" );
	}
}

static void printLnMessage( const Ln_Message* message ){
	int x;
	switch( message->opcode ){
		case LN_OPC_LOCO_SPEED:
			printf( "Locomotive speed message \n ");
			printf( "  Slot: %d\n", message->speed.slot );
			printf( "  Speed: %d\n", message->speed.speed );
			break;
		case LN_OPC_LOCO_DIR_FUNC:
			printf( "Locomotive direction/functions message: \n" );
			printf( "  Slot: %d\n", message->dirFunc.slot );
			printDirectionAndFunctions( message->dirFunc.dir_funcs );
			printf( "\n" );
			break;
		case LN_OPC_LONG_ACK:
			printf( "Long ACK \n" );
			printf( "  Long Opcode: 0x%X\n", message->ack.lopc );
			printf( "  Status: 0x%X\n", message->ack.ack );
			if( message->ack.ack == 0 ) printf( "  STATUS FAIL\n" );
			break;
		case LN_OPC_LOCO_ADDR:
			printf( "Request locomotive addr\n" );
			printf( "  Address: %d\n", message->addr.locoAddrHi << 7 | message->addr.locoAddrLo );
			break;
		case LN_OPC_SLOT_READ_DATA:
		case LN_OPC_SLOT_WRITE_DATA:
			printf( "%s slot data\n", message->opcode == LN_OPC_SLOT_READ_DATA ? "Read" : "Write"  );
			printf( "  Slot #: %d\n", message->rdSlotData.slot );
			printf( "  Slot status: 0x%X\n", message->rdSlotData.stat );
			printSlotStatus( message->rdSlotData.stat );
			printf( "  Speed: %d\n", message->rdSlotData.speed );
			printDirectionAndFunctions( message->rdSlotData.dir_funcs );
			printf( "  TRK: 0x%X\n", message->rdSlotData.track );
			printTrackStatus( message->rdSlotData.track );
			break;
		case LN_OPC_REQUEST_SLOT_DATA:
			printf( "Request slot data\n" );
			printf( "  Slot #: %d\n", message->reqSlotData.slot );
			break;
		case LN_OPC_MOVE_SLOT:
			printf( "Move slot\n" );
			printf( "  Source slot: %d\n", message->moveSlot.source );
			printf( "  Slot: %d\n", message->moveSlot.slot );
			break;
		case LN_OPC_SLOT_STAT1:
			printf( "Write stat 1\n" );
			printf( "  Slot: %d\n", message->stat1.slot );
			printf( "  Stat1: 0x%X\n", message->stat1.stat1 );
			break;
		default:
			printf( "Unimplemented print for opcode 0x%X\n", message->opcode );
	}
}

static void timerStart( uint32_t time ){
	static struct itimerspec timespec;

	memset( &timespec, 0, sizeof( struct itimerspec ) );
	timespec.it_value.tv_nsec = time * 1000; // microseconds to nanoseconds
	timespec.it_interval.tv_nsec = timespec.it_value.tv_nsec;

	if( timer_settime( loconet_timer, 0, &timespec, NULL ) < 0 ){
		perror( "timer_settime" );
	}
}

static void timerFired( int sig, siginfo_t* si, void* uc ){
	static struct itimerspec timespec;

	memset( &timespec, 0, sizeof( struct itimerspec ) );
	//disable the timer
	timer_settime( loconet_timer, 0, &timespec, NULL );

	ln_timer_fired();
}

// Cabbus support functions
static void cabDelay( uint32_t delayms ){
	usleep( delayms * 1000 );
}

// Thread Functions

static void* loconet_read_thread( void* ign ){
	uint8_t buffer[ 20 ];
	ssize_t got;
	ssize_t x;

	while( 1 ){
		got = read( loconet_fd, buffer, 20 );
		if( got < 0 ){
			perror( "read - loconet" );
			break;
		}
		for( x = 0; x < got; x++ ){
			ln_incoming_byte( buffer[ x ] );
		}
	}

	return NULL;
}

static void* cabbus_read_thread( void* ign ){
	uint8_t buffer[ 20 ];
	ssize_t got;
	ssize_t x;

	while( 1 ){
		got = read( cabbus_fd, buffer, 20 );
		if( got < 0 ){
			perror( "read - cabbus" );
			break;
		}
		for( x = 0; x < got; x++ ){
			cabbus_incoming_byte( buffer[ x ] );
		}
	}

	return NULL;
}

// Writing functions
static void loconet_write( uint8_t byte ){
	if( write( loconet_fd, &byte, 1 ) < 0 ){
		perror( "write - loconet" );
	}
}

static void cabbus_write( void* data, uint8_t len ){
	if( write( cabbus_fd, data, len ) < 0 ){
		perror( "write - cabbus" );
	}
}

//
// External Functions
//

int main( int argc, char** argv ){
	pthread_t ln_thr;
	pthread_t cab_thr;
	struct sigevent evt;
	struct sigaction sa;
	sigset_t mask;
	Ln_Message incomingMessage;
	struct Cab cab;

	//quick parse of cmdline
	if( argc < 3 ){
		fprintf( stderr, "ERROR: Need at least 2 args: <loconet port> <cabbus port>\n" );
		return 1;
	}

	//set up the timer for loconet
	memset( &sa, 0, sizeof( struct sigaction ) );
	sa.sa_flags = SA_SIGINFO;
	sa.sa_sigaction = timerFired;
	sigemptyset( &sa.sa_mask );
	if( sigaction( SIGRTMIN, &sa, NULL ) < 0 ){
		perror( "sigaction" );
		return 1;
	}

	memset( &evt, 0, sizeof( evt ) );
	evt.sigev_notify = SIGEV_SIGNAL;
	evt.sigev_signo = SIGRTMIN;
	evt.sigev_value.sival_ptr = &loconet_timer;
	if( timer_create( CLOCK_REALTIME, &evt, &loconet_timer ) < 0 ){
		perror( "timer_create" );
		return 1;
	}

	//open the TTY ports
	printf( "About to open %s for loconet use\n", argv[ 1 ] );
	loconet_fd = open( argv[ 1 ] );
	if( loconet_fd < 0 ){
		fprintf( stderr, "ERROR: Can't open.  Reason: %s\n", strerror( errno ) );
		return 1;
	}
	
	printf( "About to open %s for cabbus use\n", argv[ 2 ] );
	cabbus_fd = open( argv[ 2 ] );
	if( cabbus_fd < 0 ){
		fprintf( stderr, "ERROR: Can't open.  Reason: %s\n", strerror( errno ) );
		return 1;
	}

	//initialize loconet
	ln_init( timerStart, loconet_write, 200 );

	//initalize cabbus
	cabbus_init( cabDelay, cabbus_write );

	//start our threads
	if( pthread_create( &ln_thr, NULL, loconet_read_thread, NULL ) < 0 ){
		perror( "pthread_create - loconet" );
		return 1;
	}

	if( pthread_create( &cab_thr, NULL, cabbus_read_thread, NULL ) < 0 ){
		perror( "pthread_create - cabbus" );
	}

	//go into our main loop.
	//essentially what we do here, is we get information from the cabs on the bus,
	//and then echo that information back onto loconet.
	//we also have to parse loconet information that we get back to make sure
	//that we tell the user about stupid stuff that they are doing
	while( 1 ){

	}
}
