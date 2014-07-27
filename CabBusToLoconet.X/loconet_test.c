#include <stdio.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>

#include "loconet_buffer.h"

static volatile int fd;
static timer_t tim;

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

void* rdThread( void* param ){
	uint8_t buffer[ 20 ];
	ssize_t got;
	ssize_t x;

	while( 1 ){
		got = read( fd, buffer, 20 );
		if( got < 0 ){
			perror( "read" );
			break;
		}
		for( x = 0; x < got; x++ ){
			ln_incoming_byte( buffer[ x ] );
		}
	}

	return NULL;
}

void timerStart( uint32_t time ){
	static struct itimerspec timespec;

	memset( &timespec, 0, sizeof( struct itimerspec ) );
	timespec.it_value.tv_nsec = time * 1000; // microseconds to nanoseconds
	timespec.it_interval.tv_nsec = timespec.it_value.tv_nsec;

	if( timer_settime( tim, 0, &timespec, NULL ) < 0 ){
		perror( "timer_settime" );
	}
}

//void timerFired( union sigval sig ){
static void timerFired( int sig, siginfo_t* si, void* uc ){
	static struct itimerspec timespec;

	memset( &timespec, 0, sizeof( struct itimerspec ) );
	//disable the timer
	timer_settime( tim, 0, &timespec, NULL );

	ln_timer_fired();
}

void doWrite( uint8_t byte ){
	printf( "doWrite\n" );
	if( write( fd, &byte, 1 ) < 0 ){
		perror( "write" );
	}
}

int main( int argc, char** argv ){
	pthread_t thr;
	struct sigevent evt;
	struct sigaction sa;
	sigset_t mask;
	Ln_Message incomingMessage;


	printf( "About to open %s\n", argv[ 1 ] );
	fd = open( argv[ 1 ] );
	printf( "FD is %d\n", fd );
	if( fd < 0 ){
		fprintf( stderr, "ERROR: Can't open.  Reason: %s\n", strerror( errno ) );
		return 1;
	}

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
	evt.sigev_value.sival_ptr = &tim;
	if( timer_create( CLOCK_REALTIME, &evt, &tim ) < 0 ){
		perror( "timer_create" );
		return 1;
	}

	ln_init( timerStart, doWrite, 200 );

	pthread_create( &thr, NULL, rdThread, NULL );


	while( 1 ){
		if( ln_read_message( &incomingMessage ) == 1 ){
			printLnMessage( &incomingMessage );
			printf( "\n" );
		}

		
		usleep( 500 );
	}

}
