#ifndef cArduino_CPP
#define cArduino_CPP 1

#include "cArduino.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <string>
#include <sys/types.h>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>
	#include <sys/time.h>
using namespace std;


cArduino::cArduino()
{
}

cArduino::cArduino(ArduinoBaundRate baundRate)
{
	findArduino();

	if(MODEMDEVICE!=0)
	open(baundRate,MODEMDEVICE);
}

cArduino::cArduino(ArduinoBaundRate baundRate,char *deviceFileName)
{
	open(baundRate,deviceFileName);
}

cArduino::~cArduino()
{
	/* restore the old port settings */
	if(fd!=0)
	tcsetattr(fd,TCSANOW,&oldtio);
}

/*get Arduino Device FileName*/
char* cArduino::getDeviceName()
{
	if(MODEMDEVICE==0)	findArduino();
	return MODEMDEVICE;
}

/*Find Arduino device*/
char* cArduino::findArduino()
{
	char  dir [] = "/dev/serial/by-id/";

	DIR *d=opendir(dir);

	if(d == NULL) //Couldn't open directory
	return 0;

	struct dirent *de=NULL;
	while(de = readdir(d))
	{
		if(strstr(de->d_name,"arduino")!=0)
		{
			char s[PATH_MAX+1];
			sprintf(s,"%s%s",dir,de->d_name);

			char buf[1024];
			int len;
			if ((len = readlink(s, buf, sizeof(buf)-1)) != -1)
			   buf[len] = '\0';

			MODEMDEVICE=new char[PATH_MAX+1];
			realpath(s, MODEMDEVICE);

			closedir(d);
			return  MODEMDEVICE;
		}
	}

	closedir(d);
	return 0;
}

/*is Arduino serial port Open?*/
bool cArduino::isOpen()
{
	if(fd==0) return false;
	return true;
}

bool cArduino::open(ArduinoBaundRate baundRate)
{
	findArduino();

	return open(baundRate,0);
}

/*open serial port*/
bool cArduino::open(ArduinoBaundRate baundRate,char *DeviceFileName)
{
	int c;
	struct termios newtio;

	if(DeviceFileName==0) {
		DeviceFileName = findArduino();
	}
	MODEMDEVICE = DeviceFileName;
	/*
	Open modem device for reading and writing and not as controlling tty
	because we don't want to get killed if linenoise sends CTRL-C.
	*/
	if(MODEMDEVICE==0)
	return false;

	fd = ::open(MODEMDEVICE, O_RDWR | O_NOCTTY );
	if (fd <0) {
		perror(MODEMDEVICE);
		return false;
	}

	tcgetattr(fd,&oldtio); /* save current serial port settings */
	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	/*
	BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	CRTSCTS : output hardware flow control (only used if the cable has
	    all necessary lines. See sect. 7 of Serial-HOWTO)
	CS8     : 8n1 (8bit,no parity,1 stopbit)
	CLOCAL  : local connection, no modem contol
	CREAD   : enable receiving characters
	*/
	newtio.c_cflag =  baundRate | CRTSCTS | CS8 | CLOCAL | CREAD;

	/*
	IGNPAR  : ignore bytes with parity errors
	ICRNL   : map CR to NL (otherwise a CR input on the other computer
	    will not terminate input)
	otherwise make device raw (no other input processing)
	*/
	newtio.c_iflag = IGNPAR | ICRNL;

	/*
	Raw output.
	*/
	newtio.c_oflag = 0;

	/*
	ICANON  : enable canonical input
	disable all echo functionality, and don't send signals to calling program
	*/
	newtio.c_lflag = ICANON;

	/*
	initialize all control characters
	default values can be found in /usr/include/termios.h, and are given
	in the comments, but we don't need them here
	*/
	newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
	newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
	newtio.c_cc[VERASE]   = 0;     /* del */
	newtio.c_cc[VKILL]    = 0;     /* @ */
	newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
	newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
	newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
	newtio.c_cc[VSWTC]    = 0;     /* '\0' */
	newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
	newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
	newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
	newtio.c_cc[VEOL]     = 0;     /* '\0' */
	newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
	newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
	newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
	newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
	newtio.c_cc[VEOL2]    = 0;     /* '\0' */

	/*
	now clean the modem line and activate the settings for the port
	*/
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd,TCSANOW,&newtio);

	/*
	terminal settings done, now handle input
	*/
	return true;
}

/*zamykanie*/
void cArduino::close()
{
	::close(fd);
}

/*Flush port*/
void cArduino::flush()
{
	tcflush(fd, TCIFLUSH);
}

/*read from Arduino*/
string cArduino::read()
{
	/* read blocks program execution until a line terminating character is
	input, even if more than 255 chars are input. If the number
	of characters read is smaller than the number of chars available,
	subsequent reads will return the remaining chars. res will be set
	to the actual number of characters actually read */
	char buf[255];

	int res = ::read(fd,buf,255);
	buf[res]=0;             /* set end of string, so we can printf */

	string ret(buf);
	return ret;
}

/*read form arduino (witch timeout)
 *ret - responce
 *timeOut_MicroSec - (mikro sekundy 10-6)
 *print_error - print errors to stderr?
*/
bool cArduino::read(
	string &ret,
	unsigned long int timeOut_MicroSec,
	bool print_error
	)
{

	char buff[100];
	int len = 100;

	fd_set set;
	FD_ZERO(&set); /* clear the set */
	FD_SET(fd, &set); /* add our file descriptor to the set */

	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = timeOut_MicroSec;

	int rv = select(fd + 1, &set, NULL, NULL, &timeout);

	if(rv == -1){
		if(print_error)
		perror("arduino select"); /* an error accured */
	}
	else
	{
		if(rv == 0)
		{
			if( print_error )
			fputs("arduino timeout!\n", stderr); /* a timeout occured */
		}else{
			ret=read();
			return true; /* there was data to read */
		}
	}

	return false;
}

/*
odczytuj az do napotkania znaku / lub przekroczenia czasu
 *ret - responce
 *ultin - do jakiego znaku czytac
 *timeOut_MicroSec - (mikro sekundy 10-6)
*/
bool cArduino::read(
	string &ret,
	char until,
	unsigned long int timeOut_MicroSec)
{

	ret="";

	while(true)
	{
		string s="";
		if(!read(s,timeOut_MicroSec,false))
		break;

		ret+=s;
		if(s.find(until)!=std::string::npos)
		return true;
	}

	ret="";
	return false;
}

/*write to Arduinio*/
void cArduino::write(string text)
{
	::write(	fd,(char*)text.c_str(),(size_t)text.length() );
}

#endif
