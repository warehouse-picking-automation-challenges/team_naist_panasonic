#ifndef cArduino_H
#define cArduino_H 1

/*
Arduino Serial Port Comunication

Arduino information:
	http://playground.arduino.cc/Interfacing/LinuxTTY

	warning setting up port speed does not work :/ you nead to do in mannualy:

		stty -F /dev/ttyUSB0 cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts

Serial Conection Based On:
	Gary Frerking   gary@frerking.org
	http://www.tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html
*/


#include <termios.h>
#include <string>

using namespace std;

enum ArduinoBaundRate{
  B300bps  = B300,
  B600bps  = B600,
  B1200bps = B1200,
  B2400bps = B2400,
  B4800bps = B4800,
  B9600bps = B9600,
  B19200bps=B19200,
  B115200bps=115200
};

class cArduino{

private:
        /*serial port FileDescriptor*/
        int fd=0;

		/*memory of port settings*/
        struct termios oldtio;

        /*Arduino FileName*/
        char *MODEMDEVICE =0;

public:

	cArduino();
	cArduino(ArduinoBaundRate baundRate);
	cArduino(ArduinoBaundRate baundRate,char *deviceFileName);

	~cArduino();

	/*get Arduino Device FileName*/
	char* getDeviceName();
	/*Find Arduino device*/
	char *findArduino();

	/*is Arduino serial port Open?*/
	bool isOpen();

	/*open serial port, find device*/
	bool open(ArduinoBaundRate baundRate);

	/*open serial port*/
	bool open(ArduinoBaundRate baundRate,char *DeviceFileName);

	/*zamykanie*/
	void close();

	/*Flush port*/
	void flush();

	/*read from Arduino*/
	string read();

	/*read form arduino (witch timeout)
	 *ret - responce
	 *timeOut_MicroSec - (mikro sekundy 10-6)
	 *print_error - print errors to stderr?
	*/
	bool read(
		string &ret,
		unsigned long int timeOut_MicroSec=(1*1000000),//1sec
		bool print_error=false
		);

	/*
	odczytuj az do napotkania znaku / lub przekroczenia czasu
	 *ret - responce
	 *ultin - do jakiego znaku czytac
	 *timeOut_MicroSec - (mikro sekundy 10-6)
	*/
	bool read(
		string &ret,
		char until,
		unsigned long int timeOut_MicroSec
		);

	/*write to Arduinio*/
	void write(string text);

};

#endif
