// visca.h

#ifndef _VISCA_h
#define _VISCA_h

//_____ I N C L U D E S ________________________________________________________
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

//_____ D E F I N I T I O N S __________________________________________________
typedef enum {
	SONY_ZOOM_IN,
	SONY_ZOOM_OUT,
	SONY_SET_TITLE,
	SONY_SET_TITLE_1,
	SONY_SET_TITLE_2,
	SONY_SET_TITLE_CLEAR,
	SONY_SET_TITLE_DISP_ON,
	SONY_SET_TITLE_DISP_OFF
} commands;

//_____ M A C R O S ____________________________________________________________

//_____ D E C L A R A T I O N S ________________________________________________
class Visca
{
	public:
		//functions
		Visca(HardwareSerial &uart, unsigned long baud);

		void init();
		byte handler();
		void sendRaw(byte *cmd, byte len);
		void sendCommands(commands cmd);
		void titleCommands(float f1, float f2);
		void titleText(String s);
		void camFreeze(bool onf);
		void flush();
		bool available();
		byte read();

		//variables
	private:
		//functions
		void zoomCommand();
		byte convertChar(char c);
		void sendCommands(commands cmd, byte * data);

		//variables
		HardwareSerial *_uart;
		unsigned long _baud;
		unsigned long titleTimer;
		String resvString[][64];
		byte titleState[16] = { 0x81, 0x01, 0x04, 0x73, 0x00,
			0,0,0,0,
			0,0,0,0,0,0,0xFF },
			title1[16] = { 0x81, 0x01, 0x04, 0x73, 0x00,
			0,0,0,0,0,0,0,0,0,0,
			0xFF },
			title2[16] = { 0x81, 0x01, 0x04, 0x73, 0x00,
			0,0,0,0,0,0,0,0,0,0,
			0xFF },
			camFreezeOn[6] = { 0x81, 0x01, 0x04, 0x62, 0x02, 0xFF},
			camFreezeOff[6] = { 0x81, 0x01, 0x04, 0x62, 0x03, 0xFF }
			;
			


		String _header;
		String s;
		bool sCompleted;
		byte zoomVal;
};

#endif

