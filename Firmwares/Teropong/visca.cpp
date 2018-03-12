// 
// 
// 

//_____ I N C L U D E S ________________________________________________________
#include "visca.h"

//_____ D E F I N I T I O N S __________________________________________________
const byte sony_zoom_pos_1[9] = { 0x81, 0x01, 0x04, 0x47, 0x00, 0x00, 0x00, 0x00, 0xFF }; // max: 0x4000 (analog) 0x7AC0 (digital)
const byte sony_zoom_pos_2[9] = { 0x81, 0x01, 0x04, 0x47, 0x02, 0x00, 0x00, 0x00, 0xFF }; // max: 0x4000 (analog) 0x7AC0 (digital)
const byte sony_zoom_pos_3[9] = { 0x81, 0x01, 0x04, 0x47, 0x04, 0x00, 0x00, 0x00, 0xFF }; // max: 0x4000 (analog) 0x7AC0 (digital)
const byte sony_zoom_pos_4[9] = { 0x81, 0x01, 0x04, 0x47, 0x07, 0x0A, 0x0C, 0x00, 0xFF }; // max: 0x4000 (analog) 0x7AC0 (digital)

const byte titleClear[6] = { 0x81, 0x01, 0x04, 0x74, 0x00, 0xFF };
const byte titleOn[6] = { 0x81, 0x01, 0x04, 0x74, 0x02, 0xFF };
const byte titleOff[6] = { 0x81, 0x01, 0x04, 0x74, 0x03, 0xFF };

const byte titleSet[16] = {
	0x81, 0x01, 0x04, 0x73,		//header
	0,							//command type
	0,0,0x03,0,0,0,0,0,0,0,		// data
	0xFF						//terminator
};

const byte titleText[16] = {
	0x81, 0x01, 0x04, 0x73,		//header
	0,							//command type
	0x1B,0x1B,0x1B,0x1B,0x1B,0x1B,0x1B,0x1B,0x1B,0x1B,		// data
	0xFF						//terminator
};

//_____ F U N C T I O N S ______________________________________________________

Visca::Visca(HardwareSerial & uart, unsigned long baud):_uart(&uart)
{
	_baud = baud;
	_uart->begin(_baud);
}

void Visca::init()
{
	sendCommands(SONY_SET_TITLE);
	flush();
	sendCommands(SONY_SET_TITLE_CLEAR);
	flush();
	sendCommands(SONY_SET_TITLE_DISP_ON);
}

byte Visca::handler()
{

	//if (titleTimer && millis() > titleTimer) {
	//	titleTimer = 0;
	//	//clear title & clear display
	//	sendCommands(SONY_SET_TITLE_CLEAR);
	//}
	//return byte();
}

void Visca::sendCommands(commands cmd)
{
	byte buf[16];
	switch (cmd) {
	case SONY_ZOOM_IN:
		if (zoomVal < 3) {
			zoomVal++;
			zoomCommand();
		}
		break;
	case SONY_ZOOM_OUT:
		if (zoomVal > 0) {
			zoomVal--;
			zoomCommand();
		}
		break;
	case SONY_SET_TITLE:
		memcpy(buf, titleSet, 16);
		buf[7] = 0x03;
		sendRaw(buf, 16);
		break;
	case SONY_SET_TITLE_CLEAR:
		sendRaw((byte *)titleClear, 6);
		break;
	case SONY_SET_TITLE_DISP_ON:
		sendRaw((byte *)titleOn, 6);
		break;
	case SONY_SET_TITLE_DISP_OFF:
		sendRaw((byte *)titleOff, 6);
		break;
	}
}

void Visca::titleCommands(float f1, float f2)
{
	String s;
	char buf[20];
	char text1[10], text2[10];
	byte a;
	int i;
	float f;

	titleTimer = millis() + 15000;

	i = (int)fabs(f1);
	s += i; s += '.';
	f = fabs(f1) - i;
	f *= 60;
	i = (int)f;
	if (i < 10) s += '0';
	s += i; s += '.';
	f = f - i;
	f *= 60;
	i = (int)f;
	if (i < 10) s += '0';
	s += i;
	if (f1 >= 0.0f) s += 'N';
	else s += 'S';

	s += ' ';

	i = (int)fabs(f2);
	s += i; s += '.';
	f = fabs(f2) - i;
	f *= 60;
	i = (int)f;
	if (i < 10) s += '0';
	s += i; s += '.';
	f = f - i;
	f *= 60;
	i = (int)f;
	if (i < 10) s += '0';
	s += i;
	if (f2 >= 0.0f) s += 'E';
	else s += 'W';

	s.toCharArray(buf, 20);
	for (a = 0; a < 10; a++) {
		text1[a] = buf[a];
		text2[a] = buf[a + 10];
	}
	
	//set title text1
	sendCommands(SONY_SET_TITLE_1, (byte *)text1);
	flush();
	//set title text2
	sendCommands(SONY_SET_TITLE_2, (byte *)text2);
	flush();
}

void Visca::titleText(String s)
{
	char buf[20];
	char text1[10], text2[10];
	byte a;
	int i;

	memset(buf, 0, 20);
	s.toCharArray(buf, 20);
	for (a = 0; a < 10; a++) {
		text1[a] = buf[a];
		text2[a] = buf[a + 10];
	}
	//set title text1
	sendCommands(SONY_SET_TITLE_1, (byte *)text1);
	flush();
	//set title text2
	sendCommands(SONY_SET_TITLE_2, (byte *)text2);
	flush();
}

void Visca::camFreeze(bool onf)
{
	if (onf) sendRaw(camFreezeOn, 6);
	else sendRaw(camFreezeOff, 6);
	flush();
}

void Visca::flush()
{
	_uart->flush();
}

bool Visca::available()
{
	return _uart->available();
}

byte Visca::read()
{
	if(available())	return byte(_uart->read());
	else return 0;
}

void Visca::sendCommands(commands cmd, byte * data)
{
	byte a, b;
	byte tem[16];
	byte * pData = data;
	char c;

	switch (cmd) {
	case SONY_SET_TITLE:
		memcpy(tem, titleSet, 16);
		tem[4] = 0x00;
		pData + 5;
		for (a = 5; a < 9; a++)	tem[a] = *pData++;
		sendRaw(tem, 16);
		break;
	case SONY_SET_TITLE_1:
		memcpy(tem, titleSet, 16);
		tem[4] = 0x01;
		break;
	case SONY_SET_TITLE_2:
		memcpy(tem, titleSet, 16);
		tem[4] = 0x02;
		break;
	}

	if ((cmd == SONY_SET_TITLE_1) || (cmd == SONY_SET_TITLE_2)) {
		for (a = 5; a < 15; a++) {
			c = (char)*pData++;
			b = convertChar(c);
			tem[a] = b;
		}
		sendRaw(tem, 16);
	}
}

void Visca::zoomCommand()
{
	if (zoomVal == 0) sendRaw((byte *)sony_zoom_pos_1, 9);
	else if (zoomVal == 1)sendRaw((byte *)sony_zoom_pos_2, 9);
	else if (zoomVal == 2)sendRaw((byte *)sony_zoom_pos_3, 9);
	else if (zoomVal == 3)sendRaw((byte *)sony_zoom_pos_4, 9);
}

byte Visca::convertChar(char c)
{
	byte ret = 0x1B;

	if (c >= 'A' && c <= 'Z') ret = c - 'A';
	else if (c >= 'a' && c <= 'z') ret = c - 'a';
	else if (c >= '1'&&c <= '9') ret = c - '1' + 0x1E;
	else if (c == '&') ret = 0x1A;
	else if (c == ' ') ret = 0x1B;
	else if (c == '?') ret = 0x1C;
	else if (c == '!') ret = 0x1D;
	else if (c == '0') ret = 0x27;
	else if (c == '\"') ret = 0x49;
	else if (c == ':') ret = 0x4A;
	else if (c == '\'') ret = 0x4B;
	else if (c == '.') ret = 0x4C;
	else if (c == ',') ret = 0x4D;
	else if (c == '\\') ret = 0x4E;
	else if (c == '-') ret = 0x4F;

	return ret;
}

void Visca::sendRaw(byte * cmd, byte len)
{
	byte i = 0;
	byte * s = cmd;

	for (i = 0; i<len; i++) {
		_uart->write(*s++);
	}
}


