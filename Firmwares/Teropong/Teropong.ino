#include "Arduino.h"
/*
 Name:		teropong_20171126.ino
 Created:	11/26/2017 1:08:00 AM
 Author:	kurro

 - add imu correction
 - send data re-format

 TODO:
 - add focus button function

 Calibration.h:
 ///TEROPONG v20171002
 const int acc_off_x = 0;
 const int acc_off_y = 6;
 const int acc_off_z = -1;
 const float acc_scale_x = 259.64880715994633;
 const float acc_scale_y = 253.19561613799516;
 const float acc_scale_z = 250.57955934014902;

 const int magn_off_x = -23;
 const int magn_off_y = -114;
 const int magn_off_z = -57;
 const float magn_scale_x = 536.9715502027125;
 const float magn_scale_y = 540.6919514999853;
 const float magn_scale_z = 431.5951241975027;

 */

#define USE_BOARD				"Teropong_20171002"

#include <SPI.h>
#include <TinyGPS.h>
#include <Wire.h>
#include "FreeIMU.h"
#include "DestinationPoint.h"
#include "visca.h"

#define LED_PIN					13
#define LRF_POWER_PIN			4
#define LRF_POWER_ON			HIGH
#define LRF_POWER_OFF			LOW
#define LRF_START_COMMAND		"ON"
#define LRF_DATA_HEADER			"D="

#define DATA_SENDING			1
//#define BUTTON_DEBUG
//#define GPS_DEBUG
//#define POINT_DEBUG
//#define SONY_DEBUG
//#define LRF_DEBUG
//#define IMU_DEBUG
//#define OVERLAY_DEBUG
#define SEND_DEBUG

#define DEBOUNCE_TIMEOUT		100

TinyGPS gps;
// Set the FreeIMU object
FreeIMU my3IMU = FreeIMU();
DestPoint dp;
Visca sony(Serial1, 9600);

#if DATA_SENDING
const uint32_t IMU_STABILIZE_DELAY = 90000;
#else
const uint32_t IMU_STABILIZE_DELAY = 1000;
#endif	//#if DATA_SENDING

const float yprCorrection[3] = { 25.5f, 0.0f, 0.0f };
byte butVal = 0;
//zoomState:
// 0 -> do nothing (stop (de)zooming)
// 1 -> zoom in
// 2 -> zoom out
byte zoomState = 0;
bool stabilizedImu = 0;
float ypr[3]; // yaw pitch roll
uint16_t distance = 0;
float heading = 0.0f;
float elevation = 0.0f;
float flat = 0.0f, flon = 0.0f;
coordinate local, target;
bool targetFound = 0;
bool startMeasuring = 0;
String lrfString;
byte lrfBufSize = 64;
bool lrfReady = 0;

typedef enum
{
	BUT_LRF_ENABLE,
	BUT_LRF_START,
	BUT_FOCUS_OUT,
	BUT_FOCUS_IN,
	BUT_ZOOM_OUT,
	BUT_ZOOM_IN
} butPos;

const byte button[6] = { A12,  //LRF_ENABLE
		25,   //LRF_START
		26,   //ZOOM_IN
		27,   //ZOOM_OUT
		28,   //FOCUS_IN
		29    //FOCUS_OUT
		};

void setup()
{
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);

#if DATA_SENDING
	sendInit();
#else
	Serial.begin(115200);

	String s;
	int loc;
	s = __FILE__;
	loc = s.lastIndexOf('\\');
	s = s.substring(loc + 1);
	loc = s.indexOf('.');
	s = s.substring(0, loc);

	Serial.println();
	Serial.println(F("teropong - DDS test suite!"));
	Serial.print(F("s/w: "));
	Serial.println(s);
	Serial.print(F("Board: "));
	Serial.println(USE_BOARD);
#endif // DATA_SENDING

	delay(1000);

	//init
	buttonInit();
	gpsInit();
	overlayInit();
	sony.init();
	lrfInit();
	pointInit();
	imuInit();
}

void loop()
{
#if (DATA_SENDING == 0)
	char c;
	bool b = 0;
#endif	//#if (DATA_SENDING == 0)

#if(DATA_SENDING==0)
	if (Serial.available()) {
		c = Serial.read();
		switch (c)
		{
		case '0':
			distance = 0;
			b = 1;
			break;
		case '1':
			distance = 500;
			b = 1;
			break;
		case '2':
			distance = 1000;
			b = 1;
			break;
		case '3':
			distance = 1500;
			b = 1;
			break;
		}

		if (b) {
			Serial.print(F("distance= "));
			Serial.println(distance);
		}
	}
#endif // DATA_SENDING==0

	buttonHandler();
	imuHandler();
	gpsHandler();
	sonyHandler();
	lrfHandler();
	pointHandler();
	overlayHandler();
	sendHandler();
}

/////////////////
/// SEND DATA ///
/////////////////
void sendInit()
{
	Serial.begin(9600);

	Serial.println();
	Serial.println(F("=============="));
	Serial.println(F("Teropong - DDS"));
	Serial.println(F("=============="));
	Serial.println();
}

void sendCoordinate(coordinate* pos, coordinateFormat format)
{
	if (format == DMS_SEPARATE) {
		Serial.print(F("Lat: "));
		Serial.print(abs(pos->lat.degree));
		Serial.print('.');
		Serial.print(pos->lat.minute);
		Serial.print('\'');
		Serial.print(pos->lat.second);
		Serial.print('"');
		if (pos->lat.dms < 0)
			Serial.print('S');
		else
			Serial.print('N');
		Serial.println();

		Serial.print(F("Lon: "));
		Serial.print(abs(pos->lon.degree));
		Serial.print('.');
		Serial.print(pos->lon.minute);
		Serial.print('\'');
		Serial.print(pos->lon.second);
		Serial.print('"');
		if (pos->lon.dms < 0)
			Serial.print('W');
		else
			Serial.print('E');
		Serial.println();
	}
	else {
		Serial.println(F("Location: "));
		Serial.print(pos->lat.dms, 6);
		Serial.print(' ');
		Serial.print(pos->lon.dms, 6);
		Serial.println();
	}

}

void sendHandler()
{
}

void sendData()
{
	// send data
	Serial.println();
	// local coordinate
	Serial.println(F("coordinate Local:"));
	sendCoordinate(&local, DMS_FLOAT);
	// distance
	Serial.print("Distance= ");
	Serial.print(distance);
	Serial.println('m');
	// heading
	Serial.print("Bearing= ");
	Serial.println(heading);
	// elevation
	Serial.print("Elevation= ");
	Serial.println(elevation);
	//target
	Serial.println();
	Serial.println(F("coordinate Target:"));
	sendCoordinate(&target, DMS_FLOAT);
}

////////////////
/// POINTING ///
////////////////
void pointInit()
{
}

void pointHandler()
{
	static float prevLat = 0.0f, prevLon = 0.0f;
	static uint32_t timer = 0;

	if (targetFound) {
		if (timer == 0) {
#if DATA_SENDING
			timer = millis() + 15000;
#else
			timer = millis() + 1000;
#endif // DATA_SENDING

			// CAM_Freeze ON
			sony.camFreeze(1);

			// send distance to overlay
			overlaySendDistance(distance);
			// send target to sony
			sony.titleCommands(target.lat.dms, target.lon.dms);
			sendData();
		}
		else if (millis() > timer) {
			heading = 0;
			elevation = 0;
			distance = 0;
			targetFound = 0;
//			startMeasuring = 0;
			timer = 0;
			dp.reset(&target);

			// CAM_Freeze OFF
			sony.camFreeze(0);
			// send distance to overlay
			overlaySendDistance(distance);
			// send target to sony
			sony.sendCommands(SONY_SET_TITLE_CLEAR);
		}
	}
	else {
		targetFound = 0;
//		startMeasuring = 0;
		timer = 0;
	}

	if ((prevLat != flat) || (prevLon != flon)) {
		prevLat = flat;
		prevLon = flon;

		local.lat.dms = flat;
		local.lon.dms = flon;
		dp.update(&local, DMS_SEPARATE);
#ifdef POINT_DEBUG
		showCoordinate(&local);
#endif // POINT_DEBUG
	}
}

void targetPoint()
{
	dp.reset(&target);

#ifdef POINT_DEBUG
	Serial.println(F("start targetPoint()"));
#endif

	heading = ypr[0];
	elevation = ypr[1];

	//dp.destinationCalculation(&local, &target, distance, heading);
	dp.destinationCalculation(&local, &target, distance, heading, elevation);
	targetFound = 1;
	dp.update(&target, DMS_SEPARATE);

#ifdef POINT_DEBUG
	Serial.println(F("TARGET:"));
	Serial.println(F("======="));
	showCoordinate(&target);
#endif // POINT_DEBUG
}

#ifdef POINT_DEBUG
void showCoordinate(coordinate* pos)
{
	Serial.print(F("Lat: "));
	Serial.print(pos->lat.dms, 6);
	Serial.print(" -> ");
	Serial.print(pos->lat.degree);
	Serial.print(' ');
	Serial.print(pos->lat.minute);
	Serial.print('\'');
	Serial.print(pos->lat.second);
	Serial.println('"');

	Serial.print(F("Lon: "));
	Serial.print(pos->lon.dms, 6);
	Serial.print(" -> ");
	Serial.print(pos->lon.degree);
	Serial.print(' ');
	Serial.print(pos->lon.minute);
	Serial.print('\'');
	Serial.print(pos->lon.second);
	Serial.println('"');
}
#endif // POINT_DEBUG

///////////
/// LRF ///
///////////
void lrfInit()
{
	Serial2.begin(115200);
	lrfString.reserve(lrfBufSize);
	lrfString = "";

	pinMode(LRF_POWER_PIN, OUTPUT);
	lrfPower(LRF_POWER_ON);
	lrfReady = 1;
}

void lrfHandler()
{
	bool lrfStringCompleted = 0;
	char c;
	String s;
	static uint32_t _first_command = 3000;
	bool _command_found = 0;
	byte awal, akhir;
	String tem;
	uint16_t _d = 0;

	if (_first_command && (millis() >= _first_command)) {
		Serial2.println();
		Serial2.println(LRF_START_COMMAND);

		_first_command = 0;
	}	//(_first_command && (millis() >= _first_command))

	if (Serial2.available()) {
		c = Serial2.read();
#ifdef LRF_DEBUG
		Serial.write(c);
#endif // LRF_DEBUG
		if (c == 'D')
			lrfString = "";
		else if (c == 'm')
			lrfStringCompleted = 1;

		lrfString += c;
	}	//(Serial2.available())

	if (lrfStringCompleted) {
#ifdef LRF_DEBUG
		Serial.println(lrfString);
#endif // LRF_DEBUG

		if (lrfString.indexOf(LRF_DATA_HEADER) >= 0
				&& lrfString.indexOf('m') > lrfString.indexOf('.'))
			_command_found = 1;

		if (_command_found) {

			awal = lrfString.indexOf('=') + 1;
			akhir = lrfString.indexOf('m', awal);
			tem = lrfString.substring(awal, akhir);
			_d = tem.toInt();
#ifdef LRF_DEBUG
			Serial.print(F("D="));
			Serial.println(_d);
#endif	//#if LRF_DEBUG

			//start calculate target position
			if (startMeasuring && !targetFound) {
				distance = _d;
				targetPoint();
			}	//(_d >= 0)
		}	//(_command_found)

		lrfString = "";
	}	//(lrfStringCompleted)

}

void lrfPower(bool onf)
{
	digitalWrite(LRF_POWER_PIN, onf);
	if (onf == LRF_POWER_OFF) {
		lrfReady = 0;
	}

	lrfString = "";
}

void lrfFlush()
{
	while (Serial2.available())
		Serial2.read();
}

///////////////
/// OVERLAY ///
///////////////
void overlayInit()
{
	Serial3.print(F("$0,0*"));
	Serial3.flush();
	overlaySendDistance(0);
}

void overlayHandler()
{
	static uint32_t timer = 0;
	int h, p;
	String s = "$";

	if (targetFound) {
		h = heading * 10;
		p = elevation * 100;
	}
	else {
		h = ypr[0] * 10;
		p = ypr[1] * 100;
	}

	if (h < 0)
		h += 3600;

	if (millis() > timer) {
		timer = millis() + 500;

		s += h;
		s += ',';
		s += p;
		s += '*';
		Serial3.print(s);
#ifdef OVERLAY_DEBUG
		Serial.println(s);
#endif // OVERLAY_DEBUG

	}
}

void overlaySendDistance(uint16_t d)
{
	String s = "#";
	s += d;
	s += '*';

#ifdef OVERLAY_DEBUG
	Serial.println(s);
#endif // OVERLAY_DEBUG
	Serial3.print(s);
}

///////////
/// IMU ///
///////////
void imuInit()
{
	Wire.begin();

	delay(5);
	my3IMU.init(); // the parameter enable or disable fast mode
	delay(5);
}

void imuHandler()
{
	static bool stabilize = 0;
	static uint32_t timer = 0;
	static uint16_t prevTick = 0;
	uint16_t tick = millis() / 200;
	static uint32_t dispImuStabil = 0;

	if (millis() > timer) {
		timer = millis() + 10;

		my3IMU.getYawPitchRoll(ypr);
		imuCorrection();
#ifdef IMU_DEBUG
		Serial.print(millis() / 60000);
		Serial.print(" Yaw: ");
		Serial.print(ypr[0]);
		Serial.print(" Pitch: ");
		Serial.print(ypr[1]);
		Serial.print(" Roll: ");
		Serial.print(ypr[2]);
		Serial.println();
#endif // IMU_DEBUG
		if (millis() > IMU_STABILIZE_DELAY) {
			if (prevTick != tick) {
				prevTick = tick;
				digitalWrite(LED_PIN, !digitalRead(LED_PIN));
			}

#ifdef IMU_DEBUG
			if (!stabilize) Serial.println("IMU stabil");
#endif // IMU_DEBUG
			if (!stabilize) {
				sony.titleText("IMU Ready!");
				dispImuStabil = millis() + 10000;
			}
			if (dispImuStabil && millis() > dispImuStabil) {
				dispImuStabil = 0;
				sony.sendCommands(SONY_SET_TITLE_CLEAR);
			}
			stabilize = 1;
		}

	}

}

void imuCorrection()
{
	for ( byte i = 0; i < 3; i++ )
		ypr[i] -= yprCorrection[i];

	if (fabs(ypr[0]) > 180.0) {
		if (ypr[0] < 0)
			ypr[0] = 360.0f + ypr[0];
		else
			ypr[0] = ypr[0] - 360.0f;
	}
}

////////////
/// SONY ///
////////////
void sonyHandler()
{
	static bool displayInit = 0;

	if (millis() > 10000 && !displayInit) {
		displayInit = 1;
		sony.init();
	}

}

///////////
/// GPS ///
///////////
void gpsInit()
{
	Serial3.begin(4800);
}

void gpsHandler()
{
	bool newData = 0;
	char c;
	unsigned long age;
	String s;

	while (Serial3.available()) {
		c = Serial3.read();
		//new valid sentence arrive
		if (gps.encode(c))
			newData = 1;
	}

	if (newData) {
		gps.f_get_position(&flat, &flon, &age);
#ifdef GPS_DEBUG
		Serial.print("LAT=");
		Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
		Serial.print(" LON=");
		Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
		Serial.print(" SAT=");
		Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
		Serial.println();
#endif // GPS_DEBUG
	}
}

//////////////
/// BUTTON ///
//////////////
void buttonInit()
{
	byte a;
	for ( a = 0; a < 6; a++ )
		pinMode(button[a], INPUT_PULLUP);
}

void buttonHandler()
{
	byte a;
	static uint32_t debounceTimer = 0;

	a = butRead();
	if (debounceTimer) {
		if (millis() > debounceTimer) {
			debounceTimer = 0;
			if (a != butVal) {
				butVal = a;
#ifdef BUTTON_DEBUG
				Serial.print(F("BUT= 0b"));
				Serial.println(butVal, BIN);
#endif // BUTTON_DEBUG
				butAction();
			}
		}
	}
	else {
		if (a != butVal)
			debounceTimer = millis() + DEBOUNCE_TIMEOUT;
	}
}

void butAction()
{
	byte a, b;
	static byte prevButVal = 0;
	String s;

	///ZOOM
	//zoom in
	a = bitRead(prevButVal, BUT_ZOOM_IN);
	b = bitRead(butVal, BUT_ZOOM_IN);
	if (a != b) {
		//zoom in button has been pressed
		if (b && (zoomState == 0)) {
			zoomState = 1;
			sony.sendCommands(SONY_ZOOM_IN);
#ifdef BUTTON_DEBUG
			Serial.println(F("start zoom in"));
#endif // BUTTON_DEBUG
		}
		//zoom in button has been released
		else {
			zoomState = 0;
#ifdef BUTTON_DEBUG
			Serial.println(F("no zoom"));
#endif // BUTTON_DEBUG
		}
	}
	//zoom out
	a = bitRead(prevButVal, BUT_ZOOM_OUT);
	b = bitRead(butVal, BUT_ZOOM_OUT);
	if (a != b) {
		//zoom out button has been pressed
		if (b && (zoomState == 0)) {
			zoomState = 2;
			sony.sendCommands(SONY_ZOOM_OUT);
#ifdef BUTTON_DEBUG
			Serial.println(F("start zoom out"));
#endif // BUTTON_DEBUG
		}
		//zoom out button has been released
		else {
			zoomState = 0;
#ifdef BUTTON_DEBUG
			Serial.println(F("no zoom"));
#endif // BUTTON_DEBUG
		}
	}

	///FOCUS
	//focus in
	a = bitRead(prevButVal, BUT_FOCUS_IN);
	b = bitRead(butVal, BUT_FOCUS_IN);
	if (a != b) {
		//focus in button has been pressed
		if (b) {
		}
		//focus in button has been released
		else {
		}
	}

	//focus out

	///LRF

	//start measurement
	a = bitRead(prevButVal, BUT_LRF_START);
	b = bitRead(butVal, BUT_LRF_START);
	if (a != b) {
		if (b) {
			//lrf start button has been pressed
			startMeasuring = 1;
#ifdef BUTTON_DEBUG
			Serial.println(F("Start measuring"));
#endif	//#if BUTTON_DEBUG

		}
		else {
			if (!targetFound) {
				//clear display
				sony.sendCommands(SONY_SET_TITLE_CLEAR);
			}
			startMeasuring = 0;
		}
	}

	prevButVal = butVal;
}

byte butRead()
{
	byte ret = 0;

	for ( byte i = 0; i < 6; i++ )
		bitWrite(ret, i, !digitalRead(button[i]));

	return ret;
}
