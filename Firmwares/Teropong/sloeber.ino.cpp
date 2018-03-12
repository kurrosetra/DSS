#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2018-03-12 11:23:09

#include "Arduino.h"
#include "Arduino.h"
#define USE_BOARD				"Teropong_20171002"
#include <SPI.h>
#include <TinyGPS.h>
#include <Wire.h>
#include "FreeIMU.h"
#include "DestinationPoint.h"
#include "visca.h"

void setup() ;
void loop() ;
void sendInit() ;
void sendCoordinate(coordinate* pos, coordinateFormat format) ;
void sendHandler() ;
void sendData() ;
void pointInit() ;
void pointHandler() ;
void targetPoint() ;
void lrfInit() ;
void lrfHandler() ;
void lrfPower(bool onf) ;
void lrfFlush() ;
void overlayInit() ;
void overlayHandler() ;
void overlaySendDistance(uint16_t d) ;
void imuInit() ;
void imuHandler() ;
void imuCorrection() ;
void sonyHandler() ;
void gpsInit() ;
void gpsHandler() ;
void buttonInit() ;
void buttonHandler() ;
void butAction() ;
byte butRead() ;

#include "Teropong.ino"


#endif
