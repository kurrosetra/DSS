//_____ I N C L U D E S ________________________________________________________
#include "DestinationPoint.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ F U N C T I O N S ______________________________________________________
DestPoint::DestPoint()
{
}

void DestPoint::destinationCalculation(coordinate* local, coordinate* target, uint32_t distance, float bearing)
{
  float lat,lon,bear,a,b,c,d;

  lat= radians(local->lat.dms);
  lon= radians(local->lon.dms);
  bear= radians(bearing);

  a=float(distance)/earthRadius;
  b= sin(lat) * cos(a);
  c= cos(lat) * sin(a) * cos(bear);
  d= asin(b + c);
  target->lat.dms = degrees(d);

  b= sin(bear) * sin(a) * cos(lat);
  c= cos(a) - (sin(lat) * sin (d));
  d= lon + atan2(b,c);
  target->lon.dms= degrees(d);
  
}

void DestPoint::destinationCalculation(coordinate* local, coordinate* target, uint32_t distance, float bearing, float pitch)
{
	uint16_t corrDistance = distance * cos(radians(fabs(pitch)));
	destinationCalculation(local, target, corrDistance, bearing);
}

void DestPoint::reset(coordinate* pos)
{
  pos->lat.degree=0;
  pos->lat.minute=0;
  pos->lat.second=0;
  pos->lat.dms=0.0f;

  pos->lon.degree=0;
  pos->lon.minute=0;
  pos->lon.second=0;
  pos->lon.dms=0.0f;
}

void DestPoint::update(coordinate* pos, coordinateFormat format)
{
  float a=0.0f,b=0.0f,c=0.0f;
  int d;
  byte m;
  float s;
  
  if(format==DMS_SEPARATE){
    a= pos->lat.dms;
    d = (int)a;
    b= (fabs(a)-float(abs(d))) * 60;
    m= (byte) b;
    s= (b-m)*60;

    pos->lat.degree=d;
    pos->lat.minute=m;
    pos->lat.second=s;

    a= pos->lon.dms;
    d = (int)a;
    b= (fabs(a)-float(abs(d))) * 60;
    m= (byte) b;
    s= (b-m)*60;

    pos->lon.degree=d;
    pos->lon.minute=m;
    pos->lon.second=s;
  }
  else if(format==DMS_FLOAT){
    pos->lat.dms= float(abs(pos->lat.degree)) + float(pos->lat.minute)/60 + float(pos->lat.second)/3600;
    if(pos->lat.degree < 0) pos->lat.dms= 0-pos->lat.dms;
    pos->lon.dms= float(abs(pos->lon.degree)) + float(pos->lon.minute)/60 + float(pos->lon.second)/3600;
    if(pos->lon.degree < 0) pos->lon.dms= 0-pos->lon.dms;
  }
}

