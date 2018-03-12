#ifndef __DESTINATION_POINT_H__
#define __DESTINATION_POINT_H__

//_____ I N C L U D E S ________________________________________________________
#include "Arduino.h"

//_____ D E F I N I T I O N S __________________________________________________
typedef enum {
  DMS_SEPARATE,
  DMS_FLOAT
} coordinateFormat;

typedef struct{
  int16_t degree;
  byte minute;
  float second;
  float dms;
} position;


typedef struct{
  position lat;
  position lon;
} coordinate;


//_____ M A C R O S ____________________________________________________________

//_____ D E C L A R A T I O N S ________________________________________________
class DestPoint
{
  public:
    //functions
    DestPoint();

    void destinationCalculation(coordinate* local, coordinate* target, uint32_t distance, float bearing);
	void destinationCalculation(coordinate* local, coordinate* target, uint32_t distance, float bearing, float pitch);
    void reset(coordinate* pos);
    void update(coordinate* pos, coordinateFormat format);
    //variables
  private:  
    //functions

    //variables
    uint32_t earthRadius=6371000UL;
};

#endif  //__DESTINATION_POINT_H__
