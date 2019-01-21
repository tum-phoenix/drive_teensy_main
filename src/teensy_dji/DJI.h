#ifndef DJI_H
#define DJI_H

#include "Arduino.h"
#include "SBUS.h"

class DJI
{

public:
  enum switchState
  {
    UNKNOWN = 0,
    UP = 1,
    MIDDLE = 2,
    DOWN = 3
  };

  // constructor
  DJI(HardwareSerial &bus) : sbus(bus)
  {
  }
  
  // begin
  void begin()
  {
    sbus.begin();
  }

  // read data from DJI remote
  bool read()
  {
    return sbus.read(&channels[0], &failSafe, &lostFrames);
  }

  // get number of lost frames
  uint16_t getLostFrames()
  {
    return lostFrames;
  }

  // returns left vertical stick values between -1 and 1
  float leftVerticalStick(float last_value)
  {
    float helpVar = (float(channels[1]) - 1024) / 660;
    if (abs(helpVar) < 1.01)
    {
      return helpVar;
    }
    else
    {
      return last_value;
    }
  }

  // returns left horizontal stick values between -1 and 1
  float leftHorizontalStick(float last_value)
  {
    float helpVar = (float(channels[3]) - 1024) / 660;
    if (abs(helpVar) < 1.01)
    {
      return helpVar;
    }
    else
    {
      return last_value;
    }
  }

  // returns right vertical stick values between -1 and 1
  float rightVerticalStick(float last_value)
  {
    float helpVar = (float(channels[2]) - 1024) / 660;
    if (abs(helpVar) < 1.01)
    {
      return helpVar;
    }
    else
    {
      return last_value;
    }
  }

  // returns right horizontal stick values between -1 and 1
  float rightHorizontalStick(float last_value)
  {
    float helpVar = (float(channels[0]) - 1024) / 660;
    if (abs(helpVar) < 1.01)
    {
      return helpVar;
    }
    else
    {
      return last_value;
    }
  }

  // returns the right switch state
  switchState rightSwitch()
  {
    switch (channels[6])
    {
    case 1541:
      return UP;
      break;
    case 1024:
      return MIDDLE;
      break;
    case 511:
      return DOWN;
      break;
    }
    return UNKNOWN;
  }

  // returns the left switch state
  switchState leftSwitch()
  {
    switch (channels[5])
    {
    case 1541:
      return UP;
      break;
    case 1024:
      return MIDDLE;
      break;
    case 511:
      return DOWN;
      break;
    }
    return UNKNOWN;
  }

private:
  SBUS sbus;               // SBUS interface
  uint8_t failSafe;        // ??
  uint16_t lostFrames = 0; // number of lost frames
  uint16_t channels[16];   // current data
};

#endif
