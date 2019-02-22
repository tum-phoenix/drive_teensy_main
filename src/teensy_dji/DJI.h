#ifndef DJI_H
#define DJI_H

#include "Arduino.h"
#include "SBUS.h"

class DJI {

public:

    enum switchState {
        UNKNOWN = 0,
        DOWN = 1,
        MIDDLE = 2,
        UP = 3
    };

    // constructor
    explicit DJI(HardwareSerial &bus) : sbus(bus) {
    }

    // begin
    void begin(HardwareSerial *_serialPort1) {
        _serialPort1->begin(100000, SERIAL_8E1_RXINV_TXINV);
    }

    // read data from DJI remote
    bool read() {
        if (sbus.read(&channels[0], &failSafe, &lostFrames)) {
            ping_us += (systemClock->getMonotonic() - last_rc_receive).toUSec();
            ping_us /= 2;
            last_rc_receive = systemClock->getMonotonic();
        }
    }

    // returns left vertical stick values between -1 and 1
    float leftVerticalStick(float last_value) {
        if (failSafe == 1) return 0.0;

        float helpVar = (float(channels[1]) - 1024) / 660;
        if (fabsf(helpVar) <= 1) {
            return helpVar;
        } else {
            return last_value;
        }
    }

    // returns left horizontal stick values between -1 and 1
    float leftHorizontalStick(float last_value) {
        if (failSafe == 1) return 0.0;

        float helpVar = (float(channels[3]) - 1024) / 660;
        if (fabsf(helpVar) <= 1) {
            return helpVar;
        } else {
            return last_value;
        }
    }

    // returns right vertical stick values between -1 and 1
    float rightVerticalStick(float last_value) {
        if (failSafe == 1) return 0.0;

        float helpVar = (float(channels[2]) - 1024) / 660;
        if (fabsf(helpVar) <= 1) {
            return helpVar;
        } else {
            return last_value;
        }
    }

    // returns right horizontal stick values between -1 and 1
    float rightHorizontalStick(float last_value) {
        if (failSafe == 1) return 0.0;

        float helpVar = (float(channels[0]) - 1024) / 660;
        if (fabsf(helpVar) <= 1) {
            return helpVar;
        } else {
            return last_value;
        }
    }

    // returns the right switch state
    switchState rightSwitch() {
        if (failSafe == 1) return UNKNOWN;

        switch (channels[6]) {
            case 1541:
                return UP;
            case 1024:
                return MIDDLE;
            case 511:
                return DOWN;
            default:
                return UNKNOWN;
        }
    }

    // returns the left switch state
    switchState leftSwitch() {
        if (failSafe == 1) return UNKNOWN;

        switch (channels[5]) {
            case 1541:
                return UP;
            case 1024:
                return MIDDLE;
            case 511:
                return DOWN;
            default:
                return UNKNOWN;
        }
    }

    uint16_t get_ping() {
        return ping_us;
    }

    uint32_t get_age_last_update() {
        return (uint32_t)(systemClock->getMonotonic() - last_rc_receive).toUSec();
    }

private:
    SBUS sbus;                      // SBUS interface
    uint8_t failSafe = 1;           // init in fail save mode
    // if in fail save mode, all sticks are 0 positioned
    // if in fail save mode, all switches are in UNKNOWN state
    uint16_t lostFrames = 0;        // number of lost frames
    uint16_t channels[16] = {0};    // current data

    uint16_t ping_us = 0;
    MonotonicTime last_rc_receive = MonotonicTime::fromMSec(0);
};


#endif
