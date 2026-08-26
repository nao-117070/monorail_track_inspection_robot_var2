#ifndef RK39A1A00007_H
#define RK39A1A00007_H

#include <Arduino.h>
#include "BusLink.h"

class RK39A1A00007Sensor {
  public:
    RK39A1A00007Sensor(BusLink &bus, uint32_t baseFrameId = 0x300);
    ~RK39A1A00007Sensor();

    void begin();
    bool readAngle(uint8_t sensorId);
    bool hasNewValue(uint8_t sensorId) const;
    uint16_t getRawAngle(uint8_t sensorId) const;
    float getAngleDegrees(uint8_t sensorId) const;

  private:
    BusLink &bus;
    uint32_t baseFrameId;
    uint16_t angleValues[8];
    bool hasValue[8];
};

#endif
