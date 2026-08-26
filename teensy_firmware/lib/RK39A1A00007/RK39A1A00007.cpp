#include "RK39A1A00007.h"

RK39A1A00007Sensor::RK39A1A00007Sensor(BusLink &bus, uint32_t baseFrameId)
    : bus(bus), baseFrameId(baseFrameId) {
  for (int i = 0; i < 8; ++i) {
    angleValues[i] = 0;
    hasValue[i] = false;
  }
}

RK39A1A00007Sensor::~RK39A1A00007Sensor() {}

void RK39A1A00007Sensor::begin() {
  // No special initialization required for this sensor reader.
}

bool RK39A1A00007Sensor::readAngle(uint8_t sensorId) {
  if (sensorId == 0 || sensorId > 8) return false;
  uint32_t frameId = baseFrameId + sensorId;
  if (!bus.hasContact((uint16_t)frameId)) return false;

  uint8_t data[8] = {0};
  bus.readFrame(frameId, data);
  uint16_t raw = (uint16_t(data[0]) << 8) | uint16_t(data[1]);
  angleValues[sensorId - 1] = raw;
  hasValue[sensorId - 1] = true;
  return true;
}

bool RK39A1A00007Sensor::hasNewValue(uint8_t sensorId) const {
  if (sensorId == 0 || sensorId > 8) return false;
  return hasValue[sensorId - 1];
}

uint16_t RK39A1A00007Sensor::getRawAngle(uint8_t sensorId) const {
  if (sensorId == 0 || sensorId > 8) return 0;
  return angleValues[sensorId - 1];
}

float RK39A1A00007Sensor::getAngleDegrees(uint8_t sensorId) const {
  uint16_t raw = getRawAngle(sensorId);
  return (float)raw / 65535.0f * 360.0f;
}
