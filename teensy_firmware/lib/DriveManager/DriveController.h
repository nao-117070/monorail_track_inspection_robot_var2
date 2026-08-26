#ifndef DRIVECONTROLLER_H
#define DRIVECONTROLLER_H

#include <Arduino.h>
#include "ControlRegulator.h"
#include "BusLink.h"
#include <algorithm>

#define DRIVE_M2006 1
#define DRIVE_M3508 2

#define MAX_SPEED_M3508 10000
#define MAX_SPEED_M2006 10000
#define MAX_SPEED_M3508_POSITION 8000
#define MAX_SPEED_M2006_POSITION 5000
#define MAX_CURRENT_M3508 16384
#define MAX_CURRENT_M2006 10000

#define ENCODER_RESOLUTION 8192

typedef struct {
    int16_t angle = 0;
    int16_t previous_angle = 0;
    int64_t position = 0;
    int16_t rpm = 0;
    int16_t current = 0;
    int16_t temperature = 0;
} MotorState;

typedef struct {
    int64_t target_position = 0;
    int16_t target_speed = 0;
    int16_t target_current = 0;
    int16_t offset_current = 0;
} MotorDesired;

class DriveController {
  public:
    DriveController(BusLink &commBus, double control_cycle);
    ~DriveController();

    void configureMotorType(uint8_t id, uint8_t type);
    void enableTorque(uint8_t id, bool enable);
    void setTargetSpeed(uint8_t id, int16_t speed);
    void setTargetPosition(uint8_t id, int64_t position);

    void setSpeedGain(uint8_t id, GainParams gain);
    void setPositionGain(uint8_t id, GainParams gain);

    int16_t getAngle(uint8_t id) { return motorStateList[id-1].angle; }
    int64_t getPosition(uint8_t id) { return motorStateList[id-1].position; }
    int16_t getRpm(uint8_t id) { return motorStateList[id-1].rpm; }
    int16_t getCurrent(uint8_t id) { return motorStateList[id-1].current; }
    int16_t getTemperature(uint8_t id) { return motorStateList[id-1].temperature; }

    void resetPosition(uint8_t id, int64_t position) { motorStateList[id-1].position = 0; }
    void setOffsetCurrent(uint8_t id, int16_t offset) { desiredStateList[id-1].offset_current = offset; }

    int16_t getOutputValue(uint8_t id);
    void readMotorState(uint8_t id);
    void dispatchMotorFrames();

    static void interruptHandler();

  private:
    double control_cycle;
    uint8_t motorType[8] = {};
    static DriveController *instance;
    BusLink &commBus;
    ControlRegulator speedLoop[8];
    ControlRegulator positionLoop[8];
    MotorState motorStateList[8] = {};
    MotorDesired desiredStateList[8] = {};
    bool driveEnabled[8] = {};
    bool speedMode[8] = {};
    bool positionMode[8] = {};
    bool torqueEnabled[8] = {};

    int16_t computeVelocity(uint8_t id);
    int16_t computePower(uint8_t id);
    void updatePosition(uint8_t id);
};

#endif
