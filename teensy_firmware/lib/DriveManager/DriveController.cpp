#include <DriveController.h>

DriveController *DriveController::instance = nullptr;

DriveController::DriveController(BusLink &commBus, double control_cycle)
    : control_cycle(control_cycle), commBus(commBus) {
    commBus.initBus(1000000);
    instance = this;
}

DriveController::~DriveController() {}

void DriveController::configureMotorType(uint8_t id, uint8_t type) {
    switch (type) {
        case DRIVE_M3508:
            speedLoop[id - 1] = ControlRegulator(control_cycle, -MAX_CURRENT_M3508, MAX_CURRENT_M3508);
            positionLoop[id - 1] = ControlRegulator(control_cycle, -MAX_SPEED_M3508_POSITION, MAX_SPEED_M3508_POSITION);
            break;
        case DRIVE_M2006:
            speedLoop[id - 1] = ControlRegulator(control_cycle, -MAX_CURRENT_M2006, MAX_CURRENT_M2006);
            positionLoop[id - 1] = ControlRegulator(control_cycle, -MAX_SPEED_M2006_POSITION, MAX_SPEED_M2006_POSITION);
            break;
        default:
            break;
    }
    motorType[id - 1] = type;
    driveEnabled[id - 1] = true;
    torqueEnabled[id - 1] = true;
}

void DriveController::enableTorque(uint8_t id, bool enable) {
    this->torqueEnabled[id - 1] = enable;
}

void DriveController::setTargetSpeed(uint8_t id, int16_t speed) {
    speedMode[id - 1] = true;
    positionMode[id - 1] = false;
    desiredStateList[id - 1].target_speed = speed;
}

void DriveController::setTargetPosition(uint8_t id, int64_t position) {
    positionMode[id - 1] = true;
    speedMode[id - 1] = false;
    desiredStateList[id - 1].target_position = position;
}

void DriveController::setSpeedGain(uint8_t id, GainParams gain) {
    if (driveEnabled[id - 1] == false) return;
    speedLoop[id - 1].setGain(gain);
}

void DriveController::setPositionGain(uint8_t id, GainParams gain) {
    if (driveEnabled[id - 1] == false) return;
    positionLoop[id - 1].setGain(gain);
}

int16_t DriveController::getOutputValue(uint8_t id) {
    if (driveEnabled[id - 1] == false || commBus.hasContact(0x200 + id) == false) return 0;
    if (positionMode[id - 1]) setTargetSpeed(id, computeVelocity(id));
    return computePower(id);
}

int16_t DriveController::computeVelocity(uint8_t id) {
    int64_t error = desiredStateList[id - 1].target_position - motorStateList[id - 1].position;
    return (int16_t)positionLoop[id - 1].compute(error);
}

int16_t DriveController::computePower(uint8_t id) {
    int error = desiredStateList[id - 1].target_speed - motorStateList[id - 1].rpm;
    int control_value = (int16_t)speedLoop[id - 1].compute(error) + desiredStateList[id - 1].offset_current;
    if (motorType[id - 1] == DRIVE_M3508) control_value = std::clamp(control_value, -MAX_CURRENT_M3508, MAX_CURRENT_M3508);
    if (motorType[id - 1] == DRIVE_M2006) control_value = std::clamp(control_value, -MAX_CURRENT_M2006, MAX_CURRENT_M2006);
    return control_value;
}

void DriveController::updatePosition(uint8_t id) {
    int32_t diff = motorStateList[id - 1].angle - motorStateList[id - 1].previous_angle;
    if (diff < -(ENCODER_RESOLUTION / 2)) diff += ENCODER_RESOLUTION;
    else if (diff > (ENCODER_RESOLUTION / 2)) diff -= ENCODER_RESOLUTION;
    motorStateList[id - 1].position += diff;
    motorStateList[id - 1].previous_angle = motorStateList[id - 1].angle;
}

void DriveController::readMotorState(uint8_t id) {
    uint8_t read_data[8];
    commBus.readFrame(0x200 + id, read_data);
    motorStateList[id - 1].angle = (read_data[0] << 8) + read_data[1];
    motorStateList[id - 1].rpm = (read_data[2] << 8) + read_data[3];
    motorStateList[id - 1].current = (read_data[4] << 8) + read_data[5];
    motorStateList[id - 1].temperature = read_data[6];
    updatePosition(id);
}

void DriveController::dispatchMotorFrames() {
    CAN_message_t send_msg[2];
    send_msg[0].id = 0x200;
    send_msg[1].id = 0x1FF;
    send_msg[0].len = 8;
    send_msg[1].len = 8;

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            uint8_t index = i * 4 + j;
            send_msg[i].buf[j * 2] = desiredStateList[index].target_current >> 8;
            send_msg[i].buf[j * 2 + 1] = desiredStateList[index].target_current & 0xFF;
        }
    }

    for (int i = 0; i < 2; i++) {
        commBus.transmitFrame(send_msg[i]);
    }
}

void DriveController::interruptHandler() {
    for (int id = 1; id <= 8; id++) {
        instance->readMotorState(id);
        if (instance->torqueEnabled[id - 1] == true) {
            instance->desiredStateList[id - 1].target_current = instance->getOutputValue(id);
        } else {
            instance->desiredStateList[id - 1].target_current = 0;
        }
    }
    instance->dispatchMotorFrames();
}
