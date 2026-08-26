#include <BusLink.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

std::map<uint32_t, CAN_message_t> busBuffer1;
std::vector<CAN_message_t> busStack1;
std::map<uint32_t, CAN_message_t> busBuffer2;
std::vector<CAN_message_t> busStack2;
std::map<uint32_t, CAN_message_t> busBuffer3;
std::vector<CAN_message_t> busStack3;

std::map<uint32_t, bool> busContact1;
std::map<uint32_t, bool> busContact2;
std::map<uint32_t, bool> busContact3;
std::map<uint32_t, uint32_t> busTimestamp1;
std::map<uint32_t, uint32_t> busTimestamp2;
std::map<uint32_t, uint32_t> busTimestamp3;

void busSniff1(const CAN_message_t &msg) {
    busContact1[msg.id] = true;
    busBuffer1[msg.id] = msg;
    busTimestamp1[msg.id] = millis();
}

void busSniff2(const CAN_message_t &msg) {
    busContact2[msg.id] = true;
    busBuffer2[msg.id] = msg;
    busTimestamp2[msg.id] = millis();
}

void busSniff3(const CAN_message_t &msg) {
    busContact3[msg.id] = true;
    busBuffer3[msg.id] = msg;
    busTimestamp3[msg.id] = millis();
}

BusLink::BusLink(uint8_t node) {
    this->node = node;
    isOpen = false;
}

void BusLink::initBus(int baudrate) {
    isOpen = true;
    switch (node) {
        case 1:
            Can1.begin();
            Can1.setBaudRate(baudrate);
            Can1.setMaxMB(16);
            Can1.enableFIFO();
            Can1.enableFIFOInterrupt();
            Can1.onReceive(busSniff1);
            Can1.mailboxStatus();
            break;
        case 2:
            Can2.begin();
            Can2.setBaudRate(baudrate);
            Can2.setMaxMB(16);
            Can2.enableFIFO();
            Can2.enableFIFOInterrupt();
            Can2.onReceive(busSniff2);
            Can2.mailboxStatus();
            break;
        case 3:
            Can3.begin();
            Can3.setBaudRate(baudrate);
            Can3.setMaxMB(16);
            Can3.enableFIFO();
            Can3.enableFIFOInterrupt();
            Can3.onReceive(busSniff3);
            Can3.mailboxStatus();
            break;
        default:
            break;
    }
}

void BusLink::resetFrameTable() {
    switch (node) {
        case 1:
            busBuffer1.clear();
            break;
        case 2:
            busBuffer2.clear();
            break;
        case 3:
            busBuffer3.clear();
            break;
        default:
            break;
    }
}

void BusLink::readFrame(uint32_t id, uint8_t data[8]) {
    CAN_message_t msg;
    switch (node) {
        case 1:
            msg = busBuffer1[id];
            break;
        case 2:
            msg = busBuffer2[id];
            break;
        case 3:
            msg = busBuffer3[id];
            break;
        default:
            return;
    }
    for (int i = 0; i < 8; i++) {
        data[i] = msg.buf[i];
    }
}

void BusLink::queueFrame(uint32_t id, uint8_t data[8]) {
    bufferFrame.id = id;
    memcpy(bufferFrame.buf, data, 8);
    switch (node) {
        case 1:
            busStack1.push_back(bufferFrame);
            break;
        case 2:
            busStack2.push_back(bufferFrame);
            break;
        case 3:
            busStack3.push_back(bufferFrame);
            break;
        default:
            break;
    }
}

void BusLink::clearFrameQueue() {
    busStack1.clear();
    busStack2.clear();
    busStack3.clear();
}

void BusLink::transmitFrame(const CAN_message_t &frame) {
    if (!isOpen) return;
    CAN_message_t msg;
    msg.id = frame.id;
    msg.len = frame.len;
    memcpy(msg.buf, frame.buf, frame.len);

    switch (node) {
        case 1:
            Can1.write(msg);
            break;
        case 2:
            Can2.write(msg);
            break;
        case 3:
            Can3.write(msg);
            break;
        default:
            break;
    }
}

int8_t BusLink::transmitPendingFrames() {
    if (!isOpen) return -1;
    int8_t count = 0;
    switch (node) {
        case 1:
            for (CAN_message_t buff : busStack1) {
                Can1.write(buff);
                count++;
            }
            break;
        case 2:
            for (CAN_message_t buff : busStack2) {
                Can2.write(buff);
                count++;
            }
            break;
        case 3:
            for (CAN_message_t buff : busStack3) {
                Can3.write(buff);
                count++;
            }
            break;
        default:
            break;
    }
    clearFrameQueue();
    return count;
}

bool BusLink::hasContact(uint16_t id) {
    switch (node) {
        case 1:
            if ((millis() - busTimestamp1[id]) < BUS_TIMEOUT_MS) return true;
            break;
        case 2:
            if ((millis() - busTimestamp2[id]) < BUS_TIMEOUT_MS) return true;
            break;
        case 3:
            if ((millis() - busTimestamp3[id]) < BUS_TIMEOUT_MS) return true;
            break;
        default:
            break;
    }
    return false;
}
