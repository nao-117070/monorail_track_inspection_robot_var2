#ifndef BUSLINK_H
#define BUSLINK_H

#include <vector>
#include <map>
#include <FlexCAN_T4.h>

#define BUS_TIMEOUT_MS 10

extern std::map<uint32_t, bool> busContact1;
extern std::map<uint32_t, bool> busContact2;
extern std::map<uint32_t, bool> busContact3;
extern std::map<uint32_t, uint32_t> busTimestamp1;
extern std::map<uint32_t, uint32_t> busTimestamp2;
extern std::map<uint32_t, uint32_t> busTimestamp3;

typedef struct {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
} BusFrame_t;

class BusLink {
  public:
    BusLink(uint8_t node);

    void initBus(int baudrate);
    void resetFrameTable();
    void readFrame(uint32_t id, uint8_t data[8]);
    void queueFrame(uint32_t id, uint8_t data[8]);
    void clearFrameQueue();
    void transmitFrame(const CAN_message_t &frame);
    int8_t transmitPendingFrames();
    bool hasContact(uint16_t id);

  private:
    CAN_message_t bufferFrame;
    uint8_t node;
    bool isOpen;
    bool _bus1 = false;
    bool _bus2 = false;
    bool _bus3 = false;

  protected:
    BusLink(){};
};

#endif
