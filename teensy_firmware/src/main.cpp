#include <Arduino.h>
#include "DriveController.h"
#include <IntervalTimer.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <Metro.h>
#include <USBHost_t36.h>

const GainParams speedLoopGain = {3, 0.3, 0.5};
const double controlIntervalMs = 1.0;
const int controllerNodeId = 1;
const int canBusBaud = 1000000;

const uint8_t potentiometerPins[4] = {14, 15, 16, 17};
const double potMinAngle = 109.82;
const double potMaxAngle = 110.60;
const float minSpeedRatio = 0.7f;
const float maxSpeedRatio = 1.3f;
const double potAnalogMax = 4095.0;

// ---------- ネットワーク設定 ----------
byte mac[] = { 0x04, 0xE9, 0xE5, 0x00, 0x00, 0x01 };
IPAddress ip(192, 168, 1, 10);        // Teensy IP
IPAddress pc_ip(192, 168, 1, 20);     // ミニPC IP
unsigned int localPort = 8888;
EthernetUDP Udp;
USBHost usbHost;

// ---------- バイナリ構造体定義 ----------
#pragma pack(push, 1)
// 受信: PC -> Teensy (int16_t × 1個 = 2バイト)
struct CmdData {
  int16_t baseTargetRpm;
};

// 送信: Teensy -> PC (int16_t × 8個 = 16バイト)
struct SensorData {
  int16_t potAngles[4]; // ポテンショメータ角度 (0.01度単位, 4個)
  int16_t motorRpms[4]; // モータ回転数 (4個)
};
#pragma pack(pop)
// ----------------------------------------

int16_t targetMotorRpm = 0;
Metro statusTimer(100);    // 0.1秒(100ms)周期
BusLink busBridge(controllerNodeId);
DriveController driveControl(busBridge, controlIntervalMs);
IntervalTimer updateTimer;

int16_t calculateTargetRpm(double potAngle, int16_t baseRpm) {
  const double clampedAngle = constrain(potAngle, potMinAngle, potMaxAngle);
  const double normalized = (clampedAngle - potMinAngle) /
                           (potMaxAngle - potMinAngle);
  const double ratio = minSpeedRatio + normalized * (maxSpeedRatio - minSpeedRatio);
  const double minTargetRpm = (double)baseRpm * minSpeedRatio;
  const double maxTargetRpm = (double)baseRpm * maxSpeedRatio;
  return static_cast<int16_t>(round(constrain((double)baseRpm * ratio,
                                            minTargetRpm,
                                            maxTargetRpm)));
}

double readPotAngle(uint8_t potPin) {
  const double normalized = (double)analogRead(potPin) / potAnalogMax;
  return potMinAngle + normalized * (potMaxAngle - potMinAngle);
}

SensorData readSensorData() {
  SensorData data;
  for (uint8_t id = 1; id <= 4; ++id) {
    const double potentiometerAngle = readPotAngle(potentiometerPins[id - 1]);
    data.potAngles[id - 1] = static_cast<int16_t>(round(potentiometerAngle * 100.0));
    data.motorRpms[id - 1] = driveControl.getRpm(id);
  }
  return data;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(potentiometerPins[i], INPUT);
  }
  usbHost.begin();
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  busBridge.initBus(canBusBaud);

  for (uint8_t id = 1; id <= 4; ++id) {
    driveControl.configureMotorType(id, DRIVE_M3508);
    driveControl.setSpeedGain(id, speedLoopGain);
    driveControl.setTargetSpeed(id, targetMotorRpm);
    driveControl.enableTorque(id, true);
  }
  updateTimer.begin(DriveController::interruptHandler, controlIntervalMs * 1000);
  
  delay(1000);
  Serial.println("==========================================");
  Serial.println("  Teensy UDP Binary Node Started");
  Serial.println("==========================================");
}

void loop() {
  // 1. ミニPCからのUDPバイナリ命令を受信 (int16_t)
  int packetSize = Udp.parsePacket();
  if (packetSize == sizeof(CmdData)) {
    CmdData cmd;
    Udp.read((char*)&cmd, sizeof(CmdData));
    targetMotorRpm = cmd.baseTargetRpm;
    
    Serial.print("[RX (受信)] PCからの回転指示: ");
    Serial.println(targetMotorRpm);
  } else if (packetSize > 0) {
    Udp.flush();
  }

  for (uint8_t id = 1; id <= 4; ++id) {
    const double potentiometerAngle = readPotAngle(potentiometerPins[id - 1]);
    driveControl.setTargetSpeed(
        id, calculateTargetRpm(potentiometerAngle, targetMotorRpm));
  }

  // 2. 0.1秒(100ms)ごとにバイナリデータ送信 (int16_t × 8)
  if (statusTimer.check()) {
    const SensorData sendData = readSensorData();

    Udp.beginPacket(pc_ip, 8888);
    Udp.write((const uint8_t*)&sendData, sizeof(SensorData));
    Udp.endPacket();

    Serial.print("[TX (送信)] pot0:");
    Serial.print(sendData.potAngles[0]/100.0, 2);
    Serial.print(", pot1:");
    Serial.print(sendData.potAngles[1]/100.0, 2);
    Serial.print(", pot2:");
    Serial.print(sendData.potAngles[2]/100.0, 2);
    Serial.print(", pot3:");
    Serial.print(sendData.potAngles[3]/100.0, 2);
    Serial.print(", rpm0:");
    Serial.print(sendData.motorRpms[0]);
    Serial.print(", rpm1:");
    Serial.print(sendData.motorRpms[1]);
    Serial.print(", rpm2:");
    Serial.print(sendData.motorRpms[2]);
    Serial.print(", rpm3:");
    Serial.println(sendData.motorRpms[3]);
  }
}