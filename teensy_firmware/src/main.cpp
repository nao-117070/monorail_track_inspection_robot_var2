#include <Arduino.h>
#include "DriveController.h"
#include <IntervalTimer.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <Metro.h>
#include <USBHost_t36.h>

// モーター速度制御器のゲインと制御周期 (1 ms)
const GainParams speedLoopGain = {3, 0.3, 0.5};
const double controlIntervalMs = 1.0;
const int controllerNodeId = 1;
const int canBusBaud = 1000000;

const uint8_t potentiometerPins[4] = {14, 15, 16, 17};
// アナログ入力値 0～4095 をこの角度範囲に換算する
const double potMinAngle = 109.82;
const double potMaxAngle = 110.60;
// ポテンショメータ角度の最大偏差時に、基準RPMを最大±30%補正する
const float targetRpmGain = 0.3f;
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

// 送信: Teensy -> PC (int16_t × 8個 = 16バイト)。詰め物なしで送受信する
struct SensorData {
  int16_t potAngles[4]; // ポテンショメータ角度 (0.01度単位、4個)
  int16_t motorRpms[4]; // モータ回転数 (4個)
};
#pragma pack(pop)
// ----------------------------------------

int16_t targetMotorRpm = 0;
Metro statusTimer(100);    // 0.1秒(100ms)周期
BusLink busBridge(controllerNodeId);
DriveController driveControl(busBridge, controlIntervalMs);
IntervalTimer updateTimer;

// 基準RPMに、ポテンショメータの中央からの偏差に応じた補正を加える。
// 角度を範囲内に制限し、中央を0・両端を-1/+1とする正規化誤差を求める。
// 最後に基準RPM × (1 + ゲイン × 誤差) として補正し、整数RPMに丸める。
int16_t calculateTargetRpm(double potAngle, int16_t baseRpm) {
  // 範囲外のセンサー値でも補正率が想定範囲を超えないようにする
  const double clampedAngle = constrain(potAngle, potMinAngle, potMaxAngle);
  // 入力角度範囲の中心と、中心から端までの角度幅
  const double potCenterAngle = (potMinAngle + potMaxAngle) / 2.0;
  const double normalizedError =
      (clampedAngle - potCenterAngle) /
      ((potMaxAngle - potMinAngle) / 2.0);
  // 誤差が-1/+1のとき基準RPMをそれぞれ-30%/+30%補正する
  const double targetRpm =
      static_cast<double>(baseRpm) * (1.0 + targetRpmGain * normalizedError);

  return static_cast<int16_t>(round(targetRpm));
}

double readPotAngle(uint8_t potPin) {
  // ADC値を0～1に正規化し、ポテンショメータの角度範囲へ線形変換する
  const double normalized = (double)analogRead(potPin) / potAnalogMax;
  return potMinAngle + normalized * (potMaxAngle - potMinAngle);
}

SensorData readSensorData() {
  SensorData data;
  for (uint8_t id = 1; id <= 4; ++id) {
    const double potentiometerAngle = readPotAngle(potentiometerPins[id - 1]);
    // 角度を0.01度単位の整数にして、RPMとともに送信用データへ格納する
    data.potAngles[id - 1] = static_cast<int16_t>(round(potentiometerAngle * 100.0));
    data.motorRpms[id - 1] = driveControl.getRpm(id);
  }
  return data;
}

void setup() {
  // シリアル、ADC、入力ピン、USB、Ethernet、UDP、CANバスを初期化する
  Serial.begin(115200);
  analogReadResolution(12);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(potentiometerPins[i], INPUT);
  }
  usbHost.begin();
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  busBridge.initBus(canBusBaud);

  // 4台のモーターを設定し、初期目標RPMを与えてトルクを有効化する
  for (uint8_t id = 1; id <= 4; ++id) {
    driveControl.configureMotorType(id, DRIVE_M3508);
    driveControl.setSpeedGain(id, speedLoopGain);
    driveControl.setTargetSpeed(id, targetMotorRpm);
    driveControl.enableTorque(id, true);
  }
  // 制御器の周期処理を1 ms間隔で実行する
  updateTimer.begin(DriveController::interruptHandler, controlIntervalMs * 1000);
  
  delay(1000);
  Serial.println("==========================================");
  Serial.println("  Teensy UDP Binary Node Started");
  Serial.println("==========================================");
}

void loop() {
  // PCから2バイトの基準RPMを受信する。サイズが違うパケットは処理しない
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
  // 各ポテンショメータの角度に応じて、対応するモーターの目標RPMを更新する
  for (uint8_t id = 1; id <= 4; ++id) {
    const double potentiometerAngle = readPotAngle(potentiometerPins[id - 1]);
    driveControl.setTargetSpeed(
        id, calculateTargetRpm(potentiometerAngle, targetMotorRpm));
  }

  // 0.1秒ごとに4台分の角度とRPMをPCへバイナリ送信し、同じ値をシリアル表示する
  if (statusTimer.check()) {
    const SensorData sendData = readSensorData();

    Udp.beginPacket(pc_ip, 8888);
    Udp.write((const uint8_t*)&sendData, sizeof(SensorData));
    Udp.endPacket();

    Serial.print(millis());
    Serial.print(",");
    Serial.print(calculateTargetRpm(
      readPotAngle(potentiometerPins[3]), targetMotorRpm));
    Serial.print(",");
    Serial.println(sendData.motorRpms[3]);
  }
}