#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <Metro.h>
#include <USBHost_t36.h>

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
  int16_t potAngles[4]; // ポテンショメータ角度 (4個)
  int16_t motorRpms[4]; // モータ回転数 (4個)
};
#pragma pack(pop)
// ----------------------------------------

int16_t baseTargetRpm = 0; 
Metro statusTimer(100);    // 0.1秒(100ms)周期

void setup() {
  Serial.begin(115200);
  usbHost.begin();
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  
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
    baseTargetRpm = cmd.baseTargetRpm;
    
    Serial.print("[RX (受信)] PCからの回転指示: ");
    Serial.println(baseTargetRpm);
  } else if (packetSize > 0) {
    Udp.flush();
  }

  // 2. 0.1秒(100ms)ごとにバイナリデータ送信 (int16_t × 8)
  if (statusTimer.check()) {
    SensorData sendData;
    
    // データ格納 (ポテンショメータ4個, RPM4個)
    sendData.potAngles[0] = 10;
    sendData.potAngles[1] = 20;
    sendData.potAngles[2] = 30;
    sendData.potAngles[3] = 40;

    sendData.motorRpms[0] = baseTargetRpm;
    sendData.motorRpms[1] = baseTargetRpm;
    sendData.motorRpms[2] = baseTargetRpm;
    sendData.motorRpms[3] = baseTargetRpm;

    Udp.beginPacket(pc_ip, 8888);
    Udp.write((const uint8_t*)&sendData, sizeof(SensorData));
    Udp.endPacket();

    Serial.print("[TX (送信)] pot0:");
    Serial.print(sendData.potAngles[0]);
    Serial.print(", pot1:");
    Serial.print(sendData.potAngles[1]);
    Serial.print(", pot2:");
    Serial.print(sendData.potAngles[2]);
    Serial.print(", pot3:");
    Serial.print(sendData.potAngles[3]);
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