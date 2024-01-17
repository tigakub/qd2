#include <Arduino.h>
#include <DynamixelShield.h>
#include "Robot.hpp"
#include "QDServer.hpp"
#include "QDMessage.hpp"

//#define ENABLE_SYNC

Robot qd(42.0, 55.0, 55.0, Vector4(29.0, 0.0, 75.0), Vector4(-29.0, 0.0, 75.0), Vector4(29.0, 0.0, -75.0), Vector4(-29.0, 0.0, -75.0));

const float DXL_PROTOCOL_VERSION = 2.0;

const char *remoteIP = "127.0.0.1";
const uint16_t remotePort = 3567;

QDServer server;

const int udpSendBufLen = 36;
char udpSendBuf[udpSendBufLen];

const int udpRecvBufLen = 36;   // Size of QDCtrlMsg
char udpRecvBuf[udpRecvBufLen];

void setup() {
  Serial.begin(115200);
  delay(2000);
  // while(!Serial);

  Serial.println("qd online");
  Serial.println("joining wifi");

  if(!server.joinNetwork(5)) {
    while(true);
  }

  server.setRecvInfo(udpRecvBuf, udpRecvBufLen);
  server.startUDP(remoteIP, remotePort);

  qd.begin();
}

unsigned long lastMillis = 0;

void loop() {

  if(server.sendLoop() >= 0) {
    // Message complete, ready for next
    server.setSendInfo(udpSendBuf, udpSendBufLen);
  }

  int result = server.recvLoop();
  if(result > 0) {
    uint32_t type = ntohl(((uint32_t *) udpRecvBuf)[0]);
    switch(type) {
      case control:
        /*
        Serial.print("t: (");
        Serial.print(ctrl.translation[0]);
        Serial.print(", ");
        Serial.print(ctrl.translation[1]);
        Serial.print(") r: ");
        Serial.print(ctrl.rotation);
        Serial.print(" s: ");
        Serial.print(ctrl.stepScale);
        Serial.print(" h: ");
        Serial.print(ctrl.stepHeight);
        Serial.print(" b: ");
        Serial.print(ctrl.bodySway);
        Serial.print(" e: ");
        Serial.print(ctrl.bodyHeight);
        Serial.print(" d: ");
        Serial.println(ctrl.duration);
        */
        
        qd.setControl(*((QDCtrlMsg *) udpRecvBuf));
        break;
    }
    server.setRecvInfo(udpRecvBuf, udpRecvBufLen);
  }
  unsigned long currentMillis = millis();

  if(currentMillis - lastMillis > 250) {
    qd.tick(((float) currentMillis) * 0.001);
    lastMillis = currentMillis;
  }
}
