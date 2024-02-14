#include <Arduino.h>
#include <DynamixelShield.h>
#include "Robot.hpp"
#include "QDServer.hpp"
#include "QDMessage.hpp"
#include "LinearAlgebra.hpp"

//#define ENABLE_SYNC

Robot qd(42.0, 55.0, 55.0, Vector4(29.0, 0.0, 75.0, 1.0), Vector4(-29.0, 0.0, 75.0, 1.0), Vector4(29.0, 0.0, -75.0, 1.0), Vector4(-29.0, 0.0, -75.0, 1.0));

const float DXL_PROTOCOL_VERSION = 2.0;

const char *remoteIP = "127.0.0.1";
const uint16_t remotePort = 3567;

QDServer server;

const int udpSendBufLen = 36;
char udpSendBuf[udpSendBufLen];

const int udpRecvBufLen = 36;   // Size of QDCtrlMsg
char udpRecvBuf[udpRecvBufLen];

#if 0

void setup() {
  Serial.begin(115200);
  delay(2000);
  // while(!Serial);
  /*
  float c = cos(PIOver4);
  float s = sin(PIOver4);
  float init[] = { 
    c,   -s,    0.0,  0.0, 
    s,    c,    0.0,  0.0,
    0.0,  0.0,  1.0,  0.0,
    0.0,  0.0,  0.0,  1.0 };
  Matrix4 m4(init);

  m4.print();
  Serial.println();
  m4.cofactorMatrix().print();
  Serial.println();
  */

  float init[] = { 
    1, 1, 1, -1,
    1, 1, -1, 1, 
    1, -1, 1, 1, 
    -1, 1, 1, 1 };
  Matrix4 m4(init);

  Serial.println("Matrix");
  m4.print();
  Serial.println();
  Serial.println("Determinant");
  Serial.println(m4.determinant());
  Serial.println();
  Serial.println("Cofactor");
  m4.cofactorMatrix().print();
  Serial.println();
  Serial.println("Transposed cofactor");
  m4.cofactorMatrix().transpose().print();
  Serial.println();
  Serial.println("Inverse");
  m4.inverse().print();
  Serial.println();

  /*
  Matrix4 mat(Matrix4::rotation, Vector4::j, PIOver2);
  Matrix4 inv(mat.inverse());
  Vector4 k(mat * Vector4::i);
  Vector4 i(inv * k);
  
  Serial.print(k[0]);
  Serial.print(", ");
  Serial.print(k[1]);
  Serial.print(", ");
  Serial.println(k[2]);

  Serial.print(i[0]);
  Serial.print(", ");
  Serial.print(i[1]);
  Serial.print(", ");
  Serial.println(i[2]);
  */
}

void loop() {
}

#else

void setup() {
  Serial.begin(115200);
  delay(2000);
  // while(!Serial);

  Serial.println("qd online");

  qd.begin();
  
  Serial.println("joining wifi");

  if(!server.joinNetwork(5)) {
    while(true);
  }

  server.setRecvInfo(udpRecvBuf, udpRecvBufLen);
  server.startUDP(remoteIP, remotePort);
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
        QDCtrlMsg &ctrl = *((QDCtrlMsg *) udpRecvBuf);
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

  if(currentMillis - lastMillis > 33) {
    qd.tick(((float) currentMillis) * 0.001);
    lastMillis = currentMillis;
  }
}

#endif
