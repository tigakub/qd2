#include "QDMessage.hpp"
#include <Arduino.h>

uint32_t swap32(uint32_t iValue) {
  uint8_t *ptr = (uint8_t *) &iValue;
  ptr[0] ^= ptr[3];
  ptr[3] ^= ptr[0];
  ptr[0] ^= ptr[3];
  ptr[1] ^= ptr[2];
  ptr[2] ^= ptr[1];
  ptr[1] ^= ptr[2];
  return iValue;
}

uint32_t htonf(float iValue) {
  return htonl(*((uint32_t *) &iValue));
}

float ntohf(uint32_t iValue) {
  uint32_t hostOrder = ntohl(iValue);
  return *((float *) &hostOrder);
}

QDPose::QDPose(const QDPoseSwapped &iPose)
: type(ntohl(pose)), timestamp(ntohl(iPose.timestamp)) {
  for(int i = 0; i < 4; i++) {
    hips[i] = ntohf(iPose.hips[i]);
    shoulders[i] = ntohf(iPose.shoulders[i]);
    elbows[i] = ntohf(iPose.elbows[i]);
  }
}

QDPose2::QDPose2(const QDPose2Swapped &iPose)
: type(ntohl(iPose.type)), id(ntohl(iPose.id)), duration(ntohl(iPose.duration)), stepHeight(ntohf(iPose.stepHeight)), bodySway(ntohf(iPose.bodySway)) {
  for(int i = 0; i < 3; i++) {
    effFront[i] = ntohf(iPose.effFront[i]);
    effBack[i] = ntohf(iPose.effBack[i]);
  }
}

QDCtrlMsg::QDCtrlMsg(const QDCtrlMsgSwapped &iControl)
: type(ntohl(iControl.type)), rotation(ntohf(iControl.rotation)), stepHeight(ntohf(iControl.stepHeight)), bodySway(ntohf(iControl.bodySway)), bodyHeight(ntohf(iControl.bodyHeight)), duration(ntohf(iControl.duration)) {
  translation[0] = ntohf(iControl.translation[0]);
  translation[1] = ntohf(iControl.translation[1]);
}

QDFeedback::QDFeedback(const QDFeedbackSwapped &iFeedback)
: type(ntohl(feedback)), id(ntohl(iFeedback.id)) {
  for(int i = 0; i < 4; i++) {
    hips[i] = ntohf(iFeedback.hips[i]);
    shoulders[i] = ntohf(iFeedback.shoulders[i]);
    elbows[i] = ntohf(iFeedback.elbows[i]);
    orientation[i] = ntohf(iFeedback.orientation[i]);
  }
}

QDPoseSwapped::QDPoseSwapped(const QDPose &iPose)
: type(htonl(pose)), timestamp(htonl(iPose.timestamp)) {
  for(int i = 0; i < 4; i++) {
    hips[i] = htonf(iPose.hips[i]);
    shoulders[i] = htonf(iPose.shoulders[i]);
    elbows[i] = htonf(iPose.elbows[i]);
  }
}

QDPose2Swapped::QDPose2Swapped(const QDPose2 &iPose)
: type(htonl(iPose.type)), id(htonl(iPose.id)), duration(htonl(iPose.duration)), stepHeight(htonf(iPose.stepHeight)), bodySway(htonf(iPose.bodySway)) {
  for(int i = 0; i < 3; i++) {
    effFront[i] = htonf(iPose.effFront[i]);
    effBack[i] = htonf(iPose.effBack[i]);
  }
}

QDCtrlMsgSwapped::QDCtrlMsgSwapped(const QDCtrlMsg &iControl)
: type(htonl(iControl.type)), rotation(htonf(iControl.rotation)), stepHeight(htonf(iControl.stepHeight)), bodySway(htonf(iControl.bodySway)), bodyHeight(htonf(iControl.bodyHeight)), duration(htonf(iControl.duration)) {
  translation[0] = htonf(iControl.translation[0]);
  translation[1] = htonf(iControl.translation[1]);
}

QDFeedbackSwapped::QDFeedbackSwapped(const QDFeedback &iFeedback)
: type(htonl(feedback)), id(htonl(iFeedback.id)) {
  for(int i = 0; i < 4; i++) {
    hips[i] = htonf(iFeedback.hips[i]);
    shoulders[i] = htonf(iFeedback.shoulders[i]);
    elbows[i] = htonf(iFeedback.elbows[i]);
    orientation[i] = htonf(iFeedback.orientation[i]);
  }
}

void QDFeedbackSwapped::pack(const QDFeedback &iFeedback) {
  type = htonl(iFeedback.type);
  id = htonl(iFeedback.id);
  for(int i = 0; i < 4; i++) {
    hips[i] = htonf(iFeedback.hips[i]);
    shoulders[i] = htonf(iFeedback.shoulders[i]);
    elbows[i] = htonf(iFeedback.elbows[i]);
    orientation[i] = htonf(iFeedback.orientation[i]);
  }
}