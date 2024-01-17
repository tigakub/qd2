#ifndef __QDMESSAGE_HPP__
#define __QDMESSAGE_HPP__

#include <type_traits>
#include <typeindex>
#include <stdint.h>
#include <unordered_map>
#include <memory>

#define ntohl swap32
#define htonl swap32

uint32_t swap32(uint32_t iValue);
uint32_t htonf(float iValue);
float ntohf(uint32_t iValue);

enum PoseType {
  pose = 1,
  pose2RL = pose + 1,
  pose2LR = pose2RL + 1,
  control = pose2LR + 1,
  feedback = control + 1
};

struct QDPoseSwapped;

typedef struct QDPose {
  uint32_t type;
  float hips[4];
  float shoulders[4];
  float elbows[4];
  uint32_t timestamp;

  QDPose() {}
  QDPose(const QDPoseSwapped &iPose);
} QDPose;

struct QDPose2Swapped;

typedef struct QDPose2 {
  uint32_t type;
  uint32_t id;
  float effFront[3];
  float effBack[3];
  float stepHeight;
  float bodySway;
  uint32_t duration;

  QDPose2() { }
  QDPose2(const QDPose2Swapped &iPose);
} QDPose2;

struct QDCtrlMsgSwapped;

typedef struct QDCtrlMsg {
  uint32_t type;
  float translation[2];
  float rotation;
  float stepScale;
  float stepHeight;
  float bodySway;
  float bodyHeight;
  float duration;

  QDCtrlMsg() { }
  QDCtrlMsg(const QDCtrlMsgSwapped &iControl);
} QDCtrlMsg;

struct QDFeedbackSwapped;

typedef struct QDFeedback {
  uint32_t type;
  uint32_t id;
  float hips[4];
  float shoulders[4];
  float elbows[4];
  float orientation[4];

  QDFeedback() { }
  QDFeedback(const QDFeedbackSwapped &iFeedback);
} QDFeedback;

typedef struct QDPoseSwapped {
  uint32_t type;
  uint32_t hips[4];
  uint32_t shoulders[4];
  uint32_t elbows[4];
  uint32_t timestamp;

  QDPoseSwapped() { }
  QDPoseSwapped(const QDPose &iPose);
} QDPoseSwapped;

typedef struct QDPose2Swapped {
  uint32_t type;
  uint32_t id;
  uint32_t effFront[3];
  uint32_t effBack[3];
  uint32_t stepHeight;
  uint32_t bodySway;
  uint32_t duration;

  QDPose2Swapped() { }
  QDPose2Swapped(const QDPose2 &iPose);
} QDPose2Swapped;

typedef struct QDFeedbackSwapped {
  uint32_t type;
  uint32_t id;
  uint32_t hips[4];
  uint32_t shoulders[4];
  uint32_t elbows[4];
  uint32_t orientation[4];

  QDFeedbackSwapped() { }
  QDFeedbackSwapped(const QDFeedback &iFeedback);

  void pack(const QDFeedback &iFeedback);
} QDFeedbackSwapped;

typedef struct QDCtrlMsgSwapped {
  uint32_t type;
  uint32_t translation[2];
  uint32_t rotation;
  uint32_t stepScale;
  uint32_t stepHeight;
  uint32_t bodySway;
  uint32_t bodyHeight;
  uint32_t duration;

  QDCtrlMsgSwapped() { }
  QDCtrlMsgSwapped(const QDCtrlMsg &iControl);
} QDCtrlMsgSwapped;

#endif
