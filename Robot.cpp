//
//  Robot.cpp
//  qd
//
//  Created by Edward Janne on 10/18/23.
//

#include "Robot.hpp"
#include "QDMessage.hpp"
#include <math.h>
#include <iostream>
#include <Arduino.h>

float unitValue = 0.001533980788; // Radians
#define RAD_TO_STEP 651.8986469044
// float unitVlue = 0.087890625; // Degrees

using namespace ControlTableItem;

#define PI 3.141592653589793
float sigMin = 1.0 / (1.0 + exp(-12.0 * -0.5));
float sigRange = 1.0 / (1.0 + exp(-12.0 * 0.5)) - sigMin;

float sigmoid(float x) {
  return ((1.0 / (1.0 + (exp((x - 0.5) * -12.0)))) - sigMin) / sigRange;
}

float inverseSigmoid(float y) {
    // log() is the natural log
    if(y <= 0.0) return 0.0;
    if(y >= 1.0) return 1.0;
    return log(1.0 / (y * sigRange + sigMin) - 1.0) / -12.0 + 0.5;
}

float bell(float x) {
	float dx = (x - 0.5);
	return exp(-dx * dx / 0.045) / sqrt(0.045 * PI) / 2.6594;
}

Robot::Limb::Limb(Robot::Limb::Configuration iConfig, const Vector4 &iRootOffset, float iHipToShoulder, float iShoulderToElbow, float iElbowToToe)
: configuration(iConfig),
  rootOffset(iRootOffset),
  l0(iHipToShoulder), l0sq(l0 * l0),
  l1(iShoulderToElbow), l1sq(l1 * l1),
  l2(iElbowToToe), l2sq(l2 * l2)
{ }
        
Vector4 Robot::Limb::normalizeTarget(const Vector4 &iTarget) const {
  Vector4 normalized = iTarget - rootOffset;
  switch(configuration) {
    case frontLeft:
    case backLeft: {
        Matrix4 rotation = Matrix4(Vector4::j, PI);
        normalized = rotation * normalized;
      }
      break;
    default:
      break;
  }
  return normalized;
}

Vector4 Robot::Limb::denormalizeTarget(const Vector4 &iNormalized) const {
  Vector4 target;
  switch(configuration) {
    case frontLeft:
    case backLeft: {
        Matrix4 rotation = Matrix4(Vector4::j, -PI);
        target = rotation * iNormalized;
      }
      break;
    default:
      break;
  }
  return target + rootOffset;
}

void Robot::Limb::calcIKAngles(const Vector4 &iTarget) {
  float dsq = iTarget[0] * iTarget[0] + iTarget[1] * iTarget[1];
  float d = sqrtf(dsq);
  float e = acosf(l0 / d);
  float n = 1.0 / d;
  Vector4 unitp(iTarget[0] * n, iTarget[1] * n, 0.0);
  Matrix4 mat(Vector4::k, e);
  Vector4 v(mat * unitp * l0);
  hipAngle = -atan2f(v[1], v[0]);
  Vector4 p(iTarget - v);
  dsq = p * p;
  d = sqrtf(dsq);
  float at = asinf(iTarget[2] / d);
  float ae = (l1 + l2 > d) ? acosf(0.5 * (l1sq + l2sq - dsq) / (l1 * l2)) : PI;
  float a1 = (l1 + l2 > d) ? acosf(0.5 * (l1sq + dsq - l2sq) / (l1 * d)) : 0.0;
  
  switch(configuration) {
    case frontLeft:
    case backLeft:
      shoulderAngle = at + a1 - PIOver2;
      elbowAngle = PI - ae;
      break;
    default:
      hipAngle *= -1.0;
      shoulderAngle = PIOver2 - (at + a1);
      elbowAngle = ae - PI;
  }
  
  hipAngle += PI;
  shoulderAngle += PI;
  elbowAngle += PI;
}

float Robot::Limb::operator[](int i) const {
  switch(i) {
    case 0:
      return hipAngle;
    case 1:
      return shoulderAngle;
    default:
      return elbowAngle;
  }
}

float &Robot::Limb::operator[](int i) {
  switch(i) {
    case 0:
      return hipAngle;
    case 1:
      return shoulderAngle;
    default:
      return elbowAngle;
  }
}

Robot::Robot(float iHipToShoulder, float iShoulderToElbow, float iElbowToToe, const Vector4 &iFrontRightRootPosition, const Vector4 &iFrontLeftRootPosition, const Vector4 &iBackRightRootPosition, const Vector4 &iBackLeftRootPosition)
: currentPoseTime(0.0),
  currentProgress(1.0),
  rotationMatrix(1),
  counterRotationMatrix(1),
  limbs {
    Limb(Limb::frontRight, iFrontRightRootPosition, iHipToShoulder, iShoulderToElbow, iElbowToToe),
    Limb(Limb::frontLeft, iFrontLeftRootPosition, iHipToShoulder, iShoulderToElbow, iElbowToToe),
    Limb(Limb::backRight, iBackRightRootPosition, iHipToShoulder, iShoulderToElbow, iElbowToToe),
    Limb(Limb::backLeft, iBackLeftRootPosition, iHipToShoulder, iShoulderToElbow, iElbowToToe)
  },
  dxl()
{ }

void Robot::begin() {
  int i = 4;
  while(i--) {
    idlePose[0] = limbs[0].rootOffset - Vector4(0.0, currentBodyHeight, 0.0, 0.0);
  }

  dxl.begin(57600);

  int j = 4;
  while(j--) {
    int i = 3;
    while(i--) {
      int id = j * 10 + i;
      dxl.writeControlTableItem(RETURN_DELAY_TIME, id, 0);
      dxl.setOperatingMode(id, OP_POSITION);
      float pos = dxl.getPresentPosition(id, UNIT_DEGREE);
      dxl.setGoalPosition(id, pos, UNIT_DEGREE);
      dxl.torqueOn(id);
    }
  }
}

void Robot::setControl(const QDCtrlMsg &iCtrl) {
  currentTranslation[0] = iCtrl.translation[0];
  currentTranslation[1] = iCtrl.translation[1];
  currentRotation = iCtrl.rotation;
  currentStepScale = iCtrl.stepScale;
  currentStepHeight = iCtrl.stepHeight;
  currentBodySway = iCtrl.bodySway;
  currentDuration = iCtrl.duration;
}

void Robot::setIKTargets(const Vector4 &iFrontRightTarget, const Vector4 &iFrontLeftTarget, const Vector4 &iBackRightTarget, const Vector4 &iBackLeftTarget) {
  ikTargets[0] = limbs[0].normalizeTarget(iFrontRightTarget);
  ikTargets[1] = limbs[1].normalizeTarget(iFrontLeftTarget);
  ikTargets[2] = limbs[2].normalizeTarget(iBackRightTarget);
  ikTargets[3] = limbs[3].normalizeTarget(iBackLeftTarget);
}

void Robot::setAngles(const Vector4 &iHipAngles, const Vector4 &iShoulderAngles, const Vector4 &iElbowAngles) {
  int i = 4;
  while(i--) {
    limbs[i][0] = iHipAngles[i];
    limbs[i][1] = iShoulderAngles[i];
    limbs[i][2] = iElbowAngles[i];
  }
}

void Robot::update() {
  int i = 4;
  while(i--) {
    limbs[i].calcIKAngles(ikTargets[i]);
  }
}

void Robot::emit() {
  int j = 4;
  while(j--) {
    int i = 3;
    while(i--) {
      int id = j * 10 + i;
      float tgt = limbs[j][i] * RAD_TO_DEG;
      dxl.setGoalPosition(id, tgt, UNIT_DEGREE);
    }
  }
}

void Robot::pullNextPose() {
  int i = 4;
  while(i--) {
    currentPose[i] = nextPose[i];
  }

  currentPoseTime = ((float) millis()) * 0.001;
  
  i = 4;
  while(i--) {
    nextPose[i] = idlePose[i];
  }

  i = 2;
  while(i--) {
    latchedTranslation[i] = currentTranslation[i];
  }
  latchedRotation = currentRotation;

  rotationMatrix = Matrix4(Vector4::j, latchedRotation * 0.5);
  counterRotationMatrix = Matrix4(Vector4::j, -latchedRotation * 0.5);

  latchedStepScale = currentStepScale;
  latchedStepHeight = currentStepHeight;
  latchedBodySway = currentBodySway;
  latchedBodyHeight = currentBodyHeight;
  latchedDuration = currentDuration;

  switch(currentMode) {
    case idle:
      break;
    case starting:
      // Start body sway
      if(latchedRotation > 0.0) {
        currentPole = frontLeftBackRight;
      } else {
        currentPole = frontRightBackLeft;
      }
    case walking: {
        Vector4 step(latchedTranslation[0] * latchedStepScale, 0.0, latchedTranslation[1] * latchedStepScale, 0.0);
        switch(currentPole) {
          case frontRightBackLeft:
            nextPose[0] += step;
            nextPose[1] -= step;
            nextPose[2] -= step;
            nextPose[3] += step;
            if(latchedRotation < 0.0) {
              nextPose[0] = rotationMatrix * nextPose[0];
              nextPose[1] = counterRotationMatrix * nextPose[1];
              nextPose[2] = counterRotationMatrix * nextPose[2];
              nextPose[3] = rotationMatrix * nextPose[3];
            }
            for(int i = 0; i < 4; i++) {
              nextPose[i].v[0] += latchedBodySway;
            }
            break;
          case frontLeftBackRight:
            nextPose[0] -= step;
            nextPose[1] += step;
            nextPose[2] += step;
            nextPose[3] -= step;
            if(latchedRotation > 0.0) {
              nextPose[0] = counterRotationMatrix * nextPose[0];
              nextPose[1] = rotationMatrix * nextPose[1];
              nextPose[2] = rotationMatrix * nextPose[2];
              nextPose[3] = counterRotationMatrix * nextPose[3];
            }
            for(int i = 0; i < 4; i++) {
              nextPose[i].v[0] -= latchedBodySway;
            }
            break;
        }
      }
      break;
    case stopping:
      break;
  }

  if(currentPole == frontRightBackLeft) {
    currentPole = frontLeftBackRight;
  } else {
    currentPole = frontRightBackLeft;
  }
}

void Robot::tick(float seconds) {
  switch(currentMode) {
    case idle:
      idleTick();
      break;
    case starting:
      startTick();
      break;
    case walking:
      walkTick();
      break;
    case stopping:
      stopTick();
      break;
  }

  if(currentMode != idle) {
    float duration = latchedDuration;
    float stepHeight = latchedStepHeight;

    float progress = (seconds - currentPoseTime) / latchedDuration;
    if(progress < 0.0) progress = 0.0;
    if(progress > 1.0) progress = 1.0;

    float stepPeriod = 0.25, plantPeriod = stepPeriod * 0.5;
    float stepProgress = progress / stepPeriod;

    float frontProgress = plantPeriod + stepProgress;
    float frontBell = 0.0;
    if(frontProgress < 0.0) frontProgress = 0.0;
    else if(frontProgress > 1.0) frontProgress = 1.0;
    else {
      frontProgress = sigmoid(frontProgress);
      frontBell = bell(frontProgress);
    }

    float backProgress = 0.5 + frontProgress;
    float backBell = 0.0;
    if(backProgress < 0.0) backProgress = 0.0;
    else if(backProgress > 1.0) backProgress = 1.0;
    else {
      backProgress = sigmoid(backProgress);
      backBell = bell(backProgress);
    }

    ikTargets[0] = currentPose[0] * frontProgress + nextPose[0] * (1.0 - frontProgress);
    ikTargets[1] = currentPose[1] * frontProgress + nextPose[1] * (1.0 - frontProgress);
    ikTargets[2] = currentPose[2] * backProgress + nextPose[2] * (1.0 - backProgress);
    ikTargets[3] = currentPose[3] * backProgress + nextPose[3] * (1.0 - backProgress);

    switch(currentPole) {
      case frontRightBackLeft:
        ikTargets[0].v[1] = frontBell * currentStepHeight;
        ikTargets[3].v[1] = backBell * currentStepHeight;
        break;
      case frontLeftBackRight:
        ikTargets[1].v[1] = frontBell * currentStepHeight;
        ikTargets[2].v[1] = backBell * currentStepHeight;
        break;
    }

    /*
    int fNdx = 0, bNdx = 2;
    switch(nextPose.type) {
      case pose2LR:
        fNdx = 1; bNdx = 3;
    }

    ikTargets[fNdx] = currentPose[fNdx] * frontProgress + Vector4(nextPose.effFront, 1.0) * (1.0 - frontProgress);
    ikTargets[bNdx] = currentPose[bNdx] * backProgress + Vector4(nextPose.effBack, 1.0) * (1.0 - backProgress);

    ikTargets[fNdx].v[1] = frontBell * currentStepHeight;
    ikTargets[bNdx].v[1] = backBell * currentStepHeight;
    */

    Serial.print(" ");
    Serial.print(progress);

    if(progress == 1.0) {
      Serial.println("\n");
    }

    currentProgress = progress;
  }
  
  update();
  emit();
}

void Robot::report() {
  if((currentTranslation[0] != 0.0 || currentTranslation[1] != 0.0) && currentRotation != 0.0) {
    Serial.print("Translating and rotating");
  } else if(currentTranslation[0] != 0.0 || currentTranslation[1] != 0.0) {
    Serial.print("Translating");
  } else if(currentRotation != 0.0) {
    Serial.print("Rotating");
  }
  if(currentRotation > 0.0) {
    Serial.print(" counterClockwise");
  } else if(currentRotation < 0.0) {
    Serial.print(" clockwise");
  }
}

void Robot::idleTick() {
  currentMode == idle;
  if(currentTranslation[0] != 0.0 || currentTranslation[1] != 0.0 || currentRotation != 0.0) {
    Serial.print("Starting: ");
    report();
    Serial.println("");
    currentMode = starting;
    currentProgress = 0.0;
    pullNextPose();
  }
}

void Robot::startTick() {
  currentMode == starting;

  if(currentProgress >= 1.0) {
    Serial.print("Walking: ");
    report();
    Serial.println("");
    currentMode = walking;
    currentProgress = 0.0;
    pullNextPose();
  }
}

void Robot::walkTick() {
  currentMode == walking;
  if(currentProgress >= 1.0) {
    if(currentTranslation[0] == 0.0 && currentTranslation[1] == 0.0 && currentRotation == 0.0) {
      Serial.println("Stopping");
      currentMode = stopping;
    } else {
      Serial.print("Walking: ");
      report();
      Serial.println("");
    }
    currentProgress = 0.0;
    pullNextPose();
  }
}

void Robot::stopTick() {
  currentMode == stopping;

  if(currentProgress >= 1.0) {
    Serial.println("Idle");
    currentMode = idle;
    currentProgress = 0.0;
    pullNextPose();
  }
}

char *frontRight = "Front right: ";
char *frontLeft = "Front left: ";
char *backRight = "Back right: ";
char *backLeft = "Back left: ";

char *strings[] = { frontRight, frontLeft, backRight, backLeft };

void Robot::printIKAngles() const {
  for(int i = 0; i < 4; i++) {
    Serial.print(strings[i]);
    float hipAngle = RAD_TO_DEG * limbs[i].hipAngle;
    float shoulderAngle = RAD_TO_DEG * limbs[i].shoulderAngle;
    float elbowAngle = RAD_TO_DEG * limbs[i].elbowAngle;
    Serial.print(hipAngle);
    Serial.print(", ");
    Serial.print(shoulderAngle);
    Serial.print(", ");
    Serial.println(elbowAngle);
  }

  /*
  std::cout << "Front right: " << RAD_TO_DEG(limbs[0].hipAngle) << ", " << RAD_TO_DEG(limbs[0].shoulderAngle) << ", " << RAD_TO_DEG(limbs[0].elbowAngle) << std::endl;
  std::cout << "Front left:  " << RAD_TO_DEG(limbs[1].hipAngle) << ", " << RAD_TO_DEG(limbs[1].shoulderAngle) << ", " << RAD_TO_DEG(limbs[1].elbowAngle) << std::endl;
  std::cout << "Back right:  " << RAD_TO_DEG(limbs[2].hipAngle) << ", " << RAD_TO_DEG(limbs[2].shoulderAngle) << ", " << RAD_TO_DEG(limbs[2].elbowAngle) << std::endl;
  std::cout << "Back left:   " << RAD_TO_DEG(limbs[3].hipAngle) << ", " << RAD_TO_DEG(limbs[3].shoulderAngle) << ", " << RAD_TO_DEG(limbs[3].elbowAngle) << std::endl;
  */
}
