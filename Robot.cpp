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

char *frontRight = "Front right: ";
char *frontLeft = "Front left: ";
char *backRight = "Back right: ";
char *backLeft = "Back left: ";

char *strings[] = { frontRight, frontLeft, backRight, backLeft };

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
  l2(iElbowToToe), l2sq(l2 * l2),
  hipAngle(0.0), shoulderAngle(0.0), elbowAngle(0.0)
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
  Vector4 nTarget = normalizeTarget(iTarget);
  float dsq = nTarget[0] * nTarget[0] + nTarget[1] * nTarget[1];
  float d = sqrtf(dsq);
  float e = acosf(l0 / d);
  float n = 1.0 / d;
  Vector4 unitp(nTarget[0] * n, nTarget[1] * n, 0.0);
  Matrix4 mat(Vector4::k, e);
  Vector4 v(mat * unitp * l0);
  hipAngle = -atan2f(v[1], v[0]);
  Vector4 p(nTarget - v);
  dsq = p * p;
  d = sqrtf(dsq);
  float at = asinf(nTarget[2] / d);
  float ae = (l1 + l2 > d) ? acosf(0.5 * (l1sq + l2sq - dsq) / (l1 * l2)) : PI;
  float a1 = (l1 + l2 > d) ? acosf(0.5 * (l1sq + dsq - l2sq) / (l1 * d)) : 0.0;
  
  shoulderAngle = (at + a1) - PIOver2;
  elbowAngle = PI - ae;

  switch(configuration) {
    case frontRight:
    case backRight:
      hipAngle = -hipAngle;
      shoulderAngle = -shoulderAngle;
      elbowAngle = -elbowAngle;
      break;
    default:
      break;
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
  Serial.println("Setting idle pose");
  for(int i = 0; i < 4; i++) {
    float hipToShoulder = (i % 2) ? limbs[i].l0 : -limbs[i].l0;
    idlePose[i] = limbs[i].rootOffset - Vector4(hipToShoulder, currentBodyHeight, 0.0, 0.0);
    currentPose[i] = idlePose[i];
  }
  printPose(idlePose);
  Serial.println("Calculating IK Angles");
  setIKTargets(idlePose[0], idlePose[1], idlePose[2], idlePose[3]);
  update();
  emit();

  dxl.begin(57600);

  int j = 4;
  while(j--) {
    int i = 3;
    while(i--) {
      int id = j * 10 + i;
      dxl.setOperatingMode(id, OP_POSITION);
      dxl.writeControlTableItem(RETURN_DELAY_TIME, id, 0);
      /* THIS DOESN'T WORK
      Serial.print("Setting ");
      Serial.print(id);
      Serial.print(" to drive mode ");
      uint8_t driveMode = 0x8; // (j % 2);
      Serial.println(driveMode);
      dxl.writeControlTableItem(DRIVE_MODE, id, driveMode);
      */
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
  ikTargets[0] = iFrontRightTarget;
  ikTargets[1] = iFrontLeftTarget;
  ikTargets[2] = iBackRightTarget;
  ikTargets[3] = iBackLeftTarget;
  /*
  ikTargets[0] = limbs[0].normalizeTarget(iFrontRightTarget);
  ikTargets[1] = limbs[1].normalizeTarget(iFrontLeftTarget);
  ikTargets[2] = limbs[2].normalizeTarget(iBackRightTarget);
  ikTargets[3] = limbs[3].normalizeTarget(iBackLeftTarget);
  */
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
  // Serial.println("IK Targets");
  // printPose(ikTargets);
  int i = 4;
  while(i--) {
    limbs[i].calcIKAngles(ikTargets[i]);
  }
  Serial.println("IK Angles");
  printIKAngles();
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
    Matrix4 r(Vector4::j, latchedRotation * 0.5);

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

    for(int i = 0; i < 4; i++) {
      ikTargets[i] = currentPose[i] * (1.0 - progress) + nextPose[i] * progress;
    }

    switch(currentPole) {
      case frontRightBackLeft:
        ikTargets[0].v[1] += frontBell * currentStepHeight;
        ikTargets[3].v[1] += backBell * currentStepHeight;
        break;
      case frontLeftBackRight:
        ikTargets[1].v[1] += frontBell * currentStepHeight;
        ikTargets[2].v[1] += backBell * currentStepHeight;
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

    /*
    Serial.print(" ");
    Serial.print(progress);

    if(progress == 1.0) {
      Serial.println("\n");
    }
    */

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

void Robot::printPose(const Vector4 *iPose) const {
  for(int i = 0; i < 4; i++) {
    Serial.print(strings[i]);
    Serial.print(iPose[i][0]);
    Serial.print(", ");
    Serial.print(iPose[i][1]);
    Serial.print(", ");
    Serial.println(iPose[i][2]);
  }
}

void Robot::printIKLocators() const {
  for(int i = 0; i < 4; i++) {
    Serial.print(strings[i]);
    Serial.print(nextPose[i][0]);
    Serial.print(", ");
    Serial.print(nextPose[i][1]);
    Serial.print(", ");
    Serial.println(nextPose[i][2]);
  }
}

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
