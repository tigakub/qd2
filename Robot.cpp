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
float sigMin = 1.0 / (1.0 + exp(-6.0 * -0.5));
float sigRange = 1.0 / (1.0 + exp(-6.0 * 0.5)) - sigMin;

char *frontRight = "Front right: ";
char *frontLeft = "Front left: ";
char *backRight = "Back right: ";
char *backLeft = "Back left: ";

char *strings[] = { frontRight, frontLeft, backRight, backLeft };

float sigmoid(float x) {
  return ((1.0 / (1.0 + (exp((x - 0.5) * -6.0)))) - sigMin) / sigRange;
}

float inverseSigmoid(float y) {
    // log() is the natural log
    if(y <= 0.0) return 0.0;
    if(y >= 1.0) return 1.0;
    return log(1.0 / (y * sigRange + sigMin) - 1.0) / -6.0 + 0.5;
}

float bell(float x) {
	float dx = (x - 0.5);
	return exp(-dx * dx / 0.045) / sqrt(0.045 * PI) / 2.6594;
}

float quadratic(float x) {
  float d = 2.0 * x - 1.0;
  if(d < 0.0) d = 0.0;
  if(d > 1.0) d = 1.0;
  return 1.0 - d * d;
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
  Matrix4 t(Matrix4::translation, Vector4(-rootOffset[0], -rootOffset[1], -rootOffset[2], 0.0));

  switch(configuration) {
    case frontRight:
    case backRight:
      return t * iTarget;
  }

  Matrix4 r(Matrix4::rotation, Vector4::j, PI);
  Vector4 n(r * t * iTarget);
  n[2] *= -1.0;
  return n;

  /*
  Vector4 normalized = iTarget - rootOffset;
  
  switch(configuration) {
    case frontLeft:
    case backLeft: {
        Matrix4 rotation = Matrix4(Matrix4::rotation, Vector4::j, PI);
        normalized = rotation * normalized;
      }
      break;
    default:
      break;
  }
  
  return normalized;
  */
}

Vector4 Robot::Limb::denormalizeTarget(const Vector4 &iNormalized) const {
  Vector4 target;
  switch(configuration) {
    case frontLeft:
    case backLeft: {
        Matrix4 rotation = Matrix4(Matrix4::rotation, Vector4::j, -PI);
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
  Matrix4 mat(Matrix4::rotation, Vector4::k, e);
  Vector4 v(mat * unitp * l0);      // Shoulder position
  hipAngle = -atan2f(v[1], v[0]);   // Hip angle
  Vector4 p(nTarget - v);           // 
  dsq = p * p;                      // 
  d = sqrtf(dsq);                   // 
  float at = asinf(nTarget[2] / d);
  float ae = (l1 + l2 > d) ? acosf(0.5 * (l1sq + l2sq - dsq) / (l1 * l2)) : PI;
  float a1 = (l1 + l2 > d) ? acosf(0.5 * (l1sq + dsq - l2sq) / (l1 * d)) : 0.0;
  
  shoulderAngle = (at + a1) - PIOver2;
  elbowAngle = PI - ae;
  
  switch(configuration) {
    case frontLeft:
    case backLeft:
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

Robot::Stance operator*(const Matrix4 &iTransform, const Robot::Stance &iStance) {
  Robot::Stance newStance(iStance);
  for(int i = 0; i < 4; i++) {
    newStance[i] = iTransform * iStance[i];
  }
  return newStance;
}

Robot::StridePole operator~(Robot::StridePole iPole) {
  if(iPole == Robot::frontRightBackLeft) return Robot::frontLeftBackRight;
  return Robot::frontRightBackLeft;
}

Robot::Robot(float iHipToShoulder, float iShoulderToElbow, float iElbowToToe, const Vector4 &iFrontRightRootPosition, const Vector4 &iFrontLeftRootPosition, const Vector4 &iBackRightRootPosition, const Vector4 &iBackLeftRootPosition)
: idleStance((iFrontRightRootPosition[0] + iHipToShoulder) * 2.0, bodyHeight, iFrontRightRootPosition[2] * 2.0), 
  prevStance(idleStance), 
  nextStance(idleStance), 
  currentStance(idleStance),
  currentTranslation(),
  currentRotation(),
  currentSway(),
  currentMatrix(),
  currentInverse(),
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
    
    idleStance[i] = limbs[i].rootOffset - Vector4(hipToShoulder, bodyHeight, 0.0, 0.0);
    prevStance[i] = idleStance[i];
    nextStance[i] = idleStance[i];
    currentStance[i] = idleStance[i];
    /*
    idlePose[i] = Vector4(0.0, 0.0, 0.0, 1.0) + limbs[i].rootOffset - Vector4(hipToShoulder, currentBodyHeight, -25.0, 0.0);
    currentPose[i] = idlePose[i];
    nextPose[i] = idlePose[i];
    */
  }
  // printPose(idlePose);
  Serial.println("Calculating IK Angles");
  // setIKTargets(idlePose[0], idlePose[1], idlePose[2], idlePose[3]);
  // setIKTargets(idleStance[0], idleStance[1], idleStance[2], idleStance[3]);
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
  
  /*
  for(j = 0; j < 4; j++) {
    for(int i = 0; i < 3; i++) {
      int id = j * 10 + i;
      if(!dxl.ping(id)) {
        Serial.print("Ping ");
        Serial.print(id);
        Serial.println(" failed.");
      }
      dxl.ledOn(id);
      delay(1000);
      dxl.ledOff(id);
    }
  }
  */
  
}

void Robot::setControl(const QDCtrlMsg &iCtrl) {
  queuedHeading[0] = iCtrl.translation[0];
  queuedHeading[1] = 0.0;
  queuedHeading[2] = iCtrl.translation[1];
  queuedHeading[3] = 0.0;
  queuedAngularVelocity = iCtrl.rotation;
  queuedStepScale = iCtrl.stepScale;
  queuedSway = iCtrl.bodySway;
  queuedBodyHeight = iCtrl.bodyHeight;
  queuedStepHeight = iCtrl.stepHeight;
  queuedPeriod = iCtrl.duration;
  
  /*
  currentTranslation[0] = iCtrl.translation[0];
  currentTranslation[1] = iCtrl.translation[1];
  currentRotation = iCtrl.rotation;
  currentStepScale = iCtrl.stepScale;
  currentStepHeight = iCtrl.stepHeight;
  currentBodySway = iCtrl.bodySway;
  currentBodyHeight = iCtrl.bodyHeight;
  currentDuration = iCtrl.duration;
  */

  /*
  Serial.print("currentStepScale:   "); Serial.println(currentStepScale);
  Serial.print("currentStepHeight:  "); Serial.println(currentStepHeight);
  Serial.print("currentBodySway:    "); Serial.println(currentBodySway);
  Serial.print("currentBodyHeight:  "); Serial.println(currentBodyHeight);
  Serial.print("currentDuration:    "); Serial.println(currentDuration);
  */
}

/*
void Robot::setIKTargets(const Vector4 &iFrontRightTarget, const Vector4 &iFrontLeftTarget, const Vector4 &iBackRightTarget, const Vector4 &iBackLeftTarget) {
  ikTargets[0] = iFrontRightTarget;
  ikTargets[1] = iFrontLeftTarget;
  ikTargets[2] = iBackRightTarget;
  ikTargets[3] = iBackLeftTarget;
}

void Robot::setAngles(const Vector4 &iHipAngles, const Vector4 &iShoulderAngles, const Vector4 &iElbowAngles) {
  int i = 4;
  while(i--) {
    limbs[i][0] = iHipAngles[i];
    limbs[i][1] = iShoulderAngles[i];
    limbs[i][2] = iElbowAngles[i];
  }
}
*/

void Robot::generateNextStance() {
  heading = queuedHeading;
  angularVelocity = queuedAngularVelocity;
  stepScale = queuedStepScale;
  sway = queuedSway;
  bodyHeight = queuedBodyHeight;
  stepHeight = queuedStepHeight;
  period = queuedPeriod;

  prevTime = ((float) millis()) * 0.001;
  currentProgress = 0.0;

  prevStance = currentInverse * nextStance;
  nextStance = idleStance;

  auto v = heading * stepScale / period;
  auto a = angularVelocity / period;

  Matrix4 nextTranslation(Matrix4::translation, v);
  Matrix4 nextRotation(Matrix4::rotation, Vector4::j, a);
  Matrix4 nextMatrix(nextRotation * nextTranslation);

  switch(phase) {
    case starting:
    case walking:
      switch(polarity) {
        case frontRightBackLeft:
          nextStance[0] = nextMatrix * nextStance[0];
          nextStance[1] = prevStance[1];
          nextStance[2] = prevStance[2];
          nextStance[3] = nextMatrix * nextStance[3];
          break;
        case frontLeftBackRight:
          nextStance[0] = prevStance[0];
          nextStance[1] = nextMatrix * nextStance[1];
          nextStance[2] = nextMatrix * nextStance[2];
          nextStance[3] = prevStance[3];
      }
  }
  Serial.println("heading");
  v.print();
  Serial.println("angularVelocity");
  Serial.println(a);
  Serial.println("currentInverse");
  currentInverse.print();
  Serial.println("prevStance");
  prevStance.print();
  Serial.println("nextMatrix");
  nextMatrix.print();
  Serial.println("nextStance");
  nextStance.print(); 
}

void Robot::pause(bool iEnable) {
  if(!iEnable) {
    prevTime = ((float) millis()) * 0.001 - elapsed;
  }
  paused = iEnable;
}

#if 0
void Robot::pullNextPose() {
  int i = 4;
  while(i--) {
    currentPose[i] = nextPose[i];
  }

  currentPoseTime = ((float) millis()) * 0.001;
  
  i = 2;
  while(i--) {
    latchedTranslation[i] = currentTranslation[i];
  }
  latchedRotation = currentRotation;

  Matrix4 r(Matrix4::rotation, Vector4::j, latchedRotation);

  latchedStepScale = currentStepScale;
  latchedStepHeight = currentStepHeight;
  latchedBodySway = currentBodySway;
  latchedBodyHeight = currentBodyHeight;
  latchedDuration = currentDuration;

  i = 4;
  while(i--) {
    idlePose[i][1] = -latchedBodyHeight;
    nextPose[i] = idlePose[i];
  }

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
        Vector4 step(latchedTranslation[0] * latchedStepScale * 0.5, 0.0, latchedTranslation[1] * latchedStepScale * 0.5, 0.0);
        switch(currentPole) {
          case frontRightBackLeft:
            nextPose[0] += step;
            nextPose[1] -= step;
            nextPose[2] -= step;
            nextPose[3] += step;
            /*
            if(latchedRotation < 0.0) {
              nextPose[0] = r * nextPose[0];
              nextPose[3] = r * nextPose[3];
            }
            */
            /*
            for(int i = 0; i < 4; i++) {
              nextPose[i].v[i] += latchedBodySway;
            }
            */
            break;
          case frontLeftBackRight:
            nextPose[0] -= step;
            nextPose[1] += step;
            nextPose[2] += step;
            nextPose[3] -= step;
            /*
            if(latchedRotation > 0.0) {
              nextPose[1] = r * nextPose[1];
              nextPose[2] = r * nextPose[2];
            }
            */
            /*
            for(int i = 0; i < 4; i++) {
              nextPose[i].v[i] -= latchedBodySway;
            }
            */
            break;
        }
      }
      break;
    case stopping:
      break;
  }

  if(currentPole == frontRightBackLeft) {
    currentPole = frontLeftBackRight;
    Serial.println("frontRightBackLeft");
  } else {
    currentPole = frontRightBackLeft;
    Serial.println("frontLeftBackRight");
  }

  Serial.println("currentPose");
  printPose(currentPose);
  Serial.println("nextPose");
  printPose(nextPose);
}
#endif

void Robot::tick(float seconds) {
  switch(phase) {
    case idle:
      if(queuedHeading.magnitude() != 0 || queuedAngularVelocity != 0.0) {
        if(queuedAngularVelocity > 0.0) {
          polarity = frontRightBackLeft;
        } else {
          polarity = frontLeftBackRight;
        }
        phase = starting;
        Serial.println("Starting");
        generateNextStance();
      }
      break;
    case starting:
      if(currentProgress >= 1.0) {
        polarity = ~polarity;
        phase = walking;
        Serial.println("Walking");
        generateNextStance();
      }
      break;
    case walking:
      if(currentProgress >= 1.0) {
        polarity = ~polarity;
        if(heading.magnitude() == 0.0 && angularVelocity == 0.0) {
          phase = stopping;
          Serial.println("Stopping");
        }
        generateNextStance();
      }
      break;
    case stopping:
      if(currentProgress >= 1.0) {
        phase = idle;
        Serial.println("Idle");
        return;
      }
      break;
  }

  if(phase != idle) {
    if(!paused) {
      elapsed = ((float) millis()) * 0.001 - prevTime;
    }

    float progress = elapsed / period;
    if(progress < 0.0) progress = 0.0;
    if(progress > 1.0) progress = 1.0;

    float easedProgress = sigmoid(progress);

    float frontProgress = progress * 2.0;
    if(frontProgress < 0.0) frontProgress = 0.0;
    if(frontProgress > 1.0) frontProgress = 1.0;
    
    float easedFrontProgress = sigmoid(frontProgress);
    float quadraticFrontProgress = quadratic(frontProgress);

    float backProgress = (progress - 0.5) * 2.0;
    if(backProgress < 0.0) backProgress = 0.0;
    if(backProgress > 1.0) backProgress = 1.0; 

    float easedBackProgress = sigmoid(backProgress);
    float quadraticBackProgress = quadratic(backProgress);

    auto v = heading * stepScale * progress / period;
    auto a = angularVelocity * progress / period;

    currentMatrix = Matrix4();
    currentInverse = Matrix4();

    if(phase == walking || phase == stopping) {
      currentTranslation = Matrix4(Matrix4::translation, v);
      currentRotation = Matrix4(Matrix4::rotation, Vector4::j, a);
      currentMatrix = currentRotation * currentTranslation;
      currentInverse = currentMatrix.inverse();
      /*
      if(progress >= 1.0) {
        Serial.println("currentTranslation");
        currentTranslation.print();
        Serial.println("currentRotation");
        currentRotation.print();
        Serial.println("currentMatrix");
        currentMatrix.print();
        Serial.println("currentInverse");
        currentInverse.print();
      }
      */                              
    }

    prevStance = currentInverse * prevStance;
    nextStance = currentInverse * nextStance;

    if(progress >= 1.0) {
      Serial.println("completed prevStance");
      prevStance.print();
      Serial.println("completed nextStance");
      nextStance.print();
    }

    currentStance.interpolate(prevStance, nextStance, easedFrontProgress, easedBackProgress);

    if(phase == starting || phase == walking) {
      currentStance.liftFeet(polarity, stepHeight, quadraticFrontProgress, quadraticBackProgress);
    }
    /*
    float prevSway = ((phase == starting) || (phase == idle)) ? 0.0 : ((polarity == frontRightBackLeft) ? sway : -sway);
    float nextSway = ((phase == stopping) || (phase == idle)) ? 0.0 : ((polarity == frontRightBackLeft) ? -sway : sway);

    Matrix4 halfRotation(Matrix4::rotation, Vector4::j, a * 0.5);
    currentSway = Matrix4(Matrix4::translation, halfRotation * Vector4(prevSway * (1.0 - easedProgress) + nextSway * easedProgress));

    currentStance = currentSway * currentStance;
    */

    currentProgress = progress;
  }

  #if 0
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

    Vector4 step(latchedTranslation[0] * latchedStepScale, 0.0, latchedTranslation[1] * latchedStepScale, 0.0);
    Matrix4 t(Matrix4::translation, step * progress);
    Matrix4 r(Matrix4::rotation, Vector4::j, -latchedRotation * progress);
    Matrix4 m(r * t);

    // float stepPeriod = 0.25
    float stepPeriod = 0.5;
    float plantPeriod = (0.5 - stepPeriod) * 0.5;
    float stepProgress = progress / stepPeriod;

    float frontProgress = stepProgress - plantPeriod;
    float frontBell = 0.0;
    if(frontProgress < 0.0) {
      frontProgress = 0.0;
    } else if(frontProgress > 1.0) { 
      frontProgress = 1.0;
    } else {
      frontProgress = sigmoid(frontProgress);
      frontBell = quadratic(frontProgress);
    }

    float backProgress = (progress - 0.5) / stepPeriod;
    float backBell = 0.0;
    if(backProgress < 0.0) {
      backProgress = 0.0;
    } else if(backProgress > 1.0) {
      backProgress = 1.0;
    } else {
      backProgress = sigmoid(backProgress);
      backBell = quadratic(backProgress);
    }
    
    /*
    ikTargets[0] = currentPose[0] * (1.0 - progress) + nextPose[0] * progress;
    ikTargets[1] = currentPose[1] * (1.0 - progress) + nextPose[1] * progress;
    ikTargets[2] = currentPose[2] * (1.0 - progress) + nextPose[2] * progress;
    ikTargets[3] = currentPose[3] * (1.0 - progress) + nextPose[3] * progress;
    */

    /*
    switch(currentPole) {
      case frontRightBackLeft:
        Serial.print("Interpolating frontRightBackLeft ");
        ikTargets[0] = currentPose[0] * (1.0 - progress) + nextPose[0] * progress;
        ikTargets[1] = currentPose[1] * (1.0 - progress) + nextPose[1] * progress;
        ikTargets[2] = currentPose[2] * (1.0 - progress) + nextPose[2] * progress;
        ikTargets[3] = currentPose[3] * (1.0 - progress) + nextPose[3] * progress;
        break;
      case frontLeftBackRight:
        Serial.print("Interpolating frontLeftBackRight ");
        ikTargets[0] = currentPose[0] * (1.0 - progress) + nextPose[0] * progress;
        ikTargets[1] = currentPose[1] * (1.0 - frontProgress) + nextPose[1] * frontProgress;
        ikTargets[2] = currentPose[2] * (1.0 - backProgress) + nextPose[2] * backProgress;
        ikTargets[3] = currentPose[3] * (1.0 - progress) + nextPose[3] * progress;
        break;
    }
    */

    /*
    switch(currentPole) {
      case frontRightBackLeft:
        ikTargets[0].v[1] += quadratic(progress) * latchedStepHeight;
        ikTargets[3].v[1] += quadratic(progress) * latchedStepHeight;
        break;
      case frontLeftBackRight:
        ikTargets[1].v[1] += quadratic(progress) * latchedStepHeight;
        ikTargets[2].v[1] += quadratic(progress) * latchedStepHeight;
        break;
    }
    */

    /*
    switch(currentPole) {
      case frontRightBackLeft:
        ikTargets[0].v[1] += frontBell * latchedStepHeight;
        ikTargets[3].v[1] += backBell * latchedStepHeight;
        break;
      case frontLeftBackRight:
        ikTargets[1].v[1] += frontBell * latchedStepHeight;
        ikTargets[2].v[1] += backBell * latchedStepHeight;
        break;
    }
    */

    /*
    Serial.println("ikTargets");
    printPose(ikTargets);
    */

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
    /*
    if(currentProgress >= 1.0) {
      int i = 4;
      while(i--) {
        nextPose[i] = nextPose[i];
      }
    }
    */
    
    //Serial.print("(");
    Serial.print(progress);
    /*
    Serial.print(", ");
    Serial.print(frontProgress);
    Serial.print(", ");
    Serial.print(backProgress);
    */
    //Serial.print(")");
    if(progress == 1.0) {
      Serial.println("");
    } else {
      Serial.print(", ");
    }
  }
  #endif
  
  update();
  emit();
}

void Robot::update() {
  /*
  Serial.println("IK Targets");
  printPose(&(currentStance[0]));
  */
  int i = 4;
  while(i--) {
    limbs[i].calcIKAngles(currentStance[i]);
  }
  /*
  Serial.println("IK Angles");
  printIKAngles();
  */
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

void Robot::report() {
  float headingMag = heading.magnitude();
  if(headingMag != 0.0 && angularVelocity != 0.0) {
    Serial.print("Translating and rotating");
  } else if(headingMag != 0.0) {
    Serial.print("Translating");
  } else if(angularVelocity != 0.0) {
    Serial.print("Rotating");
  }
  if(angularVelocity > 0.0) {
    Serial.print(" counter-clockwise");
  } else if(angularVelocity < 0.0) {
    Serial.print(" clockwise");
  }
  Serial.println();
  /*
  if((currentTranslation[0] != 0.0 || currentTranslation[1] != 0.0) && currentRotation != 0.0) {
    Serial.print("Translating and rotating");
  } else if(currentTranslation[0] != 0.0 || currentTranslation[1] != 0.0) {
    Serial.print("Translating");
  } else if(currentRotation != 0.0) {
    Serial.print("Rotating");
  }
  if(currentRotation > 0.0) {
    Serial.print(" counter-clockwise");
  } else if(currentRotation < 0.0) {
    Serial.print(" clockwise");
  }
  */
}

/*
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
*/

void Robot::printPose(const Vector4 *iPose) const {
  for(int i = 0; i < 4; i++) {
    Serial.print(strings[i]);
    Serial.print(iPose[i][0]);
    Serial.print(", ");
    Serial.print(iPose[i][1]);
    Serial.print(", ");
    Serial.print(iPose[i][2]);
    Serial.print(", ");
    Serial.println(iPose[i][3]);
  }
}

void Robot::printIKLocators() const {
  for(int i = 0; i < 4; i++) {
    Serial.print(strings[i]);
    Serial.print(currentStance[i][0]);
    Serial.print(", ");
    Serial.print(currentStance[i][1]);
    Serial.print(", ");
    Serial.print(currentStance[i][2]);
    Serial.print(", ");
    Serial.println(currentStance[i][3]);
    /*
    Serial.print(strings[i]);
    Serial.print(nextPose[i][0]);
    Serial.print(", ");
    Serial.print(nextPose[i][1]);
    Serial.print(", ");
    Serial.print(nextPose[i][2]);
    Serial.print(", ");
    Serial.println(nextPose[i][3]);
    */
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
