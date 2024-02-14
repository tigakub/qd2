//
//  Robot.hpp
//  qd
//
//  Created by Edward Janne on 10/18/23.
//

#ifndef Robot_hpp
#define Robot_hpp

#define TwoPI 6.283185307179586
#define PI 3.141592653589793
#define PIOver2 1.570796326794897
#define PIOver4 0.785398163397449

#include <stdio.h>
#include <deque>

#define ENABLE_ACTUATOR_XL330

#include <DynamixelShield.h>

#include "LinearAlgebra.hpp"
#include "QDMessage.hpp"

// #define RAD_TO_DEG(rad) (rad * 180.0 / PI)

typedef struct Robot {
    typedef enum StridePole {
      frontRightBackLeft = 0,
      frontLeftBackRight = 1
    } StridePole;

    typedef enum Phase {
      idle = 1,
      starting = idle + 1,
      walking = starting + 1,
      stopping = walking + 1,
    } Phase;

    typedef struct Stance {
      Vector4 locators[4];

      Stance(float width, float height, float depth) {
        locators[0] = Vector4(width * 0.5, -height, depth * 0.5, 1.0);
        locators[1] = Vector4(-width * 0.5, -height, depth * 0.5, 1.0);
        locators[2] = Vector4(width * 0.5, -height, -depth * 0.5, 1.0);
        locators[3] = Vector4(-width * 0.5, -height, -depth * 0.5, 1.0);
      }

      Stance(const Stance &iOther) {
        int i = 4;
        while(i--) {
          locators[i] = iOther.locators[i];
        }
      }

      Vector4 operator[](int iIndex) const {
        return locators[iIndex];
      }

      Vector4 &operator[](int iIndex) {
        return locators[iIndex];
      }

      void interpolate(const Stance &iFrom, const Stance &iTo, float iFrontLateralFactor, float iBackLateralFactor) {
        locators[0] = iFrom.locators[0] * (1.0 - iFrontLateralFactor) + iTo.locators[0] * iFrontLateralFactor;
        locators[1] = iFrom.locators[1] * (1.0 - iFrontLateralFactor) + iTo.locators[1] * iFrontLateralFactor;
        locators[2] = iFrom.locators[2] * (1.0 - iBackLateralFactor) + iTo.locators[2] * iBackLateralFactor;
        locators[3] = iFrom.locators[3] * (1.0 - iBackLateralFactor) + iTo.locators[3] * iBackLateralFactor;
      }

      void liftFeet(StridePole iStridePole, float iStepHeight, float iFrontVerticalFactor, float iBackVerticalFactor) {
        switch(iStridePole) {
          case frontRightBackLeft:
            locators[0][1] += iStepHeight * iFrontVerticalFactor;
            locators[3][1] += iStepHeight * iBackVerticalFactor;
            break;
          case frontLeftBackRight:
            locators[1][1] += iStepHeight * iFrontVerticalFactor;
            locators[2][1] += iStepHeight * iBackVerticalFactor;
            break;
        }
      }

      void print() const {
        for(int i = 0; i < 4; i++) {
          locators[i].print();
        }
      }
    } Stance;

    typedef struct Limb {
        typedef enum Configuration {
            frontRight = 0,
            frontLeft = 1,
            backRight = 2,
            backLeft = 3
        } Config;
        
        Configuration configuration;
        Vector4 rootOffset;
        float l0;
        float l0sq;
        float l1;
        float l1sq;
        float l2;
        float l2sq;
        
        float hipAngle, shoulderAngle, elbowAngle;
        
        Limb(Configuration iConfig, const Vector4 &iRootOffset, float iHipToShoulder, float iShoulderToElbow, float iElbowToToe);
        
        Vector4 normalizeTarget(const Vector4 &iTarget) const;
        Vector4 denormalizeTarget(const Vector4 &iNormalized) const;
        
        void calcIKAngles(const Vector4 &iTarget);

        float operator[](int i) const;
        float &operator[](int i);
        
    } Limb;

    Limb limbs[4];

    /*
    Vector4 ikTargets[4];

    Vector4 idlePose[4];
    Vector4 currentPose[4];
    Vector4 nextPose[4];
    */

    /*
    Mode currentMode = idle;

    StridePole currentPole = frontRightBackLeft;
    */

    Vector4 queuedHeading;
    float queuedAngularVelocity = 0.0;
    float queuedStepScale = 100.0;
    float queuedSway = 30.0;
    float queuedBodyHeight = 90.0;
    float queuedStepHeight = 20.0;
    float queuedPeriod = 3.0;

    Vector4 heading;
    float angularVelocity = 0.0;
    float stepScale = 100.0;
    float sway = 30.0;
    float bodyHeight = 90.0;
    float stepHeight = 20.0;
    float period = 3.0;

    bool paused = false;
    float prevTime = 0.0;
    float elapsed = 0.0;

    float currentProgress = 0.0;

    Stance idleStance;
    Stance prevStance;
    Stance nextStance;
    Stance currentStance;

    Matrix4 currentRotation;
    Matrix4 currentTranslation;
    Matrix4 currentSway;
    Matrix4 currentMatrix;
    Matrix4 currentInverse;

    StridePole polarity = frontRightBackLeft;
    Phase phase = idle;

    // QDPose2 nextPose;
    // std::deque<QDPose2> poseBuffer;

    /*
    float currentTranslation[2] = { 0.0, 0.0 };
    float currentRotation = 0.0;
    float currentStepScale = 100.0;
    float currentStepHeight = 30.0;
    float currentBodySway = 0.0;
    float currentBodyHeight = 100.0;
    float currentDuration = 2.0;

    float latchedTranslation[2] = { 0.0, 0.0 };
    float latchedRotation = 0.0;
    float latchedStepScale = 100.0;
    float latchedStepHeight = 30.0;
    float latchedBodySway = 0.0;
    float latchedBodyHeight = 100.0;
    float latchedDuration = 2.0;

    float currentPoseTime;
    
    float currentProgress;
    */

    DynamixelShield dxl;

    Robot(float iHipToShoulder, float iShoulderToElbow, float iElbowToToe, const Vector4 &iFrontRightRootPosition, const Vector4 &iFrontLeftRootPosition, const Vector4 &iBackRightRootPosition, const Vector4 &iBackLeftRootPosition);
    
    void begin();
    
    void setControl(const QDCtrlMsg &iCtrl);
    
    /*
    void setIKTargets(const Vector4 &iFrontRightTarget, const Vector4 &iFrontLeftTarget, const Vector4 &iBackRightTarget, const Vector4 &iBackLeftTarget);
    void setAngles(const Vector4 &iHipAngles, const Vector4 &iShoulderAngles, const Vector4 &iElbowAngles);
    */

    void update();
    void emit();

    void pause(bool iEnable);
    void generateNextStance();

    void pullNextPose();
    void report();
    void tick(float seconds);

    /*
    void idleTick();
    void startTick();
    void walkTick();
    void stopTick();
    */

    void printPose(const Vector4 *iPose) const;
    void printIKLocators() const;
    void printIKAngles() const;
} Robot;

Robot::Stance operator*(const Matrix4 &iTransform, const Robot::Stance &iStance);

Robot::StridePole operator~(Robot::StridePole iPole);

#endif /* Robot_hpp */
