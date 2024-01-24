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

    typedef enum StridePole {
      frontRightBackLeft = 0,
      frontLeftBackRight = 1
    } StridePole;

    typedef enum Mode {
      idle = 1,
      starting = idle + 1,
      walking = starting + 1,
      stopping = walking + 1,
    } Mode;

    Limb limbs[4];
    Vector4 ikTargets[4];

    Vector4 idlePose[4];
    Vector4 currentPose[4];
    Vector4 nextPose[4];

    Mode currentMode = idle;
    StridePole currentPole = frontRightBackLeft;

    // QDPose2 nextPose;
    // std::deque<QDPose2> poseBuffer;

    float currentTranslation[2] = { 0.0, 0.0 };
    float currentRotation = 0.0;
    float currentStepScale = 0.0;
    float currentStepHeight = 0.0;
    float currentBodySway = 0.0;
    float currentBodyHeight = 80.0;
    float currentDuration = 2.0;

    float latchedTranslation[2] = { 0.0, 0.0 };
    float latchedRotation = 0.0;
    float latchedStepScale = 0.0;
    float latchedStepHeight = 0.0;
    float latchedBodySway = 0.0;
    float latchedBodyHeight = 80.0;
    float latchedDuration = 2.0;

    Matrix4 rotationMatrix;
    Matrix4 counterRotationMatrix;

    float currentPoseTime;
    float currentProgress;

    DynamixelShield dxl;

    Robot(float iHipToShoulder, float iShoulderToElbow, float iElbowToToe, const Vector4 &iFrontRightRootPosition, const Vector4 &iFrontLeftRootPosition, const Vector4 &iBackRightRootPosition, const Vector4 &iBackLeftRootPosition);
    
    void begin();
    
    void setControl(const QDCtrlMsg &iCtrl);
    
    void setIKTargets(const Vector4 &iFrontRightTarget, const Vector4 &iFrontLeftTarget, const Vector4 &iBackRightTarget, const Vector4 &iBackLeftTarget);
    void setAngles(const Vector4 &iHipAngles, const Vector4 &iShoulderAngles, const Vector4 &iElbowAngles);
    
    void update();
    void emit();

    void pullNextPose();
    void report();
    void tick(float seconds);

    void idleTick();
    void startTick();
    void walkTick();
    void stopTick();

    void printPose(const Vector4 *iPose) const;
    void printIKLocators() const;
    void printIKAngles() const;
} Robot;

#endif /* Robot_hpp */
