#pragma once
#include <Arduino.h>
#include "MPU9250.h"
#include <Wire.h>
#include <cmath>
#include <ArduinoLog.h>
#include <cassert>
#define LOG_LEVEL LOG_LEVEL_VERBOSE



#define NrTicks 5000

class MPU9250c {
public: 
    MPU9250c();
    void begin();
    void read_setup();
    void read();
    void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
    float getRoll();
    float getPitch();
    float getYaw();



public:
    int status;
    
    struct SensorData {
        double accelX, accelY, accelZ;
        double gyroX, gyroY, gyroZ;
        double magX, magY, magZ;
    };
    SensorData currentData;

    struct Orientation {
        double yaw, pitch, roll;
    };
    Orientation currentOrientation;
   
    MPU9250 IMU;
    double RateRoll;
    double RatePitch;
    double RateYaw;

    double Axoff;
    double Ayoff;
    double Azoff;

    double xMagoff;
    double yMagoff;
    double zMagoff;

    double Mx;
    double My;
    double Mz;

    double Rolloff;
    double Pitchoff;
    double Yawoff;
    double accel_yaw;

    double Threshold = 0.0000019;
    float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
    float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
    float Kalman1DOutput[2] = { 0, 0 };

    double finalYaw;
        
    
};