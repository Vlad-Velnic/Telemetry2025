#include "MPU9250c.hpp"




MPU9250c::MPU9250c() : IMU(Wire, 0x68){
}

void MPU9250c::begin(){
  this->status = IMU.begin();
     if (status < 0) {
    Serial.print("IMU initialization unsuccessful");
    Serial.print("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.print(status);
    assert("Setup failed");
    }
    Serial.println("**********  Setup succesful  ***********");
    Serial.println("***  Calibration for 5s please wait  ***");
    
    for (int i = 0; i < NrTicks; i++) {

    MPU9250c::read_setup();

    Axoff += currentData.accelX;
    Ayoff += currentData.accelY;
    Azoff = Azoff - currentData.accelZ - 9.80665;

    Rolloff += RateRoll;
    Pitchoff += RatePitch;
    Yawoff += RateYaw;

    xMagoff += Mx;
    yMagoff += My;
    zMagoff += Mz;

    delay(1 / portTICK_PERIOD_MS);
   }

    Serial.println("**********  Calibration done  **********");
    
    Axoff /= NrTicks;
    Ayoff /= NrTicks;
    Azoff /= NrTicks;

    Rolloff /= NrTicks;
    Pitchoff /= NrTicks;
    Yawoff /= NrTicks;

    xMagoff /= NrTicks;
    yMagoff /= NrTicks;
    zMagoff /= NrTicks;

}



void MPU9250c::read_setup(){
  IMU.readSensor();

  currentData.accelX = IMU.getAccelX_mss();
  currentData.accelY = IMU.getAccelY_mss();
  currentData.accelZ = IMU.getAccelZ_mss();

  currentData.gyroX = IMU.getGyroX_rads();
  currentData.gyroY = IMU.getGyroY_rads();
  currentData.gyroZ = IMU.getGyroZ_rads();

  currentData.magX = IMU.getMagX_uT();
  currentData.magY = IMU.getMagY_uT();
  currentData.magZ = IMU.getMagZ_uT();

  Mx = currentData.magX;
  My = currentData.magY;
  Mz = currentData.magZ;


  RateRoll = currentData.gyroX;
  RatePitch = currentData.gyroY;
  RateYaw = currentData.gyroZ * 0.0007;
  currentOrientation.roll = atan(currentData.accelY / sqrt(currentData.accelX * currentData.accelX + 
                currentData.accelZ * currentData.accelZ)) * 1 / (PI / 180);
  currentOrientation.pitch = -atan(currentData.accelX / sqrt(currentData.accelY * currentData.accelY +
                currentData.accelZ * currentData.accelZ)) * 1 / (PI / 180);

}

void MPU9250c::read() {
    IMU.readSensor();

  currentData.accelX = IMU.getAccelX_mss() - Axoff;
  currentData.accelY = IMU.getAccelY_mss() - Ayoff;
  currentData.accelZ = IMU.getAccelZ_mss() + Azoff;

  currentData.gyroX = IMU.getGyroX_rads();
  currentData.gyroY = IMU.getGyroY_rads();
  currentData.gyroZ = IMU.getGyroZ_rads();

  currentData.magX = IMU.getMagX_uT() - xMagoff;
  currentData.magY = IMU.getMagY_uT() - yMagoff;
  currentData.magZ = IMU.getMagZ_uT() - zMagoff;


  RateRoll = currentData.gyroX - Rolloff;
  RatePitch = currentData.gyroY - Pitchoff;
  RateYaw = currentData.gyroZ * 0.0007 - Yawoff;
  currentOrientation.roll = atan(currentData.accelY / sqrt(currentData.accelX * currentData.accelX + 
              currentData.accelZ * currentData.accelZ)) * 1 / (PI / 180);
  currentOrientation.pitch = -atan(currentData.accelX / sqrt(currentData.accelY * currentData.accelY +
              currentData.accelZ * currentData.accelZ)) * 1 / (PI / 180);

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, currentOrientation.roll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, currentOrientation.pitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  if (RateYaw < -Threshold) {
    currentOrientation.yaw += RateYaw * 1.003;
    }
  if (RateYaw >= Threshold) {
    currentOrientation.yaw += RateYaw * 0.978;
  }
  accel_yaw = atan2(-currentData.accelY, currentData.accelX) * 180.0 / M_PI;
  finalYaw = (currentOrientation.yaw * 257.142857) * 0.995 + accel_yaw * 0.005;
  if (finalYaw >= 360) {
    currentOrientation.yaw = 0;
  }

  if (finalYaw <= -360) {
    currentOrientation.yaw = 0;
  }
  
}

void MPU9250c::kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement){
    KalmanState = KalmanState + 0.1 * KalmanInput;

    KalmanUncertainty = KalmanUncertainty + 0.5 * 0.5 * 2 * 2;

    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 2 * 2);

    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);

    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

  float MPU9250c::getRoll() {
    return KalmanAngleRoll;
  }
  float MPU9250c::getPitch() {
    return KalmanAnglePitch;

  }
  float MPU9250c::getYaw(){
    return finalYaw;
  }
