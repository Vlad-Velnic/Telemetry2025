#pragma once
#include "driver/adc.h"
#include "esp_adc_cal.h"
//#include <Arduino.h>

//?adcObj

class adcObj {
private:
    esp_adc_cal_characteristics_t adc_chars;
    adc_unit_t unit;
    adc_atten_t atten;
    adc_bits_width_t bits;
    adc1_channel_t channel;
    float KalmanState;
    float KalmanUncertainty;
    float *KalmanOutput[2];
public:
    void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanMeasurement,  float *Kalman1DOutput[2]);
    adcObj(adc1_channel_t tempChannel, adc_unit_t tempUnit=ADC_UNIT_1, adc_atten_t tempAtten= ADC_ATTEN_DB_12,
    adc_bits_width_t tempBits = ADC_WIDTH_BIT_12);
    int getVoltage();
    float KalmanVoltage();
};
