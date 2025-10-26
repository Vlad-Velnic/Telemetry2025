#include "adcObj.hpp"

adcObj::adcObj(adc1_channel_t tempChannel, adc_unit_t tempUnit, adc_atten_t tempAtten, adc_bits_width_t tempBits) :
 channel(tempChannel), unit(tempUnit), atten(tempAtten), bits(tempBits) {
     esp_adc_cal_characterize(unit, atten, bits, 0, &adc_chars);
     esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);
     ESP_ERROR_CHECK(adc1_config_width(bits));
     ESP_ERROR_CHECK(adc1_config_channel_atten(channel, atten));
    
 }  

int adcObj::getVoltage() {
    int voltage = adc1_get_raw(channel);
    return esp_adc_cal_raw_to_voltage(adc1_get_raw(channel), &adc_chars);
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanMeasurement, float *Kalman1DOutput[2]){

    KalmanUncertainty = KalmanUncertainty + 0.5 * 0.5 * 2 * 2;

    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 2 * 2);

    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);

    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = &KalmanState;
    Kalman1DOutput[1] = &KalmanUncertainty;
}

float adcObj::KalmanVoltage() {
    int voltage = adc1_get_raw(channel);
    kalman_1d(KalmanState, KalmanUncertainty, voltage, KalmanOutput);
    return *KalmanOutput[0];
}