#include "aditional.hpp"


void convert(float x, uint8_t* data) {
    uint32_t temp = x*100;
    //std::cout << std::bitset<24>(temp) << std::endl;
    
    data[0] = (uint8_t)(temp >> 0);
    data[1] = (uint8_t)(temp >> 8);
    
}