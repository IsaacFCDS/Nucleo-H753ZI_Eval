#include "max31856.h"


float MAX31856_convertBytesToTemp(uint8_t *byte){
/*
 * Convert to a uint 32 starting point.
 */
	uint32_t temp = 0;
	float result = 0.0f;
	temp = ((uint32_t)(*byte)) | ((uint32_t)(*(byte+1))) | ((uint32_t)(*(byte+2)));
	temp /= 32; //2^5
	return (float)temp;
}
