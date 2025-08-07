#include "max31856.h"


/*
 * Convert to a uint 32 starting point.
 */
void max31856_convertBytesToTemperature(float* tempC, uint8_t *byte){
	uint32_t temp = 0;
	temp = ((uint32_t)(*byte)) | ((uint32_t)(*(byte+1))) | ((uint32_t)(*(byte+2)));
	temp /= 32; //2^5
	*tempC =  (float)temp;
}


void max31856_init(spiBusCallback cblk,chipSelectCallback csCblk){

}

uint8_t max31856_readTemp(float* tempC, spiBusCallback cblk,chipSelectCallback csCblk){
	uint8_t faultByte;

}

/*
 * Between the offset and the Q8 notation. Please use this function to convert to
 * actual temperature.
 *
 * There is an inherit assumption that
 * byte[0] - Most Significant byte
 * byte[1] - Middle Significant byte
 * byte[2] - Low Significant byte
 */

