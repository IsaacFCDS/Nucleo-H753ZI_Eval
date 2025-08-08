#include "max31856.h"

static uint8_t txData[17] = {0};
static uint8_t rxData[17] = {0};
static uint16_t adc;
static float result = -555.0f;


void max31856_init(spiBusCallback cblk,chipSelectCallback csCblk){
	MAX31856_CR0_t cr0 = {.ConvMode = MAX31856_CONV_MODE_AUTO,
						  .OneShot = MAX31856_OneShot_NONE,
						  .OpenCircuitFault = MAX31856_CR0_OPEN_CIRCUIT_FAULT_DISABLED,
						  .ColdJunctionSensorDisable = MAX31856_CJ_DISABLE,
						  .FaultMode = MAX31856_FAULT_MODE_COMP,
						  .FaultClear = MAX31856_FAULT_NoClr,
						  .NoiseRejectionFilter = MAX31856_NOISE_FILTER_60hz};


	MAX31856_CR1_t cr1 = {.ACGSEL = MAX31856_CR1_AVGSEL_16,
						  .TC_TYPE = MAX31856_CR1_TC_TYPE_K};

	MAX31856_FAULTS_MASK_t faultMask = {.CJ_High_En = 1,
									    .CJ_Low_En = 1,
									    .TC_High_En = 1,
									    .TC_Low_En = 1,
									    .OV_UV_FAULT = 1,
									    .Open_Fault = 1};

	txData[0] = MAX31856_SetWriteReg(MAX31856_REG_R_ADDR_CR0);
	txData[1] = cr0.value;
	txData[2] = cr1.value;
	txData[3] = 0x00;//faultMask.value;
	csCblk(0x01);
	cblk(txData,rxData,4);
	csCblk(0x00);
}


uint8_t max31856_readTemp(float* tempC, spiBusCallback cblk,chipSelectCallback csCblk){
	uint8_t faultByte;
	txData[0] = MAX31856_REG_R_ADDR_LTCBH;
	txData[1] = 0x00;
	txData[2] = 0x00;
	csCblk(0x01);
	cblk(txData,rxData,5);
	csCblk(0x00);
	max31856_convertBytesToTemperature(tempC, &rxData[1]);
	return rxData[4];
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
void max31856_convertBytesToTemperature(float* tempC, uint8_t *byte){
	uint32_t temp = 0;
	temp = (((uint32_t)(*byte))<<24) | (((uint32_t)(*(byte+1)))<<16) | (((uint32_t)(*(byte+2)))<<8);
	temp &= 0xFFFFE000; // Mask off lower 5 bits. Remove don't care bits at temp[4:0]
	*tempC =  ((float)temp)/(float)0x000FFFFF; //7 bits of decimal starting at location temp[11:5]
}

