#ifndef _MAX_31856_H_
#define _MAX_31856_H_

#ifdef __cplusplus
 extern "C" {
#endif
#include "stdint.h"

 /*
  * There is some fabulous shifting and scaling for the cold junction temperature. If you
  * intend to use that register, please consult the datasheet for the MAX31856.
  */
#define MAX31856_SetWriteReg(a) (a|0x80)

#define MAX31856_REG_R_ADDR_CR0    0x00
#define MAX31856_REG_R_ADDR_CR1    0x01
#define MAX31856_REG_R_ADDR_MASK   0x02
#define MAX31856_REG_R_ADDR_CJHF   0x03
#define MAX31856_REG_R_ADDR_CJLF   0x04
#define MAX31856_REG_R_ADDR_LTHFTH 0x05
#define MAX31856_REG_R_ADDR_LTHFTL 0x06
#define MAX31856_REG_R_ADDR_LTLFTH 0x07
#define MAX31856_REG_R_ADDR_LTLFTL 0x08
#define MAX31856_REG_R_ADDR_CJTO   0x09
#define MAX31856_REG_R_ADDR_CJTH   0x0A
#define MAX31856_REG_R_ADDR_CJTL   0x0B
#define MAX31856_REG_R_ADDR_LTCBH  0x0C
#define MAX31856_REG_R_ADDR_LTCBM  0x0D
#define MAX31856_REG_R_ADDR_LTCBL  0x0E
#define MAX31856_REG_R_ADDR_SR     0x0F

 typedef enum{
	 MAX31856_CR0_OPEN_CIRCUIT_FAULT_DISABLED = 0x00,
	 MAX31856_CR0_OPEN_CIRCUIT_FAULT_MODE1 = 0x01,
	 MAX31856_CR0_OPEN_CIRCUIT_FAULT_MODE2 = 0x02,
	 MAX31856_CR0_OPEN_CIRCUIT_FAULT_MODE3 = 0x03
 }MAX31856_CR0_OPEN_CIRCUIT_FAULT_t;

 typedef union{
	 uint8_t value;
	 struct{
		uint8_t ConvMode:1;
		uint8_t OneShot:1;
		uint8_t OpenCircuitFault:2;
		uint8_t ColdJunctionSensorDisable:1;
		uint8_t FaultMode:1;
		uint8_t Fault_CLEAR:1;
		uint8_t NoiseRejectionFilter_50hz:1;
	 };
 }MAX31856_CR0_t;

 typedef enum{
	 MAX31856_CR1_AVGSEL_1 = 0x00,
	 MAX31856_CR1_AVGSEL_2 = 0x01,
	 MAX31856_CR1_AVGSEL_4 = 0x02,
	 MAX31856_CR1_AVGSEL_8 = 0x03,
	 MAX31856_CR1_AVGSEL_16 = 0x04,
 }MAX31856_CR1_AVGSEL_t;

 typedef enum{
	 MAX31856_CR1_TC_TYPE_B = 0x00,
	 MAX31856_CR1_TC_TYPE_E = 0x01,
	 MAX31856_CR1_TC_TYPE_J = 0x02,
	 MAX31856_CR1_TC_TYPE_K = 0x03,
	 MAX31856_CR1_TC_TYPE_N = 0x04,
	 MAX31856_CR1_TC_TYPE_R = 0x05,
	 MAX31856_CR1_TC_TYPE_S = 0x06,
	 MAX31856_CR1_TC_TYPE_T = 0x07,
	 MAX31856_CR1_TC_TYPE_G8 = 0x08,
	 MAX31856_CR1_TC_TYPE_G32 = 0x0C
 }MAX31856_CR1_TC_TYPE_t;

 typedef union{
	 uint8_t value;
		 struct{
		 	 uint8_t reserved:1;
		 	 uint8_t ACGSEL:3;
		 	 uint8_t TC_TYPE:4;
		 };
 }MAX31856_CR1_t;

typedef union{
	uint8_t value;
	struct{
		uint8_t reserved:2;
		uint8_t CJ_High_En:1;
		uint8_t CJ_Low_En:1;
		uint8_t TC_High_En:1;
		uint8_t TC_Low_En:1;
		uint8_t OV_UV_FAULT:1;
		uint8_t Open_Fault:1;
	};
}MAX31856_FAULTS_MASK_t;

/*
 * This value is the sized Q4 number. So msb is sign, fraction starts from bit 0 to 3.
 * So sized fractional with 2^4 bits of fraction.
 */
typedef union{
	uint16_t value;
    uint8_t byte[2];
}MAX31856_LINEARIZED_TEMP_FAULT_THRESHOLD_t;

/*
 * Between the offset and the Q8 notation. Please use this function to convert to
 * actual temperature.
 *
 * There is an inherit assumption that
 * byte[0] - Most Significant byte
 * byte[1] - Middle Significant byte
 * byte[2] - Low Significant byte
 */
float MAX31856_convertBytesToTemp(uint8_t *byte);

#ifdef __cplusplus
}
#endif
#endif /* MAX31856_H_ */
