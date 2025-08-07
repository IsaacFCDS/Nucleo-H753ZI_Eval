#ifndef _MAX31865_H_
#define _MAX31865_H_

#include "stdint.h"
#ifdef __cplusplus
 extern "C" {
#endif

#define	AX31865_BIAS_ON_MASK  0x80
#define	AX31865_CONVERSION_MODE_AUTO 0x40
#define AX31865_1_SHOT_AUTO_CLEAR_ENABLE  0x20
#define	AX31865_3_WIRE  0x10
#define MAX31865_FAULT_DELAY_2 0xC0
#define MAX31865_FAULT_DELAY_1 0x08
#define MAX31865_FAULT_AUTOMATIC_DELAY 0x04
#define MAX31865_FAULT_CLR 0x02
#define MAX31865_50Hz 0x01

#define MAX31865_TABLE_ROWS 46
#define MAX31865_TABLE_COl 2
typedef int16_t RTD_Table_t[MAX31865_TABLE_ROWS][MAX31865_TABLE_COl];

/**
 * txData The array of data to be sent from master to peripheral on the SPI bus
 * rxData The array of data sent from peripheral to master on SPI bus
 * length When called this is the number of bytes to be transacted.
 */
typedef void (SpiBusCallback)(uint8_t *txData, uint8_t *rxData, uint8_t length);
typedef void (chipSelectCallback)(uint8_t isLow);

void max31865_initializeDevice(void);
float readMax31865(SpiBusCallback cblk,chipSelectCallback csCblk);
void initMax31865(SpiBusCallback cblk,chipSelectCallback csCblk);
void convertMax31865ToTemperature(float * temp, uint16_t adc);

// Used for unit testing.
RTD_Table_t* getTablePointer(void);

#ifdef __cplusplus
}
#endif

#endif /* _MAX31865_H_ */
