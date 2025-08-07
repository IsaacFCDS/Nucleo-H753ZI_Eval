#ifndef _MAX31865_H_
#define _MAX31865_H_

#include "stdint.h"
#ifdef __cplusplus
 extern "C" {
#endif


#define	MAX31865_BIAS_ON_MASK  0x80
#define	MAX31865_CONVERSION_MODE_AUTO 0x40
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
 *
 * txData
 * rxData
 * length
 */
/**
 * @fn void (SpiBusCallback) (uint8_t*, uint8_t*, uint8_t)
 * @brief This prototype is passed into the calling functions to allow data to be
 * transacted with the device.
 *
 * @pre Caller has exclusive use of spi bus. There isn't a mutex around access.
 * @post Bus is released for next user.
 * @param txData The array of data to be sent from master to peripheral on the SPI bus
 * @param rxData The array of data sent from peripheral to master on SPI bus
 * @param length When called this is the number of bytes to be transacted.
 */
typedef void (spiBusCallback)(uint8_t *txData, uint8_t *rxData, uint8_t length);

/**
 * @fn void (chipSelectCallback) (uint8_t)
 * @brief This function is provided so the calling devices chip select line can be pulled low
 * to signal a desire to talk.
 *
 * @pre None
 * @post None
 * @param isLow This is a zero if the chip select is de-selected (voltage high) or non-zero (1) if the
 * chip select should be asserted (voltage low).
 */
typedef void (chipSelectCallback)(uint8_t isLow);



/**
 * @fn uint8_t max31865_readTemp(float*, spiBusCallback, chipSelectCallback)
 * @brief This function should be called when a read of the temperature is requested.
 * This could be done in a polling manner of when the data ready pin is activated.
 *
 * @pre Device is initialized.
 * @post Temperature and status are read from peripheral.
 * @param tempC A pointer for passing the temperature in celsius back to the caller.
 * @param cblk A function to allow access to the spi bus.
 * @param csCblk A function to allow signaling the peripheral chip using the chip select line.
 * @return The status of the peripheral device.
 */
uint8_t max31865_readTemp(float* tempC, spiBusCallback cblk,chipSelectCallback csCblk);

/**
 * @fn void max31865_init(spiBusCallback, chipSelectCallback)
 * @brief This function initializes the peripheral chip to operation state.
 *
 * @pre None
 * @post MAX31865 is initialized and is periodically sampling temperature data.
 * @param cblk A function to allow access to the spi bus.
 * @param csCblk A functoin to allow signaling the peripheral chip using the chip select line.
 */
void max31865_init(spiBusCallback cblk,chipSelectCallback csCblk);

/**
 * @fn void max31865_convertBytesToTemperature(float*, uint16_t)
 * @brief Converts the data bytes read from the peripheral into temperature.
 *
 * @pre None
 * @post None
 * @param temp A pointer for passing the temperature in celsius back to the caller.
 * @param adc A pointer to the data bytes of the adc conversion.
 */
void max31865_convertBytesToTemperature(float * temp, uint16_t adc);

#ifdef __cplusplus
}
#endif

#endif /* _MAX31865_H_ */
