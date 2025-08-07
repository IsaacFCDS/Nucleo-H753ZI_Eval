#include "max31865.h"

static RTD_Table_t RTDTable_pt100= {
		{ 1517, -200},
		{ 2394, -175},
		{ 3254, -150},
		{ 4101, -125},
		{ 4936, -100},
		{ 5762, -75},
		{ 6579, -50},
		{ 6903, -40},
		{ 7227, -30},
		{ 7550, -20},
		{ 7871, -10},
		{ 8192,   0},
		{ 8512,  10},
		{ 8830,  20},
		{ 9148,  30},
		{ 9465,  40},
		{ 9781,  50},
		{10096,  60},
		{10410,  70},
		{10723,  80},
		{11025,  90},
		{11346, 100},
		{11657, 110},
		{11966, 120},
		{12274, 130},
		{12582, 140},
		{12888, 150},
		{13194, 160},
		{13498, 170},
		{13802, 180},
		{14104, 190},
		{14406, 200},
		{15156, 225},
		{15901, 250},
		{16639, 275},
		{17371, 300},
		{18098, 325},
		{18818, 350},
		{19533, 375},
		{20242, 400},
		{20945, 425},
		{21642, 450},
		{22333, 475},
		{23018, 500},
		{23697, 525},
		{24370, 550}
};
static uint8_t txData[9] = {0};
static uint8_t rxData[9] = {0};
static uint16_t adc;
static float result = -555.0f;

void max31865_init(spiBusCallback cblk,chipSelectCallback csCblk){
	//write
	txData[0] = 0x80;
	/*
	 * b[7] - VBias - On
	 * b[6] - Conversion Auto
	 * b[5] - One Shot Off
	 * b[4] - 2/4 wire
	 * b[3:2] - Auto Fault Detection 0b01
	 * b[1] - Fault Clear
	 * b[0] - 60 hz filter enabled.
	 */
	txData[1] = 0b11000110;
	csCblk(0x01);
	cblk(txData,rxData,2);
	csCblk(0x00);
}

uint8_t max31865_readTemp(float* tempC, spiBusCallback cblk,chipSelectCallback csCblk){
	txData[0] = 0x00;
	txData[1] = 0x00;
	txData[2] = 0x00;
	csCblk(0x01);
	cblk(txData,rxData,9);
	csCblk(0x00);
	adc = (int16_t)(rxData[2] << 9) | rxData[3];
	max31865_convertBytesToTemperature(&result,adc);
	*tempC = result;
	return rxData[7]; //Fault register.
}

void max31865_convertBytesToTemperature(float * temp, uint16_t adc){
	float slope = 1.0;
	uint8_t index = 0;
	//uint16_t rawTempValue = *(data+1)<<8 | *(data);

	if(adc < RTDTable_pt100[0][0]){
		*temp = (float)RTDTable_pt100[0][1]; //Return the the first temperature entry.
	}
	else if(adc > RTDTable_pt100[MAX31865_TABLE_ROWS-1][0])
	{
		*temp = (float)(RTDTable_pt100[MAX31865_TABLE_ROWS-1][1]);
	}
	else
	{
		while(RTDTable_pt100[index][0] < adc && index < MAX31865_TABLE_ROWS){
			index++;
		}
		// Found the index where the adc entry is greater than the adc value.
		slope = ((float)RTDTable_pt100[index][1] - (float)RTDTable_pt100[index-1][1]) / ((float)RTDTable_pt100[index][0] - (float)RTDTable_pt100[index-1][0]);
		*(temp) = slope * ((float)(adc - RTDTable_pt100[index-1][0])) + (float)RTDTable_pt100[index-1][1];
	}
}

