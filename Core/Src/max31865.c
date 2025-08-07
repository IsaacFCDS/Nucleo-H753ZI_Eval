#include "max31865.h"

static RTD_Table_t RTDTable= {
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

void initMax31865(SpiBusCallback cblk,chipSelectCallback csCblk){
	uint8_t txData[2];
	uint8_t rxData[2];
	//write
	txData[0] = 0x80;
	txData[1] = 0b11000010;
	csCblk(0x01);
	cblk(txData,rxData,2);
	csCblk(0x00);
}

float readMax31865(SpiBusCallback cblk, chipSelectCallback csCblk){
	uint8_t txData[3];
	uint8_t rxData[3];
	int16_t adc;
	float result = -555.0f;
	txData[0] = 0x01;
	txData[1] = 0x00;
	txData[2] = 0x00;
	//Big endian?
	adc = (int16_t)(txData[1] << 8) | txData[2];
	csCblk(0x01);
	cblk(txData,rxData,3);
	csCblk(0x00);
	convertMax31865ToTemperature(&result,adc);
	return result;
}

void convertMax31865ToTemperature(float * temp, uint16_t adc){
	float slope = 1.0;
	uint8_t index = 0;
	//uint16_t rawTempValue = *(data+1)<<8 | *(data);

	if(adc < RTDTable[0][0]){
		*temp = (float)RTDTable[0][1]; //Return the the first temperature entry.
	}
	else if(adc > RTDTable[MAX31865_TABLE_ROWS-1][0])
	{
		*temp = (float)(RTDTable[MAX31865_TABLE_ROWS-1][1]);
	}
	else
	{
		while(RTDTable[index][0] < adc && index < MAX31865_TABLE_ROWS){
			index++;
		}
		// Found the index where the adc entry is greater than the adc value.
		slope = ((float)RTDTable[index][1] - (float)RTDTable[index-1][1]) / ((float)RTDTable[index][0] - (float)RTDTable[index-1][0]);
		*(temp) = slope * ((float)(adc - RTDTable[index-1][0])) + (float)RTDTable[index-1][1];
	}
}

RTD_Table_t* getTablePointer(void){
	return RTDTable;
}
