#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

// Config registers (MPU 6050)
#define sample_rate_divider		25
#define configuration 			26
#define gyro_config				27
#define acc_config				28
#define interrupt_enable		29
#define pwr_management			107

// Acc Registers (MPU 6050)
#define acc_x	59
#define acc_y	61
#define acc_z	63

// Interrupt pin (MPU 6050)
#define INT 40

// Config registers (MAX7219)
#define shutdown_format		0x0C
#define decode_mode			0x09
#define intensity			0x0A
#define scan_limit			0x0B
#define display_test		0x0F

// Channel SPI
#define CHANNEL_SPI 0
 



uint8_t status = 1;
uint8_t point_col =  0x80;
uint8_t point_row = 0x01;

// ********** MPU6050 **********//
int mpu6050;
float Ax, Ay, Az, pitch, roll;

int16_t read_acc(unsigned char acc_reg){
	int16_t high = wiringPiI2CReadReg8(mpu6050, acc_reg);
	int16_t data = (high<<8) | wiringPiI2CReadReg8(mpu6050, acc_reg+1);
	return data; 
}

void init_mpu6050(){
	// Sample rate 100Hz
	wiringPiI2CWriteReg8(mpu6050, sample_rate_divider, 0x09);
	
	// DLPF <= 44Hz
	wiringPiI2CWriteReg8(mpu6050, configuration, 0x03);
	
	// Gyro +- 250 o/s
	wiringPiI2CWriteReg8(mpu6050, gyro_config, 0x00);

	// +-2g
	wiringPiI2CWriteReg8(mpu6050, acc_config, 0x00);
	
	// interrupt enable
	wiringPiI2CWriteReg8(mpu6050, interrupt_enable, 0x01);
	
	// PLL with X axis
	wiringPiI2CWriteReg8(mpu6050, pwr_management, 0x01);
	
}

// ********** MAX7219 MATRIX ********** //
void max7219_process(uint8_t address, uint8_t value){
	unsigned char data[2];
	data[0] = address;
	data[1] = value;
	wiringPiSPIDataRW(CHANNEL_SPI, data, 2);
}

void init_max7219(){
	// Normal operation
	max7219_process(shutdown_format, 0x01);
	// No decode mode
	max7219_process(decode_mode, 0x00);
	// Intensity 3/32
	max7219_process(intensity, 0x01);
	// Scan limit 0-7
	max7219_process(scan_limit, 0x07);
	// Normal operation
	max7219_process(display_test, 0x00);
}

void display(const uint8_t arr[]){
	for(int i=0; i<8; i++){
		max7219_process(i+1, arr[7-i]);
	}
}

void resertMatrix(){
	for(int i=0; i<8; i++){
		max7219_process(i+1, 0x00);
	}
}

// ********* MAIN INTERRUPT *********//
void INTERRUPT_MPU(){
	Ax = (float)read_acc(acc_x)/16384.0;
	Ay = (float)read_acc(acc_y)/16384.0;
	Az = (float)read_acc(acc_z)/16384.0;	
	
	pitch = atan2(Ax, sqrt(pow(Ay,2) + pow(Az,2)))*180/M_PI;
	roll = atan2(Ay, sqrt(pow(Ax,2) + pow(Az,2)))*180/M_PI;
	
	if(roll > 8){
		status = 1; // Right
	}
	else if(roll < -8){
		status = 2; // Left
	}
	else if(pitch > 8){
		status = 3; // Down
	}
	else if(pitch < -8){
		status = 4; // Up
	}
	else{
		status = 5;
	}
	
	printf("Pitch: %.2f, roll: %.2f, status: %d\n", pitch, roll, status);
}

int main(){
	// Set up pins
	wiringPiSetupPhys();
	
	pinMode(INT, INPUT);
	wiringPiISR(INT, INT_EDGE_BOTH, &INTERRUPT_MPU);	

	// Set up i2c
	mpu6050 = wiringPiI2CSetup(0x68);
	
	// Config mpu6050
	init_mpu6050();
	
	// Seup SPI
	wiringPiSPISetup(CHANNEL_SPI, 8000000);
	
	// Config Max7219
	init_max7219();
	
	Ax = (float)read_acc(acc_x)/16384.0;
	Ay = (float)read_acc(acc_y)/16384.0;
	Az = (float)read_acc(acc_z)/16384.0;	
	

	while(1){
		resertMatrix();
		max7219_process(point_row, point_col);
		switch(status){
			case 1:
				point_col >>=1;
				if(point_col == 0x00)	point_col = 0x80;
				break;
			case 2:
				point_col <<=1;
				if(point_col == 0x00)	point_col = 0x01;
				break;
			case 3:
				point_row++;
				if(point_row >8)	point_row = 0x01;
				break;
			case 4:
				point_row--;
				if(point_row <1)	point_row = 0x08;
				break;
				break;
			case 5:
				point_col = point_col;
				break;
		}
		delay(200);
		
	}
	return 0;
}