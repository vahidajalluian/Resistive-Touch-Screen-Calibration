#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "spi.h"
#include "XPT2046.h"
#define RES_SPI &hspi2

double X_CO[] = { -1.853, 0.014, 854.573};
double Y_CO[] = { -0.012, 1.243, -75.34193984 };

void resTouchCalibrateMatrixCalculation(uint16_t a[CALIBRATION_POINT_COUNT][3],uint16_t y[CALIBRATION_POINT_COUNT][2])
{
	double b[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

	for (uint8_t i = 0; i < 3; i++)
		for (uint8_t j = 0; j < 3; j++)
			for (uint8_t k = 0; k < CALIBRATION_POINT_COUNT; k++)
				b[i][j] += (double)(a[k][i]*a[k][j]);


	double det = (b[0][0]*b[1][1]*b[2][2])+(2*b[0][1]*b[1][2]*b[2][0])	-
			(b[0][0]*pow(b[1][2],2))-(b[1][1]*pow(b[0][2],2))-(b[2][2]*pow(b[0][1],2));

	double c[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	c[0][0] = ((b[1][1]*b[2][2]) - pow(b[1][2],2))/det;
	c[1][1] = ((b[0][0]*b[2][2]) - pow(b[0][2],2))/det;
	c[2][2] = ((b[0][0]*b[1][1]) - pow(b[0][1],2))/det;
	c[0][1] = ((b[0][2]*b[1][2]) - (b[0][1]*b[2][2]))/det;
	c[0][2] = ((b[0][1]*b[1][2]) - (b[0][2]*b[1][1]))/det;
	c[1][2] = ((b[0][1]*b[0][2]) - (b[1][2]*b[0][0]))/det;
	c[1][0] = c[0][1];
	c[2][0] = c[0][2];
	c[2][1] = c[1][2];
	double d[3][2] = {{0,0},{0,0},{0,0}};

	for (uint8_t i = 0; i < 3; i++)
		for (uint8_t j = 0; j < 2; j++)
			for (uint8_t k = 0; k < CALIBRATION_POINT_COUNT; k++)
				d[i][j] += (double)(a[k][i]*y[k][j]);

	double x[3][2]= {{0,0},{0,0},{0,0}};
	for (uint8_t i = 0; i < 3; i++)
		for (uint8_t j = 0; j < 2; j++)
			for (uint8_t k = 0; k < 3; k++)
				x[i][j] += c[i][k]*d[k][j];

	X_CO[0] = x[0][0];
	X_CO[1] = x[1][0];
	X_CO[2] = x[2][0];

	Y_CO[0] = x[0][1];
	Y_CO[1] = x[1][1];
	Y_CO[2] = x[2][1];
	printf ("%3.3f,%3.3f,%3.3f\n",X_CO[0],X_CO[1],X_CO[2]);
	printf ("%3.3f,%3.3f,%3.3f\n",Y_CO[0],Y_CO[1],Y_CO[2]);

}
bool touch_valid = false;
#define READ_X 0xD0
#define READ_Y 0x90
#define READ_Z1 0xB0
#define READ_Z2 0xC0
#define TouchSAMPLES 100

uint8_t instruct[2] = { 0, 0 };
union ui2c
{
	uint16_t ui;
	uint8_t c[2];
};
bool resTouchPressed(void)
{
	return HAL_GPIO_ReadPin(XPT2046_IRQ_GPIO_Port, XPT2046_IRQ_Pin)	== GPIO_PIN_RESET;
}

int cmpfunc(const void *a, const void *b)
{
   const uint16_t *A = a, *B = b;
   int16_t temp =(*A > *B) - (*A < *B);
   return temp;
}
uint16_t resTouchCoordinateRead(uint16_t *buffer, uint8_t Coordinate)
{
	uint8_t raw[2];
	uint16_t	nsamples = 0;
	instruct[0] = Coordinate;
	for (uint16_t i = 0; i < TouchSAMPLES; i++)
	{
		if (!resTouchPressed())
			break;
		nsamples++;

		HAL_SPI_TransmitReceive(RES_SPI, instruct, raw, sizeof(raw),
		HAL_MAX_DELAY);

		union ui2c temp;
		temp.c[0] = raw[0];
		temp.c[1] = raw[1];
		buffer[i] = temp.ui >> 6;
	}
	return nsamples;

}
typedef struct
{
	uint16_t x;
	uint16_t y;
}ResTouchCordinate_t;
void resTouchGetData(lv_indev_drv_t* drv, lv_indev_data_t* data)
{
	ResTouchCordinate_t cordinate;

	if(HAL_GPIO_ReadPin(RES_TOUCH_IRQ_GPIO_Port, RES_TOUCH_IRQ_Pin) == GPIO_PIN_RESET)
	{
		resTouchGetManipulatedCoordinates(&cordinate.x, &cordinate.y);
		if (( (cordinate.x == 0)  & (cordinate.y == 0)) ||
			( (cordinate.x > 800) & (cordinate.y > 480)))
		data->state = LV_INDEV_STATE_REL;
		else
		{
			data->point.x = cordinate.x;
			data->point.y = cordinate.y;
			data->state = LV_INDEV_STATE_PR;
		}
	}
	else
		data->state = LV_INDEV_STATE_REL;

}

bool resTouchGetRawCoordinates(uint16_t *x, uint16_t *y)
{
	uint16_t tempx[TouchSAMPLES], tempy[TouchSAMPLES],	tempz1[TouchSAMPLES], tempz2[TouchSAMPLES];

	instruct[0] = 0;

	HAL_SPI_Transmit(RES_SPI, instruct, 1, HAL_MAX_DELAY);

	if (resTouchCoordinateRead(tempx, READ_X) < TouchSAMPLES)
		return false;

	if (resTouchCoordinateRead(tempy, READ_Y) < TouchSAMPLES)
		return false;

	if (resTouchCoordinateRead(tempz1, READ_Z1) < TouchSAMPLES)
		return false;

	if (resTouchCoordinateRead(tempz2, READ_Z2) < TouchSAMPLES)
		return false;

	instruct[0] = 0;
	instruct[1] = 0;
	HAL_SPI_Transmit(RES_SPI, instruct, 2, HAL_MAX_DELAY);

	qsort(tempx, TouchSAMPLES,sizeof(uint16_t), cmpfunc);
	qsort(tempy, TouchSAMPLES,sizeof(uint16_t), cmpfunc);
	qsort(tempz1, TouchSAMPLES,sizeof(uint16_t), cmpfunc);
	qsort(tempz2, TouchSAMPLES,sizeof(uint16_t), cmpfunc);

	*x = tempx[TouchSAMPLES / 2];
	*y = tempy[TouchSAMPLES / 2];
//	uint32_t raw_z1 = tempz1[TouchSAMPLES / 2];
//	uint32_t raw_z2 = tempz2[TouchSAMPLES / 2];

	if (*x < XPT2046_MIN_RAW_X)
		*x = XPT2046_MIN_RAW_X;
	if (*x > XPT2046_MAX_RAW_X)
		*x = XPT2046_MAX_RAW_X;

	if (*y < XPT2046_MIN_RAW_Y)
		*y = XPT2046_MIN_RAW_Y;
	if (*y > XPT2046_MAX_RAW_Y)
		*y = XPT2046_MAX_RAW_Y;

	return true;

}
bool resTouchGetManipulatedCoordinates(uint16_t *x, uint16_t *y)
{
	resTouchGetRawCoordinates(x,y);

	uint16_t tempx,tempy;
	tempx = (X_CO[0] * (*x)) + (X_CO[1] * (*y)) + X_CO[2];
	tempy = (Y_CO[0] * (*x)) + (Y_CO[1] * (*y)) + Y_CO[2];

	if (tempx > 800)
		tempx = 800;

	if (tempy > 480)
		tempy = 480;

	*x = tempx;
	*y = tempy;
	return true;

}
