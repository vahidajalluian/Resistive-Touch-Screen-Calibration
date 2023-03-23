#ifndef XPT2046_TOUCH_H_
#define XPT2046_TOUCH_H_

#include <stdbool.h>

#include "main.h"
#include "lvgl.h"

#define CALIBRATION_POINT_COUNT 9

#define XPT2046_IRQ_Pin       RES_TOUCH_IRQ_Pin
#define XPT2046_IRQ_GPIO_Port RES_TOUCH_IRQ_GPIO_Port

#define XPT2046_SCALE_X 799
#define XPT2046_SCALE_Y 479

#define XPT2046_MIN_RAW_X 10
#define XPT2046_MAX_RAW_X 65535
#define XPT2046_MIN_RAW_Y 10
#define XPT2046_MAX_RAW_Y 65535


void resTouchCalibrateMatrixCalculation(uint16_t a[5][3],uint16_t y[5][2]);

bool resTouchPressed(void);
bool resTouchGetRawCoordinates(uint16_t *x, uint16_t *y);
bool resTouchGetManipulatedCoordinates(uint16_t *x, uint16_t *y);
void resTouchGetData(lv_indev_drv_t* drv, lv_indev_data_t* data);
uint16_t resTouchCoordinateRead(uint16_t* buffer, uint8_t Coordinate);

#endif /* XPT2046_TOUCH_H_ */
