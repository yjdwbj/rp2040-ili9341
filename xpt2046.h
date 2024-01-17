#ifndef XPT_2046_INC_H
#define XPT_2046_INC_H
#include "lvgl/lvgl.h"
#include "lv_conf.h"


void XPT2046_init(void);
void XPT2046_read(lv_indev_drv_t *drv, lv_indev_data_t *data);
#endif // XPT_2046_INC_H