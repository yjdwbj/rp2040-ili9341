#ifndef XPT_2046_INC_H
#define XPT_2046_INC_H
#include "lvgl/lvgl.h"
#include "lv_conf.h"


void XPT2046_init(void);

#if LVGL_VERSION_MAJOR >= 9
void XPT2046_read(lv_indev_data_t *drv, lv_indev_data_t *data);
#else
void XPT2046_read(lv_indev_drv_t *drv, lv_indev_data_t *data);
#endif
#endif // XPT_2046_INC_H