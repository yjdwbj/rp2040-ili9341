#include "lv_line_chart.h"

void lv_basic_line_chart(void) {
    /*Create a chart*/
    lv_obj_t *chart;
    chart = lv_chart_create(lv_scr_act());
    lv_obj_set_size(chart, 200, 150);
    lv_obj_center(chart);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE); /*Show lines and points too*/

    /*Add two data series*/
    lv_chart_series_t *ser1 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_series_t *ser2 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_SECONDARY_Y);

    uint32_t i;
    for (i = 0; i < 10; i++) {
        /*Set the next points on 'ser1'*/
        lv_chart_set_next_value(chart, ser1, lv_rand(10, 50));

        /*Directly set points on 'ser2'*/
        ser2->y_points[i] = lv_rand(50, 90);
    }

    lv_chart_refresh(chart); /*Required after direct set*/
}
