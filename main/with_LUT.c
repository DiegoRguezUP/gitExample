// oneshot_read_main_lut.c
/*
 * Canal 5 -> % Humedad usando lookup table
 * Mide tiempo del acceso a la LUT y tiempo entre logs
 */
#include <stdio.h>
#include <math.h>
#include <inttypes.h>                 // para PRId32 / PRId64
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Incluir tu LUT generada por Python
#include "lookuptable.h"

#define ADC_ATTEN_USED        ADC_ATTEN_DB_12

// ---- Selección del canal físico para “canal 5” en ADC1 ----
#if CONFIG_IDF_TARGET_ESP32
  #define SENSOR_CH   ADC_CHANNEL_5   // ADC1_CH5 en ESP32
#else
  #define SENSOR_CH   ADC_CHANNEL_3   // ajusta si tu target difiere
#endif

static inline float humidity_from_lut(int mv)
{
    if (mv < 0) mv = 0;
    if (mv >= LUT_SIZE) mv = LUT_SIZE - 1;
    return (float)lookup_table[mv];
}

static bool adc_calib_init(adc_unit_t unit, adc_channel_t ch, adc_atten_t att, adc_cali_handle_t *out)
{
    adc_cali_handle_t handle = NULL; 
    esp_err_t ret = ESP_FAIL; 
    bool ok = false;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!ok) {
        adc_cali_curve_fitting_config_t cfg = { .unit_id=unit, .chan=ch, .atten=att, .bitwidth=ADC_BITWIDTH_DEFAULT };
        ret = adc_cali_create_scheme_curve_fitting(&cfg, &handle);
        if (ret == ESP_OK) ok = true;
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!ok) {
        adc_cali_line_fitting_config_t cfg = { .unit_id=unit, .atten=att, .bitwidth=ADC_BITWIDTH_DEFAULT };
        ret = adc_cali_create_scheme_line_fitting(&cfg, &handle);
        if (ret == ESP_OK) ok = true;
    }
#endif
    *out = handle; 
    return ok;
}

void app_main(void)
{
    // Unidad ADC1
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &adc1));

    // Config canal 5
    adc_oneshot_chan_cfg_t ccfg = { .atten = ADC_ATTEN_USED, .bitwidth = ADC_BITWIDTH_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, SENSOR_CH, &ccfg));

    // Calibración para obtener mV
    adc_cali_handle_t cali = NULL;
    bool have_cal = adc_calib_init(ADC_UNIT_1, SENSOR_CH, ADC_ATTEN_USED, &cali);

    int raw = 0, mv = 0;
    int64_t t_prev_log = esp_timer_get_time();

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1, SENSOR_CH, &raw));
        if (have_cal) ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali, raw, &mv));

        // --- medir solo el acceso LUT ---
        int64_t t0 = esp_timer_get_time();
        float rh = have_cal ? humidity_from_lut(mv) : 0.0f;
        int64_t t1 = esp_timer_get_time();
        int32_t t_calc_us = (int32_t)(t1 - t0);

        // --- tiempo entre impresiones ---
        int32_t t_interval_ms = (int32_t)((t1 - t_prev_log) / 1000);
        t_prev_log = t1;

        printf("RH: %.1f %% | LUT: %" PRId32 " us | interval: %" PRId32 " ms\n",
               have_cal ? rh : -1.0f, t_calc_us, t_interval_ms);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
