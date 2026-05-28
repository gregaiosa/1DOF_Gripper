#ifndef HX711_H
#define HX711_H

#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t pd_sck;
    uint8_t dout;
    uint8_t gain;
    int32_t offset;
    float scale;
} hx711_t;

void hx711_init(hx711_t *hx, uint8_t dout, uint8_t pd_sck, uint8_t gain);
bool hx711_is_ready(hx711_t *hx);
void hx711_wait_ready(hx711_t *hx, uint32_t delay_ms);
bool hx711_wait_ready_retry(hx711_t *hx, int retries, uint32_t delay_ms);
bool hx711_wait_ready_timeout(hx711_t *hx, uint32_t timeout, uint32_t delay_ms);
void hx711_set_gain(hx711_t *hx, uint8_t gain);
int32_t hx711_read(hx711_t *hx);
int32_t hx711_read_average(hx711_t *hx, uint8_t times);
double hx711_get_value(hx711_t *hx, uint8_t times);
float hx711_get_units(hx711_t *hx, uint8_t times);
void hx711_tare(hx711_t *hx, uint8_t times);
void hx711_set_scale(hx711_t *hx, float scale);
float hx711_get_scale(hx711_t *hx);
void hx711_set_offset(hx711_t *hx, int32_t offset);
int32_t hx711_get_offset(hx711_t *hx);
void hx711_power_down(hx711_t *hx);
void hx711_power_up(hx711_t *hx);

#ifdef __cplusplus
}
#endif

#endif /* HX711_H */