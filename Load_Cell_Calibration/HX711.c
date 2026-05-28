#include "HX711.h"

void hx711_init(hx711_t *hx, uint8_t dout, uint8_t pd_sck, uint8_t gain) {
    hx->pd_sck = pd_sck;
    hx->dout = dout;
    hx->scale = 1.0f;
    hx->offset = 0;

    gpio_init(hx->pd_sck);
    gpio_set_dir(hx->pd_sck, GPIO_OUT);
    gpio_put(hx->pd_sck, 0);

    gpio_init(hx->dout);
    gpio_set_dir(hx->dout, GPIO_IN);

    hx711_set_gain(hx, gain);
}

bool hx711_is_ready(hx711_t *hx) {
    return gpio_get(hx->dout) == 0;
}

void hx711_set_gain(hx711_t *hx, uint8_t gain) {
    switch (gain) {
        case 128:
            hx->gain = 1;
            break;
        case 64:
            hx->gain = 3;
            break;
        case 32:
            hx->gain = 2;
            break;
        default:
            hx->gain = 1; // default to 128
            break;
    }
}

int32_t hx711_read(hx711_t *hx) {
    // Wait for the chip to become ready
    hx711_wait_ready(hx, 0);

    uint32_t value = 0;
    uint8_t data[3] = {0};
    uint8_t filler = 0x00;

    // Pulse the clock pin 24 times to read the data
    for (int8_t i = 2; i >= 0; i--) {
        for (int8_t j = 7; j >= 0; j--) {
            gpio_put(hx->pd_sck, 1);
            sleep_us(1); // At least 0.2us delay
            uint8_t bit = gpio_get(hx->dout);
            data[i] |= (bit << j);
            gpio_put(hx->pd_sck, 0);
            sleep_us(1); // At least 0.2us delay
        }
    }

    // Set the channel and the gain factor for the next reading using the clock pin
    for (uint8_t i = 0; i < hx->gain; i++) {
        gpio_put(hx->pd_sck, 1);
        sleep_us(1);
        gpio_put(hx->pd_sck, 0);
        sleep_us(1);
    }

    // Replicate the most significant bit to pad out a 32-bit signed integer
    if (data[2] & 0x80) {
        filler = 0xFF;
    } else {
        filler = 0x00;
    }

    // Construct a 32-bit signed integer
    value = ( (uint32_t)filler << 24
            | (uint32_t)data[2] << 16
            | (uint32_t)data[1] << 8
            | (uint32_t)data[0] );

    return (int32_t)value;
}

void hx711_wait_ready(hx711_t *hx, uint32_t delay_ms) {
    while (!hx711_is_ready(hx)) {
        sleep_ms(delay_ms);
    }
}

bool hx711_wait_ready_retry(hx711_t *hx, int retries, uint32_t delay_ms) {
    int count = 0;
    while (count < retries) {
        if (hx711_is_ready(hx)) {
            return true;
        }
        sleep_ms(delay_ms);
        count++;
    }
    return false;
}

bool hx711_wait_ready_timeout(hx711_t *hx, uint32_t timeout, uint32_t delay_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start < timeout) {
        if (hx711_is_ready(hx)) {
            return true;
        }
        sleep_ms(delay_ms);
    }
    return false;
}

int32_t hx711_read_average(hx711_t *hx, uint8_t times) {
    int64_t sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        sum += hx711_read(hx);
        sleep_ms(0); // Yield
    }
    return sum / times;
}

double hx711_get_value(hx711_t *hx, uint8_t times) {
    return hx711_read_average(hx, times) - hx->offset;
}

float hx711_get_units(hx711_t *hx, uint8_t times) {
    return hx711_get_value(hx, times) / hx->scale;
}

void hx711_tare(hx711_t *hx, uint8_t times) {
    double sum = hx711_read_average(hx, times);
    hx711_set_offset(hx, sum);
}

void hx711_set_scale(hx711_t *hx, float scale) {
    hx->scale = scale;
}

float hx711_get_scale(hx711_t *hx) {
    return hx->scale;
}

void hx711_set_offset(hx711_t *hx, int32_t offset) {
    hx->offset = offset;
}

int32_t hx711_get_offset(hx711_t *hx) {
    return hx->offset;
}

void hx711_power_down(hx711_t *hx) {
    gpio_put(hx->pd_sck, 0);
    sleep_us(1);
    gpio_put(hx->pd_sck, 1);
    sleep_us(60); // High for >60us powers down
}

void hx711_power_up(hx711_t *hx) {
    gpio_put(hx->pd_sck, 0);
}
