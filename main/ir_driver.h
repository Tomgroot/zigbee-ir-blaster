#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void      ir_driver_init(void);
esp_err_t ir_driver_send_raw(const int32_t *durations, size_t count); // signed CSV array
void      ir_driver_start_learning(void);
void      ir_driver_stop_learning(void);
bool      ir_driver_poll_decode(char *output, size_t maxlen);    // fills signed CSV

#ifdef __cplusplus
}
#endif
