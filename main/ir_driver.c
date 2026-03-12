/**
 * ir_driver.c
 *
 * Self-contained IR TX/RX driver using ESP-IDF RMT.
 * Unknown signals are encoded as signed CSV (microseconds).
 *
 * Sign convention:
 *   Positive duration → LOW  (space, carrier off) — as received from inverted IR sensor
 *   Negative duration → HIGH (mark, carrier on)
 *
 * ir_driver_send_raw() interprets: positive → HIGH (re-inverts for transmission)
 */

#include "ir_driver.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "IR_DRIVER";

#define IR_TX_GPIO  10
#define IR_RX_GPIO  11

#define RMT_RESOLUTION_HZ  1000000U
#define RX_BUFFER_SYMBOLS  2048

static rmt_channel_handle_t g_tx_channel   = NULL;
static rmt_channel_handle_t g_rx_channel   = NULL;
static rmt_encoder_handle_t g_copy_encoder = NULL;

static QueueHandle_t       g_rx_queue   = NULL;
static rmt_symbol_word_t  *g_rx_buffer  = NULL;

static bool IRAM_ATTR rx_done_callback(rmt_channel_handle_t chan,
                                        const rmt_rx_done_event_data_t *edata,
                                        void *user_data) {
    (void)chan; (void)user_data;
    BaseType_t wake = pdFALSE;
    xQueueSendFromISR(g_rx_queue, edata, &wake);
    return wake == pdTRUE;
}

static esp_err_t transmit(const rmt_symbol_word_t *symbols, size_t count) {
    rmt_transmit_config_t cfg = {.loop_count = 0};
    esp_err_t err = rmt_transmit(g_tx_channel, g_copy_encoder,
                                  symbols, count * sizeof(rmt_symbol_word_t), &cfg);
    if (err == ESP_OK) err = rmt_tx_wait_all_done(g_tx_channel, portMAX_DELAY);
    return err;
}

static void arm_rx(void) {
    rmt_receive_config_t cfg = {
        .signal_range_min_ns = 400,
        .signal_range_max_ns = 12000000,
    };
    rmt_receive(g_rx_channel, g_rx_buffer,
                RX_BUFFER_SYMBOLS * sizeof(rmt_symbol_word_t), &cfg);
}

void ir_driver_init(void) {
    rmt_tx_channel_config_t tx_cfg = {0};
    tx_cfg.gpio_num          = IR_TX_GPIO;
    tx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz     = RMT_RESOLUTION_HZ;
    tx_cfg.mem_block_symbols = 64;
    tx_cfg.trans_queue_depth = 4;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &g_tx_channel));

    rmt_carrier_config_t carrier = {0};
    carrier.frequency_hz = 38000;
    carrier.duty_cycle   = 0.33f;
    ESP_ERROR_CHECK(rmt_apply_carrier(g_tx_channel, &carrier));

    rmt_copy_encoder_config_t enc_cfg;
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &g_copy_encoder));
    ESP_ERROR_CHECK(rmt_enable(g_tx_channel));

    rmt_rx_channel_config_t rx_cfg = {0};
    rx_cfg.gpio_num          = IR_RX_GPIO;
    rx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    rx_cfg.resolution_hz     = RMT_RESOLUTION_HZ;
    rx_cfg.mem_block_symbols = 64;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &g_rx_channel));

    ESP_LOGI(TAG, "IR driver initialized (TX GPIO%d, RX GPIO%d)", IR_TX_GPIO, IR_RX_GPIO);
}

esp_err_t ir_driver_send_raw(const int32_t *durations, size_t count) {
    if (count == 0 || !g_tx_channel) return ESP_FAIL;

    size_t sym_count = (count + 1) / 2;
    rmt_symbol_word_t *items = (rmt_symbol_word_t *)calloc(sym_count, sizeof(rmt_symbol_word_t));
    if (!items) return ESP_ERR_NO_MEM;

    for (size_t i = 0; i < sym_count; i++) {
        int32_t a = durations[i * 2];
        int32_t b = (i * 2 + 1 < (int)count) ? durations[i * 2 + 1] : 0;
        items[i].level0    = (a >= 0) ? 1u : 0u;
        items[i].duration0 = (uint32_t)(a >= 0 ? a : -a);
        items[i].level1    = (b >= 0) ? 1u : 0u;
        items[i].duration1 = (uint32_t)(b >= 0 ? b : -b);
    }

    ESP_LOGI(TAG, "Sending raw IR (%d symbols)", (int)sym_count);
    esp_err_t err = transmit(items, sym_count);
    free(items);
    return err;
}

void ir_driver_start_learning(void) {
    g_rx_buffer = (rmt_symbol_word_t *)malloc(RX_BUFFER_SYMBOLS * sizeof(rmt_symbol_word_t));
    if (!g_rx_buffer) { ESP_LOGE(TAG, "OOM for RX buffer"); return; }

    g_rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = rx_done_callback;
    rmt_rx_register_event_callbacks(g_rx_channel, &cbs, NULL);

    rmt_enable(g_rx_channel);
    arm_rx();
    ESP_LOGI(TAG, "IR RX started, waiting for signal...");
}

void ir_driver_stop_learning(void) {
    rmt_disable(g_rx_channel);
    if (g_rx_queue)  { vQueueDelete(g_rx_queue);  g_rx_queue  = NULL; }
    if (g_rx_buffer) { free(g_rx_buffer);          g_rx_buffer = NULL; }
    ESP_LOGI(TAG, "IR RX stopped");
}

bool ir_driver_poll_decode(char *output, size_t maxlen) {
    if (!g_rx_queue || !g_rx_buffer) return false;

    rmt_rx_done_event_data_t ev;
    if (xQueueReceive(g_rx_queue, &ev, 0) != pdTRUE) return false;

    if (ev.num_symbols == 0) {
        arm_rx();
        return false;
    }

    ESP_LOGI(TAG, "IR received: %d symbols", (int)ev.num_symbols);

    int len = 0;
    output[0] = '\0';
    for (size_t i = 0; i < ev.num_symbols; i++) {
        int vals[2];
        vals[0] = (ev.received_symbols[i].level0 == 0)
            ? (int)ev.received_symbols[i].duration0
            : -((int)ev.received_symbols[i].duration0);
        vals[1] = (ev.received_symbols[i].level1 == 0)
            ? (int)ev.received_symbols[i].duration1
            : -((int)ev.received_symbols[i].duration1);
        for (int j = 0; j < 2; j++) {
            if (vals[j] == 0) continue;
            if (len >= (int)maxlen - 10) break;
            int written = snprintf(output + len, maxlen - len,
                                   "%s%d", (len == 0 ? "" : ","), vals[j]);
            if (written > 0) len += written;
        }
    }
    return true;
}
