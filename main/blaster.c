#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_cluster.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zcl/esp_zigbee_zcl_on_off.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "nwk/esp_zigbee_nwk.h"
#include "ir_driver.h"

#define BOOT_BUTTON_PIN         GPIO_NUM_9
#define BUTTON_HOLD_TIME_MS     3000

#define HA_ENDPOINT             1
#define ESP_MANUFACTURER_NAME   "ZigbeeHive"
#define ESP_MODEL_IDENTIFIER    "IR Blaster"

#define ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER 0xFF00

#define ESP_ZB_ZCL_ATTR_IR_SEND_ID    0x0001 // CSV string (chunked)
#define ESP_ZB_ZCL_ATTR_IR_LEARN_ID   0x0002
#define ESP_ZB_ZCL_ATTR_IR_RESULT_ID  0x0003 // Learned code CSV (chunked reports)

static const char *TAG = "IR_BLASTER";

// ZCL attribute buffers (first byte = ZCL string length)
static uint8_t ir_send_buffer[256]   = {0};
static uint8_t ir_result_buffer[256] = {0};
static bool    ir_learn_mode         = false;

static TaskHandle_t rx_task_handle = NULL;

#define IR_RECV_BUF_SIZE 4096
static char ir_recv_buf[IR_RECV_BUF_SIZE];
static int  ir_recv_total_chunks = 0;

#define REPORT_CHUNK_SIZE 60

static void report_learned_code(const char *csv) {
    size_t total_len = strlen(csv);
    if (total_len == 0) return;

    int total = (int)((total_len + REPORT_CHUNK_SIZE - 1) / REPORT_CHUNK_SIZE);

    for (int i = 0; i < total; i++) {
        char chunk_str[256];
        int prefix_len = snprintf(chunk_str, sizeof(chunk_str), "%d/%d:", i, total);
        int offset     = i * REPORT_CHUNK_SIZE;
        int data_len   = (int)total_len - offset;
        if (data_len > REPORT_CHUNK_SIZE) data_len = REPORT_CHUNK_SIZE;

        int str_len = prefix_len + data_len;
        if (str_len > 253) str_len = 253; // ZCL CharString max payload
        memcpy(chunk_str + prefix_len, csv + offset, str_len - prefix_len);
        chunk_str[str_len] = '\0';

        // Pack into ZCL string (length-prefixed)
        uint8_t zcl_str[256] = {0};
        zcl_str[0] = (uint8_t)str_len;
        memcpy(zcl_str + 1, chunk_str, str_len);

        esp_zb_lock_acquire(portMAX_DELAY);

        esp_zb_zcl_set_attribute_val(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_IR_RESULT_ID, zcl_str, false);

        esp_zb_zcl_report_attr_cmd_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.zcl_basic_cmd.src_endpoint          = HA_ENDPOINT;
        cmd.zcl_basic_cmd.dst_endpoint          = 1;
        cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
        cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        cmd.clusterID    = ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER;
        cmd.direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        cmd.attributeID  = ESP_ZB_ZCL_ATTR_IR_RESULT_ID;

        esp_err_t ret = esp_zb_zcl_report_attr_cmd_req(&cmd);
        ESP_LOGI(TAG, "Result chunk %d/%d: %s (0x%x)", i, total, esp_err_to_name(ret), ret);

        esp_zb_lock_release();

        vTaskDelay(pdMS_TO_TICKS(150)); // gap between chunks
    }
}

static void real_ir_rx_task(void *pvParameters) {
    ir_driver_start_learning();

    // Use a heap buffer — CSV can be several KB for a full IR signal
    char *decoded = malloc(IR_RECV_BUF_SIZE);
    if (!decoded) {
        ESP_LOGE(TAG, "OOM for decode buffer");
        rx_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (!ir_driver_poll_decode(decoded, IR_RECV_BUF_SIZE)) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (decoded[0] == '\0') {
            ESP_LOGW(TAG, "IR capture was empty, ignoring.");
            ir_driver_start_learning();
            free(decoded);
            decoded = malloc(IR_RECV_BUF_SIZE);
            if (!decoded) { rx_task_handle = NULL; vTaskDelete(NULL); return; }
            continue;
        }

        ESP_LOGI(TAG, "*** LEARNED: %d chars ***", (int)strlen(decoded));

        ir_driver_stop_learning();
        report_learned_code(decoded);
        free(decoded);

        // Turn off learn mode AFTER all result chunks are delivered
        esp_zb_lock_acquire(portMAX_DELAY);
        ir_learn_mode = false;
        esp_zb_zcl_set_attribute_val(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_IR_LEARN_ID, &ir_learn_mode, false);

        esp_zb_zcl_report_attr_cmd_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.zcl_basic_cmd.src_endpoint          = HA_ENDPOINT;
        cmd.zcl_basic_cmd.dst_endpoint          = 1;
        cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
        cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        cmd.clusterID    = ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER;
        cmd.direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        cmd.attributeID  = ESP_ZB_ZCL_ATTR_IR_LEARN_ID;
        esp_zb_zcl_report_attr_cmd_req(&cmd);
        esp_zb_lock_release();

        ESP_LOGI(TAG, "Learn complete. Exiting RX task.");
        rx_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
}

static void transmit_accumulated(void) {
    int32_t *durations = malloc(512 * sizeof(int32_t));
    if (!durations) { ESP_LOGE(TAG, "OOM for durations"); return; }
    int count = 0;
    char *cpy = strdup(ir_recv_buf);
    if (!cpy) { free(durations); return; }
    char *sp = NULL;
    for (char *tok = strtok_r(cpy, ",", &sp); tok && count < 512;
         tok = strtok_r(NULL, ",", &sp)) {
        durations[count++] = (int32_t)strtol(tok, NULL, 10);
    }
    free(cpy);
    if (count > 0) ir_driver_send_raw(durations, count);
    free(durations);
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) {
    if (!message) return ESP_FAIL;
    ESP_LOGI(TAG, "zb_attribute_handler: cluster=0x%04x attr=0x%04x",
             message->info.cluster, message->attribute.id);

    if (message->info.cluster != ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER) return ESP_OK;

    if (message->attribute.id == ESP_ZB_ZCL_ATTR_IR_SEND_ID) {
        uint8_t *zcl_string = (uint8_t *)message->attribute.data.value;
        if (!zcl_string || zcl_string[0] == 0) return ESP_OK;

        uint8_t len = zcl_string[0];
        char temp[256];
        memcpy(temp, zcl_string + 1, len);
        temp[len] = '\0';

        char *slash = strchr(temp, '/');
        char *colon = slash ? strchr(slash + 1, ':') : NULL;

        if (!slash || !colon) {
            // No chunk prefix: send directly (manual paste from HA)
            strncpy(ir_recv_buf, temp, IR_RECV_BUF_SIZE - 1);
            ir_recv_buf[IR_RECV_BUF_SIZE - 1] = '\0';
            transmit_accumulated();
            ir_send_buffer[0] = 0;
            esp_zb_zcl_set_attribute_val(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_IR_SEND_ID, ir_send_buffer, false);
            esp_zb_zcl_report_attr_cmd_t send_clear_cmd = {0};
            send_clear_cmd.zcl_basic_cmd.src_endpoint          = HA_ENDPOINT;
            send_clear_cmd.zcl_basic_cmd.dst_endpoint          = 1;
            send_clear_cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
            send_clear_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            send_clear_cmd.clusterID    = ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER;
            send_clear_cmd.direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
            send_clear_cmd.attributeID  = ESP_ZB_ZCL_ATTR_IR_SEND_ID;
            esp_zb_zcl_report_attr_cmd_req(&send_clear_cmd);
            return ESP_OK;
        }

        int n = atoi(temp);
        int t = atoi(slash + 1);
        char *data = colon + 1;
        size_t data_len = strlen(data);

        if (n == 0) {
            strncpy(ir_recv_buf, data, IR_RECV_BUF_SIZE - 1);
            ir_recv_buf[IR_RECV_BUF_SIZE - 1] = '\0';
            ir_recv_total_chunks = t;
        } else {
            size_t cur = strlen(ir_recv_buf);
            if (cur + 1 + data_len < IR_RECV_BUF_SIZE) {
                ir_recv_buf[cur] = ',';
                memcpy(ir_recv_buf + cur + 1, data, data_len);
                ir_recv_buf[cur + 1 + data_len] = '\0';
            }
        }

        if (n + 1 != t) {
            ESP_LOGI(TAG, "Chunk %d/%d received", n + 1, t);
            return ESP_OK;
        }

        ESP_LOGI(TAG, "All %d chunks received (%d chars). Transmitting...", t, (int)strlen(ir_recv_buf));
        transmit_accumulated();
        ir_send_buffer[0] = 0;
        esp_zb_zcl_set_attribute_val(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_IR_SEND_ID, ir_send_buffer, false);
        esp_zb_zcl_report_attr_cmd_t send_clear_cmd = {0};
        send_clear_cmd.zcl_basic_cmd.src_endpoint          = HA_ENDPOINT;
        send_clear_cmd.zcl_basic_cmd.dst_endpoint          = 1;
        send_clear_cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
        send_clear_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        send_clear_cmd.clusterID    = ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER;
        send_clear_cmd.direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        send_clear_cmd.attributeID  = ESP_ZB_ZCL_ATTR_IR_SEND_ID;
        esp_zb_zcl_report_attr_cmd_req(&send_clear_cmd);
        return ESP_OK;
    }

    if (message->attribute.id == ESP_ZB_ZCL_ATTR_IR_LEARN_ID) {
        bool learn_requested = *(bool *)message->attribute.data.value;
        ESP_LOGI(TAG, "IR Learn Mode toggled to: %d", learn_requested);

        if (learn_requested && rx_task_handle == NULL) {
            ir_learn_mode = true;
            xTaskCreate(real_ir_rx_task, "ir_rx", 8192, NULL, 10, &rx_task_handle);
        } else if (!learn_requested && rx_task_handle != NULL) {
            ir_learn_mode = false;
            ir_driver_stop_learning();
            vTaskDelete(rx_task_handle);
            rx_task_handle = NULL;
            ESP_LOGI(TAG, "IR Learn Mode aborted manually.");
        }
    }

    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        return zb_attribute_handler((const esp_zb_zcl_set_attr_value_message_t *)message);
    default:
        return ESP_OK;
    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee Stack Initialized.");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status != ESP_OK) {
            ESP_LOGW(TAG, "Stack Init Failed: %s", esp_err_to_name(err_status));
            return;
        }
        if (esp_zb_bdb_is_factory_new()) {
            ESP_LOGI(TAG, "Factory New - Steering...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGI(TAG, "Rebooted");
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Joined Network!");
        } else {
            ESP_LOGW(TAG, "Steering Failed, retrying...");
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        break;
    }
}

static void esp_zb_task(void *pvParameters) {
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
            .keep_alive = 3000,
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    // On/Off Cluster (keeps standard Zigbee networks happy)
    esp_zb_attribute_list_t *on_off_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
    bool on_off_value = false;
    esp_zb_on_off_cluster_add_attr(on_off_attr_list, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &on_off_value);
    esp_zb_cluster_list_add_on_off_cluster(cluster_list, on_off_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    ir_send_buffer[0]   = 254;
    ir_result_buffer[0] = 254;
    ir_learn_mode       = false;

    esp_zb_attribute_list_t *ir_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER);
    esp_zb_cluster_add_attr(ir_attr_list, ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER, ESP_ZB_ZCL_ATTR_IR_SEND_ID,
        ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, ir_send_buffer);
    esp_zb_cluster_add_attr(ir_attr_list, ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER, ESP_ZB_ZCL_ATTR_IR_LEARN_ID,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &ir_learn_mode);
    esp_zb_cluster_add_attr(ir_attr_list, ESP_ZB_ZCL_CLUSTER_ID_IR_BLASTER, ESP_ZB_ZCL_ATTR_IR_RESULT_ID,
        ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, ir_result_buffer);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, ir_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_basic_cluster_cfg_t basic_cfg = { .zcl_version = 0x02, .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_MAINS_SINGLE_PHASE };
    esp_zb_attribute_list_t *basic_attr_list = esp_zb_basic_cluster_create(&basic_cfg);
    esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)"\x0A" ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)"\x0A" ESP_MODEL_IDENTIFIER);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_identify_cluster_cfg_t identify_cfg = { .identify_time = 0 };
    esp_zb_attribute_list_t *identify_attr_list = esp_zb_identify_cluster_create(&identify_cfg);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);

    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_start(false);
    esp_zb_stack_main_loop();
}

static void reset_button_task(void *pvParameters) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    while (1) {
        if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
            int hold_count = 0;
            while (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                hold_count += 100;
                if (hold_count >= BUTTON_HOLD_TIME_MS) {
                    ESP_LOGW(TAG, "Factory Reset...");
                    esp_zb_factory_reset();
                    break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    esp_zb_platform_config_t config = {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    ir_driver_init();

    xTaskCreate(reset_button_task, "reset_button", 2048, NULL, 10, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
