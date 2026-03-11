#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_cluster.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zcl/esp_zigbee_zcl_on_off.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "nwk/esp_zigbee_nwk.h"
#include "freertos/semphr.h"

// Hardware Config
#define IR_TX_GPIO              GPIO_NUM_10
#define IR_RX_GPIO              GPIO_NUM_11
#define BOOT_BUTTON_PIN         GPIO_NUM_9
#define BUTTON_HOLD_TIME_MS     3000

// RMT Config
#define RMT_RESOLUTION_HZ       1000000 // 1MHz, 1 tick = 1us
#define RMT_RX_BUFFER_SIZE      2048    // High for long codes
#define RMT_TX_CARRIER_FREQ_HZ  38000   // 38kHz

// Zigbee Config
#define HA_ENDPOINT             1
#define ESP_MANUFACTURER_NAME   "ZigbeeHive"
#define ESP_MODEL_IDENTIFIER    "IR Blaster"

// Custom Attribute for IR Data (String)
#define ESP_ZB_ZCL_ATTR_IR_DATA_ID 0xFF00

static const char *TAG = "IR_BLASTER";

// Global Handles
static rmt_channel_handle_t rx_channel = NULL;
static rmt_channel_handle_t tx_channel = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;

// Buffer for IR Data Attribute
static char ir_data_buffer[256] = {0};
static bool g_learning_mode = false;

static char g_latest_ir_cmd[256] = {0};

// Synchronization & Thread Safety
static SemaphoreHandle_t g_cmd_mutex = NULL;
static SemaphoreHandle_t g_tx_mutex = NULL;       // Protects RMT TX driver
static SemaphoreHandle_t g_ir_learned_sem = NULL; // Signals when 1st command is learned

// --- IR RMT Functions ---

static void init_ir_tx() {
    ESP_LOGI(TAG, "Initializing IR TX on GPIO %d", IR_TX_GPIO);
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = IR_TX_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    ESP_LOGI(TAG, "Configuring RMT TX Channel...");
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_channel));

    rmt_carrier_config_t carrier_config = {
        .duty_cycle = 0.33,
        .frequency_hz = RMT_TX_CARRIER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_config));

    rmt_copy_encoder_config_t encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &copy_encoder));

    ESP_ERROR_CHECK(rmt_enable(tx_channel));
}

static void init_ir_rx() {
    ESP_LOGI(TAG, "Initializing IR RX on GPIO %d", IR_RX_GPIO);
    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = IR_RX_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
    };
    ESP_LOGI(TAG, "Configuring RMT RX Channel...");
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_channel));

    ESP_ERROR_CHECK(rmt_enable(rx_channel));
}

// Simple RLE parsing: "100,-100,200,-200" -> RMT symbols
static esp_err_t transmit_ir_string(const char* data) {
    if (!data || data[0] == '\0') {
        return ESP_FAIL;
    }

    // Parse tokens into a temporary int buffer
    int32_t durations[256] = {0};
    int token_count = 0;

    char *datacpy = strdup(data);
    if (!datacpy) {
        return ESP_ERR_NO_MEM;
    }
    char *saveptr = NULL;
    char *token = strtok_r(datacpy, ",", &saveptr);
    while (token && token_count < (int)(sizeof(durations) / sizeof(durations[0]))) {
        durations[token_count++] = (int32_t)strtol(token, NULL, 10);
        token = strtok_r(NULL, ",", &saveptr);
    }
    free(datacpy);

    if (token_count == 0) {
        return ESP_FAIL;
    }

    // Pair tokens into RMT symbols (two durations per symbol)
    size_t symbol_count = (token_count + 1) / 2; // round up
    rmt_symbol_word_t *items = calloc(symbol_count, sizeof(rmt_symbol_word_t));
    if (!items) {
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < symbol_count; i++) {
        int32_t first = durations[i * 2];
        int32_t second = (i * 2 + 1 < token_count) ? durations[i * 2 + 1] : 0;

        items[i].level0 = (first >= 0) ? 1 : 0;
        items[i].duration0 = (uint32_t)abs(first);
        items[i].level1 = (second >= 0) ? 1 : 0;
        items[i].duration1 = (uint32_t)abs(second);
    }

    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
    };

    esp_err_t err = ESP_OK;

    // Mutex to prevent concurrent transmissions crashing the RMT driver
    if (g_tx_mutex) xSemaphoreTake(g_tx_mutex, portMAX_DELAY);

    ESP_LOGI(TAG, "Transmitting IR (%d symbols)...", (int)symbol_count);
    err = rmt_transmit(tx_channel, copy_encoder, items, symbol_count * sizeof(rmt_symbol_word_t), &transmit_config);
    if (err == ESP_OK) {
        err = rmt_tx_wait_all_done(tx_channel, -1);
    }

    if (g_tx_mutex) xSemaphoreGive(g_tx_mutex);

    free(items);
    return err;
}

static QueueHandle_t rx_queue = NULL;

static bool rx_done_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    xQueueSendFromISR(rx_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static void real_ir_rx_task(void *pvParameters) {
    rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rx_done_callback,
    };
    rmt_rx_register_event_callbacks(rx_channel, &cbs, NULL);
    
    rmt_symbol_word_t *buffer = malloc(RMT_RX_BUFFER_SIZE * sizeof(rmt_symbol_word_t));
    
    // Accept a wide but sane pulse range for IR (0.4us - 12ms)
    rmt_receive_config_t rx_config = {
        .signal_range_min_ns = 400,
        .signal_range_max_ns = 12000000,
    };

    while(1) {
        rmt_receive(rx_channel, buffer, RMT_RX_BUFFER_SIZE * sizeof(rmt_symbol_word_t), &rx_config);
        
        rmt_rx_done_event_data_t event_data;
        if (xQueueReceive(rx_queue, &event_data, portMAX_DELAY)) {
            // Process Data
            ESP_LOGI(TAG, "IR Received! %d symbols", event_data.num_symbols);
            
            // Encode to String using local buffer
            char local_buffer[256];
            int len = 0;
            local_buffer[0] = '\0';
            
            for(size_t i=0; i<event_data.num_symbols; i++) {
                // Each RMT symbol holds two durations; flatten both
                int vals[2];
                vals[0] = (buffer[i].level0 == 0) ? (int)buffer[i].duration0 : -((int)buffer[i].duration0);
                vals[1] = (buffer[i].level1 == 0) ? (int)buffer[i].duration1 : -((int)buffer[i].duration1);

                for (int j = 0; j < 2; j++) {
                    if (vals[j] == 0) continue; 
                    if (len >= (int)sizeof(local_buffer) - 20) break;
                    
                    int written = snprintf(local_buffer + len, sizeof(local_buffer) - len, "%s%d", (len == 0 ? "" : ","), vals[j]);
                    if (written > 0) len += written;
                }
            }
            
            if (len == 0) {
                rmt_symbol_word_t *sym = buffer;
                ESP_LOGW(TAG, "IR capture was empty, ignoring (symbols=%d)", event_data.num_symbols);
                continue; // Wait for a valid signal
            }

            ESP_LOGI(TAG, "\n\n*** LEARNED COMMAND ***\n%s\n***********************\n", local_buffer);

            if (g_cmd_mutex) {
                xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
                strncpy(g_latest_ir_cmd, local_buffer, sizeof(g_latest_ir_cmd) - 1);
                g_latest_ir_cmd[sizeof(g_latest_ir_cmd) - 1] = '\0';
                xSemaphoreGive(g_cmd_mutex);
            }

            // Update Zigbee Attribute
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_set_attribute_val(
                HA_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_IR_DATA_ID,
                local_buffer,
                false
            );
            esp_zb_lock_release();

            // Notify the TX task that a command is learned
            if (g_ir_learned_sem) {
                xSemaphoreGive(g_ir_learned_sem);
            }

            // Shut down RX to prevent it from running alongside TX
            ESP_LOGI(TAG, "First IR command learned. Stopping RX and exiting listen task.");
            rmt_disable(rx_channel);
            free(buffer);
            vQueueDelete(rx_queue);
            vTaskDelete(NULL); // Terminate this task completely
        }
    }
}

// --- Zigbee Functions ---

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) {
    if (!message) return ESP_FAIL;
    
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF && 
        message->attribute.id == ESP_ZB_ZCL_ATTR_IR_DATA_ID) {
        
        ESP_LOGI(TAG, "IR Data Attribute Written!");
        
        uint8_t *zcl_string = (uint8_t*)message->attribute.data.value;
        if (zcl_string) {
            uint8_t len = ir_data_buffer[0];
            if (len > 0) {
                char temp[256];
                memcpy(temp, ir_data_buffer + 1, len);
                temp[len] = '\0';
                ESP_LOGI(TAG, "Transmitting via Zigbee Trigger: %s", temp);
                transmit_ir_string(temp);
            }
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
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO Signal: 0x%x", sig_type);
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
    
    // Create Attribute List for On/Off Cluster
    esp_zb_attribute_list_t *on_off_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
    
    // Standard On/Off Attribute
    bool on_off_value = false;
    esp_zb_on_off_cluster_add_attr(on_off_attr_list, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &on_off_value);
    
    // Initialize buffer with length 0
    ir_data_buffer[0] = 0; 
    
    esp_zb_cluster_add_attr(
        on_off_attr_list,
        ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        ESP_ZB_ZCL_ATTR_IR_DATA_ID,
        ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        ir_data_buffer
    );

    // Create Cluster List
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_on_off_cluster(cluster_list, on_off_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    // Basic Cluster
    esp_zb_basic_cluster_cfg_t basic_cfg = { .zcl_version = 0x02, .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_MAINS_SINGLE_PHASE };
    esp_zb_attribute_list_t *basic_attr_list = esp_zb_basic_cluster_create(&basic_cfg);
    esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)"\x0A" ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)"\x0D" ESP_MODEL_IDENTIFIER);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Identify Cluster
    esp_zb_identify_cluster_cfg_t identify_cfg = { .identify_time = 0 };
    esp_zb_attribute_list_t *identify_attr_list = esp_zb_identify_cluster_create(&identify_cfg);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Endpoint
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);

    esp_zb_device_register(ep_list);
    
    // Register Action Handler for Attribute Writes
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
            bool reset_triggered = false;
            while (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                hold_count += 100;
                if (hold_count >= BUTTON_HOLD_TIME_MS) {
                    ESP_LOGW(TAG, "Factory Reset...");
                    esp_zb_factory_reset(); 
                    reset_triggered = true;
                    break;
                }
            }
            
            // Short press detected
            if (!reset_triggered && hold_count > 50) { 
                g_learning_mode = !g_learning_mode;
                if (g_learning_mode) {
                    ESP_LOGI(TAG, "Learning Mode: ENABLED");
                } else {
                    ESP_LOGI(TAG, "Learning Mode: DISABLED");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

static void ir_test_task(void *arg) {
    ESP_LOGI(TAG, "IR TX Task waiting for first command to be learned...");
    
    // Block indefinitely until the RX task signals us
    xSemaphoreTake(g_ir_learned_sem, portMAX_DELAY);

    char cmd_to_send[256] = {0};

    // Safely copy the newly learned command
    if (g_cmd_mutex) {
        xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
        strncpy(cmd_to_send, g_latest_ir_cmd, sizeof(cmd_to_send) - 1);
        cmd_to_send[sizeof(cmd_to_send) - 1] = '\0';
        xSemaphoreGive(g_cmd_mutex);
    }

    ESP_LOGI(TAG, "IR TX Task starting 1-second pulse loop...");

    while (1) {
        if (strlen(cmd_to_send) > 0) {
            transmit_ir_string(cmd_to_send);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1-second pulse interval
    }
}

void app_main(void) {
    esp_zb_platform_config_t config = {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // Initialize synchronization primitives
    g_cmd_mutex = xSemaphoreCreateMutex();
    g_tx_mutex = xSemaphoreCreateMutex();
    g_ir_learned_sem = xSemaphoreCreateBinary();

    if (g_cmd_mutex == NULL || g_tx_mutex == NULL || g_ir_learned_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create Semaphores/Mutexes!");
        return;
    }

    // Init IR
    init_ir_tx();
    init_ir_rx();

    // Start Tasks
    xTaskCreate(reset_button_task, "reset_button", 2048, NULL, 10, NULL);
    xTaskCreate(real_ir_rx_task, "ir_rx", 4096, NULL, 10, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    xTaskCreate(ir_test_task, "ir_test", 4096, NULL, 10, NULL);
}