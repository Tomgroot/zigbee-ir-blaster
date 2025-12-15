#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
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

// Hardware Config
#define IR_RX_GPIO              GPIO_NUM_10
#define IR_TX_GPIO              GPIO_NUM_11
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
// Using a custom attribute ID in the On/Off Cluster
#define ESP_ZB_ZCL_ATTR_IR_DATA_ID 0xFF00

static const char *TAG = "IR_BLASTER";

// Global Handles
static rmt_channel_handle_t rx_channel = NULL;
static rmt_channel_handle_t tx_channel = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;

// Buffer for IR Data Attribute
static char ir_data_buffer[256] = {0};

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
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_channel));

    ESP_ERROR_CHECK(rmt_enable(rx_channel));
}

// RMT Item Helper
// Casting to specific struct to avoid union issues with different IDF versions
typedef struct {
    uint32_t duration : 15;
    uint32_t level : 1;
    uint32_t : 16;
} rmt_item_legacy_t;

// ...

// Simple RLE parsing: "100,-100,200,-200" -> RMT symbols
static esp_err_t transmit_ir_string(const char* data) {
    if (!data || strlen(data) == 0) return ESP_FAIL;

    // Estimate count
    int count = 1;
    for(const char* p = data; *p; p++) {
        if(*p == ',') count++;
    }

    rmt_symbol_word_t* items = malloc(sizeof(rmt_symbol_word_t) * count);
    if (!items) return ESP_ERR_NO_MEM;
    
    // Treat as legacy items for easy access
    rmt_item_legacy_t* legacy_items = (rmt_item_legacy_t*)items;

    char* datacpy = strdup(data);
    char* token = strtok(datacpy, ",");
    int i = 0;
    
    while(token && i < count) {
        int val = atoi(token);
        if (val > 0) {
            // Mark (Pulse)
            legacy_items[i].duration = val;
            legacy_items[i].level = 1; 
        } else {
            // Space (Gap)
            legacy_items[i].duration = -val; // Make positive for duration
            legacy_items[i].level = 0;
        }
        token = strtok(NULL, ",");
        i++;
    }
    
    free(datacpy);

    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
    };
    
    ESP_LOGI(TAG, "Transmitting IR (%d symbols)...", i);
    ESP_ERROR_CHECK(rmt_transmit(tx_channel, copy_encoder, items, i * sizeof(rmt_symbol_word_t), &transmit_config));
    
    free(items);
    return ESP_OK;
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
    rmt_item_legacy_t *legacy_buffer = (rmt_item_legacy_t*)buffer;
    
    rmt_receive_config_t rx_config = {
        .signal_range_min_ns = 1250,
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
                // Limit buffer size
                if (len >= sizeof(local_buffer) - 20) break; 
                
                int val = legacy_buffer[i].duration;
                if (legacy_buffer[i].level == 0) val = -val; // Space
                
                int written = snprintf(local_buffer + len, sizeof(local_buffer) - len, "%s%d", (i==0?"":","), val);
                if (written > 0) len += written;
            }
            
            ESP_LOGI(TAG, "Encoded IR: %s", local_buffer);

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
        }
    }
}

// --- Zigbee Functions ---

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) {
    if (!message) return ESP_FAIL;
    
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF && 
        message->attribute.id == ESP_ZB_ZCL_ATTR_IR_DATA_ID) {
        
        ESP_LOGI(TAG, "IR Data Attribute Written!");
        
        // info type fix: message->info is esp_zb_device_cb_common_info_t
        // But for set_attr_value, it might be different?
        // Let's just access message->info directly.
        
        uint8_t *zcl_string = (uint8_t*)message->attribute.data.value;
        if (zcl_string) {
            uint8_t len = ir_data_buffer[0];
            // Removed always-true check if buffer size > 255
            if (len > 0) {
                char temp[256];
                memcpy(temp, ir_data_buffer + 1, len);
                temp[len] = '\0';
                ESP_LOGI(TAG, "Transmitting: %s", temp);
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
    
    // Custom Attribute: IR Data (Char String)
    // Value is pointer to our buffer. Buffer first byte = Length.
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
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID, // Use On/Off Switch device ID
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
            while (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                hold_count += 100;
                if (hold_count >= BUTTON_HOLD_TIME_MS) {
                    ESP_LOGW(TAG, "Factory Reset...");
                    esp_zb_factory_reset(); 
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

    // Init IR
    init_ir_tx();
    init_ir_rx();

    // Start Tasks
    xTaskCreate(reset_button_task, "reset_button", 2048, NULL, 10, NULL);
    xTaskCreate(real_ir_rx_task, "ir_rx", 4096, NULL, 10, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
