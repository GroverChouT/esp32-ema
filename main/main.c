#include <string.h>

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "mqtt_client.h"

#define ESP_WIFI_SSID               CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASSWORD           CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY           CONFIG_ESP_MAXIMUM_RETRY
#define REMOTE_SERVICE_UUID         0xFFE0
#define REMOTE_CHAR_NOTIFY_UUID     0xFFE1
#define PROFILE_NUM                 1
#define PROFILE_A_APP_ID            0
#define INVALID_HANDLE              0
#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_FAIL_BIT               BIT1

static const char *TAG = "EMA";
static const char *TOPIC = "yitong/energy_meter/home";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static esp_mqtt_client_handle_t mqtt_client;
static bool mqtt_connected = false;
static const char remote_device_name[] = "UD18-BLE";
static bool ble_connected = false;
static bool ble_service_found = false;
static esp_gattc_char_elem_t *char_elem_result = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                        esp_ble_gattc_cb_param_t *param);

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_notify_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_CHAR_NOTIFY_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

struct meter_data_t {
    uint8_t device_type;
    float voltage;
    float current;
    float power;
    float energy;
    float unit_price;
    float cumulative_price;
    float frequency;
    float power_factor;
    int temperature;
    uint8_t back_light_mode;
};

static struct meter_data_t meter_data;

uint16_t p16tou16(const uint8_t *p16) {
    return p16[0] << 8 | p16[1];
}

uint32_t p24tou32(const uint8_t *p24) {
    return p24[0] << 16 | p24[1] << 8 | p24[2];
}

uint32_t p32tou32(const uint8_t *p32) {
    return p32[0] << 24 | p32[1] << 16 | p32[2] << 8 | p32[3];
}

void print_meter_data(struct meter_data_t *data) {
    ESP_LOGI(TAG,
             "\n\tVoltage: %.2fV\n\tCurrent: %.2fA\n\tPower: %.2fW\n\tEnergy: %.2fkWh\n\tUnit Price: %.2f"
             "\n\tCumulative Price: %.2f\n\tFrequency: %.2f\n\tPower Factor: %.2f\n\tTemperature: %d°C",
             data->voltage, data->current, data->power, data->energy, data->unit_price,
             data->cumulative_price, data->frequency, data->power_factor, data->temperature);
}

void process_notify_data(const uint8_t *data, const uint16_t len) {
    static uint8_t buf[36];
    if (len == 20 && data[0] == 0xFF && data[2] == 0x01) {
        memcpy(buf, data, len);
    } else if (len == 16) {
        memcpy(&buf[20], data, len);

        uint8_t checksum = buf[2];
        for (int i = 3; i < 35; i++) {
            checksum += buf[i];
        }
        if (((checksum & 0xFF) ^ 0x44) != buf[35]) return;

        meter_data.device_type = buf[3];
        meter_data.voltage = (float) p24tou32(&buf[4]) / 10;
        meter_data.current = (float) p24tou32(&buf[7]) / 1000;
        meter_data.power = (float) p24tou32(&buf[10]) / 10;
        meter_data.energy = (float) p32tou32(&buf[13]) / 100;
        meter_data.unit_price = (float) p24tou32(&buf[17]) / 100;
        meter_data.cumulative_price = meter_data.energy * meter_data.unit_price;
        meter_data.frequency = (float) p16tou16(&buf[20]) / 10;
        meter_data.power_factor = (float) p16tou16(&buf[22]) / 1000;
        meter_data.temperature = p16tou16(&buf[24]);
        print_meter_data(&meter_data);
    }
}

static void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                        esp_ble_gattc_cb_param_t *param) {
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *) param;

    switch (event) {
        case ESP_GATTC_REG_EVT:
            ESP_LOGI(TAG, "ESP_GATTC_REG_EVT");
            esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
            if (scan_ret) {
                ESP_LOGE(TAG, "set scan params error, error code = %x", scan_ret);
            }
            break;
        case ESP_GATTC_CONNECT_EVT: {
            ESP_LOGI(TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
            gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
            memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
            ESP_LOGI(TAG, "REMOTE BDA:");
                esp_log_buffer_hex(TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
            esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->connect.conn_id);
            if (mtu_ret) {
                ESP_LOGE(TAG, "config MTU error, error code = %x", mtu_ret);
            }
            break;
        }
        case ESP_GATTC_OPEN_EVT:
            if (param->open.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "open failed, status %d", p_data->open.status);
                break;
            }
            ESP_LOGI(TAG, "open success");
            break;
        case ESP_GATTC_DIS_SRVC_CMPL_EVT:
            if (param->dis_srvc_cmpl.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
                break;
            }
            ESP_LOGI(TAG, "discover service complete conn_id %d", param->dis_srvc_cmpl.conn_id);
            esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id,
                                         &remote_filter_service_uuid);
            break;
        case ESP_GATTC_CFG_MTU_EVT:
            if (param->cfg_mtu.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "config mtu failed, error status = %x", param->cfg_mtu.status);
            }
            ESP_LOGI(TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d",
                     param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
            break;
        case ESP_GATTC_SEARCH_RES_EVT: {
            ESP_LOGI(TAG, "SEARCH RES: conn_id = %x is primary service %d",
                     p_data->search_res.conn_id,
                     p_data->search_res.is_primary);
            ESP_LOGI(TAG, "start handle %d end handle %d current handle value %d",
                     p_data->search_res.start_handle, p_data->search_res.end_handle,
                     p_data->search_res.srvc_id.inst_id);
            if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
                p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
                ESP_LOGI(TAG, "ble service found");
                ble_service_found = true;
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
                ESP_LOGI(TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
            }
            break;
        }
        case ESP_GATTC_SEARCH_CMPL_EVT:
            if (p_data->search_cmpl.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
                break;
            }
            if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
                ESP_LOGI(TAG, "Get service information from remote device");
            } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
                ESP_LOGI(TAG, "Get service information from flash");
            } else {
                ESP_LOGI(TAG, "unknown service source");
            }
            ESP_LOGI(TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
            if (ble_service_found) {
                uint16_t count = 0;
                esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
                    gattc_if,
                    p_data->search_cmpl.conn_id,
                    ESP_GATT_DB_CHARACTERISTIC,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    INVALID_HANDLE,
                    &count);
                if (status != ESP_GATT_OK) {
                    ESP_LOGE(TAG, "esp_ble_gattc_get_attr_count error");
                }

                if (count > 0) {
                    char_elem_result = (esp_gattc_char_elem_t *) malloc(sizeof(esp_gattc_char_elem_t) * count);
                    if (!char_elem_result) {
                        ESP_LOGE(TAG, "gattc no mem");
                    } else {
                        status = esp_ble_gattc_get_char_by_uuid(
                            gattc_if,
                            p_data->search_cmpl.conn_id,
                            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                            remote_filter_char_notify_uuid,
                            char_elem_result,
                            &count);
                        if (status != ESP_GATT_OK) {
                            ESP_LOGE(TAG, "esp_ble_gattc_get_char_by_uuid error");
                        }

                        /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                        if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)) {
                            gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                            esp_ble_gattc_register_for_notify(gattc_if,
                                                              gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                                                              char_elem_result[0].char_handle);
                        }
                    }
                    /* free char_elem_result */
                    free(char_elem_result);
                } else {
                    ESP_LOGE(TAG, "no char found");
                }
            }
            break;
        case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
            ESP_LOGI(TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
            if (p_data->reg_for_notify.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
            } else {
                uint16_t count = 0;
                uint16_t notify_en = 1;
                esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(
                    gattc_if,
                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                    ESP_GATT_DB_DESCRIPTOR,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                    &count);
                if (ret_status != ESP_GATT_OK) {
                    ESP_LOGE(TAG, "esp_ble_gattc_get_attr_count error");
                }
                if (count > 0) {
                    descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                    if (!descr_elem_result) {
                        ESP_LOGE(TAG, "malloc error, gattc no mem");
                    } else {
                        ret_status = esp_ble_gattc_get_descr_by_char_handle(
                            gattc_if,
                            gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                            p_data->reg_for_notify.handle,
                            notify_descr_uuid,
                            descr_elem_result,
                            &count);
                        if (ret_status != ESP_GATT_OK) {
                            ESP_LOGE(TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                        }
                        /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                        if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 &&
                            descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
                            ret_status = esp_ble_gattc_write_char_descr(
                                gattc_if,
                                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                descr_elem_result[0].handle,
                                sizeof(notify_en),
                                (uint8_t *) &notify_en,
                                ESP_GATT_WRITE_TYPE_RSP,
                                ESP_GATT_AUTH_REQ_NONE);
                        }

                        if (ret_status != ESP_GATT_OK) {
                            ESP_LOGE(TAG, "esp_ble_gattc_write_char_descr error");
                        }

                        /* free descr_elem_result */
                        free(descr_elem_result);
                    }
                } else {
                    ESP_LOGE(TAG, "decsr not found");
                }

            }
            break;
        }
        case ESP_GATTC_NOTIFY_EVT:
            ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive %s value:",
                     p_data->notify.is_notify ? "notify" : "indicate");
            ESP_LOG_BUFFER_HEX(TAG, p_data->notify.value, p_data->notify.value_len);
            if (p_data->notify.is_notify) {
                process_notify_data(p_data->notify.value, p_data->notify.value_len);
            }
            break;
        case ESP_GATTC_WRITE_DESCR_EVT:
            if (p_data->write.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "write descr failed, error status = %x", p_data->write.status);
                break;
            }
            ESP_LOGI(TAG, "write descr success ");
            uint8_t write_char_data[35];
            for (int i = 0; i < sizeof(write_char_data); ++i) {
                write_char_data[i] = i % 256;
            }
            esp_ble_gattc_write_char(gattc_if,
                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                     gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                     sizeof(write_char_data),
                                     write_char_data,
                                     ESP_GATT_WRITE_TYPE_RSP,
                                     ESP_GATT_AUTH_REQ_NONE);
            break;
        case ESP_GATTC_SRVC_CHG_EVT: {
            esp_bd_addr_t bda;
            memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
            ESP_LOGI(TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
                esp_log_buffer_hex(TAG, bda, sizeof(esp_bd_addr_t));
            break;
        }
        case ESP_GATTC_WRITE_CHAR_EVT:
            if (p_data->write.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "write char failed, error status = %x", p_data->write.status);
                break;
            }
            ESP_LOGI(TAG, "write char success ");
            break;
        case ESP_GATTC_DISCONNECT_EVT:
            ble_connected = false;
            ble_service_found = false;
            ESP_LOGI(TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
            break;
        default:
            break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:;
            uint32_t duration = 30;
            esp_ble_gap_start_scanning(duration);
            break;
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
                break;
            }
            ESP_LOGI(TAG, "scan start success");
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT:;
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *) param;
            switch (scan_result->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT:
                        esp_log_buffer_hex(TAG, scan_result->scan_rst.bda, 6);
                    ESP_LOGI(TAG, "searched Adv Data Len %d, Scan Response Len %d",
                             scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
                    adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                        ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
                    ESP_LOGI(TAG, "searched Device Name Len %d", adv_name_len);
                            esp_log_buffer_char(TAG, adv_name, adv_name_len);
                    ESP_LOGI(TAG, "\n");
                    if (adv_name != NULL) {
                        if (strlen(remote_device_name) == adv_name_len &&
                            strncmp((char *) adv_name, remote_device_name, adv_name_len) == 0) {
                            ESP_LOGI(TAG, "searched device %s\n", remote_device_name);
                            if (ble_connected == false) {
                                ble_connected = true;
                                ESP_LOGI(TAG, "connect to the remote device.");
                                esp_ble_gap_stop_scanning();
                                esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                                   scan_result->scan_rst.bda,
                                                   scan_result->scan_rst.ble_addr_type,
                                                   true);
                            }
                        }
                    }
                    break;
                case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                default:
                    break;
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
                break;
            }
            ESP_LOGI(TAG, "stop scan successfully");
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
                break;
            }
            ESP_LOGI(TAG, "stop adv successfully");
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG,
                     "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                     param->update_conn_params.status,
                     param->update_conn_params.min_int,
                     param->update_conn_params.max_int,
                     param->update_conn_params.conn_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE ||
                /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

static void ble_init() {
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret) {
        ESP_LOGE(TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret) {
        ESP_LOGE(TAG, "set local MTU failed, error code = %x", local_mtu_ret);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t) event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            mqtt_connected = false;
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",
                                     event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

static void mqtt_init(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASSWORD);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

void mqtt_publish_task(void *param) {
    char buf[256];

    while (1) {
        if (mqtt_connected && ble_service_found) {
            snprintf(buf, 256,
                     "{\"voltage\":%.2f,\"current\":%.2f,\"power\":%.2f,\"energy\":%.2f,\"unit_price\":%.2f,"
                     "\"cumulative_price\":%.2f,\"frequency\":%.2f,\"power_factor\":%.2f,\"temperature\":%d}",
                     meter_data.voltage, meter_data.current, meter_data.power, meter_data.energy, meter_data.unit_price,
                     meter_data.cumulative_price, meter_data.frequency, meter_data.power_factor, meter_data.temperature);
            int msg_id = esp_mqtt_client_publish(mqtt_client, TOPIC, buf, 0, 0, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_LOGI(TAG, "Startup...");
    ESP_LOGI(TAG, "Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();
    mqtt_init();
    ble_init();
    xTaskCreatePinnedToCore(&mqtt_publish_task, "mqtt_publish_task", 4096,
                            NULL, 5, NULL, 1);
}
