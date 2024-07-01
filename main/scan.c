/* 
    Projektarbeit Linus Norden 2024
    Adaption der Espressif iBeacon Demo
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_ibeacon_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// Lower Level access für UART
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

// Serielle Schnittstellen definieren
#define BOARD2BOARD_TXD 3
#define BOARD2BOARD_RXD 4
#define BOARD2BOARD_RTS (UART_PIN_NO_CHANGE) // -1, ungenutzt
#define BOARD2BOARD_CTS (UART_PIN_NO_CHANGE) // -1, ungenutzt

#define BOARD2BOARD_PORT_NUM 1 //esp32c3 bietet Ports 0-2 an
#define BOARD2BOARD_BAUD_RATE 115200 
#define BOARD2BOARD_STACK_SIZE 2048 // UART-Stack Größe, mehr siehe: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html
#define BOARD2BOARD_BUF_SIZE 1024 // Zwischenspeicher für Nachrichten


uart_config_t uart_config = {
    .baud_rate = BOARD2BOARD_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
    .source_clk = UART_SCLK_DEFAULT
};
int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

static const char* APP_TAG = "SENSOR_SCANNER";
// Das hier konfigurierte Scan-Board kann anhand der eingestellten UUID explizit ausschließlich nach Geräten suchen, die dem eigenen Pool entstammen
static const uint8_t OWNED_UUID[] = {0x61,0x6e,0x50,0xac,0x4a,0xfa,0x4e,0x41,0xba,0x2a,0x8c,0x82,0xf3,0xe6,0x56,0xca};
extern esp_ble_ibeacon_vendor_t vendor_config;

// Statische Funktionen deklarieren
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
// BLE Scan Einstellungen
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

// BT event handler
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        // Einheit sind Sekunden, 0 -> permanent scannen
        uint32_t duration = 0;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        // Indikator für Scan-Start erfolgreich oder fehlerhaft
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(APP_TAG, "Scan start failed: %s", esp_err_to_name(err));
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        /* Wenn es BLE-Scanergebnisse gibt, müssen einige Werte aus dem advertising data Paket geholt werden.
           - die Mac-Adresse als eindeutige Kennung
           - Buttonstatus, falls vorhanden
           - Batteriestand in Prozent, falls angegeben
           - RSSI als Entfernungsindikator
        */
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
            case ESP_GAP_SEARCH_INQ_RES_EVT:
                // dieses Ereignis wird ausgelöst, wenn z.B. zusätzliche Werbedaten gefunden wurden 
                // Suche nach BLE iBeacon-Paket
                if (esp_ble_is_ibeacon_packet(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len)){
                    esp_ble_ibeacon_t *ibeacon_data = (esp_ble_ibeacon_t*)(scan_result->scan_rst.ble_adv);
                    // Für Debugging: Namen lesen
//					uint8_t *adv_name = NULL;
//					uint8_t adv_name_len = 0;
                    uint8_t *adv_service_data = NULL;
                    uint8_t adv_service_data_len = 0;
//                    adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
//					if (adv_name ) ESP_LOGI(APP_TAG,"adv_name: %s", adv_name );
                    /* Wenn die UUID übereinstimmt, wurde ein zugehöriger Sensor identifiziert. Gegebene Parameter für MAC-Adresse, Tastenzustand, 
                    RSSI und Batteriestand müssen über SoftwareSerial an das WLAN-Board übertragen werden.
                    */
                    if ( memcmp(OWNED_UUID, ibeacon_data->ibeacon_vendor.proximity_uuid, ESP_UUID_LEN_128) == 0){
                        uint8_t serial_data[9]; // MAC + Batt + Button + RSSI
                        // Hinzufügen der MAC-Adresse zu den seriellen Daten - Schritt 1: Kopieren der uint_8-Daten in den char- Buffer, Schritt 2: Verkettung
                        adv_service_data = esp_ble_resolve_adv_data(param->scan_rst.ble_adv, ESP_BLE_AD_TYPE_SERVICE_DATA, &adv_service_data_len);
                        memcpy(serial_data, scan_result->scan_rst.bda, 6);
                        if (adv_service_data ) {
//                            ESP_LOGI(APP_TAG, "Battery: %d %%", adv_service_data[3]);
                            serial_data[6] = adv_service_data[3];
//                            ESP_LOGI(APP_TAG, "Button pressed: %d", adv_service_data[13]);
                            serial_data[7] = adv_service_data[13];
//                            ESP_LOGI(APP_TAG, "RSSI of packet:%d dbm", scan_result->scan_rst.rssi);
                            serial_data[8] = scan_result->scan_rst.rssi;
                            esp_log_buffer_hex("Write data: ", serial_data, 9 );
                            
                            // Daten schreiben und auf das Senden des Pakets warten
                            // Wegen möglichen Problemen mit dem Timing könnte dies eventuell besser mit einer asynchronen Methode übertragen werden.
                            uart_write_bytes(BOARD2BOARD_PORT_NUM, (const char *)serial_data, 9);
                            ESP_ERROR_CHECK(uart_wait_tx_done(BOARD2BOARD_PORT_NUM, 100)); // Timeout sind 100 RTOS ticks (TickType_t)
                        }
                        // Im 'else'-Fall fehlen advertising-Daten
                        // Es braucht einen temporären Buffer für eingehende Daten
                        // uint8_t *data_in = (uint8_t *) malloc(BOARD2BOARD_BUF_SIZE);
                        // Daten aus dem UART lesen
                        // int len = uart_read_bytes(BOARD2BOARD_PORT_NUM, data_in, (BOARD2BOARD_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
                        // if (len) {
                        //    esp_log_buffer_hex("IBEACON_DEMO: read data:", data_in, len);
                        // }                    
                    }
                }
                break;
            default:
             ESP_LOGI(APP_TAG, "Search event type: %d", scan_result->scan_rst.search_evt);
                break;
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(APP_TAG, "Scan stop failed: %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI(APP_TAG, "Stop scan successfully");
        }
        break;

    default:
        break;
    }
}


void ble_ibeacon_appRegister(void)
{
    esp_err_t status;
    ESP_LOGI(APP_TAG, "register callback");
    // Registriere callback-Funktion zum gap-Modul
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(APP_TAG, "gap register error: %s", esp_err_to_name(status));
        return;
    }
}

void ble_ibeacon_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    ble_ibeacon_appRegister();
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    
    // UART Treiber installieren
    ESP_ERROR_CHECK( uart_driver_install(BOARD2BOARD_PORT_NUM, BOARD2BOARD_BUF_SIZE *2,0, 0, NULL, intr_alloc_flags) ); // no event queue defined (NULL)
    ESP_ERROR_CHECK(uart_param_config(BOARD2BOARD_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BOARD2BOARD_PORT_NUM, BOARD2BOARD_TXD, BOARD2BOARD_RXD, BOARD2BOARD_RTS, BOARD2BOARD_CTS));

    ble_ibeacon_init();
    /* Scan Parameter setzen */
    esp_ble_gap_set_scan_params(&ble_scan_params);

}
