/**
 */

#include "esp_log.h"
#include "esp32-rmt-ir.h"
#include "mqtt_client.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "led_strip.h"      // RGB LED via RMT (no driver/gpio.h needed)

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>

static const char *TAG = "irsender";

static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

static esp_mqtt_client_handle_t mqtt_client;

static char topic_rc5[96];
static char topic_nec_hisense[96];
static char topic_nec_pleio[96];
static char topic_nec_ugreen[96];

static void build_topics(void)
{
    snprintf(topic_rc5, sizeof(topic_rc5), "irsender/%s/rc5", CONFIG_IRSENDER_DEVICE_ID);
    snprintf(topic_nec_hisense, sizeof(topic_nec_hisense), "irsender/%s/nec/hisense", CONFIG_IRSENDER_DEVICE_ID);
    snprintf(topic_nec_pleio, sizeof(topic_nec_pleio), "irsender/%s/nec/pleio", CONFIG_IRSENDER_DEVICE_ID);
    snprintf(topic_nec_ugreen, sizeof(topic_nec_ugreen), "irsender/%s/nec/ugreen", CONFIG_IRSENDER_DEVICE_ID);
}


/* ===================== RGB Status LED (SK6812) ===================== */

#define STATUS_LED_GPIO CONFIG_IRSENDER_STATUS_LED_GPIO

typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_MQTT_CONNECTING,
    LED_MODE_MQTT_CONNECTED
} led_mode_t;

static led_strip_handle_t s_led_strip = NULL;
static volatile led_mode_t s_led_mode = LED_MODE_OFF;

static void configureLed(led_strip_handle_t* led_strip)
{
    const led_strip_config_t strip_config = {
        .strip_gpio_num = STATUS_LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_SK6812,
        .color_component_format = { .format_id = 0 },
        .flags = { .invert_out = 0 }
    };
    const led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_APB,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 0,
        .flags = { .with_dma = 0 }
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, led_strip));
}

static void switchOffLed(led_strip_handle_t led_strip)
{
    if (!led_strip) return;
    led_strip_clear(led_strip);
}

static void set_led_connecting(led_strip_handle_t led_strip, bool on)
{
    if (!led_strip) return;
    if (on) {
        // Amber blink
        led_strip_set_pixel(led_strip, 0, 32, 16, 0);
        led_strip_refresh(led_strip);
    } else {
        led_strip_clear(led_strip);
    }
}

static void set_led_connected(led_strip_handle_t led_strip)
{
    if (!led_strip) return;
    // Solid green
    led_strip_set_pixel(led_strip, 0, 0, 32, 0);
    led_strip_refresh(led_strip);
}

static void status_led_task(void *arg)
{
    bool blink_on = false;

    while (1) {
        led_mode_t mode = s_led_mode;

        switch (mode) {
        case LED_MODE_OFF:
            switchOffLed(s_led_strip);
            vTaskDelay(pdMS_TO_TICKS(250));
            break;

        case LED_MODE_MQTT_CONNECTING:
            blink_on = !blink_on;
            set_led_connecting(s_led_strip, blink_on);
            vTaskDelay(pdMS_TO_TICKS(blink_on ? 200 : 800));
            break;

        case LED_MODE_MQTT_CONNECTED:
            set_led_connected(s_led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
            break;

        default:
            switchOffLed(s_led_strip);
            vTaskDelay(pdMS_TO_TICKS(250));
            break;
        }
    }
}

/* ===================== IR command tables ===================== */

typedef struct {
    const char *name;
    const char *label;
    uint16_t    code;     // 13-bit Raw-Data from sniffer (field+toggle+addr+cmd)
} rc5_command_t;

typedef struct {
    const char *name;
    const char *label;
    uint32_t    code;     // 32-bit NEC frame (MSB-first)
} nec_command_t;

static const rc5_command_t rc5_commands[] = {
    { "Back",      "TV, Back",            0x184A },
    { "Blue",      "TV, Blue",            0x1874 },
    { "Channel-",  "TV, Channel Down",    0x1861 },
    { "Channel+",  "TV, Channel Up",      0x1860 },
    { "0",         "TV, Digit 0",         0x1840 },
    { "1",         "TV, Digit 1",         0x1841 },
    { "2",         "TV, Digit 2",         0x1842 },
    { "3",         "TV, Digit 3",         0x1843 },
    { "4",         "TV, Digit 4",         0x1844 },
    { "5",         "TV, Digit 5",         0x1845 },
    { "6",         "TV, Digit 6",         0x1846 },
    { "7",         "TV, Digit 7",         0x1847 },
    { "8",         "TV, Digit 8",         0x1848 },
    { "9",         "TV, Digit 9",         0x1849 },
    { "Disney",    "TV, Disney +",        0x184B },
    { "Down",      "TV, Down",            0x1853 },
    { "Exit",      "TV, Exit",            0x1865 },
    { "Green",     "TV, Green",           0x1876 },
    { "Guide",     "TV, Guide / EPG",     0x186F },
    { "Info",      "TV, Info",            0x1852 },
    { "Left",      "TV, Left",            0x1855 },
    { "Home",      "TV, Menu",            0x1870 },
    { "Mute",      "TV, Mute",            0x184D },
    { "Netflix",   "TV, Netflix",         0x1867 },
    { "Ok",        "TV, Ok",              0x1875 },
    { "Pause",     "TV, Play/Pause",      0x1859 },
    { "Play",      "TV, Play/Pause",      0x1871 },
    { "Standby",   "TV, Standby",         0x184C },
    { "Red",       "TV, Red",             0x1877 },
    { "Right",     "TV, Right",           0x1856 },
    { "Settings",  "TV, Settings",        0x184E },
    { "Source",    "TV, Source",          0x1878 },
    { "Up",        "TV, Up",              0x1854 },
    { "Vol-",      "TV, Volume Down",     0x1851 },
    { "Vol+",      "TV, Volume Up",       0x1850 },
    { "Yellow",    "TV, Yellow",          0x1872 },
    { "Youtube",   "TV, Youtube",         0x1869 }
};

static const size_t rc5_commands_count =
        sizeof(rc5_commands) / sizeof(rc5_commands[0]);

static const nec_command_t nec_commands_hisense[] = {
    { "Back",      "HISENSE, Back",              0xFD12ED },
    { "Blue",      "HISENSE, Blue",              0xFDAA55 },
    { "Channel-",  "HISENSE, Channel Down",      0xFDD22D },
    { "Channel+",  "HISENSE, Channel Up",        0xFD52AD },
    { "0",         "HISENSE, Digit 0",           0xFD00FF },
    { "1",         "HISENSE, Digit 1",           0xFD807F },
    { "2",         "HISENSE, Digit 2",           0xFD40BF },
    { "3",         "HISENSE, Digit 3",           0xFDC03F },
    { "4",         "HISENSE, Digit 4",           0xFD20DF },
    { "5",         "HISENSE, Digit 5",           0xFDA05F },
    { "6",         "HISENSE, Digit 6",           0xFD609F },
    { "7",         "HISENSE, Digit 7",           0xFDE01F },
    { "8",         "HISENSE, Digit 8",           0xFD10EF },
    { "9",         "HISENSE, Digit 9",           0xFD906F },
    { "Disney",    "HISENSE, Disney +",          0xFD56A9 },
    { "Down",      "HISENSE, Down",              0xFDE817 },
    { "Exit",      "HISENSE, Exit",              0xFD3AC5 },
    { "Freely",    "HISENSE, Freely",            0x5DF20D },
    { "Green",     "HISENSE, Green",             0xFDCA35 },
    { "Guide",     "HISENSE, Guide / EPG",       0xFD5CA3 },
    { "Info",      "HISENSE, Info",              0xFD30CF },
    { "Left",      "HISENSE, Left",              0xFD9867 },
    { "Home",      "HISENSE, Menu",              0xFD04FB },
    { "Mute",      "HISENSE, Mute",              0xFD708F },
    { "Netflix",   "HISENSE, Netflix",           0xFDB44B },
    { "Ok",        "HISENSE, Ok",                0xFDA857 },
    { "Pause",     "HISENSE, Play/Pause",        0xFD53AC },
    { "Play",      "HISENSE, Play/Pause",        0xFD53AC },
    { "Standby",   "HISENSE, Standby",           0xFDB04F },
    { "Red",       "HISENSE, Red",               0xFD4AB5 },
    { "Right",     "HISENSE, Right",             0xFD18E7 },
    { "Settings",  "HISENSE, Settings",          0xFD28D7 },
    { "Source",    "HISENSE, Source",            0xFD48B7 },
    { "Up",        "HISENSE, Up",                0xFD6897 },
    { "Vol-",      "HISENSE, Volume Down",       0xFDC23D },
    { "Vol+",      "HISENSE, Volume Up",         0xFD22DD },
    { "Yellow",    "HISENSE, Yellow",            0xFD2AD5 },
    { "Youtube",   "HISENSE, Youtube",           0xFD55AA }
};

static const size_t nec_commands_hisense_count =
        sizeof(nec_commands_hisense) / sizeof(nec_commands_hisense[0]);

static const nec_command_t nec_commands_pleio[] = {
    { "Back",      "PLEIO, Back",               0x11EE12ED },
    { "Blue",      "PLEIO, Blue",               0x11EE32CD },
    { "Channel-",  "PLEIO, Channel Down",       0x11EE2CD3 },
    { "Channel+",  "PLEIO, Channel Up",         0x11EECC33 },
    { "0",         "PLEIO, Digit 0",            0x11EE50AF },
    { "1",         "PLEIO, Digit 1",            0x11EE807F },
    { "2",         "PLEIO, Digit 2",            0x11EE40BF },
    { "3",         "PLEIO, Digit 3",            0x11EEC03F },
    { "4",         "PLEIO, Digit 4",            0x11EE20DF },
    { "5",         "PLEIO, Digit 5",            0x11EEA05F },
    { "6",         "PLEIO, Digit 6",            0x11EE609F },
    { "7",         "PLEIO, Digit 7",            0x11EEE01F },
    { "8",         "PLEIO, Digit 8",            0x11EE10EF },
    { "9",         "PLEIO, Digit 9",            0x11EE906F },
    { "Down",      "PLEIO, Down",               0x11EE6897 },
    { "Freely",    "PLEIO, Freely",             0x11EEC639 },
    { "Green",     "PLEIO, Green",              0x11EE52AD },
    { "Guide",     "PLEIO, Guide / EPG",        0x11EE4CB3 },
    { "Info",      "PLEIO, Info",               0x11EE946B },
    { "Left",      "PLEIO, Left",               0x11EEE817 },
    { "Home",      "PLEIO, Menu",               0x11EEE21D },
    { "Mute",      "PLEIO, Mute",               0x11EEA45B },
    { "Netflix",   "PLEIO, Netflix",            0x11EE26D9 },
    { "Ok",        "PLEIO, Ok",                 0x11EE9867 },
    { "Standby",   "PLEIO, Standby",            0x11EE847B },
    { "Red",       "PLEIO, Red",                0x11EED22D },
    { "Right",     "PLEIO, Right",              0x11EE18E7 },
    { "Settings",  "PLEIO, Settings",           0x11EEF00F },
    { "Source",    "PLEIO, Source",             0x11EEEA15 },
    { "Up",        "PLEIO, Up",                 0x11EEA857 },
    { "Vol-",      "PLEIO, Volume Down",        0x11EE24DB },
    { "Vol+",      "PLEIO, Volume Up",          0x11EEC43B },
    { "Yellow",    "PLEIO, Yellow",             0x11EE926D }
};

static const size_t nec_commands_pleio_count =
        sizeof(nec_commands_pleio) / sizeof(nec_commands_pleio[0]);

static const nec_command_t nec_commands_ugreen[] = {
    { "1",         "UGREEN, Digit 1",           0x1FE40BF },
    { "2",         "UGREEN, Digit 2",           0x1FE20DF },
    { "3",         "UGREEN, Digit 3",           0x1FE609F },
    { "Next",      "UGREEN, Next",              0x1FE10EF }
};

static const size_t nec_commands_ugreen_count =
        sizeof(nec_commands_ugreen) / sizeof(nec_commands_ugreen[0]);

/* ===================== RC5 toggle handling ===================== */

// Global toggle state for RC5: flips on "new press"
static bool rc5_toggle = false;

// Track last send to detect duplicates/repeats
static uint16_t rc5_last_raw13 = 0;
static int64_t  rc5_last_send_us = 0;

static void sendRC5(const char* text, uint16_t raw13)
{
    // Time window used to decide whether this is a "new press"
    // or a repeat / duplicate of the same command (e.g. MQTT resend).
    const int64_t REPEAT_WINDOW_US = 300000; // 300 ms
    int64_t now = esp_timer_get_time();

    // Check whether the incoming command is the same as the last one sent
    bool is_same_command = (raw13 == rc5_last_raw13);

    // Check whether it arrived quickly after the last transmission
    bool within_repeat_window =
        (now - rc5_last_send_us) < REPEAT_WINDOW_US;

    // Treat same command within the repeat window as a "repeat"
    // (do NOT flip RC5 toggle bit for repeats)
    bool quick_repeat = is_same_command && within_repeat_window;

    // RC5 toggle bit flips ONLY on a new button press.
    // Repeats/duplicates must keep the same toggle value.
    if (!quick_repeat) {
        rc5_toggle = !rc5_toggle;
    }

    // Save state for next call
    rc5_last_raw13 = raw13;
    rc5_last_send_us = now;

    // raw13 contains address + command bits, but we do NOT trust
    // whatever value it has for the RC5 toggle bit (bit 11).
    //
    // Clear bit 11 first so we can explicitly control it.
    uint16_t frame13 = raw13 & ~(1u << 11);

    // If the current press requires toggle=1, set bit 11.
    // Bitwise OR sets this bit without affecting any others.
    if (rc5_toggle) {
        frame13 |= (1u << 11);
    }

    // RC5 frames are 14 bits total.
    // Bit 13 is the RC5 start bit and must always be 1.
    //
    // (1u << 13) sets the start bit.
    // Masking with 0x1FFF ensures only bits 0–12 from frame13 are used.
    uint16_t cmd14 =
        (1u << 13) |        // RC5 start bit (bit 13)
        (frame13 & 0x1FFF); // lower 13 bits: field, toggle, address, command

    sendIR(RC5, cmd14, 14);
}


static void sendNEC(const char *text, uint32_t code)
{
    ESP_LOGI(TAG, "TX NEC: %s as 0x%08" PRIx32, text, code);
    sendIR(NEC, code, 32);
}

static void handle_rc5_command(const char *payload, int payload_len)
{
    for (size_t i = 0; i < rc5_commands_count; i++) {
        const rc5_command_t *cmd = &rc5_commands[i];
        size_t name_len = strlen(cmd->name);

        if (payload_len == (int)name_len &&
            strncmp(payload, cmd->name, name_len) == 0)
        {
            sendRC5(cmd->label, cmd->code);
            return;
        }
    }
    ESP_LOGW(TAG, "Unknown RC5 command: %.*s", payload_len, payload);
}

static void handle_nec_command(const nec_command_t *table,
                               size_t count,
                               const char *payload,
                               int payload_len)
{
    for (size_t i = 0; i < count; i++) {
        const nec_command_t *cmd = &table[i];
        size_t name_len = strlen(cmd->name);

        if (payload_len == (int)name_len &&
            strncmp(payload, cmd->name, name_len) == 0)
        {
            sendNEC(cmd->label, cmd->code);
            return;
        }
    }
    ESP_LOGW(TAG, "Unknown NEC command: %.*s", payload_len, payload);
}

/* ===================== Wi-Fi ===================== */

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(const char* ssid, const char* pass)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_wifi_event_group = xEventGroupCreate();
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ===================== MQTT ===================== */

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event->event_id) {

    case MQTT_EVENT_BEFORE_CONNECT:
        s_led_mode = LED_MODE_MQTT_CONNECTING;
        break;

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to broker, subscribing...");
        s_led_mode = LED_MODE_MQTT_CONNECTED;

        esp_mqtt_client_subscribe(event->client, topic_rc5, 0);
        esp_mqtt_client_subscribe(event->client, topic_nec_hisense, 0);
        esp_mqtt_client_subscribe(event->client, topic_nec_pleio, 0);
        esp_mqtt_client_subscribe(event->client, topic_nec_ugreen, 0);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Disconnected from broker, trying again...");
        s_led_mode = LED_MODE_MQTT_CONNECTING;
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGW(TAG, "MQTT error");
        s_led_mode = LED_MODE_MQTT_CONNECTING;
        break;

    case MQTT_EVENT_DATA: {
        const char *topic = event->topic;
        int tlen = event->topic_len;

        if (tlen == (int)strlen(topic_rc5) &&
            strncmp(topic, topic_rc5, tlen) == 0)
        {
            handle_rc5_command(event->data, event->data_len);
        }
        else if (tlen == (int)strlen(topic_nec_hisense) &&
                 strncmp(topic, topic_nec_hisense, tlen) == 0)
        {
            handle_nec_command(nec_commands_hisense, nec_commands_hisense_count,
                               event->data, event->data_len);
        }
        else if (tlen == (int)strlen(topic_nec_pleio) &&
                 strncmp(topic, topic_nec_pleio, tlen) == 0)
        {
            handle_nec_command(nec_commands_pleio, nec_commands_pleio_count,
                               event->data, event->data_len);
        }
        else if (tlen == (int)strlen(topic_nec_ugreen) &&
                 strncmp(topic, topic_nec_ugreen, tlen) == 0)
        {
            handle_nec_command(nec_commands_ugreen, nec_commands_ugreen_count,
                               event->data, event->data_len);
        }
        else {
            ESP_LOGW(TAG, "Unknown topic: %.*s", tlen, topic);
        }

        break;
    }

    default:
        break;
    }
}

static void start_mqtt_tls(void)
{
    esp_mqtt_client_config_t cfg = {};

    cfg.broker.address.uri = CONFIG_IRSENDER_MQTT_URI;

    if (strlen(CONFIG_IRSENDER_MQTT_USERNAME) > 0) {
        cfg.credentials.username = CONFIG_IRSENDER_MQTT_USERNAME;
    }
    if (strlen(CONFIG_IRSENDER_MQTT_PASSWORD) > 0) {
        cfg.credentials.authentication.password = CONFIG_IRSENDER_MQTT_PASSWORD;
    }

    mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);

    // As soon as MQTT starts, show “connecting” until CONNECTED arrives
    s_led_mode = LED_MODE_MQTT_CONNECTING;

    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

/* ===================== app_main ===================== */

extern "C" void app_main(void)
{
    irTxPin = CONFIG_IRSENDER_IR_TX_GPIO;

    // Configure RGB status LED and start its task
    configureLed(&s_led_strip);
    switchOffLed(s_led_strip);
    xTaskCreate(status_led_task, "status_led", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Starting Wi-Fi...");
    build_topics();
    wifi_init_sta(CONFIG_IRSENDER_WIFI_SSID, CONFIG_IRSENDER_WIFI_PASSWORD);

    ESP_LOGI(TAG, "Starting MQTT (TLS) IR sender...");
    start_mqtt_tls();
}
