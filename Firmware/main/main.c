#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <ctype.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_err.h"

#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_sntp.h"

#include "esp_http_server.h"

#include "lwip/ip4_addr.h"

static const char *TAG = "IV3_CLOCK";

#ifndef MIN
#define MIN(a,b) (( (a) < (b) ) ? (a) : (b))
#endif

#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW  0
#endif

/* ------------------------------------------------------------
   Pin mapping ESP32-S3
   ------------------------------------------------------------ */
#define PIN_GRID0    2   // A0
#define PIN_GRID1    1   // A1
#define PIN_GRID2    7   // A2
#define PIN_GRID3    6   // A3
#define PIN_SEG_A   18   // D2
#define PIN_SEG_B   17   // D3
#define PIN_SEG_C   19   // D4
#define PIN_SEG_D   20   // D5
#define PIN_SEG_E    3   // D6
#define PIN_SEG_F   14   // D7
#define PIN_SEG_G   21   // D8
#define PIN_DOT     46   // D9
#define PIN_LEDS    10   // D10

/* ------------------------------------------------------------
    Digit segment 
    Order: [A,B,C,D,E,F,G]
   ------------------------------------------------------------ */
static const uint8_t digit_seg_data[11][7] = {
    {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,  LOW},  // 0
    { LOW, HIGH, HIGH,  LOW,  LOW,  LOW,  LOW},  // 1
    {HIGH, HIGH,  LOW, HIGH, HIGH,  LOW, HIGH},  // 2
    {HIGH, HIGH, HIGH, HIGH,  LOW,  LOW, HIGH},  // 3
    { LOW, HIGH, HIGH,  LOW,  LOW, HIGH, HIGH},  // 4
    {HIGH,  LOW, HIGH, HIGH,  LOW, HIGH, HIGH},  // 5
    {HIGH,  LOW, HIGH, HIGH, HIGH, HIGH, HIGH},  // 6
    {HIGH, HIGH, HIGH,  LOW,  LOW,  LOW,  LOW},  // 7
    {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH},  // 8
    {HIGH, HIGH, HIGH, HIGH,  LOW, HIGH, HIGH},  // 9
    { LOW,  LOW,  LOW,  LOW,  LOW,  LOW, HIGH}   // Hyphen
};

typedef struct {
    uint8_t digit;  // 0..9, 10=hyphen
    uint8_t dot;    // HIGH/LOW
} TUBE;

/* Display state (shared mit ISR) */
static volatile TUBE tube_list[4] = {
    {10, LOW},
    {10, LOW},
    {10, LOW},
    {10, LOW},
};

/* ISR-Vars */
static volatile uint8_t cur_tube     = 3;
static volatile uint8_t tube_toggle  = 0;
static volatile uint8_t led_pwm_step = 0;  // 0..7
static volatile uint8_t led_pwm_off  = 2;  // 0..8 (8=always on)

static portMUX_TYPE tube_mux = portMUX_INITIALIZER_UNLOCKED;

static const int seg_pins[7] = {
    PIN_SEG_A, PIN_SEG_B, PIN_SEG_C, PIN_SEG_D,
    PIN_SEG_E, PIN_SEG_F, PIN_SEG_G
};

static const int grid_pins[4] = {
    PIN_GRID0, PIN_GRID1, PIN_GRID2, PIN_GRID3
};

/* Time set? */
static volatile bool time_set = false;

/* ------------------------------------------------------------
   IRAM-safe GPIO setting in ISR (no gpio_set_level())
   ------------------------------------------------------------ */
static inline void IRAM_ATTR gpio_set_level_isr(int pin, int level)
{
    if (pin < 32) {
        if (level) GPIO.out_w1ts = (1U << pin);
        else       GPIO.out_w1tc = (1U << pin);
    } else {
        uint32_t shift = pin - 32;
        if (level) GPIO.out1_w1ts.val = (1U << shift);
        else       GPIO.out1_w1tc.val = (1U << shift);
    }
}

/* ------------------------------------------------------------
   Tube drive ISR @250 Hz
   ------------------------------------------------------------ */
static void IRAM_ATTR isr_tubes(void)
{
    // Everything turned off to avoid ghosting
    for (int i = 0; i < 4; i++) gpio_set_level_isr(grid_pins[i], 0);
    for (int i = 0; i < 7; i++) gpio_set_level_isr(seg_pins[i], 0);
    gpio_set_level_isr(PIN_DOT, 0);

    esp_rom_delay_us(40);

    // Next tube
    cur_tube = (cur_tube + 1) & 0x03;

    uint8_t digit, dot;
    portENTER_CRITICAL_ISR(&tube_mux);
    digit = tube_list[cur_tube].digit;
    dot   = tube_list[cur_tube].dot;
    portEXIT_CRITICAL_ISR(&tube_mux);

    if (digit > 10) digit = 10;
    const uint8_t *seg = digit_seg_data[digit];

    for (int i = 0; i < 7; i++) gpio_set_level_isr(seg_pins[i], seg[i]);
    gpio_set_level_isr(PIN_DOT, dot);

    gpio_set_level_isr(grid_pins[cur_tube], 1);
}

/* ------------------------------------------------------------
   LED PWM ISR @500 Hz (8 steps)
   ------------------------------------------------------------ */
static void IRAM_ATTR isr_leds(void)
{
    uint8_t off;
    portENTER_CRITICAL_ISR(&tube_mux);
    off = led_pwm_off;
    portEXIT_CRITICAL_ISR(&tube_mux);

    if (led_pwm_step == off) {
        gpio_set_level_isr(PIN_LEDS, 0);
    } else if (led_pwm_step == 0) {
        gpio_set_level_isr(PIN_LEDS, 1);
    }

    led_pwm_step++;
    if (led_pwm_step == 8) led_pwm_step = 0;
}

/* ------------------------------------------------------------
   GPTimer alarm callback @500 Hz
   ------------------------------------------------------------ */
static bool IRAM_ATTR timer_on_alarm(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx)
{
    tube_toggle ^= 1;
    if (tube_toggle) isr_tubes();  // 250 Hz effective
    isr_leds();                    // 500 Hz

    return false;
}

/* ------------------------------------------------------------
   GPIO init
   ------------------------------------------------------------ */
static void init_gpios(void)
{
    uint64_t out_mask =
        (1ULL<<PIN_GRID0) | (1ULL<<PIN_GRID1) |
        (1ULL<<PIN_GRID2) | (1ULL<<PIN_GRID3) |
        (1ULL<<PIN_SEG_A) | (1ULL<<PIN_SEG_B) | (1ULL<<PIN_SEG_C) |
        (1ULL<<PIN_SEG_D) | (1ULL<<PIN_SEG_E) | (1ULL<<PIN_SEG_F) |
        (1ULL<<PIN_SEG_G) | (1ULL<<PIN_DOT)   | (1ULL<<PIN_LEDS);

    gpio_config_t out_conf = {
        .pin_bit_mask = out_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&out_conf));

    ESP_LOGI(TAG, "GPIOs initialisiert (alle Grids/Segmente als OUTPUT).");
}

/* ------------------------------------------------------------
   GPTimer init @500 Hz
   ------------------------------------------------------------ */
static void init_timer_500hz(void)
{
    gptimer_handle_t gptimer = NULL;

    gptimer_config_t tconf = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000  // 1 MHz
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tconf, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_on_alarm
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    gptimer_alarm_config_t alarm_conf = {
        .alarm_count = 2000, // 2000 us => 500 Hz
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_conf));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

/* ------------------------------------------------------------
   Display functions
   ------------------------------------------------------------ */
static void no_time(void)
{
    portENTER_CRITICAL(&tube_mux);
    for (int i = 0; i < 4; i++) {
        tube_list[i].digit = 10; // Hyphen
        tube_list[i].dot   = LOW;
    }
    portEXIT_CRITICAL(&tube_mux);
}

static void display_time(void)
{
    time_t t = time(NULL);
    struct tm tmv;
    localtime_r(&t, &tmv);   // Local time (time zone via TZ/TZSET)

    uint32_t ms = (uint32_t)((esp_timer_get_time() / 1000) % 1000);
    uint8_t dot = (ms < 500) ? HIGH : LOW;

    portENTER_CRITICAL(&tube_mux);

    uint8_t u = (uint8_t)tmv.tm_sec;
    if (u >= 50 && u <= 54) {
        // show DD.MM
        u = (uint8_t)tmv.tm_mday;
        tube_list[1].digit = u % 10; tube_list[1].dot = dot;
        tube_list[0].digit = u / 10; tube_list[0].dot = dot;

        u = (uint8_t)(tmv.tm_mon + 1);
        tube_list[3].digit = u % 10; tube_list[3].dot = dot;
        tube_list[2].digit = u / 10; tube_list[2].dot = dot;
    } else {
        // show HH:MM
        u = (uint8_t)tmv.tm_hour;
        tube_list[1].digit = u % 10; tube_list[1].dot = dot;
        tube_list[0].digit = u / 10; tube_list[0].dot = LOW;

        u = (uint8_t)tmv.tm_min;
        tube_list[3].digit = u % 10; tube_list[3].dot = LOW;
        tube_list[2].digit = u / 10; tube_list[2].dot = LOW;
    }

    portEXIT_CRITICAL(&tube_mux);
}

static void display_task(void *arg)
{
    while (1) {
        if (!time_set) no_time();
        else           display_time();

        vTaskDelay(pdMS_TO_TICKS(20)); // ~50 Hz Refresh
    }
}

/* ------------------------------------------------------------
   Config in NVS (WLAN + TZ)
   ------------------------------------------------------------ */

typedef struct {
    char ssid[32];
    char password[64];
    char tz[32];
    bool has_wifi;
} clock_config_t;

static clock_config_t g_cfg;

/* AP-IP as string for display in web UI */
static char g_ap_ip_str[16] = "192.168.4.1";

static void config_set_defaults(void)
{
    memset(&g_cfg, 0, sizeof(g_cfg));
    // Standard: Germany / Central Europe with summer time
    strcpy(g_cfg.tz, "CET-1CEST,M3.5.0,M10.5.0/3");
    g_cfg.has_wifi = false;
}

static void config_load(void)
{
    config_set_defaults();

    nvs_handle_t h;
    esp_err_t err = nvs_open("clock", NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS: keine vorhandene Konfiguration, benutze Defaults.");
        return;
    }

    size_t len;

    len = sizeof(g_cfg.ssid);
    if (nvs_get_str(h, "ssid", g_cfg.ssid, &len) == ESP_OK && g_cfg.ssid[0] != '\0') {
        g_cfg.has_wifi = true;
    }

    len = sizeof(g_cfg.password);
    if (nvs_get_str(h, "pass", g_cfg.password, &len) != ESP_OK) {
        g_cfg.password[0] = '\0';
    }

    len = sizeof(g_cfg.tz);
    if (nvs_get_str(h, "tz", g_cfg.tz, &len) != ESP_OK) {
        // remains default
    }

    nvs_close(h);

    ESP_LOGI(TAG, "Konfiguration geladen: has_wifi=%d, ssid='%s', tz='%s'",
             g_cfg.has_wifi, g_cfg.ssid, g_cfg.tz);
}

static void config_save(void)
{
    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open("clock", NVS_READWRITE, &h));
    ESP_ERROR_CHECK(nvs_set_str(h, "ssid", g_cfg.ssid));
    ESP_ERROR_CHECK(nvs_set_str(h, "pass", g_cfg.password));
    ESP_ERROR_CHECK(nvs_set_str(h, "tz",   g_cfg.tz));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);

    ESP_LOGI(TAG, "Konfiguration gespeichert.");
}

/* ------------------------------------------------------------
   WiFi + SNTP + Web server
   ------------------------------------------------------------ */

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_MAX_RETRY     5

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool s_ap_mode = false;

static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif  = NULL;

/* Forward Decl for SNTP */
static void initialize_sntp(void);

/* SNTP callback: Time is synchronized */
static void time_sync_notification_cb(struct timeval *tv)
{
    time_set = true;
    ESP_LOGI(TAG, "Zeit per SNTP synchronisiert.");
}

/* Initialize SNTP */
static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "SNTP initialisieren...");
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

/* WiFi Event Handler */
static void wifi_event_handler(void* arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "WiFi-STA: Retry %d", s_retry_num);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        s_ap_mode = false;

        // Once IP address is available: Start NTP
        initialize_sntp();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "SoftAP gestartet.");
        s_ap_mode = true;
    }
}

/* WiFi basic init (netif, event loop, driver) */
static void wifi_init_all(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif  = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &wifi_event_handler, NULL, &instance_got_ip));
}

/* Start STA mode with saved SSID/password */
static void wifi_start_sta(void)
{
    ESP_LOGI(TAG, "Starte WiFi im STA-Modus, SSID='%s'", g_cfg.ssid);

    wifi_config_t wifi_config = { 0 };
    strncpy((char*)wifi_config.sta.ssid,
            g_cfg.ssid,
            sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password,
            g_cfg.password,
            sizeof(wifi_config.sta.password) - 1);

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* Start AP mode for setup */
static void wifi_start_ap(void)
{
    ESP_LOGI(TAG, "Starte WiFi im AP-Modus für Setup.");

    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
    }

    wifi_config_t ap_config = { 0 };
    strcpy((char*)ap_config.ap.ssid, "NixieClock-Setup");
    ap_config.ap.ssid_len = strlen((char*)ap_config.ap.ssid);
    strcpy((char*)ap_config.ap.password, "12345678");
    ap_config.ap.max_connection = 4;
    ap_config.ap.channel = 1;
    ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    if (strlen("12345678") == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Remember AP-IP for control and display
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(s_ap_netif, &ip_info);
    ESP_LOGI(TAG, "AP IP: " IPSTR ", Gateway: " IPSTR,
             IP2STR(&ip_info.ip), IP2STR(&ip_info.gw));
    ip4addr_ntoa_r((const ip4_addr_t *)&ip_info.ip, g_ap_ip_str, sizeof(g_ap_ip_str));
}

/* ------------------------------------------------------------
   HTTP-Server
   ------------------------------------------------------------ */

static httpd_handle_t s_http_server = NULL;

/* URL decoding (very easy) */
static void url_decode(char *dst, const char *src)
{
    char a, b;
    while (*src) {
        if (*src == '%' &&
            (a = src[1]) && (b = src[2]) &&
            isxdigit((unsigned char)a) && isxdigit((unsigned char)b)) {

            a = tolower((unsigned char)a);
            b = tolower((unsigned char)b);
            a = (a >= 'a') ? (a - 'a' + 10) : (a - '0');
            b = (b >= 'a') ? (b - 'a' + 10) : (b - '0');
            *dst++ = (char)(16 * a + b);
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}

/* Root page: Status + AP IP address + some nice CSS */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTP: GET /");

    // Fetch large HTML buffers from the heap (not the stack!)
    const size_t HTML_BUF_SIZE = 4096;
    char *html = malloc(HTML_BUF_SIZE);
    if (!html) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    time_t now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);

    char time_str[64];
    if (time_set) {
        strftime(time_str, sizeof(time_str),
                 "%Y-%m-%d %H:%M:%S", &tmv);
    } else {
        strcpy(time_str, "Time not yet set");
    }

    const char *mode_str = s_ap_mode ? "Access Point (Setup Mode)"
                                     : "Station (connected to Wi-Fi)";

    char sta_ip_str[16] = "-";
    if (!s_ap_mode && s_sta_netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK) {
            ip4addr_ntoa_r((const ip4_addr_t *)&ip_info.ip, sta_ip_str, sizeof(sta_ip_str));
        }
    }

    const char *ap_hint_html = "";
    if (s_ap_mode) {
        static char hint_buf[256];
        snprintf(hint_buf, sizeof(hint_buf),
            "<div class=\"hint\">"
            "<strong>Setup-AP aktiv:</strong> <code>NixieClock-Setup</code><br>"
            "Standard-IP: <strong>http://%s</strong>"
            "</div>",
            g_ap_ip_str);
        ap_hint_html = hint_buf;
    }

// Render HTML to the heap buffer
    snprintf(html, HTML_BUF_SIZE,
        "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
        "<!--Copyright (c) 2025 Erik Lauter-->"
        "<title>Nixie Clock</title>"
        "<style>"
        "body{margin:0;font-family:system-ui,-apple-system,BlinkMacSystemFont,"
        "Segoe UI,sans-serif;background:#0f172a;color:#e5e7eb;"
        "display:flex;align-items:center;justify-content:center;"
        "min-height:100vh;padding:16px;box-sizing:border-box;}"
        ".card{background:#020617;padding:24px 22px;border-radius:16px;"
        "box-shadow:0 18px 45px rgba(0,0,0,0.6);max-width:420px;width:100%%;}"
        "h1{margin:0 0 12px;font-size:1.6rem;color:#f9fafb;}"
        "p{margin:6px 0 10px;font-size:0.9rem;}"
        ".label{font-size:0.75rem;text-transform:uppercase;"
        "letter-spacing:0.08em;color:#9ca3af;margin-top:10px;}"
        ".value{font-size:0.95rem;color:#e5e7eb;}"
        ".badge{display:inline-block;padding:3px 8px;border-radius:999px;"
        "font-size:0.7rem;background:#111827;color:#9ca3af;margin-left:8px;}"
        ".hint{margin-top:12px;padding:10px 12px;border-radius:12px;"
        "background:#111827;font-size:0.8rem;color:#e5e7eb;}"
        "a{color:#60a5fa;text-decoration:none;font-size:0.9rem;}"
        "a:hover{text-decoration:underline;}"
        ".footer{margin-top:16px;font-size:0.7rem;color:#6b7280;}"
        "</style>"
        "</head><body>"
        "<div class=\"card\">"
        "<h1>Nixie Clock<span class=\"badge\">%s</span></h1>"
        "%s"
        "<div class=\"label\">WiFi</div>"
        "<div class=\"value\">%s</div>"
        "<div class=\"label\">Timezone</div>"
        "<div class=\"value\"><code>%s</code></div>"
        "<div class=\"label\">Current time</div>"
        "<div class=\"value\">%s</div>"
        "<div class=\"label\">Device IP</div>"
        "<div class=\"value\">%s</div>"
        "<p style=\"margin-top:14px;\"><a href=\"/config\">WiFi &amp; Timezone Settings &raquo;</a></p>"
        "<div class=\"footer\">Copyright (c) 2025 Erik Lauter</div>"
        "</div></body></html>",
        mode_str,
        ap_hint_html,
        g_cfg.has_wifi ? g_cfg.ssid : "(not configured)",
        g_cfg.tz,
        time_str,
        s_ap_mode ? g_ap_ip_str : sta_ip_str
    );

    httpd_resp_set_type(req, "text/html");
    esp_err_t err = httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);

    free(html);
    return err;
}

/* Time zone options for dropdown */
typedef struct {
    const char *label;
    const char *tz;
} tz_option_t;

static const tz_option_t tz_options[] = {
    { "UTC", "UTC0" }, 
    { "Europe - Berlin (CET/CEST)", "CET-1CEST,M3.5.0,M10.5.0/3" }, 
    { "Europe - London", "GMT0BST,M3.5.0/1,M10.5.0" }, 
    { "USA - Eastern (New York)", "EST5EDT,M3.2.0,M11.1.0" }, 
    { "USA - Pacific (Los Angeles)", "PST8PDT,M3.2.0,M11.1.0" }, 
    { "Japan (Tokyo)", "JST-9" }
};
static const size_t TZ_OPTION_COUNT = sizeof(tz_options)/sizeof(tz_options[0]);

/* Configuration page (GET) with CSS + TZ dropdown */
static esp_err_t config_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTP: GET /config");

    const size_t HTML_BUF_SIZE = 4096;
    char *html = malloc(HTML_BUF_SIZE);
    if (!html) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    char tz_opts_html[512];
    tz_opts_html[0] = '\0';

    size_t offset = 0;
    for (size_t i = 0; i < TZ_OPTION_COUNT; ++i) {
        const char *sel = (strcmp(g_cfg.tz, tz_options[i].tz) == 0) ? " selected" : "";
        int written = snprintf(tz_opts_html + offset,
                               sizeof(tz_opts_html) - offset,
                               "<option value=\"%s\"%s>%s</option>",
                               tz_options[i].tz,
                               sel,
                               tz_options[i].label);
        if (written < 0 || (size_t)written >= sizeof(tz_opts_html) - offset) {
            break;
        }
        offset += (size_t)written;
    }

    snprintf(html, HTML_BUF_SIZE,
        "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
        "<!--Copyright (c) 2025 Erik Lauter-->"
        "<title>Nixie Config</title>"
        "<style>"
        "body{margin:0;font-family:system-ui,-apple-system,BlinkMacSystemFont,"
        "Segoe UI,sans-serif;background:#020617;color:#e5e7eb;"
        "display:flex;align-items:center;justify-content:center;"
        "min-height:100vh;padding:16px;box-sizing:border-box;}"
        ".card{background:#020617;padding:24px 22px;border-radius:16px;"
        "box-shadow:0 18px 45px rgba(0,0,0,0.6);max-width:440px;width:100%%;}"
        "h1{margin:0 0 14px;font-size:1.5rem;color:#f9fafb;}"
        "label{display:block;margin-top:12px;font-size:0.8rem;"
        "text-transform:uppercase;letter-spacing:0.08em;color:#9ca3af;}"
        "input,select{width:100%%;padding:8px 10px;border-radius:10px;"
        "border:1px solid #374151;background:#020617;color:#e5e7eb;"
        "margin-top:4px;box-sizing:border-box;font-size:0.9rem;}"
        "input:focus,select:focus{outline:none;border-color:#60a5fa;"
        "box-shadow:0 0 0 1px rgba(96,165,250,0.5);}"
        "input[type=submit]{margin-top:18px;background:#3b82f6;border:none;"
        "color:#f9fafb;font-weight:600;cursor:pointer;border-radius:999px;}"
        "input[type=submit]:hover{background:#2563eb;}"
        ".back{margin-top:12px;font-size:0.85rem;}"
        "a{color:#60a5fa;text-decoration:none;}"
        "a:hover{text-decoration:underline;}"
        ".small{font-size:0.75rem;color:#9ca3af;margin-top:4px;}"
        "</style>"
        "</head><body>"
        "<div class=\"card\">"
        "<h1>WiFi &amp; Time zone</h1>"
        "<form method=\"POST\" action=\"/config\">"
        "<label for=\"ssid\">WiFi SSID</label>"
        "<input id=\"ssid\" name=\"ssid\" value=\"%s\">"
        "<label for=\"password\">WiFi Password</label>"
        "<input id=\"password\" type=\"password\" name=\"password\" value=\"%s\">"
        "<label for=\"tz\">Time zone</label>"
        "<select id=\"tz\" name=\"tz\">"
        "%s"
        "</select>"
        "<div class=\"small\">"
        ""
        ""
        "</div>"
        "<input type=\"submit\" value=\"Save &amp; Restart\">"
        "</form>"
        "<div class=\"back\"><a href=\"/\">&laquo; Zur&uuml;ck</a></div>"
        "</div></body></html>",
        g_cfg.has_wifi ? g_cfg.ssid : "",
        g_cfg.has_wifi ? g_cfg.password : "",
        tz_opts_html
    );

    httpd_resp_set_type(req, "text/html");
    esp_err_t err = httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);

    free(html);
    return err;
}

/* Save configuration (POST) */
static esp_err_t config_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTP: POST /config");

    char content[256];

    int recv_len = MIN(req->content_len, sizeof(content) - 1);
    int ret = httpd_req_recv(req, content, recv_len);
    if (ret <= 0) {
        return ESP_FAIL;
    }
    content[recv_len] = '\0';

    char ssid[32] = {0};
    char pass[64] = {0};
    char tz[32]   = {0};

    char *rest = content;
    char *token;
    while ((token = strtok_r(rest, "&", &rest))) {
        char *eq = strchr(token, '=');
        if (!eq) continue;
        *eq = '\0';
        char *key = token;
        char *val = eq + 1;

        char decoded[128];
        url_decode(decoded, val);

        if (strcmp(key, "ssid") == 0) {
            strncpy(ssid, decoded, sizeof(ssid) - 1);
        } else if (strcmp(key, "password") == 0) {
            strncpy(pass, decoded, sizeof(pass) - 1);
        } else if (strcmp(key, "tz") == 0) {
            strncpy(tz, decoded, sizeof(tz) - 1);
        }
    }

    if (tz[0] == '\0') {
        strcpy(tz, "UTC0");
    }

    strncpy(g_cfg.ssid, ssid, sizeof(g_cfg.ssid) - 1);
    strncpy(g_cfg.password, pass, sizeof(g_cfg.password) - 1);
    strncpy(g_cfg.tz, tz, sizeof(g_cfg.tz) - 1);
    g_cfg.has_wifi = (g_cfg.ssid[0] != '\0');

    config_save();

    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req,
        "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
        "<meta http-equiv=\"refresh\" content=\"5;url=/\"/></head>"
        "<body><p>Konfiguration gespeichert. Neustart in 2 Sekunden...</p>"
        "</body></html>");

    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ESP_OK;
}

/* Start HTTP server */
static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = root_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t cfg_get_uri = {
            .uri      = "/config",
            .method   = HTTP_GET,
            .handler  = config_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &cfg_get_uri);

        httpd_uri_t cfg_post_uri = {
            .uri      = "/config",
            .method   = HTTP_POST,
            .handler  = config_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &cfg_post_uri);

        ESP_LOGI(TAG, "HTTP-Server gestartet.");
    } else {
        ESP_LOGE(TAG, "HTTP-Server konnte nicht gestartet werden.");
    }
    return server;
}

/* ------------------------------------------------------------
   app_main
   ------------------------------------------------------------ */
void app_main(void)
{
    // NVS for WiFi/Config
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    config_load();

    // Set time zone (for localtime_r)
    setenv("TZ", g_cfg.tz, 1);
    tzset();

    // Advertisement
    init_gpios();
    init_timer_500hz();
    xTaskCreate(display_task, "display_task", 4096, NULL, 9, NULL);

    // WiFi + network
    wifi_init_all();

    if (g_cfg.has_wifi) {
        wifi_start_sta();

        // Waiting for connection or failed attempt
        EventBits_t bits = xEventGroupWaitBits(
            s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(15000)); // 15s Timeout

        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "Mit WLAN verbunden, HTTP-Server im STA-Modus.");
        } else {
            ESP_LOGW(TAG, "WLAN-STA fehlgeschlagen, starte SoftAP.");
            esp_wifi_stop();
            wifi_start_ap();
        }
    } else {
        //No Wi-Fi configured yet -> direct access point
        wifi_start_ap();
    }

    s_http_server = start_webserver();

    ESP_LOGI(TAG, "Clock gestartet. Web-UI aufrufen zum Konfigurieren.");
}
