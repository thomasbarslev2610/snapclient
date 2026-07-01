/*
 * activitytimer.c
 *
 * Controls a relay GPIO and an RGB WS2812 LED to indicate Snapcast
 * playback activity.
 *
 *  - Playing starts  : relay HIGH, LED green immediately
 *  - Playing stops   : start countdown (relay_timeout_s); on expiry:
 *                      relay LOW, LED red
 *
 * The relay GPIO number and inactivity timeout are read from NVS at
 * init time (set via the web UI under General Settings).
 *
 * The WS2812 LED GPIO is set at compile time via Kconfig
 * (CONFIG_ACTIVITYTIMER_LED_GPIO, default 48 for ESP32-S3-DevKitC-1).
 * Set to -1 to disable LED control entirely.
 */

#include "activitytimer.h"
#include "settings_manager.h"

#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "ACTIVITYTIMER";

/* ---- WS2812 RMT config ------------------------------------------------- */
/*
 * 10 MHz clock → 100 ns / tick
 * WS2812B spec (typ):
 *   T0H = 400 ns → 4 ticks   T0L = 850 ns → 9 ticks (round up)
 *   T1H = 800 ns → 8 ticks   T1L = 450 ns → 5 ticks (round up)
 *   Reset > 50 µs            → 560 ticks
 */
#define LED_RMT_RESOLUTION_HZ  10000000U
#define WS2812_T0H  4
#define WS2812_T0L  9
#define WS2812_T1H  8
#define WS2812_T1L  5
#define WS2812_RST  560

/* 24 data bits (GRB) + 1 reset symbol */
#define LED_SYMBOL_COUNT 25

static rmt_channel_handle_t s_led_chan    = NULL;
static rmt_encoder_handle_t s_copy_enc   = NULL;
static rmt_symbol_word_t    s_symbols[LED_SYMBOL_COUNT];

/* ---- State ------------------------------------------------------------- */
static TimerHandle_t    s_timer     = NULL;
static SemaphoreHandle_t s_mutex    = NULL;
static int              s_relay_gpio = -1;
static uint32_t         s_timeout_ms = 5000;
static bool             s_playing    = false;

/* ---- LED helpers -------------------------------------------------------- */

static void build_symbols(uint8_t r, uint8_t g, uint8_t b)
{
    /* WS2812 expects GRB bit order, MSB first */
    const uint8_t grb[3] = {g, r, b};
    for (int byte = 0; byte < 3; byte++) {
        for (int bit = 7; bit >= 0; bit--) {
            int idx = byte * 8 + (7 - bit);
            bool one = (grb[byte] >> bit) & 1u;
            s_symbols[idx].level0    = 1;
            s_symbols[idx].duration0 = one ? WS2812_T1H : WS2812_T0H;
            s_symbols[idx].level1    = 0;
            s_symbols[idx].duration1 = one ? WS2812_T1L : WS2812_T0L;
        }
    }
    /* Reset / latch pulse */
    s_symbols[24].level0    = 0;
    s_symbols[24].duration0 = WS2812_RST;
    s_symbols[24].level1    = 0;
    s_symbols[24].duration1 = 0;
}

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led_chan || !s_copy_enc) return;
    build_symbols(r, g, b);
    rmt_transmit_config_t tx_cfg = {.loop_count = 0};
    esp_err_t err = rmt_transmit(s_led_chan, s_copy_enc,
                                 s_symbols,
                                 LED_SYMBOL_COUNT * sizeof(rmt_symbol_word_t),
                                 &tx_cfg);
    if (err == ESP_OK) {
        rmt_tx_wait_all_done(s_led_chan, pdMS_TO_TICKS(50));
    }
}

/* ---- Output helpers ----------------------------------------------------- */

static void output_active(void)
{
    if (s_relay_gpio >= 0) gpio_set_level(s_relay_gpio, 1);
    led_set(0, 255, 0);  /* green */
    ESP_LOGI(TAG, "ACTIVE  – relay HIGH, LED green");
}

static void output_inactive(void)
{
    if (s_relay_gpio >= 0) gpio_set_level(s_relay_gpio, 0);
    led_set(255, 0, 0);  /* red */
    ESP_LOGI(TAG, "INACTIVE – relay LOW, LED red");
}

/* ---- Timer callback ----------------------------------------------------- */

static void inactivity_cb(TimerHandle_t t)
{
    (void)t;
    output_inactive();
}

/* ---- Public API --------------------------------------------------------- */

esp_err_t activitytimer_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    /* Read settings from NVS */
    int32_t gpio_num    = -1;
    int32_t timeout_s   = 5;
    settings_get_relay_gpio(&gpio_num);
    settings_get_relay_timeout_s(&timeout_s);

    s_relay_gpio = (int)gpio_num;
    s_timeout_ms = (uint32_t)((timeout_s > 0 ? timeout_s : 1)) * 1000u;

    ESP_LOGI(TAG, "relay_gpio=%d  timeout=%lus", s_relay_gpio,
             (unsigned long)(s_timeout_ms / 1000));

    /* Configure relay GPIO */
    if (s_relay_gpio >= 0) {
        gpio_config_t io = {
            .pin_bit_mask  = 1ULL << s_relay_gpio,
            .mode          = GPIO_MODE_OUTPUT,
            .pull_up_en    = GPIO_PULLUP_DISABLE,
            .pull_down_en  = GPIO_PULLDOWN_DISABLE,
            .intr_type     = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        gpio_set_level(s_relay_gpio, 0);
    }

    /* Configure WS2812 LED via RMT */
    int led_gpio = CONFIG_ACTIVITYTIMER_LED_GPIO;
    if (led_gpio >= 0) {
        rmt_tx_channel_config_t chan_cfg = {
            .gpio_num         = led_gpio,
            .clk_src          = RMT_CLK_SRC_DEFAULT,
            .resolution_hz    = LED_RMT_RESOLUTION_HZ,
            .mem_block_symbols = 64,
            .trans_queue_depth = 4,
        };
        esp_err_t err = rmt_new_tx_channel(&chan_cfg, &s_led_chan);
        if (err == ESP_OK) {
            rmt_copy_encoder_config_t enc_cfg = {};
            err = rmt_new_copy_encoder(&enc_cfg, &s_copy_enc);
        }
        if (err == ESP_OK) {
            err = rmt_enable(s_led_chan);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "WS2812 init failed: %s", esp_err_to_name(err));
            s_led_chan  = NULL;
            s_copy_enc = NULL;
        } else {
            ESP_LOGI(TAG, "WS2812 LED on GPIO %d", led_gpio);
        }
    }

    /* Create one-shot inactivity timer */
    s_timer = xTimerCreate("act_tmr",
                           pdMS_TO_TICKS(s_timeout_ms),
                           pdFALSE,   /* one-shot */
                           NULL,
                           inactivity_cb);
    if (!s_timer) {
        ESP_LOGE(TAG, "Failed to create timer");
        return ESP_ERR_NO_MEM;
    }

    /* Start in inactive state */
    output_inactive();
    return ESP_OK;
}

void activitytimer_notify_playing(bool playing)
{
    if (!s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool was_playing = s_playing;
    s_playing = playing;
    xSemaphoreGive(s_mutex);

    if (playing) {
        /* Cancel any pending inactivity countdown */
        if (s_timer) xTimerStop(s_timer, 0);
        if (!was_playing) output_active();
    } else if (was_playing) {
        /* Start / restart the inactivity countdown */
        if (s_timer) {
            xTimerChangePeriod(s_timer, pdMS_TO_TICKS(s_timeout_ms), 0);
            xTimerStart(s_timer, 0);
        }
    }
}
