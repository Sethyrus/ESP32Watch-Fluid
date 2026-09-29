#include "watch_rtc.h"

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "bsp/esp-bsp.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define RTC_ADDR 0x51
#define RTC_REG_SECONDS 0x04 // seconds, minutes, hours, days, weekdays, months, years
#define RTC_OS_FLAG 0x80     // in the seconds register: oscillator stopped, time invalid
#define RTC_TIMEOUT_MS 50

static const char *TAG = "watch_rtc";
static i2c_master_dev_handle_t s_dev;

static inline int from_bcd(uint8_t v)
{
    return (v >> 4) * 10 + (v & 0x0f);
}

static inline uint8_t to_bcd(int v)
{
    return (uint8_t)(((v / 10) << 4) | (v % 10));
}

static esp_err_t read_tm(struct tm *tm, bool *valid)
{
    uint8_t reg = RTC_REG_SECONDS;
    uint8_t r[7];
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, r, sizeof(r), RTC_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    memset(tm, 0, sizeof(*tm));
    *valid = !(r[0] & RTC_OS_FLAG);
    tm->tm_sec = from_bcd(r[0] & 0x7f);
    tm->tm_min = from_bcd(r[1] & 0x7f);
    tm->tm_hour = from_bcd(r[2] & 0x3f); // 24 h mode (Control_1 12_24 = 0, the default)
    tm->tm_mday = from_bcd(r[3] & 0x3f);
    tm->tm_wday = r[4] & 0x07;
    tm->tm_mon = from_bcd(r[5] & 0x1f) - 1;
    tm->tm_year = from_bcd(r[6]) + 100; // 2000..2099
    return ESP_OK;
}

// Writing the seconds register also clears the OS flag.
static esp_err_t write_tm(const struct tm *tm)
{
    uint8_t w[8] = {
        RTC_REG_SECONDS,
        to_bcd(tm->tm_sec),
        to_bcd(tm->tm_min),
        to_bcd(tm->tm_hour),
        to_bcd(tm->tm_mday),
        (uint8_t)(tm->tm_wday & 0x07),
        to_bcd(tm->tm_mon + 1),
        to_bcd(tm->tm_year % 100),
    };
    return i2c_master_transmit(s_dev, w, sizeof(w), RTC_TIMEOUT_MS);
}

static void set_system_clock(const struct tm *tm)
{
    struct tm copy = *tm;
    struct timeval tv = {.tv_sec = mktime(&copy), .tv_usec = 0}; // no TZ set: local == UTC
    settimeofday(&tv, NULL);
}

static void build_time(struct tm *tm)
{
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char mon[4] = {0};
    int day = 1;
    int year = 2026;
    int h = 0;
    int m = 0;
    int s = 0;
    memset(tm, 0, sizeof(*tm));
    sscanf(__DATE__, "%3s %d %d", mon, &day, &year);
    sscanf(__TIME__, "%d:%d:%d", &h, &m, &s);
    const char *p = strstr(months, mon);
    tm->tm_mon = p != NULL ? (int)(p - months) / 3 : 0;
    tm->tm_mday = day;
    tm->tm_year = year - 1900;
    tm->tm_hour = h;
    tm->tm_min = m;
    tm->tm_sec = s;
    struct tm copy = *tm;
    mktime(&copy); // fills the weekday
    tm->tm_wday = copy.tm_wday;
}

esp_err_t watch_rtc_init(bool set_from_build)
{
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (bus == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = RTC_ADDR,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &s_dev);
    if (err != ESP_OK) {
        return err;
    }

    struct tm tm;
    bool valid = false;
    err = read_tm(&tm, &valid);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RTC read failed: %s", esp_err_to_name(err));
        return err;
    }
    if (!valid || set_from_build) {
        build_time(&tm);
        err = write_tm(&tm);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "RTC write failed: %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "RTC %s, set to build time", valid ? "overridden" : "had lost the time");
    }
    set_system_clock(&tm);
    ESP_LOGI(TAG, "Time %04d-%02d-%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
             tm.tm_min, tm.tm_sec);
    return ESP_OK;
}

esp_err_t watch_rtc_set_time(int hours, int minutes)
{
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    tm.tm_hour = hours;
    tm.tm_min = minutes;
    tm.tm_sec = 0;
    esp_err_t err = write_tm(&tm);
    set_system_clock(&tm); // keep the session consistent even if the RTC write failed
    return err;
}
