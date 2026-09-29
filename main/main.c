#include "fluid_app.h"

#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "ESP32WatchFluid";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting fluid simulation");

    esp_err_t err = fluid_app_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start fluid app: %s", esp_err_to_name(err));
    }
}
