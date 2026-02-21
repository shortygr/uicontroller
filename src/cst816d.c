#include "cst816d.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "CST816D";

// LVGL input device callback
static void cst816d_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    static uint32_t callback_count = 0;
    
    cst816d_handle_t *handle = (cst816d_handle_t *)lv_indev_get_user_data(indev);
    if (!handle) {
        ESP_LOGE(TAG, "cst816d_read_cb: Invalid handle!");
        data->state = LV_INDEV_STATE_REL;
        return;
    }

    cst816d_touch_point_t touch_point = {0};
    
    if (cst816d_read_touch(handle, &touch_point)) {
        data->point.x = touch_point.x;
        data->point.y = touch_point.y;
        data->state = touch_point.pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
        
        // Log callback invocations occasionally
        if (callback_count++ % 100 == 0) {
            ESP_LOGI(TAG, "Callback #%lu: Last touch x=%u, y=%u, pressed=%d, state=%d",
                     callback_count, touch_point.x, touch_point.y, touch_point.pressed, data->state);
        }
        
        if (touch_point.pressed) {
            ESP_LOGI(TAG, ">>> TOUCH DETECTED: x=%u, y=%u, pressure=%u, state=%d <<<",
                     touch_point.x, touch_point.y, touch_point.pressure, data->state);
        }
    } else {
        ESP_LOGW(TAG, "cst816d_read_cb: Failed to read touch data");
        data->state = LV_INDEV_STATE_REL;
    }
}

/**
 * @brief Write a register to CST816D
 */
static bool cst816d_write_reg(cst816d_handle_t *handle, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    esp_err_t err = i2c_master_transmit(handle->i2c_dev_handle, data, sizeof(data), -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

/**
 * @brief Read a register from CST816D
 */
static bool cst816d_read_reg(cst816d_handle_t *handle, uint8_t reg, uint8_t *value)
{
    esp_err_t err;
    
    // Write register address
    err = i2c_master_transmit(handle->i2c_dev_handle, &reg, 1, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register address 0x%02X: %s", reg, esp_err_to_name(err));
        return false;
    }
    
    // Read register value
    err = i2c_master_receive(handle->i2c_dev_handle, value, 1, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(err));
        return false;
    }
    
    return true;
}

/**
 * @brief Read multiple registers from CST816D
 */
static bool cst816d_read_regs(cst816d_handle_t *handle, uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t err;
    
    // Write register address
    err = i2c_master_transmit(handle->i2c_dev_handle, &reg, 1, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register address 0x%02X: %s", reg, esp_err_to_name(err));
        return false;
    }
    
    // Read register values
    err = i2c_master_receive(handle->i2c_dev_handle, data, len, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read %u registers from 0x%02X: %s", len, reg, esp_err_to_name(err));
        return false;
    }
    
    return true;
}

cst816d_handle_t* cst816d_init(i2c_master_bus_handle_t i2c_bus_handle,
                                gpio_num_t interrupt_pin,
                                gpio_num_t reset_pin,
                                uint16_t width,
                                uint16_t height)
{
    if (!i2c_bus_handle) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return NULL;
    }

    // Allocate handle structure
    cst816d_handle_t *handle = (cst816d_handle_t *)malloc(sizeof(cst816d_handle_t));
    if (!handle) {
        ESP_LOGE(TAG, "Failed to allocate CST816D handle");
        return NULL;
    }

    handle->i2c_bus_handle = i2c_bus_handle;
    handle->interrupt_pin = interrupt_pin;
    handle->reset_pin = reset_pin;
    handle->width = width;
    handle->height = height;
    handle->indev_handle = NULL;

    // Add device to I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CST816D_I2C_ADDR,
        .scl_speed_hz = 400000,  // 400 kHz clock
    };

    esp_err_t err = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &handle->i2c_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add CST816D device to I2C bus: %s", esp_err_to_name(err));
        free(handle);
        return NULL;
    }

    ESP_LOGI(TAG, "CST816D device added to I2C bus at address 0x%02X", CST816D_I2C_ADDR);

    // Configure GPIO pins
    if (reset_pin != GPIO_NUM_NC) {
        gpio_config_t gpio_cfg = {
            .pin_bit_mask = 1ULL << reset_pin,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&gpio_cfg);
        ESP_LOGI(TAG, "Reset GPIO pin %d configured", reset_pin);
    }

    if (interrupt_pin != GPIO_NUM_NC) {
        gpio_config_t gpio_cfg = {
            .pin_bit_mask = 1ULL << interrupt_pin,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&gpio_cfg);
        ESP_LOGI(TAG, "Interrupt GPIO pin %d configured", interrupt_pin);
    }

    // Reset the device
    if (!cst816d_reset(handle)) {
        ESP_LOGE(TAG, "Failed to reset CST816D device");
        free(handle);
        return NULL;
    }

    // Verify device communication
    uint8_t chip_id = 0, fw_version = 0;
    if (!cst816d_get_info(handle, &chip_id, &fw_version)) {
        ESP_LOGE(TAG, "Failed to read CST816D chip info");
        free(handle);
        return NULL;
    }

    ESP_LOGI(TAG, "CST816D initialized successfully - Chip ID: 0x%02X, FW Version: 0x%02X", chip_id, fw_version);

    return handle;
}

bool cst816d_reset(cst816d_handle_t *handle)
{
    if (!handle) {
        return false;
    }

    if (handle->reset_pin != GPIO_NUM_NC) {
        // Pull reset pin low
        gpio_set_level(handle->reset_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        
        // Release reset pin (pull high)
        gpio_set_level(handle->reset_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(100));  // Wait for device to boot
        
        ESP_LOGI(TAG, "Hardware reset performed");
    }

    // Software reset
    if (!cst816d_write_reg(handle, CST816D_REG_RESET, 0x01)) {
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
    return true;
}

bool cst816d_read_touch(cst816d_handle_t *handle, cst816d_touch_point_t *touch_point)
{
    if (!handle || !touch_point) {
        return false;
    }

    uint8_t data[6];
    
    // Read touch status and coordinates
    if (!cst816d_read_regs(handle, CST816D_REG_STATUS, data, sizeof(data))) {
        ESP_LOGW(TAG, "cst816d_read_touch: Failed to read registers");
        return false;
    }

    uint8_t status = data[0];
    uint8_t point_num = data[1];

    // Extract touch point data
    uint8_t xh = data[2];
    uint8_t xl = data[3];
    uint8_t yh = data[4];
    uint8_t yl = data[5];

    // Combine high and low bytes for X coordinate
    touch_point->x = ((uint16_t)(xh & 0x0F) << 8) | xl;
    
    // Combine high and low bytes for Y coordinate
    touch_point->y = ((uint16_t)(yh & 0x0F) << 8) | yl;

    // Read pressure (optional)
    uint8_t pressure = 0;
    cst816d_read_reg(handle, CST816D_REG_POINT1_PRESSURE, &pressure);
    touch_point->pressure = pressure;

    // Determine if touch is pressed (check if point_num > 0)
    touch_point->pressed = (point_num > 0) ? true : false;

    // Debug: Log raw touch data
    static uint32_t call_count = 0;
    if (call_count++ % 50 == 0) {  // Log every 50th call to avoid spam
        ESP_LOGI(TAG, "Touch data: status=0x%02X, point_num=%u, x=%u, y=%u, pressure=%u, pressed=%d",
                 status, point_num, touch_point->x, touch_point->y, touch_point->pressure, touch_point->pressed);
    }

    return true;
}

bool cst816d_get_info(cst816d_handle_t *handle, uint8_t *chip_id, uint8_t *fw_version)
{
    if (!handle) {
        return false;
    }

    if (chip_id && !cst816d_read_reg(handle, CST816D_REG_CHIP_ID, chip_id)) {
        return false;
    }

    if (fw_version && !cst816d_read_reg(handle, CST816D_REG_FW_VERSION, fw_version)) {
        return false;
    }

    return true;
}

void cst816d_deinit(cst816d_handle_t *handle)
{
    if (!handle) {
        return;
    }

    if (handle->i2c_dev_handle) {
        i2c_master_bus_rm_device(handle->i2c_dev_handle);
    }

    free(handle);
    ESP_LOGI(TAG, "CST816D deinitialized");
}

lv_indev_t* cst816d_register_lvgl_indev(cst816d_handle_t *handle)
{
    if (!handle) {
        ESP_LOGE(TAG, "Invalid CST816D handle");
        return NULL;
    }

    lv_indev_t *indev = lv_indev_create();
    if (!indev) {
        ESP_LOGE(TAG, "Failed to create LVGL input device");
        return NULL;
    }
    ESP_LOGI(TAG, "LVGL input device created");

    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, cst816d_read_cb);
    lv_indev_set_user_data(indev, (void *)handle);

    handle->indev_handle = indev;
    ESP_LOGI(TAG, "CST816D LVGL input device configured and ready");

    return indev;
}
