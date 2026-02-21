#ifndef CST816D_H
#define CST816D_H

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

// CST816D I2C Device Configuration
#define CST816D_I2C_ADDR 0x15

// Register Definitions
#define CST816D_REG_STATUS 0x00
#define CST816D_REG_POINT_NUM 0x01
#define CST816D_REG_POINT1_XH 0x02
#define CST816D_REG_POINT1_XL 0x03
#define CST816D_REG_POINT1_YH 0x04
#define CST816D_REG_POINT1_YL 0x05
#define CST816D_REG_POINT1_PRESSURE 0x06
#define CST816D_REG_CHIP_ID 0xA7
#define CST816D_REG_FW_VERSION 0xA9
#define CST816D_REG_RESET 0xFA

// Touch Point Structure
typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t pressure;
    bool pressed;
} cst816d_touch_point_t;

// CST816D Configuration Structure
typedef struct {
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_master_dev_handle_t i2c_dev_handle;
    gpio_num_t interrupt_pin;
    gpio_num_t reset_pin;
    uint16_t width;
    uint16_t height;
    lv_indev_t *indev_handle;
} cst816d_handle_t;

/**
 * @brief Initialize CST816D touchscreen
 *
 * @param i2c_bus_handle Initialized I2C bus handle
 * @param interrupt_pin GPIO pin for touch interrupt
 * @param reset_pin GPIO pin for touch reset
 * @param width Display width
 * @param height Display height
 * @return Pointer to CST816D handle, NULL if failed
 */
cst816d_handle_t* cst816d_init(i2c_master_bus_handle_t i2c_bus_handle,
                                gpio_num_t interrupt_pin,
                                gpio_num_t reset_pin,
                                uint16_t width,
                                uint16_t height);

/**
 * @brief Reset CST816D device
 *
 * @param handle CST816D handle
 * @return True if successful, false otherwise
 */
bool cst816d_reset(cst816d_handle_t *handle);

/**
 * @brief Read touch point data from CST816D
 *
 * @param handle CST816D handle
 * @param touch_point Pointer to store touch point data
 * @return True if touch data read successfully, false otherwise
 */
bool cst816d_read_touch(cst816d_handle_t *handle, cst816d_touch_point_t *touch_point);

/**
 * @brief Get chip/firmware information
 *
 * @param handle CST816D handle
 * @param chip_id Pointer to store chip ID
 * @param fw_version Pointer to store firmware version
 * @return True if successful, false otherwise
 */
bool cst816d_get_info(cst816d_handle_t *handle, uint8_t *chip_id, uint8_t *fw_version);

/**
 * @brief Deinitialize CST816D
 *
 * @param handle CST816D handle
 */
void cst816d_deinit(cst816d_handle_t *handle);

/**
 * @brief Register LVGL input device for CST816D
 *
 * @param handle CST816D handle
 * @return LVGL input device handle
 */
lv_indev_t* cst816d_register_lvgl_indev(cst816d_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // CST816D_H
