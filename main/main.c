#include <stdio.h>
#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "spi_setup.h"
#include "cst816d.h"
#include "ui.h"

// Tag for console logging
static const char *TAG = "uicontroller";

// GC9A01 Display Configuration
#define LCD_HOST  SPI2_HOST
#define LCD_CLK_GPIO 10
#define LCD_MOSI_GPIO 11
#define LCD_CS_GPIO 9
#define LCD_DC_GPIO 3
#define LCD_RST_GPIO 14
#define LCD_BL_GPIO 46
#define POWER1_GPIO 1
#define POWER2_GPIO 2   

// CST816D Touch Configuration
#define TOUCH_I2C_NUM I2C_NUM_0
#define TOUCH_I2C_SCL_GPIO 7
#define TOUCH_I2C_SDA_GPIO 6
#define TOUCH_INT_GPIO 5
#define TOUCH_RST_GPIO 13



#define LCD_H_RES 240
#define LCD_V_RES 240
#define LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define SCREEN_BACKLIGHT_PIN 46

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL 0


#define LVGL_DRAW_BUF_LINES    20 // number of display lines in each draw buffer
#define LVGL_TICK_PERIOD_MS    2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1000 / CONFIG_FREERTOS_HZ
#define LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LVGL_TASK_PRIORITY     2


static lv_display_t *disp = NULL;

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) 
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    
    int x1 = area->x1;
    int y1 = area->y1;
    int x2 = area->x2;
    int y2 = area->y2;

    // Pass the draw buffer to the LCD driver
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, px_map);
    lv_display_flush_ready(disp);
}

static void lvgl_task(void *pv)
{
    (void) pv;
    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


void app_main(void)
{
    // Initialize logging
//    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    
    ESP_LOGI(TAG, "===============================");
    ESP_LOGI(TAG, "UI Controller Starting Up");
    ESP_LOGI(TAG, "===============================");
    
    // Initialize LVGL
    lv_init();
    ESP_LOGI(TAG, "LVGL initialized");

    //Configure backlight PWM


//    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << SCREEN_BACKLIGHT_PIN
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    ESP_LOGI(TAG, "Backlight GPIO configured");

    gpio_config_t power1_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << POWER1_GPIO
    };
    ESP_ERROR_CHECK(gpio_config(&power1_gpio_config));

    gpio_config_t power2_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << POWER2_GPIO
    };
    ESP_ERROR_CHECK(gpio_config(&power2_gpio_config));


    gpio_set_level(POWER1_GPIO, 1);
    gpio_set_level(POWER2_GPIO, 1);

    // Initialize SPI bus and LCD panel
    esp_lcd_panel_handle_t panel_handle = spi_setup_initialize();
    ESP_LOGI(TAG, "SPI and LCD panel initialized");

    // Reset and initialize LCD
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_LOGI(TAG, "LCD panel reset");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_LOGI(TAG, "LCD panel initialized");
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_LOGI(TAG, "Color inversion disabled");
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false)); // horizontal, vertical
    // Mirror the display (adjust parameters as needed)

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_LOGI(TAG, "Display turned on");

//    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(SCREEN_BACKLIGHT_PIN, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
    ESP_LOGI(TAG, "Backlight turned on to level %d", EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    // Initialize I2C bus for CST816D touch
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TOUCH_I2C_NUM,
        .scl_io_num = TOUCH_I2C_SCL_GPIO,
        .sda_io_num = TOUCH_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    i2c_master_bus_handle_t i2c_mst_handle = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_mst_handle));
    ESP_LOGI(TAG, "I2C master bus initialized on pins SCL=%d, SDA=%d", TOUCH_I2C_SCL_GPIO, TOUCH_I2C_SDA_GPIO);

    // Initialize CST816D touch sensor
    cst816d_handle_t *touch_handle = cst816d_init(i2c_mst_handle, TOUCH_INT_GPIO, TOUCH_RST_GPIO, LCD_H_RES, LCD_V_RES);
    if (!touch_handle) {
        ESP_LOGE(TAG, "Failed to initialize CST816D touch sensor");
    } else {
        ESP_LOGI(TAG, "CST816D touch sensor initialized");
    }

    // Setup LVGL display buffer

    disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    ESP_LOGI(TAG, "LVGL display created with resolution %dx%d", LCD_H_RES, LCD_V_RES);


    size_t draw_buffer_sz = LCD_H_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);
    ESP_LOGI(TAG, "Draw buffer size: %zu bytes", draw_buffer_sz);

    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf1);
    ESP_LOGI(TAG, "Allocated LVGL draw buffer 1");
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf2);
    ESP_LOGI(TAG, "Allocated LVGL draw buffer 2");
    // initialize LVGL draw buffers
    lv_display_set_buffers(disp, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // associate the mipi panel handle to the display
    lv_display_set_user_data(disp, panel_handle);
    // set color depth
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(disp, lvgl_flush_cb);
    ESP_LOGI(TAG, "LVGL display buffers and callbacks configured");

    // Register CST816D touch sensor NOW (after display is created)
    if (touch_handle) {
        lv_indev_t *indev = cst816d_register_lvgl_indev(touch_handle);
        if (indev) {
            // Associate input device with display
            lv_indev_set_display(indev, disp);
            ESP_LOGI(TAG, "CST816D input device registered and associated with display");
        }
    }

    // Create LVGL display
    ui_init();

/*     // Simple test UI
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text_static(label, LV_SYMBOL_REFRESH"Hello, LVGL!");
    lv_obj_set_style_text_color(label, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_obj_center(label);
    ESP_LOGI(TAG, "UI test label created"); */

    // Start LVGL handler in its own FreeRTOS task so it yields properly
//    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 4096, NULL, 5, NULL, 1);
    ESP_LOGI(TAG, "LVGL task created");

    ESP_LOGI(TAG, "===============================");
    ESP_LOGI(TAG, "UI Controller Ready");
    ESP_LOGI(TAG, "===============================");

    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(20));
     }
    // Delete this initialization task to avoid blocking CPU 0 idle
}
