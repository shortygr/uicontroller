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
#include "esp_log.h"
#include "esp_log.h"

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

    // Initialize SPI bus
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_CLK_GPIO,
        .mosi_io_num = LCD_MOSI_GPIO,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
//        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
//      .flags = SPI_DEVICE_3WIRE,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI bus initialized on host %d", LCD_HOST);

    // Attach LCD to SPI bus
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = LCD_CS_GPIO,
        .dc_gpio_num = LCD_DC_GPIO,
        .spi_mode = 0,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
    ESP_LOGI(TAG, "LCD panel IO initialized at pixel clock %"PRIu32" Hz", LCD_PIXEL_CLOCK_HZ);


    // Create LCD panel
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST_GPIO,
        .rgb_endian = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
    ESP_LOGI(TAG, "GC9A01 LCD panel created successfully");


    // Reset and initialize LCD
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_LOGI(TAG, "LCD panel reset");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_LOGI(TAG, "LCD panel initialized");
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_LOGI(TAG, "Color inversion enabled");
    // Mirror the display (adjust parameters as needed)

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_LOGI(TAG, "Display turned on");

//    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(SCREEN_BACKLIGHT_PIN, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
    ESP_LOGI(TAG, "Backlight turned on to level %d", EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

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

    
    // Create LVGL display

    // Simple test UI
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text_static(label, LV_SYMBOL_REFRESH"Hello, LVGL!");
    lv_obj_set_style_text_color(label, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_obj_center(label);
    ESP_LOGI(TAG, "UI test label created");

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
