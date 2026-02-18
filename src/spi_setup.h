#ifndef SPI_SETUP_H
#define SPI_SETUP_H

#include "driver/spi_master.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_io_spi.h"

// GC9A01 Display Configuration
#define LCD_HOST  SPI2_HOST
#define LCD_CLK_GPIO 10
#define LCD_MOSI_GPIO 11
#define LCD_CS_GPIO 9
#define LCD_DC_GPIO 3
#define LCD_RST_GPIO 14
#define LCD_BL_GPIO 46

#define LCD_H_RES 240
#define LCD_V_RES 240
#define LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)

/**
 * Initialize SPI bus and LCD panel
 * @return panel_handle - esp_lcd_panel_handle_t for the initialized LCD panel
 */
esp_lcd_panel_handle_t spi_setup_initialize(void);

#endif // SPI_SETUP_H