# CST816D Touch Device Support

This project now includes full support for the **CST816D capacitive touch sensor** IC, which is commonly used in ESP32-based display applications.

## Features

- **I2C Communication**: CST816D communicates via I2C (standard 400 kHz clock)
- **LVGL Integration**: Touch input is automatically registered as an LVGL input device
- **Multi-point Touch**: Supports single and multiple touch points
- **Pressure Sensing**: Reads touch pressure information
- **Hardware Reset**: Supports GPIO-based hardware reset
- **Interrupt Support**: GPIO interrupt pin for event notification
- **Chip Information**: Reads chip ID and firmware version for verification

## GPIO Configuration

The following GPIO pins are used for the CST816D touch interface (defined in [main.c](main/main.c)):

| Function | GPIO | Adjustable |
|----------|------|-----------|
| I2C SCL (Clock) | GPIO 8 | Yes - `TOUCH_I2C_SCL_GPIO` |
| I2C SDA (Data) | GPIO 4 | Yes - `TOUCH_I2C_SDA_GPIO` |
| Touch Interrupt | GPIO 7 | Yes - `TOUCH_INT_GPIO` |
| Touch Reset | GPIO 6 | Yes - `TOUCH_RST_GPIO` |
| I2C Port | I2C_NUM_0 | Yes - `TOUCH_I2C_NUM` |

### Customizing GPIO Pins

To use different GPIO pins, edit the following defines in [main/main.c](main/main.c):

```c
#define TOUCH_I2C_NUM I2C_NUM_0        // I2C port
#define TOUCH_I2C_SCL_GPIO 8           // I2C Clock pin
#define TOUCH_I2C_SDA_GPIO 4           // I2C Data pin
#define TOUCH_INT_GPIO 7               // Touch interrupt pin
#define TOUCH_RST_GPIO 6               // Touch reset pin
```

## I2C Configuration

The CST816D communicates at the following I2C address:

- **Device Address**: 0x15 (7-bit addressing)
- **Clock Speed**: 400 kHz (standard I2C)
- **Internal Pull-ups**: Enabled in software

The I2C bus is initialized in [main.c](main/main.c) with internal pull-up resistors enabled.

## Driver Implementation

The CST816D driver is implemented in two files:

### [src/cst816d.h](src/cst816d.h)
Public API header file containing:
- `cst816d_init()` - Initialize the touch sensor
- `cst816d_reset()` - Reset the device
- `cst816d_read_touch()` - Read touch coordinates and pressure
- `cst816d_get_info()` - Read chip ID and firmware version
- `cst816d_register_lvgl_indev()` - Register with LVGL
- `cst816d_deinit()` - Clean up resources

### [src/cst816d.c](src/cst816d.c)
Implementation file containing:
- I2C register read/write functions
- Touch data parsing
- LVGL input device callback
- Device initialization and reset logic

## LVGL Integration

The CST816D is automatically registered as an LVGL input device after initialization. Touch events are:
- Automatically passed to LVGL as pointer input events
- Converted to LVGL coordinate system
- Support both touch press and release events

### Touch Input Flow

```
CST816D (I2C) 
    ↓
cst816d_read_touch()
    ↓
cst816d_read_cb() (LVGL callback)
    ↓
LVGL Input Processing
    ↓
UI Event Handlers
```

## Register Map

The CST816D uses the following register addresses:

| Register | Address | Purpose |
|----------|---------|---------|
| Status | 0x00 | Touch status information |
| Point Count | 0x01 | Number of active touch points |
| Point 1 X High | 0x02 | X coordinate high byte |
| Point 1 X Low | 0x03 | X coordinate low byte |
| Point 1 Y High | 0x04 | Y coordinate high byte |
| Point 1 Y Low | 0x05 | Y coordinate low byte |
| Point 1 Pressure | 0x06 | Touch pressure value |
| Chip ID | 0xA7 | Chip identification |
| FW Version | 0xA9 | Firmware version |
| Reset | 0xFA | Software reset register |

## Troubleshooting

### Touch Not Responding

1. **Check I2C Communication**:
   - Verify GPIO pins are correctly defined
   - Ensure I2C pull-up resistors are present on the board
   - Check ESP-IDF logs for I2C initialization messages

2. **Check Device Detection**:
   - Look for log message: "CST816D initialized successfully - Chip ID: 0xXX, FW Version: 0xXX"
   - If initialization fails, verify I2C address and device power supply

3. **Check LVGL Integration**:
   - Verify log message: "CST816D touch sensor registered with LVGL"
   - Test by touching the display and watching for debug output

### Logging

Enable debug logging by adjusting the log level in [main.c](main/main.c):

```c
esp_log_level_set("CST816D", ESP_LOG_DEBUG);  // Enable CST816D debug logs
```

Debug output will show:
- Touch coordinates and pressure
- I2C read/write operations
- Device initialization progress
- LVGL input events

## Example Usage

The CST816D is initialized automatically in `app_main()`. To use touch input:

1. **In UI Code**: Handle LVGL events normally - touch input is transparent
2. **Custom Touch Processing**: Implement a custom LVGL input callback if needed
3. **Raw Touch Reading**: Call `cst816d_read_touch()` directly if bypassing LVGL

## Hardware Requirements

- **CST816D Module** with I2C interface
- **Pull-up Resistors**: 4.7-10kΩ on both SDA and SCL (usually on the module)
- **Power Supply**: 3.3V with adequate current capacity
- **Decoupling Capacitors**: 100nF between VDD and GND recommended

## Notes

- Touch coordinates are provided in pixel space matching the display resolution
- Pressure values are 0-255, with 0 indicating no touch
- Multiple touch points can be detected (though currently only Point 1 is processed)
- The driver is interrupt-driven but uses polling in the LVGL callback for simplicity
