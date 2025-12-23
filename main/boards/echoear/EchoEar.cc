#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "display/emote_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "backlight.h"
#include "esp32_camera.h"
#include "assets/lang_config.h"

#include <esp_log.h>

#include <driver/i2c_master.h>
#include <driver/i2c.h>
#include "i2c_device.h"
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_st77916.h>
#include "esp_lcd_touch_cst816s.h"
#include "touch.h"

#include "driver/temperature_sensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <cmath>
#include <ssid_manager.h>
#include <esp_wifi.h>

// BMI270 configuration firmware (required for accelerometer to function)
extern "C" {
extern const uint8_t bmi270_config_file[];
}

#define TAG "EchoEar"


temperature_sensor_handle_t temp_sensor = NULL;
static const st77916_lcd_init_cmd_t vendor_specific_init_yysj[] = {
    {0xF0, (uint8_t []){0x28}, 1, 0},
    {0xF2, (uint8_t []){0x28}, 1, 0},
    {0x73, (uint8_t []){0xF0}, 1, 0},
    {0x7C, (uint8_t []){0xD1}, 1, 0},
    {0x83, (uint8_t []){0xE0}, 1, 0},
    {0x84, (uint8_t []){0x61}, 1, 0},
    {0xF2, (uint8_t []){0x82}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x01}, 1, 0},
    {0xF1, (uint8_t []){0x01}, 1, 0},
    {0xB0, (uint8_t []){0x56}, 1, 0},
    {0xB1, (uint8_t []){0x4D}, 1, 0},
    {0xB2, (uint8_t []){0x24}, 1, 0},
    {0xB4, (uint8_t []){0x87}, 1, 0},
    {0xB5, (uint8_t []){0x44}, 1, 0},
    {0xB6, (uint8_t []){0x8B}, 1, 0},
    {0xB7, (uint8_t []){0x40}, 1, 0},
    {0xB8, (uint8_t []){0x86}, 1, 0},
    {0xBA, (uint8_t []){0x00}, 1, 0},
    {0xBB, (uint8_t []){0x08}, 1, 0},
    {0xBC, (uint8_t []){0x08}, 1, 0},
    {0xBD, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x80}, 1, 0},
    {0xC1, (uint8_t []){0x10}, 1, 0},
    {0xC2, (uint8_t []){0x37}, 1, 0},
    {0xC3, (uint8_t []){0x80}, 1, 0},
    {0xC4, (uint8_t []){0x10}, 1, 0},
    {0xC5, (uint8_t []){0x37}, 1, 0},
    {0xC6, (uint8_t []){0xA9}, 1, 0},
    {0xC7, (uint8_t []){0x41}, 1, 0},
    {0xC8, (uint8_t []){0x01}, 1, 0},
    {0xC9, (uint8_t []){0xA9}, 1, 0},
    {0xCA, (uint8_t []){0x41}, 1, 0},
    {0xCB, (uint8_t []){0x01}, 1, 0},
    {0xD0, (uint8_t []){0x91}, 1, 0},
    {0xD1, (uint8_t []){0x68}, 1, 0},
    {0xD2, (uint8_t []){0x68}, 1, 0},
    {0xF5, (uint8_t []){0x00, 0xA5}, 2, 0},
    {0xDD, (uint8_t []){0x4F}, 1, 0},
    {0xDE, (uint8_t []){0x4F}, 1, 0},
    {0xF1, (uint8_t []){0x10}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x02}, 1, 0},
    {0xE0, (uint8_t []){0xF0, 0x0A, 0x10, 0x09, 0x09, 0x36, 0x35, 0x33, 0x4A, 0x29, 0x15, 0x15, 0x2E, 0x34}, 14, 0},
    {0xE1, (uint8_t []){0xF0, 0x0A, 0x0F, 0x08, 0x08, 0x05, 0x34, 0x33, 0x4A, 0x39, 0x15, 0x15, 0x2D, 0x33}, 14, 0},
    {0xF0, (uint8_t []){0x10}, 1, 0},
    {0xF3, (uint8_t []){0x10}, 1, 0},
    {0xE0, (uint8_t []){0x07}, 1, 0},
    {0xE1, (uint8_t []){0x00}, 1, 0},
    {0xE2, (uint8_t []){0x00}, 1, 0},
    {0xE3, (uint8_t []){0x00}, 1, 0},
    {0xE4, (uint8_t []){0xE0}, 1, 0},
    {0xE5, (uint8_t []){0x06}, 1, 0},
    {0xE6, (uint8_t []){0x21}, 1, 0},
    {0xE7, (uint8_t []){0x01}, 1, 0},
    {0xE8, (uint8_t []){0x05}, 1, 0},
    {0xE9, (uint8_t []){0x02}, 1, 0},
    {0xEA, (uint8_t []){0xDA}, 1, 0},
    {0xEB, (uint8_t []){0x00}, 1, 0},
    {0xEC, (uint8_t []){0x00}, 1, 0},
    {0xED, (uint8_t []){0x0F}, 1, 0},
    {0xEE, (uint8_t []){0x00}, 1, 0},
    {0xEF, (uint8_t []){0x00}, 1, 0},
    {0xF8, (uint8_t []){0x00}, 1, 0},
    {0xF9, (uint8_t []){0x00}, 1, 0},
    {0xFA, (uint8_t []){0x00}, 1, 0},
    {0xFB, (uint8_t []){0x00}, 1, 0},
    {0xFC, (uint8_t []){0x00}, 1, 0},
    {0xFD, (uint8_t []){0x00}, 1, 0},
    {0xFE, (uint8_t []){0x00}, 1, 0},
    {0xFF, (uint8_t []){0x00}, 1, 0},
    {0x60, (uint8_t []){0x40}, 1, 0},
    {0x61, (uint8_t []){0x04}, 1, 0},
    {0x62, (uint8_t []){0x00}, 1, 0},
    {0x63, (uint8_t []){0x42}, 1, 0},
    {0x64, (uint8_t []){0xD9}, 1, 0},
    {0x65, (uint8_t []){0x00}, 1, 0},
    {0x66, (uint8_t []){0x00}, 1, 0},
    {0x67, (uint8_t []){0x00}, 1, 0},
    {0x68, (uint8_t []){0x00}, 1, 0},
    {0x69, (uint8_t []){0x00}, 1, 0},
    {0x6A, (uint8_t []){0x00}, 1, 0},
    {0x6B, (uint8_t []){0x00}, 1, 0},
    {0x70, (uint8_t []){0x40}, 1, 0},
    {0x71, (uint8_t []){0x03}, 1, 0},
    {0x72, (uint8_t []){0x00}, 1, 0},
    {0x73, (uint8_t []){0x42}, 1, 0},
    {0x74, (uint8_t []){0xD8}, 1, 0},
    {0x75, (uint8_t []){0x00}, 1, 0},
    {0x76, (uint8_t []){0x00}, 1, 0},
    {0x77, (uint8_t []){0x00}, 1, 0},
    {0x78, (uint8_t []){0x00}, 1, 0},
    {0x79, (uint8_t []){0x00}, 1, 0},
    {0x7A, (uint8_t []){0x00}, 1, 0},
    {0x7B, (uint8_t []){0x00}, 1, 0},
    {0x80, (uint8_t []){0x48}, 1, 0},
    {0x81, (uint8_t []){0x00}, 1, 0},
    {0x82, (uint8_t []){0x06}, 1, 0},
    {0x83, (uint8_t []){0x02}, 1, 0},
    {0x84, (uint8_t []){0xD6}, 1, 0},
    {0x85, (uint8_t []){0x04}, 1, 0},
    {0x86, (uint8_t []){0x00}, 1, 0},
    {0x87, (uint8_t []){0x00}, 1, 0},
    {0x88, (uint8_t []){0x48}, 1, 0},
    {0x89, (uint8_t []){0x00}, 1, 0},
    {0x8A, (uint8_t []){0x08}, 1, 0},
    {0x8B, (uint8_t []){0x02}, 1, 0},
    {0x8C, (uint8_t []){0xD8}, 1, 0},
    {0x8D, (uint8_t []){0x04}, 1, 0},
    {0x8E, (uint8_t []){0x00}, 1, 0},
    {0x8F, (uint8_t []){0x00}, 1, 0},
    {0x90, (uint8_t []){0x48}, 1, 0},
    {0x91, (uint8_t []){0x00}, 1, 0},
    {0x92, (uint8_t []){0x0A}, 1, 0},
    {0x93, (uint8_t []){0x02}, 1, 0},
    {0x94, (uint8_t []){0xDA}, 1, 0},
    {0x95, (uint8_t []){0x04}, 1, 0},
    {0x96, (uint8_t []){0x00}, 1, 0},
    {0x97, (uint8_t []){0x00}, 1, 0},
    {0x98, (uint8_t []){0x48}, 1, 0},
    {0x99, (uint8_t []){0x00}, 1, 0},
    {0x9A, (uint8_t []){0x0C}, 1, 0},
    {0x9B, (uint8_t []){0x02}, 1, 0},
    {0x9C, (uint8_t []){0xDC}, 1, 0},
    {0x9D, (uint8_t []){0x04}, 1, 0},
    {0x9E, (uint8_t []){0x00}, 1, 0},
    {0x9F, (uint8_t []){0x00}, 1, 0},
    {0xA0, (uint8_t []){0x48}, 1, 0},
    {0xA1, (uint8_t []){0x00}, 1, 0},
    {0xA2, (uint8_t []){0x05}, 1, 0},
    {0xA3, (uint8_t []){0x02}, 1, 0},
    {0xA4, (uint8_t []){0xD5}, 1, 0},
    {0xA5, (uint8_t []){0x04}, 1, 0},
    {0xA6, (uint8_t []){0x00}, 1, 0},
    {0xA7, (uint8_t []){0x00}, 1, 0},
    {0xA8, (uint8_t []){0x48}, 1, 0},
    {0xA9, (uint8_t []){0x00}, 1, 0},
    {0xAA, (uint8_t []){0x07}, 1, 0},
    {0xAB, (uint8_t []){0x02}, 1, 0},
    {0xAC, (uint8_t []){0xD7}, 1, 0},
    {0xAD, (uint8_t []){0x04}, 1, 0},
    {0xAE, (uint8_t []){0x00}, 1, 0},
    {0xAF, (uint8_t []){0x00}, 1, 0},
    {0xB0, (uint8_t []){0x48}, 1, 0},
    {0xB1, (uint8_t []){0x00}, 1, 0},
    {0xB2, (uint8_t []){0x09}, 1, 0},
    {0xB3, (uint8_t []){0x02}, 1, 0},
    {0xB4, (uint8_t []){0xD9}, 1, 0},
    {0xB5, (uint8_t []){0x04}, 1, 0},
    {0xB6, (uint8_t []){0x00}, 1, 0},
    {0xB7, (uint8_t []){0x00}, 1, 0},
    {0xB8, (uint8_t []){0x48}, 1, 0},
    {0xB9, (uint8_t []){0x00}, 1, 0},
    {0xBA, (uint8_t []){0x0B}, 1, 0},
    {0xBB, (uint8_t []){0x02}, 1, 0},
    {0xBC, (uint8_t []){0xDB}, 1, 0},
    {0xBD, (uint8_t []){0x04}, 1, 0},
    {0xBE, (uint8_t []){0x00}, 1, 0},
    {0xBF, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x10}, 1, 0},
    {0xC1, (uint8_t []){0x47}, 1, 0},
    {0xC2, (uint8_t []){0x56}, 1, 0},
    {0xC3, (uint8_t []){0x65}, 1, 0},
    {0xC4, (uint8_t []){0x74}, 1, 0},
    {0xC5, (uint8_t []){0x88}, 1, 0},
    {0xC6, (uint8_t []){0x99}, 1, 0},
    {0xC7, (uint8_t []){0x01}, 1, 0},
    {0xC8, (uint8_t []){0xBB}, 1, 0},
    {0xC9, (uint8_t []){0xAA}, 1, 0},
    {0xD0, (uint8_t []){0x10}, 1, 0},
    {0xD1, (uint8_t []){0x47}, 1, 0},
    {0xD2, (uint8_t []){0x56}, 1, 0},
    {0xD3, (uint8_t []){0x65}, 1, 0},
    {0xD4, (uint8_t []){0x74}, 1, 0},
    {0xD5, (uint8_t []){0x88}, 1, 0},
    {0xD6, (uint8_t []){0x99}, 1, 0},
    {0xD7, (uint8_t []){0x01}, 1, 0},
    {0xD8, (uint8_t []){0xBB}, 1, 0},
    {0xD9, (uint8_t []){0xAA}, 1, 0},
    {0xF3, (uint8_t []){0x01}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0x21, (uint8_t []){}, 0, 0},
    {0x11, (uint8_t []){}, 0, 0},
    {0x00, (uint8_t []){}, 0, 120},
};
float tsens_value;
gpio_num_t AUDIO_I2S_GPIO_DIN = AUDIO_I2S_GPIO_DIN_1;
gpio_num_t AUDIO_CODEC_PA_PIN = AUDIO_CODEC_PA_PIN_1;
gpio_num_t QSPI_PIN_NUM_LCD_RST = QSPI_PIN_NUM_LCD_RST_1;
gpio_num_t TOUCH_PAD2 = TOUCH_PAD2_1;
gpio_num_t UART1_TX = UART1_TX_1;
gpio_num_t UART1_RX = UART1_RX_1;

// ============================================================================
// LED Controller - 绿色状态指示灯
// ============================================================================
class StatusLed {
public:
    StatusLed(gpio_num_t pin) : pin_(pin), current_state_(false) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pin_),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        Off();
    }

    void On() {
        gpio_set_level(pin_, 0); // Active Low: 0 is ON
        current_state_ = true;
    }

    void Off() {
        gpio_set_level(pin_, 1); // Active Low: 1 is OFF
        current_state_ = false;
    }

    void Toggle() {
        if (current_state_) Off();
        else On();
    }

    void Blink(int times, int delay_ms = 200) {
        for (int i = 0; i < times; i++) {
            On();
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            Off();
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }

    bool IsOn() const { return current_state_; }

private:
    gpio_num_t pin_;
    bool current_state_;
};

// ============================================================================
// MPU6050 6轴陀螺仪+加速度计
// ============================================================================
class Mpu6050 : public I2cDevice {
public:
    enum GestureEvent {
        GESTURE_NONE = 0,
        GESTURE_FLIP,        // 翻转（屏幕朝下）
        GESTURE_UNFLIP,      // 翻转恢复（屏幕朝上）
        GESTURE_SHAKE,       // 摇晃
        GESTURE_FAST_SHAKE,  // 快速摇晃 (交互触发)
        GESTURE_DOUBLE_TAP,  // 双击
        GESTURE_TAP,         // 单击
        GESTURE_LIFT         // 抬腕/拿起
    };

    enum ChipType {
        TYPE_NONE = 0,
        TYPE_MPU6050,
        TYPE_BMI270
    };

    Mpu6050(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr), address_(addr), bus_handle_(i2c_bus)
    {
        Init();
    }

    bool Init() {
        uint8_t chip_id = 0;
        
        ESP_LOGI("IMU", "=== IMU Init Start (addr=0x%02X) ===", address_);
        
        // 尝试读取 WHO_AM_I (0x75) - MPU6050
        ReadRegs(0x75, &chip_id, 1);
        ESP_LOGI("IMU", "Read WHO_AM_I (0x75): 0x%02X", chip_id);
        
        if (chip_id == 0x68 || chip_id == 0x11 || chip_id == 0x69) {
            chip_type_ = TYPE_MPU6050;
            ESP_LOGI("IMU", "=== MPU6050/ICM Detected! ===");
            ESP_LOGI(TAG, "  Vendor: InvenSense (TDK)");
            
            // MPU6050 Init Sequence
            WriteReg(0x6B, 0x00); // Wake up
            vTaskDelay(pdMS_TO_TICKS(100));
            WriteReg(0x6B, 0x01); // Set Clock Source
            vTaskDelay(pdMS_TO_TICKS(10));
            WriteReg(0x1C, 0x00); // Accel Range +/- 2g
            WriteReg(0x1B, 0x00); // Gyro Range +/- 250 deg/s
            
            ESP_LOGI("IMU", "MPU6050 configuration complete.");
            initialized_ = true;
            return true;
        } 
        
        // 尝试读取 BMI270 Chip ID (0x00)
        ReadRegs(0x00, &chip_id, 1);
        ESP_LOGI("IMU", "Read Chip ID (0x00): 0x%02X", chip_id);
        
        if (chip_id == 0x24) {
            chip_type_ = TYPE_BMI270;
            ESP_LOGI("IMU", "=== BMI270 Detected! ===");
            ESP_LOGI(TAG, "  Vendor: Bosch Sensortec");
            
            // ============================================
            // BMI270 Full Initialization with Firmware Upload (Burst Write)
            // ============================================
            ESP_LOGI("IMU", "Uploading BMI270 firmware (Burst Write)...");

            // Create a temporary device handle for raw burst writes
            i2c_master_dev_handle_t bmi_dev_handle = nullptr;
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = address_,
                .scl_speed_hz = 100000, // Safe speed for config upload
            };
            if (i2c_master_bus_add_device(bus_handle_, &dev_cfg, &bmi_dev_handle) != ESP_OK) {
                ESP_LOGE("IMU", "Failed to create temp I2C device for BMI270");
                return false;
            }
            
            // Step 1: Soft Reset
            // WriteReg(0x7E, 0xB6);
            uint8_t reset_cmd[] = {0x7E, 0xB6};
            i2c_master_transmit(bmi_dev_handle, reset_cmd, 2, -1);
            vTaskDelay(pdMS_TO_TICKS(50));
            
            // Wait for boot (check status)
            // Can use existing ReadRegs/WriteReg or the new handle
            // Switch to new handle for consistency for Init sequence
            
            // Step 2: Disable advanced power save mode for config upload
            uint8_t pwr_conf[] = {0x7C, 0x00};
            i2c_master_transmit(bmi_dev_handle, pwr_conf, 2, -1);
            vTaskDelay(pdMS_TO_TICKS(1));
            
            // Step 3: Prepare for config load
            uint8_t init_ctrl[] = {0x59, 0x00};
            i2c_master_transmit(bmi_dev_handle, init_ctrl, 2, -1);
            
            // Step 4: Upload config file (8KB) using BURST Writes
            const size_t config_size = 8192;
            const size_t chunk_size = 64; // Max burst size
            uint8_t burst_buffer[chunk_size + 1]; // Reg + Data

            for (size_t offset = 0; offset < config_size; offset += chunk_size) {
                // Set burst write address
                uint8_t addr_low = (offset / 2) & 0x0F;
                uint8_t addr_high = ((offset / 2) >> 4) & 0xFF;
                
                uint8_t set_addr_0[] = {0x5B, addr_low};
                i2c_master_transmit(bmi_dev_handle, set_addr_0, 2, -1);
                
                uint8_t set_addr_1[] = {0x5C, addr_high};
                i2c_master_transmit(bmi_dev_handle, set_addr_1, 2, -1);
                
                // Write chunk to INIT_DATA register (0x5E)
                size_t current_chunk_size = (offset + chunk_size <= config_size) ? chunk_size : (config_size - offset);
                
                burst_buffer[0] = 0x5E; // INIT_DATA register
                memcpy(&burst_buffer[1], &bmi270_config_file[offset], current_chunk_size);
                
                // Transmit [Reg, Data...] in one transaction
                esp_err_t ret = i2c_master_transmit(bmi_dev_handle, burst_buffer, current_chunk_size + 1, -1);
                if (ret != ESP_OK) {
                     ESP_LOGE("IMU", "Firmware upload failed at offset %d: %s", offset, esp_err_to_name(ret));
                     i2c_master_bus_rm_device(bmi_dev_handle);
                     return false;
                }
            }
            
            // Step 5: Complete initialization
            uint8_t init_complete[] = {0x59, 0x01};
            i2c_master_transmit(bmi_dev_handle, init_complete, 2, -1);
            vTaskDelay(pdMS_TO_TICKS(150));
            
            // Cleanup temp handle
            i2c_master_bus_rm_device(bmi_dev_handle);

            // Step 6: Check INTERNAL_STATUS (back to standard methods)
            uint8_t init_status = 0;
            ReadRegs(0x21, &init_status, 1);
            ESP_LOGI("IMU", "BMI270 Init Status (0x21): 0x%02X", init_status);

            if ((init_status & 0x01) != 0x01) {
                ESP_LOGE("IMU", "BMI270 init failed! Status: 0x%02X", init_status);
                initialized_ = false;
                return false;
            }
            ESP_LOGI("IMU", "BMI270 firmware upload complete.");
            
            // Step 7: Enable accelerometer
            WriteReg(0x7D, 0x04);  // PWR_CTRL: enable ACC
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // Step 8: Configure accelerometer
            // ACC_CONF (0x40): ODR=100Hz, BWP=normal, filter_perf=high
            WriteReg(0x40, 0xA8);
            // ACC_RANGE (0x41): +/- 2g
            WriteReg(0x41, 0x00);
            
            ESP_LOGI("IMU", "BMI270 configuration complete.");
            initialized_ = true;
            return true;
        }

        ESP_LOGE("IMU", "=== NO IMU FOUND at 0x%02X! ===", address_);
        initialized_ = false;
        return false;
    }

    void Update() {
        if (!initialized_) {
            static int not_init_counter = 0;
            if (++not_init_counter >= 50) {
                not_init_counter = 0;
                ESP_LOGW("IMU", "Update skipped: NOT INITIALIZED! chip_type=%d", chip_type_);
            }
            return;
        }
        
        float ax_raw_f = 0, ay_raw_f = 0, az_raw_f = 0;
        if (!ReadRawAccel(ax_raw_f, ay_raw_f, az_raw_f)) return;

        // 1. 摇晃检测 (优先级最高！必须使用原始数据，避免滤波器平滑掉高频信号)
        DetectShakeRaw(ax_raw_f, ay_raw_f, az_raw_f);

        // 2. 低通滤波 (Alpha Filter) - 用于姿态判断
        const float alpha = 0.2f;
        ax_ = (1.0f - alpha) * ax_ + alpha * ax_raw_f;
        ay_ = (1.0f - alpha) * ay_ + alpha * ay_raw_f;
        float az_prev = az_;
        az_ = (1.0f - alpha) * az_ + alpha * az_raw_f;

        // 3. 抬腕检测 (从垂直/侧卧回到水平正面 Z ~ -1)
        DetectLift();

        // 4. 翻转检测 (监测面朝下 Z ~ +1)
        DetectFlip();

        // 5. 敲击检测 (最后检测，受 DetectShake 生成的屏蔽保护)
        float az_delta = fabsf(az_raw_f - az_prev);
        DetectTap(az_delta);

        // 6. 调试输出 (减低频率)
        static int debug_check_counter = 0;
        if (++debug_check_counter >= 50) {
            debug_check_counter = 0;
            float mag = sqrtf(ax_*ax_ + ay_*ay_ + az_*az_);
            ESP_LOGI("IMU_DEBUG", "Filt: X=%.2f Y=%.2f Z=%.2f Mag=%.2f Flip=%d", ax_, ay_, az_, mag, is_flipped_);
        }
    }

private:
    bool ReadRawAccel(float &ax, float &ay, float &az) {
        int16_t rax=0, ray=0, raz=0;
        uint8_t buffer[6];

        if (chip_type_ == TYPE_MPU6050) {
            ReadRegs(0x3B, buffer, 6);
            rax = (int16_t)((buffer[0] << 8) | buffer[1]);
            ray = (int16_t)((buffer[2] << 8) | buffer[3]);
            raz = (int16_t)((buffer[4] << 8) | buffer[5]);
        } else if (chip_type_ == TYPE_BMI270) {
            ReadRegs(0x0C, buffer, 6);
            rax = (int16_t)((buffer[1] << 8) | buffer[0]);
            ray = (int16_t)((buffer[3] << 8) | buffer[2]);
            raz = (int16_t)((buffer[5] << 8) | buffer[4]);
        } else return false;

        ax = rax / 16384.0f;
        ay = ray / 16384.0f;
        az = raz / 16384.0f;
        return true;
    }

    void DetectFlip() {
        // 根据实测：Z=-1 是正面，Z=+1 是背面。所以 Z > 0.5 为翻转朝下
        bool new_flipped = (az_ > 0.5f);  
        if (new_flipped != is_flipped_) {
            if (++flip_stable_count_ >= 3) {
                is_flipped_ = new_flipped;
                flip_stable_count_ = 0;
                last_gesture_ = is_flipped_ ? GESTURE_FLIP : GESTURE_UNFLIP;
                ESP_LOGI("IMU", "Flip Event: %s", is_flipped_ ? "FACE DOWN (Muted)" : "FACE UP (Active)");
            }
        } else {
            flip_stable_count_ = 0;
        }
    }

    void DetectShakeRaw(float ax_raw, float ay_raw, float az_raw) {
        // 使用原始加速度值计算模长，避免滤波器削弱高频信号
        float magnitude = sqrtf(ax_raw*ax_raw + ay_raw*ay_raw + az_raw*az_raw);
        
        // 调试：每50次采样输出一次原始模长
        static int raw_debug_counter = 0;
        if (++raw_debug_counter >= 50) {
            raw_debug_counter = 0;
            ESP_LOGI("IMU_RAW", "RawMag=%.2fg (threshold: 1.4g=shake, 1.6g=fast)", magnitude);
        }
        
        // 强效护盾逻辑：只要设备在动（模长偏离 1.0g 较大），就更新摇晃时间戳，保护 DetectTap
        if (magnitude > 1.25f || magnitude < 0.75f) {
            last_shake_time_raw_ = esp_timer_get_time() / 1000;
        }

        // 快速摇晃抱怨 (降低阈值到 1.6g)
        if (magnitude > 1.6f) {
            last_gesture_ = GESTURE_FAST_SHAKE;
            ESP_LOGI("IMU", ">>> FAST SHAKE! Mag: %.2fg <<<", magnitude);
        } else if (magnitude > 1.4f) { // 普通摇晃亮屏 (降低阈值到 1.4g)
            if (++shake_stable_count_ >= 2) { 
                 last_gesture_ = GESTURE_SHAKE;
                 shake_stable_count_ = 0;
                 ESP_LOGI("IMU", "Shake detected, Mag: %.2fg", magnitude);
            }
        } else {
            shake_stable_count_ = 0;
        }
    }

    void DetectLift() {
        static float last_az = -1.0f;
        if (last_az > -0.2f && az_ < -0.85f) {
            last_gesture_ = GESTURE_LIFT;
            ESP_LOGI("IMU", "Lift to Wake (Back to Home) Detected!");
        }
        last_az = az_;
    }

    void DetectTap(float delta) {
        static int64_t last_tap_time = 0;
        static int64_t last_double_tap_time = 0;
        int64_t now = esp_timer_get_time() / 1000;

        // 核心防御：如果 800ms 内有明显的设备位移/摇晃，严禁识别成双击敲击
        if (now - last_shake_time_raw_ < 800) {
            last_tap_time = 0; 
            return;
        }

        // 双击后的冷却时间
        if (now - last_double_tap_time < 1000) return;

        // 提高阈值到 1.0g，过滤掉普通手持波动
        if (delta > 1.0f) {
            int64_t interval = now - last_tap_time;
            if (interval > 150 && interval < 600) {
                last_gesture_ = GESTURE_DOUBLE_TAP;
                ESP_LOGI("IMU", "Double Tap Detected! Interval: %d ms", (int)interval);
                last_double_tap_time = now;
                last_tap_time = 0;
            } else {
                last_tap_time = now;
            }
        }
    }

public:
    GestureEvent GetLastGesture() {
        GestureEvent gesture = last_gesture_;
        last_gesture_ = GESTURE_NONE;
        return gesture;
    }

    bool IsFlipped() const { return is_flipped_; }
    bool IsSideStanding() const {
        return (fabsf(az_) < 0.6f) && (fabsf(ax_) > 0.7f || fabsf(ay_) > 0.7f);
    }
    float GetAccelX() const { return ax_; }
    float GetAccelY() const { return ay_; }
    float GetAccelZ() const { return az_; }

private:
    uint8_t address_;  // I2C address
    i2c_master_bus_handle_t bus_handle_;
    bool initialized_ = false;
    ChipType chip_type_ = TYPE_NONE;
    float ax_ = 0, ay_ = 0, az_ = 1.0f;
    bool is_flipped_ = false;
    int flip_stable_count_ = 0;
    int shake_stable_count_ = 0;
    int64_t last_shake_time_raw_ = 0;
    GestureEvent last_gesture_ = GESTURE_NONE;
};


        
        class Charge : public I2cDevice {
public:
    Charge(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        // 初始化滤波器缓冲区
        for (int i = 0; i < FILTER_SIZE; i++) {
            voltage_buffer_[i] = 4200;
        }
        last_update_time_ = esp_timer_get_time();
    }

    void Update()
    {
        // 读取 I2C 寄存器获取电压电流数据
        ReadRegs(0x08, read_buffer_, 2);
        ReadRegs(0x0c, read_buffer_ + 2, 2);
        
        // 转换数据 (假设是 mV)
        uint16_t raw_voltage = static_cast<uint16_t>(read_buffer_[1] << 8 | read_buffer_[0]);
        // 读取电流 (假设单位 mA，有符号)
        int16_t raw_current = static_cast<int16_t>(read_buffer_[3] << 8 | read_buffer_[2]);

        // 滑动平均滤波
        voltage_buffer_[buffer_index_] = raw_voltage;
        buffer_index_ = (buffer_index_ + 1) % FILTER_SIZE;
        
        // 计算平均电压
        uint32_t sum = 0;
        for (int i = 0; i < FILTER_SIZE; i++) {
            sum += voltage_buffer_[i];
        }
        voltage_mV_ = sum / FILTER_SIZE;
        current_mA_ = raw_current;

        // 判断充电状态 - 使用更可靠的判断
        bool new_charging = (current_mA_ > 50 && voltage_mV_ < 4300);
        
        // 检测充电状态变化
        if (new_charging != is_charging_) {
            charging_state_count_++;
            if (charging_state_count_ >= 3) {  // 连续3次状态一致才改变
                // 状态切换时，记录开路电压（OCV）对应的真实电量
                if (!new_charging && is_charging_) {
                    // 从充电切换到放电：等待一小段时间让电压稳定
                    ocv_stable_count_ = 0;
                } else if (new_charging && !is_charging_) {
                    // 从放电切换到充电：记录当前电量作为基准
                    charge_start_percentage_ = percentage_;
                    charge_accumulated_mAh_ = 0;
                }
                is_charging_ = new_charging;
                charging_state_count_ = 0;
            }
        } else {
            charging_state_count_ = 0;
        }

        // 计算时间差（秒）
        int64_t current_time = esp_timer_get_time();
        float delta_time_s = (current_time - last_update_time_) / 1000000.0f;
        last_update_time_ = current_time;

        if (is_charging_) {
            // 充电模式：使用库仑计法估算电量增加
            // 假设电池容量约 1000mAh
            const float BATTERY_CAPACITY_mAh = 1000.0f;
            
            // 累积充电量 (mAh)
            if (current_mA_ > 50) {
                float charge_delta = (current_mA_ * delta_time_s) / 3600.0f;  // mAh
                charge_accumulated_mAh_ += charge_delta;
                
                // 计算充电增加的百分比
                int charge_increase = (int)(charge_accumulated_mAh_ / BATTERY_CAPACITY_mAh * 100);
                int estimated_percentage = charge_start_percentage_ + charge_increase;
                
                // 限制在合理范围
                if (estimated_percentage > 100) estimated_percentage = 100;
                if (estimated_percentage < charge_start_percentage_) {
                    estimated_percentage = charge_start_percentage_;
                }
                
                // 平滑更新
                if (abs(estimated_percentage - percentage_) >= 1) {
                    percentage_ = estimated_percentage;
                }
            }
            
            // 充电完成检测：电流很小且电压高
            if (current_mA_ < 100 && voltage_mV_ >= 4150) {
                percentage_ = 100;
            }
        } else {
            // 放电模式：使用电压-电量曲线
            // 等待电压稳定（刚拔掉充电器时电压会下降）
            if (ocv_stable_count_ < 5) {
                ocv_stable_count_++;
                return;  // 等待稳定，不更新电量
            }
            
            int new_percentage = VoltageToPercentage(voltage_mV_);
            
            // 防抖动：只有变化超过阈值才更新
            if (abs(new_percentage - percentage_) >= PERCENTAGE_THRESHOLD) {
                percentage_ = new_percentage;
            }
        }

        // 限制范围
        if (percentage_ > 100) percentage_ = 100;
        if (percentage_ < 0) percentage_ = 0;
    }

    void GetBatteryInfo(int &level, bool &charging) {
        level = percentage_;
        charging = is_charging_;
    }

    int16_t GetCurrent() { return current_mA_; }

private:
    static const int FILTER_SIZE = 5;
    static const int PERCENTAGE_THRESHOLD = 2;
    
    // 电压到电量的映射（放电曲线）
    int VoltageToPercentage(uint16_t voltage_mV) {
        if (voltage_mV >= 4100) {
            return 100;
        } else if (voltage_mV >= 3950) {
            // 3.95V-4.1V: 90%-100%
            return 90 + (voltage_mV - 3950) * 10 / 150;
        } else if (voltage_mV >= 3850) {
            // 3.85V-3.95V: 70%-90%
            return 70 + (voltage_mV - 3850) * 20 / 100;
        } else if (voltage_mV >= 3750) {
            // 3.75V-3.85V: 40%-70%
            return 40 + (voltage_mV - 3750) * 30 / 100;
        } else if (voltage_mV >= 3650) {
            // 3.65V-3.75V: 20%-40%
            return 20 + (voltage_mV - 3650) * 20 / 100;
        } else if (voltage_mV >= 3500) {
            // 3.5V-3.65V: 5%-20%
            return 5 + (voltage_mV - 3500) * 15 / 150;
        } else if (voltage_mV >= 3300) {
            // 3.3V-3.5V: 0%-5%
            return (voltage_mV - 3300) * 5 / 200;
        } else {
            return 0;
        }
    }
    
    uint8_t read_buffer_[4];
    int percentage_ = 100;
    uint16_t voltage_mV_ = 4200;
    int16_t current_mA_ = 0;
    bool is_charging_ = false;
    
    // 滑动平均滤波器
    uint16_t voltage_buffer_[FILTER_SIZE];
    int buffer_index_ = 0;
    
    // 充电状态防抖动
    int charging_state_count_ = 0;
    
    // 库仑计相关
    int64_t last_update_time_;  // 微秒
    int charge_start_percentage_ = 0;  // 开始充电时的电量
    float charge_accumulated_mAh_ = 0;  // 累积充电量
    
    // OCV稳定计数
    int ocv_stable_count_ = 0;
};

// ... (Cst816s update skippied as it is unchanged, referring to original file content if needed) ...

class Cst816s : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };

    enum TouchEvent {
        TOUCH_NONE,
        TOUCH_PRESS,
        TOUCH_RELEASE,
        TOUCH_HOLD
    };

    Cst816s(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        was_touched_ = false;
        press_count_ = 0;

        // Create touch interrupt semaphore
        touch_isr_mux_ = xSemaphoreCreateBinary();
        if (touch_isr_mux_ == NULL) {
            ESP_LOGE("EchoEar", "Failed to create touch semaphore");
        }
    }

    ~Cst816s()
    {
        // Delete semaphore if it exists
        if (touch_isr_mux_ != NULL) {
            vSemaphoreDelete(touch_isr_mux_);
            touch_isr_mux_ = NULL;
        }
    }

    void UpdateTouchPoint()
    {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    const TouchPoint_t &GetTouchPoint()
    {
        return tp_;
    }

    TouchEvent CheckTouchEvent()
    {
        bool is_touched = (tp_.num > 0);
        TouchEvent event = TOUCH_NONE;

        if (is_touched && !was_touched_) {
            // Press event (transition from not touched to touched)
            press_count_++;
            event = TOUCH_PRESS;
            ESP_LOGI("EchoEar", "TOUCH PRESS - count: %d, x: %d, y: %d", press_count_, tp_.x, tp_.y);
        } else if (!is_touched && was_touched_) {
            // Release event (transition from touched to not touched)
            event = TOUCH_RELEASE;
            ESP_LOGI("EchoEar", "TOUCH RELEASE - total presses: %d", press_count_);
        } else if (is_touched && was_touched_) {
            // Continuous touch (hold)
            event = TOUCH_HOLD;
            ESP_LOGD("EchoEar", "TOUCH HOLD - x: %d, y: %d", tp_.x, tp_.y);
        }

        // Update previous state
        was_touched_ = is_touched;
        return event;
    }

    int GetPressCount() const
    {
        return press_count_;
    }

    void ResetPressCount()
    {
        press_count_ = 0;
    }

    // Semaphore management methods
    SemaphoreHandle_t GetTouchSemaphore()
    {
        return touch_isr_mux_;
    }

    bool WaitForTouchEvent(TickType_t timeout = portMAX_DELAY)
    {
        if (touch_isr_mux_ != NULL) {
            return xSemaphoreTake(touch_isr_mux_, timeout) == pdTRUE;
        }
        return false;
    }

    void NotifyTouchEvent()
    {
        if (touch_isr_mux_ != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(touch_isr_mux_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

private:
    uint8_t read_buffer_[6];
    TouchPoint_t tp_;

    // Touch state tracking
    bool was_touched_;
    int press_count_;

    // Touch interrupt semaphore
    SemaphoreHandle_t touch_isr_mux_;
};

class EchoEar : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Cst816s* cst816s_;
    Charge* charge_;
    Mpu6050* mpu6050_ = nullptr;
    StatusLed* status_led_ = nullptr;
    Button boot_button_;
    Display* display_ = nullptr;
    PwmBacklight* backlight_ = nullptr;
    esp_timer_handle_t touchpad_timer_;
    esp_lcd_touch_handle_t tp;   // LCD touch handle
    Esp32Camera* camera_ = nullptr;
    
    // 记录最后一次交互时间 (tick count)
    uint32_t last_interaction_tick_ = 0;
    bool screen_off_ = false;
    bool flip_muted_ = false;  // 翻转静音状态
    TickType_t last_shake_protection_tick_ = 0;  // 摇晃保护时间戳

    void InitializeI2c()
    {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
        ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
        ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

        // I2C总线扫描 - 检测所有连接的硬件设备
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "    I2C Bus Scan - Hardware Detection");
        ESP_LOGI(TAG, "========================================");
        
        // 已知设备地址映射
        struct I2cDevice {
            uint8_t addr;
            const char* name;
            const char* description;
            bool required;
        };
        
        const I2cDevice known_devices[] = {
            // 音频芯片
            {0x18, "ES8311", "Audio DAC (Speaker)", true},
            {0x40, "ES7210", "Audio ADC (Microphone)", true},
            // 触摸
            {0x15, "CST816S", "Touch Controller", true},
            // 电源管理
            {0x55, "Charge IC", "Battery/Charge Management", true},
            // 常见传感器 (可能存在)
            {0x68, "MPU6050/ICM20600", "6-Axis Gyroscope+Accelerometer", false},
            {0x69, "MPU6050 Alt", "6-Axis Gyroscope+Accelerometer (Alt Addr)", false},
            {0x1E, "HMC5883L", "3-Axis Magnetometer", false},
            {0x0D, "QMC5883L", "3-Axis Magnetometer (Alt)", false},
            {0x76, "BME280/BMP280", "Temp/Humidity/Pressure Sensor", false},
            {0x77, "BMP180/BME280 Alt", "Pressure Sensor (Alt Addr)", false},
            {0x23, "BH1750", "Ambient Light Sensor", false},
            {0x44, "SHT30/SHT40", "Temp/Humidity Sensor", false},
            {0x48, "ADS1115", "16-bit ADC", false},
            {0x50, "EEPROM", "24C32 EEPROM", false},
            {0x57, "MAX30102", "Heart Rate/SpO2 Sensor", false},
            {0x5A, "MLX90614", "IR Temperature Sensor", false},
            {0x29, "VL53L0X", "ToF Distance Sensor", false},
            {0x38, "AHT20", "Temp/Humidity Sensor", false},
            {0x39, "APDS9960", "Gesture/Color/Proximity Sensor", false},
            {0x3C, "SSD1306", "OLED Display", false},
        };
        
        int detected_count = 0;
        int required_missing = 0;
        
        ESP_LOGI(TAG, "Scanning I2C bus (SDA:%d, SCL:%d)...", AUDIO_CODEC_I2C_SDA_PIN, AUDIO_CODEC_I2C_SCL_PIN);
        ESP_LOGI(TAG, "----------------------------------------");
        
        // 扫描所有可能的I2C地址
        for (uint8_t addr = 0x08; addr < 0x78; addr++) {
            esp_err_t ret = i2c_master_probe(i2c_bus_, addr, 100);
            if (ret == ESP_OK) {
                // 查找已知设备名
                const char* name = "Unknown";
                const char* desc = "";
                bool is_known = false;
                
                for (const auto& dev : known_devices) {
                    if (dev.addr == addr) {
                        name = dev.name;
                        desc = dev.description;
                        is_known = true;
                        break;
                    }
                }
                
                if (is_known) {
                    ESP_LOGI(TAG, "  [0x%02X] ✓ %s - %s", addr, name, desc);
                } else {
                    ESP_LOGW(TAG, "  [0x%02X] ? Unknown device detected", addr);
                }
                detected_count++;
            }
        }
        
        ESP_LOGI(TAG, "----------------------------------------");
        ESP_LOGI(TAG, "Total devices found: %d", detected_count);
        
        // 检查必需设备是否存在
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Required devices check:");
        for (const auto& dev : known_devices) {
            if (dev.required) {
                esp_err_t ret = i2c_master_probe(i2c_bus_, dev.addr, 100);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "  [0x%02X] ✓ %s - OK", dev.addr, dev.name);
                } else {
                    ESP_LOGE(TAG, "  [0x%02X] ✗ %s - MISSING!", dev.addr, dev.name);
                    required_missing++;
                }
            }
        }
        
        // 检查未使用的GPIO
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "    Unused Hardware (Not in code)");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGW(TAG, "  GPIO43 - Green LED (LED_G) - Available");
        ESP_LOGW(TAG, "  GPIO17/16/38 - SD Card (MISO/SCK/MOSI) - Available");
        ESP_LOGW(TAG, "  GPIO5/6 - UART1 (TX/RX) - Available");
        ESP_LOGW(TAG, "  GPIO6/7 - Capacitive Touch Pads - Available");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "");

    }
    uint8_t DetectPcbVersion()
    {
        esp_err_t ret = i2c_master_probe(i2c_bus_, 0x18, 100);
        uint8_t pcb_verison = 0;
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "PCB verison V1.0");
            pcb_verison = 0;
        } else {
            gpio_config_t gpio_conf = {
                .pin_bit_mask = (1ULL << GPIO_NUM_48),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            ESP_ERROR_CHECK(gpio_config(&gpio_conf));
            ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_48, 1));
            vTaskDelay(pdMS_TO_TICKS(100));
            ret = i2c_master_probe(i2c_bus_, 0x18, 100);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "PCB verison V1.2");
                pcb_verison = 1;
                AUDIO_I2S_GPIO_DIN = AUDIO_I2S_GPIO_DIN_2;
                AUDIO_CODEC_PA_PIN = AUDIO_CODEC_PA_PIN_2;
                QSPI_PIN_NUM_LCD_RST = QSPI_PIN_NUM_LCD_RST_2;
                TOUCH_PAD2 = TOUCH_PAD2_2;
                UART1_TX = UART1_TX_2;
                UART1_RX = UART1_RX_2;
            } else {
                ESP_LOGE(TAG, "PCB version detection error");

            }
        }
        return pcb_verison;
    }

    static void touch_isr_callback(void* arg)
    {
        Cst816s* touchpad = static_cast<Cst816s*>(arg);
        if (touchpad != nullptr) {
            touchpad->NotifyTouchEvent();
        }
    }

    static void touch_event_task(void* arg)
    {
        Cst816s* touchpad = static_cast<Cst816s*>(arg);
        if (touchpad == nullptr) {
            ESP_LOGE(TAG, "Invalid touchpad pointer in touch_event_task");
            vTaskDelete(NULL);
            return;
        }

        while (true) {
            if (touchpad->WaitForTouchEvent()) {
                auto &app = Application::GetInstance();
                auto &board = (EchoEar &)Board::GetInstance();

                ESP_LOGI(TAG, "Touch event, TP_PIN_NUM_INT: %d", gpio_get_level(TP_PIN_NUM_INT));
                touchpad->UpdateTouchPoint();
                auto touch_event = touchpad->CheckTouchEvent();

                if (touch_event == Cst816s::TOUCH_RELEASE) {
                    if (app.GetDeviceState() == kDeviceStateStarting) {
                        board.EnterWifiConfigMode();
                        continue;
                    }

                    // Double-tap and Single-tap logic moved here to ensure it only runs on Release
                    static uint32_t last_tap_time = 0;
                    uint32_t current_time = xTaskGetTickCount();
                    
                    if (board.screen_off_) {
                        // Screen detected OFF: Single tap -> Turn ON screen only.
                        board.screen_off_ = false;
                        board.GetBacklight()->RestoreBrightness();
                        ESP_LOGI(TAG, "Screen OFF -> ON (Single Tap)");
                        // Do NOT toggle AI here.
                        last_tap_time = current_time; 
                    } else {
                        // Screen detected ON.
                        auto state = app.GetDeviceState();
                        
                        // Case 1: AI is Speaking/Listening -> Single tap should interrupt.
                        if (state == kDeviceStateSpeaking || state == kDeviceStateListening) {
                             ESP_LOGI(TAG, "AI Active, Single Tap -> Interrupt/Stop");
                             app.SetDeviceState(kDeviceStateIdle);
                             last_tap_time = 0; // Reset double tap timer
                        } 
                        // Case 2: AI is Idle -> Check for Double Tap to Wake AI.
                        else {
                            if (current_time - last_tap_time < pdMS_TO_TICKS(400)) {
                                // Double tap detected!
                                ESP_LOGI(TAG, "Double Tap -> Toggle Chat State (Wake AI)");
                                app.ToggleChatState();
                                last_tap_time = 0; // Reset
                            } else {
                                // First tap recorded
                                last_tap_time = current_time;
                                ESP_LOGI(TAG, "Single Tap recorded, waiting for second...");
                            }
                        }
                    }
                    
                    // Always update interaction time
                    board.last_interaction_tick_ = current_time;
                }
            }
        }
    }

    static void power_save_task(void* arg) {
        EchoEar* board = static_cast<EchoEar*>(arg);
        int led_blink_counter = 0;
        
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(50)); // 从 10Hz 提升到 20Hz，手势更灵敏
            
            // 手动执行 Charge 的更新
            if (board->charge_) {
                board->charge_->Update();
            }

            // ============================================
            // 网络重连检测 (每30秒检查一次)
            // ============================================
            static int network_check_counter = 0;
            network_check_counter++;
            if (network_check_counter >= 600) {  // 50ms * 600 = 30s
                network_check_counter = 0;
                
                wifi_ap_record_t ap_info;
                if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
                    // WiFi未连接，尝试重连
                    ESP_LOGW(TAG, "WiFi disconnected, attempting reconnect...");
                    esp_wifi_connect();
                }
            }

            int battery_level = 0;
            bool charging = false;
            if (board->charge_) {
                board->charge_->GetBatteryInfo(battery_level, charging);
            }

            auto& app = Application::GetInstance();
            auto state = app.GetDeviceState();

            // ============================================
            // MPU6050 陀螺仪手势检测
            // ============================================
            static int mpu_debug_counter = 0;
            if (board->mpu6050_) {
                board->mpu6050_->Update();
                
                // 每5秒打印一次加速度值用于调试
                mpu_debug_counter++;
                if (mpu_debug_counter >= 100) {  // 50ms * 100 = 5s
                    mpu_debug_counter = 0;
                    ESP_LOGI(TAG, "MPU6050: ax=%.2f ay=%.2f az=%.2f flipped=%d", 
                             board->mpu6050_->GetAccelX(),
                             board->mpu6050_->GetAccelY(),
                             board->mpu6050_->GetAccelZ(),
                             board->mpu6050_->IsFlipped());
                }
                
                auto gesture = board->mpu6050_->GetLastGesture();
                
                // 辅助 Lambda：唤醒屏幕
                auto wakeScreen = [board]() {
                    board->last_interaction_tick_ = xTaskGetTickCount();
                    // 强制唤醒屏幕，不依赖当前 screen_off_ 状态
                    // 解决偶尔的状态不同步导致屏幕无法再次点亮的问题
                    bool was_off = board->screen_off_;
                    board->screen_off_ = false;
                    board->GetBacklight()->RestoreBrightness();
                    if (was_off) {
                        ESP_LOGI(TAG, "wakeScreen: Screen was OFF -> ON");
                    } else {
                        ESP_LOGD(TAG, "wakeScreen: Screen was already ON (refreshing brightness)");
                    }
                };
                
                // 处理手势事件
                switch (gesture) {
                    case Mpu6050::GESTURE_FLIP:
                        if (!board->flip_muted_) {
                            board->flip_muted_ = true;
                            ESP_LOGI(TAG, "Flip mute: MUTED");
                            if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
                                app.AbortSpeaking(kAbortReasonNone);
                                app.SetDeviceState(kDeviceStateIdle);
                            }
                            if (board->status_led_) board->status_led_->Blink(1, 100);
                        }
                        break;
                        
                    case Mpu6050::GESTURE_UNFLIP:
                        if (board->flip_muted_) {
                            board->flip_muted_ = false;
                            ESP_LOGI(TAG, "Flip mute: UNMUTED");
                            wakeScreen();
                            if (board->status_led_) board->status_led_->Blink(2, 100);
                        }
                        break;
                        
                    case Mpu6050::GESTURE_DOUBLE_TAP:
                        ESP_LOGI(TAG, "IMU double-tap: Toggle chat state");
                        wakeScreen();
                        app.ToggleChatState();
                        break;
                        
                    case Mpu6050::GESTURE_SHAKE:
                        ESP_LOGI(TAG, "IMU shake: Wake up screen");
                        wakeScreen();
                        break;
                        
                    case Mpu6050::GESTURE_FAST_SHAKE: {
                        // AI 对话进行中时忽略摇晃（避免打断会话）
                        if (state == kDeviceStateSpeaking || state == kDeviceStateListening) {
                            ESP_LOGD(TAG, "Ignoring shake during active conversation");
                            break;
                        }
                        
                        // 快摇事件冷却机制：防止 5 秒内反复触发
                        static TickType_t last_fast_shake_tick = 0;
                        TickType_t now = xTaskGetTickCount();
                        if (now - last_fast_shake_tick > pdMS_TO_TICKS(5000)) {
                            last_fast_shake_tick = now;
                            
                            // 记录摇晃保护时间戳，防止后续 6 秒内被误判为隐私姿态
                            board->last_shake_protection_tick_ = now;
                            
                            ESP_LOGI(TAG, "IMU fast shake: Triggering AI complaint response");
                            wakeScreen();
                            
                            // 向 AI 发送事件触发词，AI 会根据角色设定给出撒娇/抱怨回复
                            app.SendIoTEvent("有人摇我，告诉他别摇了", "");
                        }
                        break;
                    }
                        
                    case Mpu6050::GESTURE_LIFT:
                        ESP_LOGI(TAG, "IMU lift: Wake up screen");
                        wakeScreen();
                        break;
                        
                    default:
                        break;
                }
                
                // 侧立/翻转判定 -> 自动熄屏并关闭对话
                // 注意：在摇晃后 6 秒内跳过检测，防止对话被误中断（摇晃保护）
                static int off_confirm_counter = 0;
                TickType_t current_tick = xTaskGetTickCount();
                bool in_shake_protection = (current_tick - board->last_shake_protection_tick_) < pdMS_TO_TICKS(6000);
                
                bool is_privacy_pose = !in_shake_protection && 
                                       (board->mpu6050_->IsSideStanding() || board->mpu6050_->IsFlipped());
                
                if (is_privacy_pose) {
                    if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
                        ESP_LOGI(TAG, "Privacy pose -> Closing dialogue");
                        app.AbortSpeaking(kAbortReasonNone);
                        app.SetDeviceState(kDeviceStateIdle);
                    }
                    
                    if (!board->screen_off_) {
                        if (++off_confirm_counter > 20) { // 1秒确认
                            off_confirm_counter = 0;
                            ESP_LOGI(TAG, "Privacy pose -> Auto screen off");
                            board->screen_off_ = true;
                            board->GetBacklight()->SetBrightness(0);
                        }
                    }
                } else {
                    off_confirm_counter = 0; // 恢复正面时重置计数
                }
            }

            // ============================================
            // LED 状态指示
            // ============================================
            if (board->status_led_) {
                led_blink_counter++;
                
                if (charging) {
                    // Charging: Short blink every 4s
                    // Active Low Mode in StatusLed: On()=Lit(Low), Off()=Dark(High).
                    if ((led_blink_counter % 40) == 0) {
                         board->status_led_->On(); 
                    } else {
                         board->status_led_->Off(); 
                    }
                } else if (battery_level < 20) {
                    // Low Battery: Short blink every 1s
                    if ((led_blink_counter % 10) == 0) {
                         board->status_led_->On(); 
                    } else {
                         board->status_led_->Off();
                    }
                } else if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
                    // Speaking/Listening: Toggle
                    if (led_blink_counter % 4 == 0) {
                         board->status_led_->Toggle();
                    }
                } else {
                    // Normal state: LED OFF
                    board->status_led_->Off();
                }
            }

            // ============================================
            // 原有的电源管理逻辑 (每秒执行一次)
            // ============================================
            static int power_check_counter = 0;
            power_check_counter++;
            if (power_check_counter < 10) continue;  // 100ms * 10 = 1s
            power_check_counter = 0;

            // 如果正在充电 -> 始终亮屏，保持唤醒
            if (charging) {
                if (board->screen_off_) {
                    board->screen_off_ = false;
                    board->GetBacklight()->RestoreBrightness();
                }
                board->last_interaction_tick_ = xTaskGetTickCount();
                continue;
            }

            // 如果处于翻转静音状态，不做超时处理
            if (board->flip_muted_) {
                continue;
            }

            // 如果未充电，检测是否处于非空闲状态
            if (state == kDeviceStateListening || state == kDeviceStateSpeaking || state == kDeviceStateConnecting) {
                board->last_interaction_tick_ = xTaskGetTickCount();
            }
            
            // 检查超时 (15秒)
            uint32_t now = xTaskGetTickCount();
            uint32_t diff = (now - board->last_interaction_tick_) * portTICK_PERIOD_MS;

            if (diff > 15000) {
                if (!board->screen_off_) {
                    ESP_LOGI(TAG, "Auto screen off after 15s inactivity");
                    if (state != kDeviceStateIdle) {
                        app.SetDeviceState(kDeviceStateIdle);
                    }
                    board->GetBacklight()->SetBrightness(0);
                    board->screen_off_ = true;
                }
            } else {
                if (state != kDeviceStateIdle && board->screen_off_) {
                    board->screen_off_ = false;
                    board->GetBacklight()->RestoreBrightness();
                }
            }
        }
    }

    void ScanI2C() {
        ESP_LOGI(TAG, "Starting I2C Scan...");
        for (uint8_t addr = 1; addr < 127; addr++) {
            esp_err_t ret = i2c_master_probe(i2c_bus_, addr, 50);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "I2C Device found at address 0x%02X", addr);
                // Try to read first byte (register 0x00)
                uint8_t reg = 0x00;
                uint8_t val = 0xFF;
                
                // We need a temporary device handle to read
                i2c_device_config_t dev_cfg = {
                    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                    .device_address = addr,
                    .scl_speed_hz = 50000,
                };
                i2c_master_dev_handle_t dev_handle;
                if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev_handle) == ESP_OK) {
                    if (i2c_master_transmit_receive(dev_handle, &reg, 1, &val, 1, 50) == ESP_OK) {
                        ESP_LOGI(TAG, "   WARN: Read Reg 0x00 = 0x%02X", val);
                    } else {
                        ESP_LOGW(TAG, "   WARN: Read Reg 0x00 FAILED");
                    }
                    i2c_master_bus_rm_device(dev_handle);
                }
            }
        }
        ESP_LOGI(TAG, "I2C Scan Complete");
    }

    void InitializeCharge()
    {
        // 扫描I2C设备，确认硬件是否存在
        // 扫描I2C设备，确认硬件是否存在
        // ScanI2C(); // 禁用扫描以消除启动警告与延迟

        charge_ = new Charge(i2c_bus_, 0x55);
        
        // --- 增强: 自动探测 MPU6050/ICM 地址 ---
        ESP_LOGI(TAG, "=== IMU Detection Start ===");
        uint8_t mpu_addr = 0;
        esp_err_t probe_68 = i2c_master_probe(i2c_bus_, 0x68, 50);
        esp_err_t probe_69 = i2c_master_probe(i2c_bus_, 0x69, 50);
        
        ESP_LOGI(TAG, "Probe 0x68: %s", probe_68 == ESP_OK ? "FOUND" : "NOT FOUND");
        ESP_LOGI(TAG, "Probe 0x69: %s", probe_69 == ESP_OK ? "FOUND" : "NOT FOUND");
        
        if (probe_68 == ESP_OK) {
            mpu_addr = 0x68;
        } else if (probe_69 == ESP_OK) {
            mpu_addr = 0x69;
        }

        if (mpu_addr != 0) {
            mpu6050_ = new Mpu6050(i2c_bus_, mpu_addr);
            ESP_LOGI(TAG, "MPU6050 object created at 0x%02X", mpu_addr);
        } else {
            ESP_LOGE(TAG, "IMU NOT FOUND! Will try 0x68 anyway for debugging.");
            // 强制创建以便调试
            mpu6050_ = new Mpu6050(i2c_bus_, 0x68);
        }
        
        // 初始化状态LED (绿色)
        status_led_ = new StatusLed(LED_G);
        status_led_->Blink(2, 100);  // 启动时闪烁2次
        ESP_LOGI(TAG, "Status LED initialized on GPIO%d", LED_G);
        
        // 启动电源管理任务
        last_interaction_tick_ = xTaskGetTickCount();
        xTaskCreatePinnedToCore(power_save_task, "power_save", 4 * 1024, this, 1, NULL, 0);
    }

    void InitializeCst816sTouchPad()
    {
        cst816s_ = new Cst816s(i2c_bus_, 0x15);

        xTaskCreatePinnedToCore(touch_event_task, "touch_task", 4 * 1024, cst816s_, 5, NULL, 1);

        const gpio_config_t int_gpio_config = {
            .pin_bit_mask = (1ULL << TP_PIN_NUM_INT),
            .mode = GPIO_MODE_INPUT,
            // .intr_type = GPIO_INTR_NEGEDGE
            .intr_type = GPIO_INTR_ANYEDGE
        };
        gpio_config(&int_gpio_config);
        gpio_install_isr_service(0);
        gpio_intr_enable(TP_PIN_NUM_INT);
        gpio_isr_handler_add(TP_PIN_NUM_INT, EchoEar::touch_isr_callback, cst816s_);
    }

    void InitializeSpi()
    {
        const spi_bus_config_t bus_config = TAIJIPI_ST77916_PANEL_BUS_QSPI_CONFIG(QSPI_PIN_NUM_LCD_PCLK,
                                                                                  QSPI_PIN_NUM_LCD_DATA0,
                                                                                  QSPI_PIN_NUM_LCD_DATA1,
                                                                                  QSPI_PIN_NUM_LCD_DATA2,
                                                                                  QSPI_PIN_NUM_LCD_DATA3,
                                                                                  QSPI_LCD_H_RES * 80 * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(QSPI_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));
    }

    void Initializest77916Display(uint8_t pcb_verison)
    {

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        const esp_lcd_panel_io_spi_config_t io_config = ST77916_PANEL_IO_QSPI_CONFIG(QSPI_PIN_NUM_LCD_CS, NULL, NULL);
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io));
        st77916_vendor_config_t vendor_config = {
            .init_cmds = vendor_specific_init_yysj,
            .init_cmds_size = sizeof(vendor_specific_init_yysj) / sizeof(st77916_lcd_init_cmd_t),
            .flags = {
                .use_qspi_interface = 1,
            },
        };
        const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = QSPI_PIN_NUM_LCD_RST,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = QSPI_LCD_BIT_PER_PIXEL,
            .flags = {
                .reset_active_high = pcb_verison,
            },
            .vendor_config = &vendor_config,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st77916(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_disp_on_off(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
#else
        display_ = new SpiLcdDisplay(panel_io, panel,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#endif
        backlight_ = new PwmBacklight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        backlight_->RestoreBrightness();
    }

    void InitializeButtons()
    {
        boot_button_.OnClick([this]() {
            auto &app = Application::GetInstance();
            
            // 按键唤醒屏幕
            last_interaction_tick_ = xTaskGetTickCount();
            if (screen_off_) {
                screen_off_ = false;
                backlight_->RestoreBrightness();
                // 唤醒只亮屏，不触发其他操作，或者视情况而定？
                // 根据当前逻辑，如果不触发其他操作，按键原本逻辑还是会执行
            }

            if (app.GetDeviceState() == kDeviceStateStarting) {
                ESP_LOGI(TAG, "Boot button pressed, enter WiFi configuration mode");
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        gpio_config_t power_gpio_config = {
            .pin_bit_mask = (BIT64(POWER_CTRL)),
            .mode = GPIO_MODE_OUTPUT,

        };
        ESP_ERROR_CHECK(gpio_config(&power_gpio_config));

        gpio_set_level(POWER_CTRL, 0);
    }

#ifdef CONFIG_ESP_VIDEO_ENABLE_USB_UVC_VIDEO_DEVICE
    void InitializeCamera() {
        esp_video_init_usb_uvc_config_t usb_uvc_config = {
            .uvc = {
                .uvc_dev_num = 1,
                .task_stack = 4096,
                .task_priority = 5,
                .task_affinity = -1,
            },
            .usb = {
                .init_usb_host_lib = true,
                .task_stack = 4096,
                .task_priority = 5,
                .task_affinity = -1,
            },
        };

        esp_video_init_config_t video_config = {
            .usb_uvc = &usb_uvc_config,
        };

        camera_ = new Esp32Camera(video_config);
    }
#endif // CONFIG_ESP_VIDEO_ENABLE_USB_UVC_VIDEO_DEVICE

public:
    EchoEar() : boot_button_(BOOT_BUTTON_GPIO)
    {
        InitializeI2c();
        uint8_t pcb_verison = DetectPcbVersion();
        InitializeCharge();
        InitializeCst816sTouchPad();

        InitializeSpi();
        Initializest77916Display(pcb_verison);
        InitializeButtons();
#ifdef CONFIG_ESP_VIDEO_ENABLE_USB_UVC_VIDEO_DEVICE
        InitializeCamera();
#endif // CONFIG_ESP_VIDEO_ENABLE_USB_UVC_VIDEO_DEVICE

        // 自动配置WiFi (如果没有保存的SSID则添加预设的)
        AutoConfigureWifi();
        
        // 设置默认音量 50%
        GetAudioCodec()->SetOutputVolume(50);
    }

    void AutoConfigureWifi() {
        auto& ssid_manager = SsidManager::GetInstance();
        auto ssid_list = ssid_manager.GetSsidList();
        
        // 预设WiFi配置
        const char* preset_ssid = "CK";
        const char* preset_password = "Chai962464";
        
        // 检查是否已有此SSID
        bool has_preset = false;
        for (const auto& item : ssid_list) {
            if (item.ssid == preset_ssid) {
                has_preset = true;
                break;
            }
        }
        
        // 如果没有添加预设WiFi
        if (!has_preset) {
            ssid_manager.AddSsid(preset_ssid, preset_password);
            ESP_LOGI(TAG, "Auto WiFi configured: %s", preset_ssid);
        }
    }

    virtual AudioCodec* GetAudioCodec() override
    {
        static BoxAudioCodec audio_codec(
            i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN,
            AUDIO_CODEC_ES8311_ADDR,
            AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override
    {
        return display_;
    }

    Cst816s* GetTouchpad()
    {
        return cst816s_;
    }

    virtual Backlight* GetBacklight() override
    {
        return backlight_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override
    {
        if (charge_) {
            charge_->GetBatteryInfo(level, charging);
            discharging = !charging;
            return true;
        }
        return false;
    }


};

DECLARE_BOARD(EchoEar);
