
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "sdkconfig.h"

#include "esp_random.h"
#include "esp_event.h"
#include "esp_log.h"

#include "usb/usb_host.h"
#include "fs_hid_host.h"
#include "fs_hid_usage_logitech_joystick.h"

#include "fs_usb_handler.h"

// The current state of the device
int state = 0;

// Variables with regards to connection to remote
volatile bool _receivedRemoteHandshake = false; // Have we connected to the remote?
volatile long _lastCommunicationWithRemote = 0; // Last time we heard from remote - used to infer connection issues

volatile uint8_t lastButtonPress = 0;    // Joystick button, trigger = 1, BTN 2 = 2, BTN 3 = 4, BTN 4 = 8, BTN 5 = 0x10, BTN 6 = 0x20
volatile uint8_t lastHatSwitchPress = 8; // Joystick hat switch, neutral = 8, right = 2, left = 6, up = 0, down = 4
volatile uint8_t throttlePos = 0xFF;     // Throttle position, for some reason is reversed, 0xFF is zero pos, 0 is max
volatile uint16_t lastStickR = 0x1FF;    // Joystick roll axis, 0x0 is left, 0x1FF is center, 0x3FF is right
volatile uint16_t lastStickP = 0x1FF;    // Joystick pitch axis, 0x0 is forward, 0x1FF is center, 0x3FF is back
volatile uint8_t lastStickY = 0x80;      // Joystick yaw axis, 0x0 is left, 0x80 is center, 0xFF is right

volatile hid_stick_input_report_boot_t last_joystick_message; // Previous message... (initialize with all 0)

// Illustration of how the joystick can be used to control an aircraft
volatile float _cmdPitch = 0; // Pitch  (0 = level) DEG/s
volatile float _cmdRoll = 0; // Roll  (0 = level) DEG/s
volatile float _cmdYaw = 0; // Yaw (0 = forward) DEG/s
volatile float _cmdThrottlePercentage = 0; // Throttle strength in percent (0 - 100)

/// @brief Parse the input (which was automatically placed into last_joystick_message)
/// This is the 
void handle_stick_input()
{
    if (lastStickP != last_joystick_message.y)
    {
        _cmdPitch = (float)last_joystick_message.y - 0x1FF;
        _cmdPitch /= 0x3FF;
        //_cmdPitch *= 150/M_PI;
        _cmdPitch /= 2.0f;
        ESP_LOGI("CMDPITCH", "%.02f", _cmdPitch);
    }
    if (lastStickR != last_joystick_message.x)
    {
        _cmdRoll = (float)last_joystick_message.x - 0x1FF;
        _cmdRoll /= 0x3FF;
        _cmdRoll /= 2.0f;
        //_cmdRoll *= 150/M_PI;
        ESP_LOGI("CMDROLL", "%.02f", _cmdRoll);
    }
    if (lastStickY != last_joystick_message.twist)
    {
        _cmdYaw = (float)last_joystick_message.twist - 0x80;
        _cmdYaw /= 0xFF;
        _cmdYaw /= 4;
        //_cmdYawRate *= 150/M_PI;
        ESP_LOGI("CMDYAW", "%.02f", _cmdYaw);
    }
    if (throttlePos != last_joystick_message.slider)
    {
        _cmdThrottlePercentage = 0xFF - (float)last_joystick_message.slider;
        _cmdThrottlePercentage = _cmdThrottlePercentage / 255;
        ESP_LOGI("CMDTHRTL", "%.02f", _cmdThrottlePercentage);
    }
    if (lastHatSwitchPress != last_joystick_message.hat)
    {
        switch (last_joystick_message.hat)
        {
        case 0:
            ESP_LOGI("HAT", "UP");
            break;

        case 2:
            ESP_LOGI("HAT", "RIGHT");
            break;

        case 4:
            ESP_LOGI("HAT", "DOWN");
            break;

        case 6:
            ESP_LOGI("HAT", "LEFT");
            break;

        case 8:
            ESP_LOGI("HAT", "CENTER");
            break;

        default:
            break;
        }
    }
    if (lastButtonPress != last_joystick_message.buttons_a)
    {
        switch (last_joystick_message.buttons_a)
        {
        case 0:
            ESP_LOGI("BTN", "NONE");
            break;

        case 1:
            ESP_LOGI("BTN", "TRIGGER");

            break;

        case 2:
            ESP_LOGI("BTN", "[2]");
            break;

        case 4:
            ESP_LOGI("BTN", "[3]");
            break;

        case 8:
            ESP_LOGI("BTN", "[4]");
            break;

        case 16:
            ESP_LOGI("BTN", "[5]");
            break;

        case 32:
            ESP_LOGI("BTN", "[6]");
            break;
        default:
            break;
        }
    }

    lastStickR = last_joystick_message.x;
    lastStickP = last_joystick_message.y;
    lastStickY = last_joystick_message.twist;
    throttlePos = last_joystick_message.slider;
    lastHatSwitchPress = last_joystick_message.hat;
    lastButtonPress = last_joystick_message.buttons_a;
}

/// @brief Remote control update task
/// @param pvParam
static void remote_control_task(void *pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(20);

    while (true)
    {
        handle_stick_input();
        vTaskDelayUntil(&lastTaskTime, delay_time);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI("MAIN", "Initializing!");
    initialize_usb_joystick(&last_joystick_message);
    xTaskCreatePinnedToCore(remote_control_task, "remote_control_task", 4096, (void *)1, 9, NULL, 0); // Remote control input task
}