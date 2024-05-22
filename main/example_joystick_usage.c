/*
    Remote for twin rotor helicopter
    Last updated 4/14/2023 - Now uses the USB joystick "Logitech Extreme 3D Pro"
*/
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
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "usb/usb_host.h"
#include "fs_hid_host.h"
#include "fs_hid_usage_logitech_joystick.h"

// The current state of the device
int state = 0;

// Variables with regards to connection to remote
volatile bool _receivedRemoteHandshake = false; // Have we connected to the remote?
volatile long _lastCommunicationWithRemote = 0; // Last time we heard from remote - used to infer connection issues

// Variables with regards to USB joystick...
#define APP_QUIT_PIN GPIO_NUM_0
#define APP_QUIT_PIN_POLL_MS 500

#define READY_TO_UNINSTALL (HOST_NO_CLIENT | HOST_ALL_FREE)

volatile uint8_t lastButtonPress = 0;    // Joystick button, trigger = 1, BTN 2 = 2, BTN 3 = 4, BTN 4 = 8, BTN 5 = 10, BTN 6 = 20
volatile uint8_t lastHatSwitchPress = 8; // Joystick hat switch, neutral = 8, right = 2, left = 6, up = 0, down = 4
volatile uint8_t throttlePos = 0xFF;     // Throttle position, for some reason is reversed, 0xFF is zero pos, 0 is max
volatile uint16_t lastStickR = 0x1FF;    // Joystick roll axis, 0x0 is left, 0x1FF is center, 0x3FF is right
volatile uint16_t lastStickP = 0x1FF;    // Joystick pitch axis, 0x0 is forward, 0x1FF is center, 0x3FF is back
volatile uint8_t lastStickY = 0x80;      // Joystick yaw axis, 0x0 is left, 0x80 is center, 0xFF is right

volatile hid_stick_input_report_boot_t last_stick_report; // Previous message... (initialize with all 0)

// Illustration of how the joystick can be used to control an aircraft
volatile float _cmdPitch = 0; // Pitch  (0 = level) DEG/s
volatile float _cmdRoll = 0; // Roll  (0 = level) DEG/s
volatile float _cmdYaw = 0; // Yaw (0 = forward) DEG/s
volatile float _cmdThrottlePercentage = 0; // Throttle strength in percent (0 - 100)

/**
 * @brief Application Event from USB Host driver
 *
 */
typedef enum
{
    HOST_NO_CLIENT = 0x1,
    HOST_ALL_FREE = 0x2,
    DEVICE_CONNECTED = 0x4,
    DEVICE_DISCONNECTED = 0x8,
    DEVICE_ADDRESS_MASK = 0xFF0,
} app_event_t;

#define USB_EVENTS_TO_WAIT (DEVICE_CONNECTED | DEVICE_ADDRESS_MASK | DEVICE_DISCONNECTED)

// static const char *TAG = "example";
static EventGroupHandle_t usb_flags;
static bool is_hid_device_connected = false;
static hid_host_interface_handle_t mouse_handle = NULL;

/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto)
{
    static hid_protocol_t prev_proto_output = HID_PROTOCOL_NONE;
    if (prev_proto_output != proto)
    {
        prev_proto_output = proto;
        printf("\r\n");
        printf(":%02X", proto);
        fflush(stdout);
    }
    ESP_LOGI("HID_NEW_DEVICE_REPORT", "!");
}

/**
 * @brief The joystick callback handler...
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_flight_stick_cb(const uint8_t *const dat, const int length)
{
    hid_stick_input_report_boot_t *stick_rep = (hid_stick_input_report_boot_t *)dat;

    if (length < sizeof(hid_stick_input_report_boot_t))
    {
        ESP_LOGE("FS-CB", "Wrong size!");
        return;
    }

    last_stick_report = *stick_rep;
    // printf("P %X, R %X, Y %X, T %X, H %X, BA %X, BB %X", stick_rep->x, stick_rep->y, stick_rep->twist, stick_rep->slider, stick_rep->hat, stick_rep->buttons_a, stick_rep->buttons_b);
    // fflush(stdout);
}

/**
 * @brief USB HID Host event callback. Handle such event as device connection and removing
 *
 * @param[in] event  HID device event
 * @param[in] arg    Pointer to arguments, does not used
 */
void hid_host_event_callback(const hid_host_event_t *event, void *arg)
{
    if (event->event == HID_DEVICE_CONNECTED)
    {
        // Obtained USB device address is placed after application events
        ESP_LOGI("USB-DETECT", "DEVADDRESS: %X", event->device.address);
        xEventGroupSetBits(usb_flags, DEVICE_CONNECTED | (event->device.address << 4));
    }
    else if (event->event == HID_DEVICE_DISCONNECTED)
    {
        ESP_LOGI("USB-DISCONNECT", "DEVADDRESS: %X", event->device.address);
        xEventGroupSetBits(usb_flags, DEVICE_DISCONNECTED);
    }
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] event  HID interface event
 * @param[in] arg    Pointer to arguments, does not used
 */
void hid_host_interface_event_callback(const hid_host_interface_event_t *event2, void *arg)
{
    ESP_LOGI("HID_HOST_EVENT", "Proto: %X", event2->interface.proto);

    switch (event2->event)
    {
    case HID_DEVICE_INTERFACE_INIT:
        ESP_LOGI("HID_HOST_EVENT", "Claim interface attempt");
        const hid_host_interface_config_t hid_stick_config = {
            .proto = event2->interface.proto, // Aviation stick
            .callback = hid_flight_stick_cb,
        };

        hid_host_claim_interface(&hid_stick_config, &mouse_handle);
        break;
    case HID_DEVICE_INTERFACE_TRANSFER_ERROR:
        ESP_LOGD("PPS", "Interface number %d, transfer error",
                 event2->interface.num);
        break;

    case HID_DEVICE_INTERFACE_CLAIM:
    case HID_DEVICE_INTERFACE_RELEASE:
        ESP_LOGI("PPS", "Claim/Release");
        break;

    default:
        ESP_LOGI("PPS", "%s Unhandled event %X, Interface number %d",
                 __FUNCTION__,
                 event2->event,
                 event2->interface.num);
        break;
    }
}

/**
 * @brief Handle common USB host library events
 *
 * @param[in] args  Pointer to arguments, does not used
 */
void handle_usb_events(void *args)
{
    uint32_t event_flags;

    while (1)
    {
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        // Release devices once all clients has deregistered
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            usb_host_device_free_all();
            xEventGroupSetBits(usb_flags, HOST_NO_CLIENT);
        }
        // Give ready_to_uninstall_usb semaphore to indicate that USB Host library
        // can be deinitialized, and terminate this task.
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            xEventGroupSetBits(usb_flags, HOST_ALL_FREE);
        }

        // vTaskDelay(25);
    }

    vTaskDelete(NULL);
}

bool wait_for_event(EventBits_t event, TickType_t timeout)
{
    return xEventGroupWaitBits(usb_flags, event, pdTRUE, pdTRUE, timeout) & event;
}

void parseInput() // The input parsing test function
{
    // hid_print_new_device_report_header(HID_PROTOCOL_NONE);
    if (lastStickP != last_stick_report.y)
    {
        _cmdPitch = (float)last_stick_report.y - 0x1FF;
        _cmdPitch /= 0x3FF;
        //_cmdPitch *= 150/M_PI;
        _cmdPitch /= 2.0f;
        ESP_LOGI("CMDPITCH", "%.02f", _cmdPitch);
    }
    if (lastStickR != last_stick_report.x)
    {
        _cmdRoll = (float)last_stick_report.x - 0x1FF;
        _cmdRoll /= 0x3FF;
        _cmdRoll /= 2.0f;
        //_cmdRoll *= 150/M_PI;
        ESP_LOGI("CMDROLL", "%.02f", _cmdRoll);
    }
    if (lastStickY != last_stick_report.twist)
    {
        _cmdYaw = (float)last_stick_report.twist - 0x80;
        _cmdYaw /= 0xFF;
        _cmdYaw /= 4;
        //_cmdYawRate *= 150/M_PI;
        ESP_LOGI("CMDYAWRATE", "%.02f", _cmdYaw);
    }
    if (throttlePos != last_stick_report.slider)
    {
        _cmdThrottlePercentage = 0xFF - (float)last_stick_report.slider;
        _cmdThrottlePercentage = _cmdThrottlePercentage / 255;
        ESP_LOGI("CMDTHRTL", "%.02f", _cmdThrottlePercentage);
    }
    if (lastHatSwitchPress != last_stick_report.hat)
    {
        switch (last_stick_report.hat)
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
    if (lastButtonPress != last_stick_report.buttons_a)
    {
        switch (last_stick_report.buttons_a)
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

    lastStickR = last_stick_report.x;
    lastStickP = last_stick_report.y;
    lastStickY = last_stick_report.twist;
    throttlePos = last_stick_report.slider;
    lastHatSwitchPress = last_stick_report.hat;
    lastButtonPress = last_stick_report.buttons_a;
}

/// @brief Check connection to remote control (Every 96 ms)
/// @param pvParam
static void remote_conn(void *pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(96);

    TaskHandle_t search_for_remote_task = NULL; // The esp_now_ping task

    uint8_t macADDRTemp[ESP_NOW_ETH_ALEN] = {0, 0, 0, 0, 0, 0};

    while (true)
    {
        // ESP_LOGI("RAM USAGE","N");
        switch (state)
        {
        case 0:
            // ESP_LOGI("[RC-INFO]", "Awaiting connection!");

            if (is_hid_device_connected)
            {
                state = 1; // Move to state 1 once an USB connection is identified...
            }
            break;
        case 1:
            parseInput();
            // In case 1, we wait for the pingback task to give us a connection to a flight controller
            // In the meantime, we do nothing...
            break;
        default:
            // It should never reach this case...
            break;
        }
        vTaskDelayUntil(&lastTaskTime, delay_time);
        // vTaskDelay(delay_time);
    }
    vTaskDelete(NULL);
}

static void usb_core_task(void *p)
{
    TaskHandle_t usb_events_task_handle;
    hid_host_device_handle_t hid_device;

    BaseType_t task_created;

    usb_flags = xEventGroupCreate();
    assert(usb_flags);

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1};

    ESP_ERROR_CHECK(usb_host_install(&host_config));
    vTaskDelay(pdMS_TO_TICKS(105));
    task_created = xTaskCreatePinnedToCore(handle_usb_events, "usb_events", 4096, NULL, 8, &usb_events_task_handle, 1);
    assert(task_created);

    // hid host driver config
    const hid_host_driver_config_t hid_host_config = {
        .create_background_task = true,
        .task_priority = 8,
        .stack_size = 4096,
        .core_id = 1,
        .callback = hid_host_event_callback,
        .callback_arg = NULL};

    vTaskDelay(pdMS_TO_TICKS(88));

    ESP_ERROR_CHECK(hid_host_install(&hid_host_config));
    vTaskDelay(pdMS_TO_TICKS(88));

    EventBits_t event;

    do
    {
        event = xEventGroupWaitBits(usb_flags, USB_EVENTS_TO_WAIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(725));
        if (event & DEVICE_CONNECTED)
        {
            ESP_LOGI("USB", "USB device connected...!");
            xEventGroupClearBits(usb_flags, DEVICE_CONNECTED);
            is_hid_device_connected = true;
            ESP_LOGI("USB", "USB device connected...!");
        }

        if (event & DEVICE_ADDRESS_MASK)
        {

            ESP_LOGI("USB", "USB device address %X", (uint8_t)(event & (DEVICE_ADDRESS_MASK >> 4)));
            xEventGroupClearBits(usb_flags, DEVICE_ADDRESS_MASK);
            const hid_host_device_config_t hid_host_device_config = {
                .dev_addr = 0x1, // Device address
                .iface_event_cb = hid_host_interface_event_callback,
                .iface_event_arg = NULL,
            };

            ESP_ERROR_CHECK(hid_host_install_device(&hid_host_device_config, &hid_device));

            const usb_intf_desc_t ifd = {
                .bLength = 0x09,
                .bDescriptorType = 0x04,
                .bInterfaceNumber = 0x00,
                .bAlternateSetting = 0x00,
                .bNumEndpoints = 0x01,
                .bInterfaceClass = 0x03,
                .bInterfaceSubClass = 0x01,
                .bInterfaceProtocol = 0x00,
                .iInterface = 0x00};

            const hid_descriptor_t hidd = {
                .bLength = 0x09,
                .bDescriptorType = 0x21,
                .bcdHID = 0x0111,
                .bCountryCode = 0x00,
                .bNumDescriptors = 0x01,
                .bReportDescriptorType = 0x22,
                .wReportDescriptorLength = 0x007A};

            const usb_ep_desc_t espdesc = {
                .bLength = 0x07,
                .bDescriptorType = 0x05,
                .bEndpointAddress = 0x81,
                .bmAttributes = 0x03,
                .wMaxPacketSize = 0x0007,
                .bInterval = 0x01};

            ESP_ERROR_CHECK(create_interface_new(&hid_host_device_config, &hid_device, &ifd, &hidd, &espdesc));
            ESP_LOGI("USB", "USB device address %X", hid_host_device_config.dev_addr);
        }

        if (event & DEVICE_DISCONNECTED)
        {

            ESP_LOGI("USB", "Lost USB connection...!");
            xEventGroupClearBits(usb_flags, DEVICE_DISCONNECTED);

            hid_host_release_interface(mouse_handle);

            ESP_ERROR_CHECK(hid_host_uninstall_device(hid_device));

            is_hid_device_connected = false;

            ESP_LOGI("USB", "Unloaded USB device...!");
        }

        vTaskDelay(pdMS_TO_TICKS(22));
    } while (true);

    if (is_hid_device_connected)
    {
        ESP_LOGI("PPS", "Uninitializing HID Device");
        hid_host_release_interface(mouse_handle);
        ESP_ERROR_CHECK(hid_host_uninstall_device(hid_device));
        is_hid_device_connected = false;
    }

    ESP_LOGI("PPS", "Uninitializing USB");
    ESP_ERROR_CHECK(hid_host_uninstall());
    wait_for_event(READY_TO_UNINSTALL, portMAX_DELAY);
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(usb_events_task_handle);
    vEventGroupDelete(usb_flags);
    ESP_LOGI("PPS", "Done");
    vTaskDelete(NULL);
}

/// @brief Initialization
/// @param
void app_main(void)
{
    mouse_handle = NULL;
    ESP_LOGI("MAIN", "Initializing!");
    xTaskCreatePinnedToCore(remote_conn, "remote_conn", 4096, (void *)1, 9, NULL, 0); // Task for remote control
    vTaskDelay(500 / portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(usb_core_task, "usb_init", 4096, (void *)1, 8, NULL, 1); // Task for USB stuff
}