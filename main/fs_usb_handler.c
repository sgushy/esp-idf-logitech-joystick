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

// Variables with regards to USB joystick...
#define APP_QUIT_PIN GPIO_NUM_0
#define APP_QUIT_PIN_POLL_MS 500

#define READY_TO_UNINSTALL (HOST_NO_CLIENT | HOST_ALL_FREE)
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

static EventGroupHandle_t usb_flags;
static bool is_hid_device_connected = false;
static hid_host_interface_handle_t mouse_handle = NULL;

volatile hid_stick_input_report_boot_t *last_stick_report;

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

/// @brief The callback for handling flight stick inputs (should not modify)
/// @param dat 
/// @param length 
static void hid_flight_stick_cb(const uint8_t *const dat, const int length)
{
    hid_stick_input_report_boot_t *stick_rep = (hid_stick_input_report_boot_t *)dat;

    if (length < sizeof(hid_stick_input_report_boot_t))
    {
        ESP_LOGE("FS-CB", "Wrong size!");
        return;
    }
    memcpy(last_stick_report, stick_rep, sizeof(hid_stick_input_report_boot_t));
}

/// @brief Callback for handling USB HID host events (do not modify)
/// @param event 
/// @param arg 
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

/// @brief Callback for handling USB HID host interface events (do not modify)
/// @param event 
/// @param arg 
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

/// @brief USB host library event handler (do not modify)
/// @param args 
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

/// @brief Initialize the joystick USB driver
/// @param lst The pointer to the storage container for joystick status
void initialize_usb_joystick(hid_stick_input_report_boot_t *lst)
{
    last_stick_report = lst;
    vTaskDelay(50 / portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(usb_core_task, "usb_init", 4096, (void *)1, 8, NULL, 1); // USB task
    vTaskDelay(50 / portTICK_PERIOD_MS);
}