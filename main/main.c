/* HTTP File Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/param.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "soc/soc_caps.h"
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "esp_spi_flash.h"
#include "protocol_examples_common.h"
#include "addr_from_stdin.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "mpu6050/GY521.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include <stdbool.h>
#include <string.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "lwip/sys.h"
#include "freertos/message_buffer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "vs1053.h"
#include "math.h"
#if SOC_SDMMC_HOST_SUPPORTED
#include "driver/sdmmc_host.h"
#include <dirent.h>
#include <sys/types.h>
#endif

/* This example demonstrates how to create file server
 * using esp_http_server. This file has only startup code.
 * Look in file_server.c for the implementation */

#define MOUNT_POINT "/sdcard"
static const char *TAG="example";
/* ESP32-S2/C3 doesn't have an SD Host peripheral, always use SPI,
 * ESP32 can choose SPI or SDMMC Host, SPI is used by default: */

#ifndef CONFIG_EXAMPLE_USE_SDMMC_HOST
// #define USE_SPI_MODE
#endif
// DMA channel to be used by the SPI peripheral
#if CONFIG_IDF_TARGET_ESP32
#define SPI_DMA_CHAN    1
// on ESP32-S2, DMA channel must be the same as host id
#elif CONFIG_IDF_TARGET_ESP32S2
#define SPI_DMA_CHAN    host.slot
#elif CONFIG_IDF_TARGET_ESP32C3
// on ESP32-C3, DMA channels are shared with all other peripherals
#define SPI_DMA_CHAN    1
#endif //CONFIG_IDF_TARGET_ESP32

// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.
#ifdef CONFIG_EXAMPLE_MOUNT_SD_CARD
static sdmmc_card_t* mount_card = NULL;
static char * mount_base_path = MOUNT_POINT;
#endif
// #ifdef USE_SPI_MODE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
// #if CONFIG_IDF_TARGET_ESP32C3
// #define PIN_NUM_MISO 2
// #define PIN_NUM_MOSI 7
// #define PIN_NUM_CLK  6
// #define PIN_NUM_CS   10
// #else
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   13
// #endif // CONFIG_IDF_TARGET_ESP32C3
// #endif //USE_SPI_MODE

/* Function to initialize SPIFFS */
static esp_err_t init_spiffs(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,   // This decides the maximum number of files that can be created on the storage
      .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    return ESP_OK;
}

/* Declare the function which starts the file server.
 * Implementation of this function is to be found in
 * file_server.c */
esp_err_t start_file_server(const char *base_path);
#ifdef CONFIG_EXAMPLE_MOUNT_SD_CARD
void sdcard_mount(void)
{
    /*sd_card part code*/
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    sdmmc_card_print_info(stdout, card);

}

static esp_err_t unmount_card(const char* base_path, sdmmc_card_t* card)
{
#ifdef USE_SPI_MODE
    esp_err_t err = esp_vfs_fat_sdcard_unmount(base_path, card);
#else
    esp_err_t err = esp_vfs_fat_sdmmc_unmount();
#endif
    ESP_ERROR_CHECK(err);
#ifdef USE_SPI_MODE
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    err = spi_bus_free(host.slot);
#endif
    ESP_ERROR_CHECK(err);

    return err;
}

#endif //CONFIG_EXAMPLE_MOUNT_SD_CARD

void app_main(void)
{
    /*Mount the SDcard first if needed.*/
#ifdef CONFIG_EXAMPLE_MOUNT_SD_CARD
    sdcard_mount();
#endif

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    /* Initialize file storage */
    // ESP_ERROR_CHECK(init_spiffs());

    /* Start the file server */
    ESP_ERROR_CHECK(start_file_server("/sdcard"));
    
    ESP_LOGE("11","1123123123123");
#ifdef CONFIG_EXAMPLE_MOUNT_SD_CARD
    //deinitialize the bus after all devices are removed
    // ESP_ERROR_CHECK(unmount_card(mount_base_path, mount_card));
#endif
}
