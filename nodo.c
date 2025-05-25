#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "MFRC522.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#define BUF_SIZE 128
#define WAKEUP_GPIO GPIO_NUM_2      // GPIO del botón
#define LED_GPIO    GPIO_NUM_15     // GPIO del LED
#define LORA_GPIO   GPIO_NUM_3      // GPIO del LoRa
#define RFID_GPIO   GPIO_NUM_6      // GPIO del RFID
#define UART_PORT UART_NUM_1
#define TX_PIN GPIO_NUM_4
#define RX_PIN GPIO_NUM_S
#define SPI_CLK 21
#define SPI_MOSI 22
#define SPI_MISO 23
#define SPI_SS 8

const char* AULA = "0009";

static const char* TAG = "RC522";

void interrupt_handler(void* arg) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO, 0);

    gpio_config_t lora_conf = {
        .pin_bit_mask = (1ULL << LORA_GPIO) | (1ULL << RFID_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&lora_conf);
    
    gpio_set_level(LORA_GPIO, 0);
    gpio_set_level(RFID_GPIO, 0);

    gpio_config_t wakeup_io_conf = {
        .pin_bit_mask = (1ULL << WAKEUP_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&wakeup_io_conf);

    //Habilita el despertar en nivel BAJO
    esp_deep_sleep_enable_gpio_wakeup((1ULL << WAKEUP_GPIO), ESP_GPIO_WAKEUP_GPIO_LOW);

    ESP_LOGI(TAG, "Entrando en deep sleep...");
    esp_deep_sleep_start();
}

void app_main(void)
{
    bool detected = false;
    char str[21] = "";
    //Obtener la causa del último despertar
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "Motivo del despertar: %d", wakeup_reason);

    // Inicializa el GPIO del LED
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO, 1);

    if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
        gpio_config_t lora_conf = {
            .pin_bit_mask = (1ULL << LORA_GPIO) | (1ULL << RFID_GPIO),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&lora_conf);
        
        gpio_set_level(LORA_GPIO, 0);
        gpio_set_level(RFID_GPIO, 1);

        uart_config_t uart_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
        };

        // Inicializar el UART
        uart_param_config(UART_PORT, &uart_config);
        uart_set_pin(UART_PORT, 5, 4, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);

        uart_flush(UART_PORT);
        char str[20];
        
        esp_err_t ret;
        spi_device_handle_t spi;
        spi_bus_config_t buscfg={
            .miso_io_num=SPI_MISO,
            .mosi_io_num=SPI_MOSI,
            .sclk_io_num=SPI_CLK,
            .quadwp_io_num=-1,
            .quadhd_io_num=-1
        };
        spi_device_interface_config_t devcfg={
            .clock_speed_hz=5000000,                //Clock out at 5 MHz
            .mode=0,                                //SPI mode 0
            .spics_io_num=SPI_SS,                   //CS pin
            .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        };
        //Inicializar el SPI
        ret=spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
        assert(ret==ESP_OK);
        ret=spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
        assert(ret==ESP_OK);
    
        PCD_Init(spi);

        const esp_timer_create_args_t timer_args = {
            .callback = &interrupt_handler,
            .name = "OneShotTimer"
        };

        esp_timer_handle_t timer;
        esp_timer_create(&timer_args, &timer);
        esp_timer_start_once(timer, 10 * 1000000);  //10 * microseg

        while(!detected)
        {
            if(PICC_IsNewCardPresent(spi))                      //Checking for new card
            {
                GetStatusCodeName(PICC_Select(spi,&uid,0));
                PICC_DumpToSerial(spi,&uid);                    //DETAILS OF UID ALONG WITH SECTORS
                uint8_t status;
                uint8_t byteCount;
                uint8_t buffer[18];
                uint8_t i;
                for (uint8_t page = 0; page < 32; page +=4) {
                    byteCount = sizeof(buffer);
                    status = MIFARE_Read(spi,page, buffer, &byteCount);
                    if (status != STATUS_OK) {
                        printf("MIFARE_Read() failed: ");
                        break;
                    }
                    //Dump data
                    for (uint8_t offset = 0; offset < 4; offset++) {
                        i = page + offset;
                        for (uint8_t index = 0; index < 4; index++) {
                            i = 4 * offset + index;

                            printf("%c",buffer[i]);
                        }
                    }
                }
                printf("\r\n");
                gpio_set_level(LORA_GPIO, 1);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                uart_write_bytes(UART_PORT, "Aula ", strlen("Aula "));
                uart_write_bytes(UART_PORT, AULA, strlen(AULA));
                for (uint8_t i = 0; i < uid.size; i++) {
                    if(uid.uidByte[i] < 0x10)
                        sprintf(str, " 0%x",uid.uidByte[i]);
                    else
                        sprintf(str, " %x",uid.uidByte[i]);
                    uart_write_bytes(UART_PORT, (const char*)str, strlen(str));
                }
                gpio_set_level(LORA_GPIO, 0);
                gpio_set_level(RFID_GPIO, 0);
                detected = true;
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }

    // Configura el GPIO del botón como entrada sin interrupciones
    gpio_config_t wakeup_io_conf = {
        .pin_bit_mask = (1ULL << WAKEUP_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&wakeup_io_conf);

    // Habilita el despertar en nivel BAJO
    esp_deep_sleep_enable_gpio_wakeup((1ULL << WAKEUP_GPIO), ESP_GPIO_WAKEUP_GPIO_LOW);

    ESP_LOGI(TAG, "Entrando en deep sleep...");
    esp_deep_sleep_start();
}