#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#define BOARD_NAME "ESP2"

#define UART_PORT UART_NUM_1
#define UART_TX_PIN 22
#define UART_RX_PIN 21
#define UART_BAUD 115200
#define UART_BUF_SIZE 2048

static void uart_init_simple(void)
{
    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(
        UART_PORT,
        UART_BUF_SIZE,
        UART_BUF_SIZE,
        0,
        NULL,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));

    ESP_ERROR_CHECK(uart_set_pin(
        UART_PORT,
        UART_TX_PIN,
        UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    uart_flush_input(UART_PORT);
}

static void tx_task(void *arg)
{
    uint32_t count = 0;
    char msg[128];

    while (1)
    {
        int len = snprintf(
            msg,
            sizeof(msg),
            "%s UART TEST count=%lu\r\n",
            BOARD_NAME,
            (unsigned long)count
        );

        uart_write_bytes(UART_PORT, msg, len);

        printf("[%s] TX: %s", BOARD_NAME, msg);

        count++;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void rx_task(void *arg)
{
    uint8_t data[128];

    while (1)
    {
        int len = uart_read_bytes(
            UART_PORT,
            data,
            sizeof(data) - 1,
            pdMS_TO_TICKS(1000)
        );

        if (len > 0)
        {
            data[len] = '\0';

            printf("[%s] RX TEXT: %s", BOARD_NAME, (char *)data);
            printf("[%s] RX HEX : ", BOARD_NAME);

            for (int i = 0; i < len; i++)
            {
                printf("%02X ", data[i]);
            }

            printf("\n");
        }
        else
        {
            printf("[%s] Waiting on RX GPIO%d...\n", BOARD_NAME, UART_RX_PIN);
        }
    }
}

void app_main(void)
{
    printf("\n[%s] UART test started\n", BOARD_NAME);
    printf("[%s] TX GPIO%d\n", BOARD_NAME, UART_TX_PIN);
    printf("[%s] RX GPIO%d\n", BOARD_NAME, UART_RX_PIN);

    uart_init_simple();

    xTaskCreate(tx_task, "tx_task", 4096, NULL, 5, NULL);
    xTaskCreate(rx_task, "rx_task", 4096, NULL, 5, NULL);
}