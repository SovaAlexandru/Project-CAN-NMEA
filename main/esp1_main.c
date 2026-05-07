#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_timer.h"

// ─────────────────────────────────────────────
// CONFIG
// ─────────────────────────────────────────────

#define CAN_TX_PIN       4
#define CAN_RX_PIN       5

#define CAN_BITRATE      250000
#define UART_BAUDRATE    115200

#define RX_QUEUE_LEN     200

// ─────────────────────────────────────────────
// TYPES
// ─────────────────────────────────────────────

typedef struct {
    uint32_t id;
    bool extended;
    bool rtr;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_info_t;

// ─────────────────────────────────────────────
// GLOBALS
// ─────────────────────────────────────────────

static twai_node_handle_t twai_node = NULL;
static QueueHandle_t rx_queue = NULL;

static volatile uint32_t frame_count = 0;
static volatile uint32_t dropped_frames = 0;

static uint64_t last_time_us = 0;

// ─────────────────────────────────────────────
// CALCULATE CAN FRAME BITS
// ─────────────────────────────────────────────

static uint32_t calculate_can_bits(bool extended,
                                   uint8_t dlc)
{
    uint32_t bits;

    if (extended) {

        // Extended CAN frame overhead
        bits = 67;

    } else {

        // Standard CAN frame overhead
        bits = 47;
    }

    // Add payload
    bits += dlc * 8;

    // Approximate bit stuffing
    bits = (bits * 12) / 10;

    return bits;
}

// ─────────────────────────────────────────────
// EXTRACT PGN
// ─────────────────────────────────────────────

static uint32_t extract_pgn(uint32_t id)
{
    uint8_t pf = (id >> 16) & 0xFF;
    uint8_t ps = (id >> 8) & 0xFF;
    uint8_t dp = (id >> 24) & 0x01;

    if (pf < 240) {

        return ((uint32_t)dp << 16) |
               ((uint32_t)pf << 8);
    }

    return ((uint32_t)dp << 16) |
           ((uint32_t)pf << 8) |
           ps;
}

// ─────────────────────────────────────────────
// RX CALLBACK ISR
// ─────────────────────────────────────────────

static bool twai_rx_callback(
    twai_node_handle_t handle,
    const twai_rx_done_event_data_t *edata,
    void *user_ctx)
{
    uint8_t rx_buffer[8] = {0};

    twai_frame_t rx_frame = {
        .buffer = rx_buffer,
        .buffer_len = sizeof(rx_buffer),
    };

    if (twai_node_receive_from_isr(handle,
                                   &rx_frame) == ESP_OK) {

        can_frame_info_t frame = {0};

        frame.id       = rx_frame.header.id;
        frame.extended = rx_frame.header.ide;
        frame.rtr      = rx_frame.header.rtr;
        frame.dlc      = rx_frame.buffer_len > 8 ?
                         8 : rx_frame.buffer_len;

        memcpy(frame.data,
               rx_buffer,
               frame.dlc);

        BaseType_t woken = pdFALSE;

        if (xQueueSendFromISR(rx_queue,
                              &frame,
                              &woken) != pdTRUE) {

            dropped_frames++;
        }

        return woken == pdTRUE;
    }

    return false;
}

// ─────────────────────────────────────────────
// PRINT TASK
// ─────────────────────────────────────────────

static void can_print_task(void *arg)
{
    can_frame_info_t frame;

    while (1) {

        if (xQueueReceive(rx_queue,
                          &frame,
                          portMAX_DELAY) == pdTRUE) {

            frame_count++;

            // ─────────────────────────────
            // TIME
            // ─────────────────────────────

            uint64_t now =
                esp_timer_get_time();

            float fps = 0.0f;

            if (last_time_us != 0) {

                uint64_t diff =
                    now - last_time_us;

                if (diff > 0) {

                    fps =
                        1000000.0f /
                        (float)diff;
                }
            }

            last_time_us = now;

            // ─────────────────────────────
            // CALCULATE FRAME BITS
            // ─────────────────────────────

            uint32_t frame_bits =
                calculate_can_bits(
                    frame.extended,
                    frame.dlc);

            // ─────────────────────────────
            // UART COUNTER
            // ─────────────────────────────

            uint32_t uart_bytes = 0;

            uart_bytes += printf(
                "\n========== CAN FRAME #%lu ==========\n",
                (unsigned long)frame_count);

            // ─────────────────────────────
            // FRAME TYPE
            // ─────────────────────────────

            if (frame.extended) {

                uint32_t pgn =
                    extract_pgn(frame.id);

                uart_bytes += printf(
                    "Type : EXTENDED 29-bit\n");

                uart_bytes += printf(
                    "ID   : 0x%08lX\n",
                    (unsigned long)frame.id);

                uart_bytes += printf(
                    "PGN  : %lu\n",
                    (unsigned long)pgn);

            } else {

                uart_bytes += printf(
                    "Type : STANDARD 11-bit\n");

                uart_bytes += printf(
                    "ID   : 0x%03lX\n",
                    (unsigned long)frame.id);
            }

            // ─────────────────────────────
            // FRAME INFO
            // ─────────────────────────────

            uart_bytes += printf(
                "RTR  : %s\n",
                frame.rtr ? "YES" : "NO");

            uart_bytes += printf(
                "DLC  : %u\n",
                frame.dlc);

            uart_bytes += printf(
                "Data : ");

            for (int i = 0; i < frame.dlc; i++) {

                uart_bytes += printf(
                    "%02X ",
                    frame.data[i]);
            }

            uart_bytes += printf("\n");

            // ─────────────────────────────
            // LOAD CALCULATIONS
            // ─────────────────────────────

            float bus_load =
                ((float)(frame_bits * fps)
                / (float)CAN_BITRATE)
                * 100.0f;

            float pc_load =
                ((float)(uart_bytes * fps * 10)
                / (float)UART_BAUDRATE)
                * 100.0f;

            // ─────────────────────────────
            // PRINT STATS
            // ─────────────────────────────

            uart_bytes += printf(
                "Frame bits : %lu bits\n",
                (unsigned long)frame_bits);

            uart_bytes += printf(
                "FPS        : %.2f frames/sec\n",
                fps);

            uart_bytes += printf(
                "Bus Load   : %.2f %%\n",
                bus_load);

            uart_bytes += printf(
                "PC Load    : %.2f %%\n",
                pc_load);

            uart_bytes += printf(
                "Dropped    : %lu\n",
                (unsigned long)dropped_frames);

            uart_bytes += printf(
                "UART Bytes : %lu\n",
                (unsigned long)uart_bytes);

            uart_bytes += printf(
                "====================================\n");

            fflush(stdout);
        }
    }
}

// ─────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────

void app_main(void)
{
    // ─────────────────────────────
    // CREATE QUEUE
    // ─────────────────────────────

    rx_queue = xQueueCreate(
        RX_QUEUE_LEN,
        sizeof(can_frame_info_t));

    if (rx_queue == NULL) {

        printf("Failed to create queue\n");
        return;
    }

    // ─────────────────────────────
    // TWAI CONFIG
    // ─────────────────────────────

    twai_onchip_node_config_t node_config = {

        .io_cfg.tx = CAN_TX_PIN,
        .io_cfg.rx = CAN_RX_PIN,

        .bit_timing.bitrate =
            CAN_BITRATE,

        .tx_queue_depth = 5,

        // Sniffer mode
        .flags.enable_listen_only = true,
    };

    ESP_ERROR_CHECK(
        twai_new_node_onchip(
            &node_config,
            &twai_node));

    // ─────────────────────────────
    // REGISTER CALLBACK
    // ─────────────────────────────

    twai_event_callbacks_t callbacks = {
        .on_rx_done = twai_rx_callback,
    };

    ESP_ERROR_CHECK(
        twai_node_register_event_callbacks(
            twai_node,
            &callbacks,
            NULL));

    // ─────────────────────────────
    // ACCEPT ALL FRAMES
    // ─────────────────────────────

    twai_mask_filter_config_t filter_config = {

        .id = 0,
        .mask = 0,
        .is_ext = true,
    };

    ESP_ERROR_CHECK(
        twai_node_config_mask_filter(
            twai_node,
            0,
            &filter_config));

    // ─────────────────────────────
    // START TWAI
    // ─────────────────────────────

    ESP_ERROR_CHECK(
        twai_node_enable(twai_node));

    // ─────────────────────────────
    // START MESSAGE
    // ─────────────────────────────

    printf("\n");
    printf("╔════════════════════════════════════╗\n");
    printf("║         CAN SNIFFER READY         ║\n");
    printf("╠════════════════════════════════════╣\n");

    printf("║ CAN Bitrate : %-8d kbps       ║\n",
           CAN_BITRATE / 1000);

    printf("║ UART Baud   : %-8d            ║\n",
           UART_BAUDRATE);

    printf("║ TX GPIO     : %-8d            ║\n",
           CAN_TX_PIN);

    printf("║ RX GPIO     : %-8d            ║\n",
           CAN_RX_PIN);

    printf("║ Queue Size  : %-8d            ║\n",
           RX_QUEUE_LEN);

    printf("╚════════════════════════════════════╝\n");

    fflush(stdout);

    // ─────────────────────────────
    // CREATE TASK
    // ─────────────────────────────

    xTaskCreate(
        can_print_task,
        "can_print",
        4096,
        NULL,
        10,
        NULL);
}