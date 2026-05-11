#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_random.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "psa/crypto.h"

// ─────────────────────────────────────────────
// CONFIG
// ─────────────────────────────────────────────

#define CAN_TX_PIN        4
#define CAN_RX_PIN        5

#define CAN_BITRATE       250000
#define UART_BAUDRATE     921600

#define RX_QUEUE_LEN      512

// AES-GCM
#define AES_KEY_SIZE      16
#define AES_NONCE_SIZE    12

// ─────────────────────────────────────────────
// AES KEY
// ─────────────────────────────────────────────

static const uint8_t AES_KEY[AES_KEY_SIZE] = {

    0x2B, 0x7E, 0x15, 0x16,
    0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88,
    0x09, 0xCF, 0x4F, 0x3C
};

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

static psa_key_id_t aes_key_id = 0;

static volatile uint32_t frame_count = 0;

static volatile uint32_t dropped_frames = 0;

static volatile uint64_t total_can_bits = 0;

static volatile uint64_t total_uart_bytes = 0;

// ─────────────────────────────────────────────
// AES-GCM INIT
// ─────────────────────────────────────────────

static void aes_gcm_init(void)
{
    psa_status_t status;

    status = psa_crypto_init();

    if (status != PSA_SUCCESS) {

        printf("PSA crypto init failed\n");
        return;
    }

    psa_key_attributes_t attributes =
        PSA_KEY_ATTRIBUTES_INIT;

    psa_set_key_usage_flags(
        &attributes,
        PSA_KEY_USAGE_ENCRYPT);

    psa_set_key_algorithm(
        &attributes,
        PSA_ALG_GCM);

    psa_set_key_type(
        &attributes,
        PSA_KEY_TYPE_AES);

    psa_set_key_bits(
        &attributes,
        128);

    status = psa_import_key(

        &attributes,

        AES_KEY,
        sizeof(AES_KEY),

        &aes_key_id);

    psa_reset_key_attributes(
        &attributes);

    if (status != PSA_SUCCESS) {

        printf("AES key import failed\n");

    } else {

        printf("AES-GCM initialized\n");
    }
}

// ─────────────────────────────────────────────
// AES-GCM ENCRYPT
// ─────────────────────────────────────────────

static bool encrypt_frame_gcm(

    const uint8_t *plain,
    size_t plain_len,

    uint8_t *cipher,
    size_t *cipher_len)
{
    psa_status_t status;

    uint8_t nonce[AES_NONCE_SIZE];

    esp_fill_random(
        nonce,
        sizeof(nonce));

    size_t output_len = 0;

    status = psa_aead_encrypt(

        aes_key_id,

        PSA_ALG_GCM,

        nonce,
        sizeof(nonce),

        NULL,
        0,

        plain,
        plain_len,

        cipher,
        64,
        &output_len);

    if (status != PSA_SUCCESS) {

        return false;
    }

    *cipher_len = output_len;

    return true;
}

// ─────────────────────────────────────────────
// CALCULATE CAN BITS
// ─────────────────────────────────────────────

static uint32_t calculate_can_bits(
    bool extended,
    uint8_t dlc)
{
    uint32_t bits;

    if (extended) {

        bits = 67;

    } else {

        bits = 47;
    }

    bits += dlc * 8;

    // bit stuffing approximation
    bits = (bits * 12) / 10;

    return bits;
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

    if (twai_node_receive_from_isr(
            handle,
            &rx_frame) == ESP_OK) {

        can_frame_info_t frame = {0};

        frame.id =
            rx_frame.header.id;

        frame.extended =
            rx_frame.header.ide;

        frame.rtr =
            rx_frame.header.rtr;

        frame.dlc =
            rx_frame.buffer_len > 8 ?
            8 : rx_frame.buffer_len;

        memcpy(
            frame.data,
            rx_buffer,
            frame.dlc);

        BaseType_t woken = pdFALSE;

        if (xQueueSendFromISR(
                rx_queue,
                &frame,
                &woken) != pdTRUE) {

            dropped_frames++;
        }

        return woken == pdTRUE;
    }

    return false;
}

// ─────────────────────────────────────────────
// CAN PRINT TASK
// ─────────────────────────────────────────────

static void can_print_task(void *arg)
{
    can_frame_info_t frame;

    uint64_t start_time =
        esp_timer_get_time();

    while (1) {

        if (xQueueReceive(
                rx_queue,
                &frame,
                portMAX_DELAY) == pdTRUE) {

            frame_count++;

            uint32_t frame_bits =
                calculate_can_bits(
                    frame.extended,
                    frame.dlc);

            total_can_bits += frame_bits;

            uint64_t now =
                esp_timer_get_time();

            uint64_t ms =
                (now - start_time) / 1000;

            // AES-GCM encryption
            uint8_t encrypted[64];

            size_t encrypted_len = 0;

            encrypt_frame_gcm(
                frame.data,
                frame.dlc,
                encrypted,
                &encrypted_len);

            // ONE LINE OUTPUT
            printf(
                "I (%llu) CAP: ms=%llu,id=0x%08lX,ext=%d,dlc=%u,data=",

                (unsigned long long)ms,

                (unsigned long long)ms,

                (unsigned long)frame.id,

                frame.extended,

                frame.dlc
            );

            // print encrypted payload
            for (int i = 0;
                 i < encrypted_len;
                 i++) {

                printf("%02X ",
                       encrypted[i]);
            }

            printf("\n");

            fflush(stdout);

            // UART estimate
            total_uart_bytes += 100;
        }
    }
}

// ─────────────────────────────────────────────
// STATUS TASK
// ─────────────────────────────────────────────

static void status_task(void *arg)
{
    uint32_t last_frames = 0;

    uint32_t last_drops = 0;

    uint64_t last_can_bits = 0;

    uint64_t last_uart = 0;

    while (1) {

        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t fps =
            frame_count - last_frames;

        uint32_t drops =
            dropped_frames - last_drops;

        uint64_t can_bits_sec =
            total_can_bits - last_can_bits;

        uint64_t uart_bytes_sec =
            total_uart_bytes - last_uart;

        float bus_load =
            ((float)can_bits_sec /
            (float)CAN_BITRATE)
            * 100.0f;

        float pc_load =
            ((float)(uart_bytes_sec * 10) /
            (float)UART_BAUDRATE)
            * 100.0f;

        printf(
            "STATS: fps=%lu,bits=%llu,bus=%.2f%%,pc=%.2f%%,drop=%lu\n",

            (unsigned long)fps,

            (unsigned long long)can_bits_sec,

            bus_load,

            pc_load,

            (unsigned long)drops
        );

        fflush(stdout);

        last_frames = frame_count;

        last_drops = dropped_frames;

        last_can_bits = total_can_bits;

        last_uart = total_uart_bytes;
    }
}

// ─────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────

void app_main(void)
{
    // AES init
    aes_gcm_init();

    // create queue
    rx_queue = xQueueCreate(
        RX_QUEUE_LEN,
        sizeof(can_frame_info_t));

    if (rx_queue == NULL) {

        printf("Queue creation failed\n");

        return;
    }

    // TWAI config
    twai_onchip_node_config_t node_config = {

        .io_cfg.tx = CAN_TX_PIN,

        .io_cfg.rx = CAN_RX_PIN,

        .bit_timing.bitrate =
            CAN_BITRATE,

        .tx_queue_depth = 5,

        .flags.enable_listen_only = true,
    };

    ESP_ERROR_CHECK(
        twai_new_node_onchip(
            &node_config,
            &twai_node));

    // callback
    twai_event_callbacks_t callbacks = {

        .on_rx_done =
            twai_rx_callback,
    };

    ESP_ERROR_CHECK(
        twai_node_register_event_callbacks(
            twai_node,
            &callbacks,
            NULL));

    // accept all frames
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

    // enable TWAI
    ESP_ERROR_CHECK(
        twai_node_enable(
            twai_node));

    printf("\n");
    printf("CAN AES-GCM Sniffer Started\n");

    // tasks
    xTaskCreate(
        can_print_task,
        "can_print",
        6144,
        NULL,
        10,
        NULL);

    xTaskCreate(
        status_task,
        "status",
        4096,
        NULL,
        5,
        NULL);
}