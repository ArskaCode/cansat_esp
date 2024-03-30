#include "lora.h"

#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"


/*
 * Lora operating mode
 * bit 0 is M1, bit 1 is M0
 */
typedef enum {
    LORA_MODE_TRANSMISSION      = 0, // 0b00
    LORA_MODE_WOR_TRANSMITTING  = 1, // 0b01
    LORA_MODE_WOR_RECEIVING     = 2, // 0b10
    LORA_MODE_SLEEP             = 3, // 0b11
} lora_mode_t;


/*
 * Lora registers
 * See the E220 documentation for more info
 */
typedef enum {
    LORA_REG_ADDH = 0,
    LORA_REG_ADDL = 1,
    LORA_REG_REG0 = 2,
    LORA_REG_REG1 = 3,
    LORA_REG_REG2 = 4,
    LORA_REG_REG3 = 5,
    LORA_REG_PID  = 8,
} lora_reg_t;


static const char* TAG = "lora";


static struct {
    lora_mode_t mode;
} lora_state;


static void lora_read_bytes(void*, size_t);
static void lora_write_bytes(const void*, size_t);
static void lora_wait_aux(void);
static void lora_set_mode(lora_mode_t);


static TaskHandle_t aux_wait_task = NULL;
static void IRAM_ATTR gpio_aux_isr(void* arg)
{
    assert(aux_wait_task != NULL);
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(aux_wait_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}


void lora_init(bool *inits)
{
    // uart config
    const uart_port_t uart_num = UART_NUM_2;
    const int uart_buffer_size = (1024 * 2); // TODO: is this good

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    inits[0] &= LORA_SEND_ERROR(TAG, uart_param_config(uart_num, &uart_config));
    inits[0] &= LORA_SEND_ERROR(TAG, uart_set_pin(UART_NUM_2, CONFIG_LORA_TX, CONFIG_LORA_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    inits[0] &= LORA_SEND_ERROR(TAG, uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, NULL, 0));

    // M0 & M1 pin config
    gpio_config_t io_config_mode = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << CONFIG_LORA_M0) | (1ULL << CONFIG_LORA_M1),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    inits[0] &= LORA_SEND_ERROR(TAG, gpio_config(&io_config_mode));

/*
    // aux pin config
    gpio_config_t io_config_aux = {
        // disabled here because setting this to posedge would enable interrupts immediately
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CONFIG_LORA_AUX),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    LORA_SEND_ERROR(TAG, gpio_config(&io_config_aux));
    LORA_SEND_ERROR(TAG, gpio_set_intr_type(CONFIG_LORA_AUX, GPIO_INTR_POSEDGE));
    LORA_SEND_ERROR(TAG, gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    LORA_SEND_ERROR(TAG, gpio_isr_handler_add(CONFIG_LORA_AUX, gpio_aux_isr, NULL));
 */   
    lora_wait_aux();

    lora_state.mode = LORA_MODE_TRANSMISSION;

    LORA_SEND_LOG(TAG, "Initialized");
}


void lora_get_info(lora_info_t* info)
{
    // TODO: bettter error check(?), timeout
    if (lora_state.mode != LORA_MODE_SLEEP)
    {
        lora_set_mode(LORA_MODE_SLEEP);
        lora_wait_aux();
    }

    // Lora info is 3 bytes
    uint8_t command[3] = { 0xC1, LORA_REG_PID, 3 };
    uint8_t response[3+3];

    lora_write_bytes(command, sizeof(command));
    lora_read_bytes(response, sizeof(response));

    memcpy(info, response + 3, sizeof(lora_info_t));
}


void lora_set_address(uint16_t address)
{
    // TODO: bettter error check(?), timeout
    if (lora_state.mode != LORA_MODE_SLEEP)
    {
        lora_set_mode(LORA_MODE_SLEEP);
        lora_wait_aux();
    }

    uint8_t command[5] = { 0xC0, LORA_REG_ADDH, 2, (uint8_t)(address >> 8), (uint8_t)address };
    uint8_t response[5];

    lora_write_bytes(command, sizeof(command));
    lora_read_bytes(response, sizeof(response));
}


void lora_set_channel(uint8_t channel)
{
        // TODO: bettter error check(?), timeout
    if (lora_state.mode != LORA_MODE_SLEEP)
    {
        lora_set_mode(LORA_MODE_SLEEP);
        lora_wait_aux();
    }

    uint8_t command[4] = { 0xC0, LORA_REG_REG2, 1, channel };
    uint8_t response[4];

    lora_write_bytes(command, sizeof(command));
    lora_read_bytes(response, sizeof(response));
}


void lora_transmit(const void* data, size_t size)
{
    if (lora_state.mode != LORA_MODE_TRANSMISSION)
    {
        lora_set_mode(LORA_MODE_TRANSMISSION);
        lora_wait_aux();
    }

    lora_write_bytes(data, size);
}


void lora_receive(void* buf, size_t size)
{
    // TODO: bettter error check(?), timeout
    if (lora_state.mode != LORA_MODE_TRANSMISSION)
    {
        lora_set_mode(LORA_MODE_TRANSMISSION);
        lora_wait_aux();
    }

    lora_read_bytes(buf, size);
}


void lora_read_bytes(void* ptr, size_t size)
{
    if (uart_read_bytes(UART_NUM_2, ptr, size, portMAX_DELAY) < 0)
    {
        LORA_SEND_LOG(TAG, "UART read parameter error");
        abort();
    }
}

void lora_write_bytes(const void* ptr, size_t size)
{
    if (uart_write_bytes(UART_NUM_2, ptr, size) < 0)
    {
        LORA_SEND_LOG(TAG, "UART write parameter error");
        abort();
    }
}

void lora_wait_aux(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    return;

    aux_wait_task = xTaskGetCurrentTaskHandle();

    while (1)
    {
        LORA_SEND_ERROR(TAG, gpio_intr_enable(CONFIG_LORA_AUX));
        // 100 ms timeout if the rising edge of the AUX pin happened already
        // so this doesnt get stuck
        BaseType_t result = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        LORA_SEND_ERROR(TAG, gpio_intr_disable(CONFIG_LORA_AUX));
        // if the rising edge actually happened
        if (result == pdTRUE)
        {
            break;
        }
        else // timeout
        {
            // missed the rising edge maybe
            if (gpio_get_level(CONFIG_LORA_AUX) == 1)
            {
                break;
            }
            // else timeout, continue
        }
    }
    aux_wait_task = NULL;
    // "Therefore, the general recommendation is to detect the output state of the AUX pin
    // and switch after 2ms when the output is high" - Lora docs
    // This will wait 10-20 ms
    // the tick number is not 1 because it would be possible that it waits under 2 ms
    vTaskDelay(2);
}


void lora_set_mode(lora_mode_t mode)
{
    lora_state.mode = mode;

    LORA_SEND_LOG(TAG, "changing lora mode");

    LORA_SEND_ERROR(TAG, gpio_set_level(CONFIG_LORA_M0, (mode >> 1) & 0b1));
    LORA_SEND_ERROR(TAG, gpio_set_level(CONFIG_LORA_M1, mode & 0b1));
}