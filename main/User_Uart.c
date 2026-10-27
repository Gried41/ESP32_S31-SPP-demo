#include "User_Uart.h"

#include <inttypes.h>
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define MY_UART       UART_NUM_0
#define UART_TX_PIN   GPIO_NUM_58
#define UART_RX_PIN   GPIO_NUM_59

#define RX_SPP_CHUNK_SIZE   128

// UART driver creates this event queue in uart_driver_install.
static QueueHandle_t rx_to_spp_queue;
static portMUX_TYPE spp_handle_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t spp_connection_handle;
static bool spp_connected;


void uart_init(void)
{
    uart_config_t config = 
    {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /*
    MY_UART     : 要安装驱动的串口编号
    128         ：RX环形缓冲区大小，单位为字节
    0           ：TX环形缓冲区大小，0表示不创建
    20          ：UART事件队列最多容纳20个事件项
    spp_tx_queue：将创建的UART事件队列句柄写入 spp_tx_queue 队列
    0           ：使用默认的中断分配标志

    事件项目        ：当收到数据，硬件FIFO溢出等都会产生事件保存到队列句柄 spp_tx_queue 中 ，之后可以 if(event.type == UART_DATA) 来处理对应数据
    20          ：代表最多容纳20个事件项存储到队列句柄中
    这里并不会将RX串口收到的信息放入队列中，而是放入到RX环形缓冲区，之后由事件提醒触发
    */
    ESP_ERROR_CHECK(uart_driver_install(MY_UART, 1024, 0, 20, &rx_to_spp_queue, 0));
    //配置指定串口的参数，例如波特率、数据位、停止位、校验位和硬件流控
    ESP_ERROR_CHECK(uart_param_config(MY_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(MY_UART, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    //创建任务
    BaseType_t task_created = xTaskCreate(uart_task, "uart_task", 3072, NULL, 10, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

}

void uart_spp_set_connection(uint32_t handle, bool connected)
{
    portENTER_CRITICAL(&spp_handle_mux);
    if (connected) 
    {
        spp_connection_handle = handle;
        spp_connected = true;
    } 
    else if (spp_connected && spp_connection_handle == handle) 
    {
        spp_connected = false;
    }
    portEXIT_CRITICAL(&spp_handle_mux);
}

void uart_task(void *arg)
{
    (void)arg;
    uart_event_t event;
    uint8_t data[RX_SPP_CHUNK_SIZE];

    while (1) 
    {
        if (xQueueReceive(rx_to_spp_queue, &event, portMAX_DELAY) != pdTRUE) 
        {
            continue;
        }

        if (event.type == UART_DATA && event.size > 0) 
        {
            size_t remaining = event.size;
            while (remaining > 0) 
            {
                size_t requested = remaining > sizeof(data) ? sizeof(data) : remaining;
                int received = uart_read_bytes(MY_UART, data, (uint32_t)requested, pdMS_TO_TICKS(20));
                if (received <= 0) 
                {
                    ESP_LOGW("USER_UART", "UART read returned %d", received);
                    break;
                }
                uint32_t handle;
                bool connected;
                portENTER_CRITICAL(&spp_handle_mux);
                handle = spp_connection_handle;
                connected = spp_connected;
                portEXIT_CRITICAL(&spp_handle_mux);

                if (connected) 
                {
                    char buf[128];
                    esp_err_t ret = esp_spp_write(handle, received, data);
                    memcpy(buf , data , received);
                    buf[received] = '\0';
                    printf("send data : %s\r\n",buf);
                    if (ret != ESP_OK) 
                    {
                        ESP_LOGE("USER_UART", "esp_spp_write failed: %s", esp_err_to_name(ret));
                    }
                } 
                else 
                {
                    ESP_LOGW("USER_UART", "SPP is not connected; UART data discarded");
                }

                remaining -= (size_t)received;
            }
        }
    }
}