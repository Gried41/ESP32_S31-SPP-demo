#include "mcu_to_im.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define UART1          UART_NUM_1
#define UART1_TX_PIN   GPIO_NUM_13
#define UART1_RX_PIN   GPIO_NUM_12

QueueHandle_t im_uart_queue;

void uart1_init()
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
    ESP_ERROR_CHECK(uart_driver_install(UART1, 1024, 0, 20, &im_uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART1, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART1, UART1_TX_PIN, UART1_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    BaseType_t task_created = xTaskCreate(im_uart_to_mcu_task, "im_uart_to_mcu_task", 3072, NULL, 10, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
void im_uart_to_mcu_task(void *arg)
{
    //阻塞等待事件
    uint8_t data[128];
    //接收完整报文缓冲区
    uint8_t log_buf[2048];
    size_t  buf_len = 0;
    while(1)
    {
        uart_event_t event;
        if (xQueueReceive(im_uart_queue , &event, portMAX_DELAY) != pdTRUE) 
        {
            continue;
        }
        if(event.type == UART_DATA)
        {
            uint16_t len = event.size;  //缓冲区存在阈值，目前看来可能是128，导致最多读取到120字节就会返回一次事件
            while(len > 0)
            {
                uint16_t real_len = len > sizeof(data) ? sizeof(data) : len;
                uart_read_bytes(UART1 , data , real_len , portMAX_DELAY);
                //判断是否有\r\n 有则代表输出的包已满 可以换行分割
                char *star = memchr(data , '\r' , real_len);
                char *next = memchr(data , '\n' , real_len);
                if( star != NULL && next != NULL && (next - star == 1))//找到了报文结束,一次性打印报文
                {
                    if(real_len != 120)
                    {
                        data[real_len] = '\0';
                        real_len++;
                        memcpy(log_buf + buf_len , data , real_len);//前面加过\0，这里不用在加一次
                        buf_len += real_len;
                        printf("%s" , log_buf);
                        log_buf[0] = '\0';
                        buf_len = 0;        //清空缓冲区
                    }
                    else
                    {
                        data[real_len] = '\0';
                        real_len++;
                        memcpy(log_buf + buf_len, data , real_len);
                        buf_len += real_len;
                        printf("%s" , log_buf);
                        log_buf[0] = '\0';
                        buf_len = 0;        //清空缓冲区
                    }
                    break;
                }
                else//没找到，当前包还有下文
                {
                    if(real_len != 120)
                    {
                        memcpy(log_buf + buf_len, data , real_len);
                        buf_len += real_len;
                    }
                    else
                    {
                        memcpy(log_buf + buf_len, data , real_len);
                        buf_len += real_len;
                    } 
                }
                len -= real_len;
            }
        }
    }
}