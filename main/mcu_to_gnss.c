#include "mcu_to_gnss.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "mcu_to_im.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static uint32_t crc32_table[256];

#define MY_UART        UART_NUM_0
#define UART_TX_PIN    GPIO_NUM_58
#define UART_RX_PIN    GPIO_NUM_59


#define UART2          UART_NUM_2
#define UART3          UART_NUM_3
#define UART2_TX_PIN   GPIO_NUM_9
#define UART2_RX_PIN   GPIO_NUM_8
#define UART3_TX_PIN   GPIO_NUM_11
#define UART3_RX_PIN   GPIO_NUM_10

static QueueHandle_t rx_to_gnss_queue;
static QueueHandle_t gnss_uart2_queue;
static QueueHandle_t gnss_uart3_queue;
extern QueueHandle_t im_uart_queue;
/*
报文数量统计，判断丢包辅助验证
*/
#if UM980_LOG_CHECK_IS
static uint8_t uart2_gga_count;
static uint8_t uart2_rmc_count;
static uint8_t uart2_zda_count;
static uint8_t uart3_gga_count;
static uint8_t uart3_rmc_count;
static uint8_t uart3_zda_count;
#endif
//串口打印互斥锁
static SemaphoreHandle_t uart_printf_mutex = NULL;

/*
@fn : 串口初始化
*/
void mcu_uart_init(void)
{
    //串口0初始化 用于串口打印和接受指令
    uart_config_t config = 
    {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /*            可以理解为串口驱动配置
    MY_UART     : 要安装驱动的串口编号
    1024        ：RX环形缓冲区大小，单位为字节
    1024        ：TX环形缓冲区大小，0表示不创建，缓冲区不能过小，会出问题
    20          ：UART事件队列最多容纳20个事件项
    spp_tx_queue：将创建的UART事件队列句柄写入 spp_tx_queue 队列
    0           ：使用默认的中断分配标志

    事件项目     ：当收到数据，硬件FIFO溢出等都会产生事件保存到队列句柄 spp_tx_queue 中 ，之后可以 if(event.type == UART_DATA) 来处理对应数据
    20          ：代表最多容纳20个事件项存储到队列句柄中
    这里并不会将RX串口收到的信息放入队列中，而是放入到RX环形缓冲区，之后由事件提醒触发
    内部有超时判断，从第一个字节进入开始，要么溢出要么达到超时时间就会触发事件，任务检查到事件才会停止阻塞去执行。
    */
    uart_driver_install(MY_UART, 1024, 1024, 20, &rx_to_gnss_queue, 0);
    uart_param_config(MY_UART, &config);
    uart_set_pin(MY_UART, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/*
@fn:创建一个专门用于检查UART0 RX的任务，待rx_to_gnss_queue队列出现数据时调用串口发送
*/
void rx_to_gnss_task_init(void)
{
    BaseType_t task_created = xTaskCreate(rx_to_gnss_task, "rx_to_gnss_task", 3072, NULL, 10, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

void rx_to_gnss_task(void *arg)
{
    uart_event_t event;
    uint8_t data[128];
    //阻塞等待事件
    while(1)
    {
        /*
        xQueueReceive(rx_to_gnss_queue , &event, portMAX_DELAY);本身阻塞等待，这里避免
        函数返回失败，当结束阻塞但是返回失败时返回值为0，加上if条件限制避免这种情况，返回失败则继续下一轮阻塞
        */
        if (xQueueReceive(rx_to_gnss_queue , &event, portMAX_DELAY) != pdTRUE) 
        {
            continue;
        }
        if(event.type == UART_DATA)
        {
            uint8_t len = event.size;
            while(len > 0)
            {
                uint8_t real_len = len > sizeof(data) ? sizeof(data) : len;
                //从rx接收缓冲区中取出数据存放到 data 中， 取出长度为 real_len，最长等待20ms，也会读取到\r\n 
                uart_read_bytes(MY_UART , data , (uint32_t) real_len , pdMS_TO_TICKS(20));
                //对数据进行处理，处理后决定通过哪个串口发送出去
                Get_cmd_go(data , real_len);
                len -= real_len;
            }
        }
    }
}
/*
@fn: 处理串口接收到的数据,通过指令决定通过哪个串口发送
*/
void Get_cmd_go(uint8_t *data , uint8_t len)
{
    char cmd[129];
    if (len > sizeof(cmd) - 1) 
    {
        return;
    }
    memcpy(cmd, data, len);
    cmd[len] = '\0';
    if (len >= 5 && (memcmp(cmd, "com2 ", 5) == 0 || memcmp(cmd, "COM2 ", 5) == 0))
    {
        char *real_cmd = cmd + 5;
        uint8_t payload_len = len - 5;
        int written = uart_write_bytes(UART2, real_cmd, payload_len);
        if (written != payload_len) 
        {
            printf("uart2_write error \r\n");    //不需要\r\n 原本的data就在末尾有
        }
    }
    else if (len >= 5 && (memcmp(cmd, "com3 ", 5) == 0 || memcmp(cmd, "COM3 ", 5) == 0))
    {
        char *real_cmd = cmd + 5;
        uint8_t payload_len = len - 5;
        int written = uart_write_bytes(UART3, real_cmd, payload_len);
        if (written != payload_len) 
        {
            printf("uart1_write error \r\n");
        }
    }
    else if(len >= 3 && memcmp(cmd , "AT+" , 3) == 0)
    {
        int  written = uart_write_bytes(UART_NUM_1 , cmd , len);
        if(written != len)
        {
             printf("uart3_write error \r\n");
        }
    }
    else
    {
        printf("cmd unfind!\r\n");
    }
}
/*
UARY1 主要任务是接收gnss发送的数据，接着打印
*/
void uart2_init(void)
{
    uart_config_t config = 
    {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART2, 1024, 0, 20, &gnss_uart2_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART2, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART2, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    BaseType_t task_created = xTaskCreate(gnss_uart2_to_mcu_task, "gnss_uart2_to_mcu_task", 3072, NULL, 10, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

#if !UM980_LOG_CHECK_IS
/*
解决了一个事件最后字节因为被\0代替而导致打印数据丢失问题
*/
void gnss_uart2_to_mcu_task(void *arg)
{
    //阻塞等待事件
    uint8_t data[128];
    //接收完整报文缓冲区
    uint8_t *log_buf = malloc(4096);        //存放于堆空间
    size_t  buf_len = 0;
    while(1)
    {
        uart_event_t event;
        if (xQueueReceive(gnss_uart2_queue , &event, portMAX_DELAY) != pdTRUE) 
        {
            continue;
        }
        if(event.type == UART_DATA)
        {
            uint16_t len = event.size;  //缓冲区存在阈值，目前看来可能是128，导致最多读取到120字节就会返回一次事件
            while(len > 0)
            {
                uint16_t real_len = len > sizeof(data) ? sizeof(data) : len;
                uart_read_bytes(UART2 , data , real_len , portMAX_DELAY);
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
                        Get_Log_Type(log_buf , buf_len);
                        //printf("%s\r\n",data);
                        log_buf[0] = '\0';
                        buf_len = 0;        //清空缓冲区
                    }
                    else
                    {
                        data[real_len] = '\0';
                        real_len++;
                        memcpy(log_buf + buf_len, data , real_len);
                        buf_len += real_len;
                        Get_Log_Type(log_buf , buf_len);

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
#endif
#if UM980_LOG_CHECK_IS
/*
 * 处理一条完整的 GNSS NMEA 报文
 * uart_num = 1 表示你当前的 COM2 接收通道
 */
static void process_com2_nmea_line(char *line,uint32_t *checked_count,uint32_t *pass_count,uint32_t *fail_count,uint32_t *invalid_count)
{
    // 统计 GGA、RMC、ZDA 报文数量
    //check_log_format(line, 2);

    // 检查报文格式及异或校验
    int result = log_check(line);

    if (result == 1)
    {
        // 格式正确且校验通过
        (*checked_count)++;
        (*pass_count)++;
    }
    else if (result == 0)
    {
        // 格式正确，但校验失败
        (*checked_count)++;
        (*fail_count)++;

        printf("GNSS COM2 checksum fail: %s\r\n", line);
    }
    else
    {
        // 格式错误
        (*invalid_count)++;

        printf("GNSS COM2 invalid message: %s\r\n", line);
    }
}
void gnss_uart2_to_mcu_task(void *arg)
{
    uint32_t checked_count = 0;
    uint32_t pass_count = 0;
    uint32_t fail_count = 0;
    uint32_t invalid_count = 0;

    TickType_t start_tick = xTaskGetTickCount();
    TickType_t last_report_tick = start_tick;

    const TickType_t report_period = pdMS_TO_TICKS(60000);

    uart_event_t event;

    // UART 数据分块读取缓冲区，不需要添加 '\0'
    uint8_t data[128];

    /*
     * NMEA 完整报文缓存区。
     * 必须放在 while 循环外，才能保留跨多次 UART 接收的数据。
     */
    char nmea_line[256];
    size_t nmea_len = 0;

    /*
     * 1 表示当前报文超长或含有非法的 NUL 字节。
     * 此时丢弃后续字节，直到遇到 '\n' 再恢复。
     */
    int discard_line = 0;

    while (1)
    {
        /*
         * 最多等待 1 秒。
         * 即使超时，也不能直接 continue，
         * 否则会跳过下面的 60 秒统计检查。
         */
        BaseType_t queue_result = xQueueReceive(gnss_uart2_queue,&event,pdMS_TO_TICKS(1000));
        if (queue_result == pdTRUE)
        {
            switch (event.type)
            {
                case UART_DATA:
                {
                    /*
                     * event.size 可能大于 data 数组。
                     * 因此必须分块读取，并使用 size_t 保存长度。
                     */
                    size_t remaining = (size_t)event.size;

                    while (remaining > 0)
                    {
                        size_t want =
                            remaining > sizeof(data)
                            ? sizeof(data)
                            : remaining;

                        int read_len = uart_read_bytes(
                            UART2,
                            data,
                            (uint32_t)want,
                            pdMS_TO_TICKS(50)
                        );

                        if (read_len <= 0)
                        {
                            printf("GNSS COM2 UART read failed: %d\r\n",read_len);
                            break;
                        }

                        // 按实际读取长度更新剩余数据
                        remaining -= (size_t)read_len;

                        /*
                         * 逐字节解析。
                         * 一次读取可能包含半条、一条或多条 NMEA 报文。
                         */
                        for (int i = 0; i < read_len; i++)
                        {
                            char ch = (char)data[i];

                            // 当前报文已损坏或超长，丢弃直到行结束
                            if (discard_line)
                            {
                                if (ch == '\n')
                                {
                                    discard_line = 0;
                                    nmea_len = 0;
                                }

                                continue;
                            }

                            // 收到换行符，表示一条 NMEA 报文结束
                            if (ch == '\n')
                            {
                                // 去掉结尾的 '\r'
                                if (nmea_len > 0 &&
                                    nmea_line[nmea_len - 1] == '\r')
                                {
                                    nmea_len--;
                                }

                                // 添加 C 字符串结束符
                                nmea_line[nmea_len] = '\0';

                                // 忽略空行
                                if (nmea_len > 0)
                                {
                                    process_com3_nmea_line(
                                        nmea_line,
                                        &checked_count,
                                        &pass_count,
                                        &fail_count,
                                        &invalid_count
                                    );
                                }

                                // 当前报文处理完毕，准备下一条
                                nmea_len = 0;
                                continue;
                            }

                            // NMEA 文本正常情况下不应包含 NUL 字节
                            if (ch == '\0')
                            {
                                nmea_len = 0;
                                discard_line = 1;
                                invalid_count++;

                                printf(
                                    "GNSS COM2 invalid NUL byte\r\n"
                                );
                                continue;
                            }

                            // 检查缓存区容量，预留一个字节给 '\0'
                            if (nmea_len < sizeof(nmea_line) - 1)
                            {
                                nmea_line[nmea_len++] = ch;
                            }
                            else
                            {
                                // 报文超过缓存容量，丢弃当前报文
                                nmea_len = 0;
                                discard_line = 1;
                                invalid_count++;

                                printf(
                                    "GNSS COM2 message too long\r\n"
                                );
                            }
                        }
                    }

                    break;
                }

                case UART_FIFO_OVF:
                {
                    printf("GNSS COM2 UART FIFO overflow\r\n");

                    // 数据已经可能丢失，清理接收缓冲区并重置解析状态
                    uart_flush_input(UART2);
                    xQueueReset(gnss_uart2_queue);

                    nmea_len = 0;
                    discard_line = 0;
                    break;
                }

                case UART_BUFFER_FULL:
                {
                    printf("GNSS COM2 RX buffer full\r\n");

                    uart_flush_input(UART2);
                    xQueueReset(gnss_uart2_queue);

                    nmea_len = 0;
                    discard_line = 0;
                    break;
                }

                case UART_FRAME_ERR:
                    printf("GNSS COM2 UART frame error\r\n");
                    break;

                case UART_PARITY_ERR:
                    printf("GNSS COM2 UART parity error\r\n");
                    break;

                default:
                    break;
            }
        }

        /*
         * 定时检查放在事件处理之外。
         * 无论这一轮收到事件还是等待超时，都有机会执行。
         */
        TickType_t now = xTaskGetTickCount();

        if ((TickType_t)(now - last_report_tick) >= report_period)
        {
            uint32_t fail_rate_x100 = 0;

            if (checked_count > 0)
            {
                fail_rate_x100 =
                    (uint32_t)(
                        ((uint64_t)fail_count * 10000U)
                        / checked_count
                    );
            }

            uint32_t elapsed_seconds =
                (uint32_t)(
                    (now - start_tick) / configTICK_RATE_HZ
                );

            /*
             * 尽量避免等待打印互斥锁而长时间阻塞 UART 接收任务。
             * 如果锁正在使用，下次循环会再次尝试打印统计信息。
             */
            BaseType_t can_print = pdTRUE;

            if (uart_printf_mutex != NULL)
            {
                can_print = xSemaphoreTake(
                    uart_printf_mutex,
                    pdMS_TO_TICKS(10)
                );
            }

            if (can_print == pdTRUE)
            {
                printf(
                    "\r\n===== GNSS COM2 CHECKSUM STATISTICS =====\r\n"
                    "Elapsed       : %lu seconds\r\n"
                    "Checked       : %lu\r\n"
                    "Pass          : %lu\r\n"
                    "Fail          : %lu\r\n"
                    "Failure rate  : %lu.%02lu%%\r\n"
                    "Invalid format: %lu\r\n"
                    "=========================================\r\n",

                    (unsigned long)elapsed_seconds,
                    (unsigned long)checked_count,
                    (unsigned long)pass_count,
                    (unsigned long)fail_count,
                    (unsigned long)(fail_rate_x100 / 100),
                    (unsigned long)(fail_rate_x100 % 100),
                    (unsigned long)invalid_count

                    // 当前任务接收的是 COM2，因此使用 UART3 对应计数器
                );

                if (uart_printf_mutex != NULL)
                {
                    xSemaphoreGive(uart_printf_mutex);
                }

                last_report_tick = now;
            }
        }
    }
}
#endif

/*
UARY3 主要任务是接收gnss发送的数据，接着打印
*/
void uart3_init(void)
{
    uart_config_t config = 
    {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART3, 1024, 0, 20, &gnss_uart3_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART3, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART3, UART3_TX_PIN, UART3_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    BaseType_t task_created = xTaskCreate(gnss_uart3_to_mcu_task, "gnss_uart3_to_mcu_task", 3072, NULL, 10, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}


/*
@fn:判断报文类别，带有返回值,返回值为报文类型，枚举
*/
void Get_Log_Type(uint8_t *data , size_t len)
{
    //进入的为整个报文数据
    switch (Log_Type(data , len))
    {
        case MSG_RMC:
            RMC_Info rmc;
            RMC_MSG_Parse(data , len , &rmc);
            break;

        case MSG_GGA:
            GGA_Info gga;
            GGA_MSG_Parse(data , len , &gga);
            break;

        case MSG_OBSVMCMPA:
            CMPA_Info cmpa;
            OBSVMCMPA_MSG_Parse(data , &cmpa);
            //printf("%s",data);
            break;

        default:
            printf("%s",data);
            break;
    }
}
GNSS_MSG_TYPE Log_Type(uint8_t *data , size_t len)
{
    if(memcmp(data , "$GNRMC" , 6) == 0)
    {
        return MSG_RMC;
    }
    else if(memcmp(data , "$GNGGA" , 6) == 0)
    {
        return MSG_GGA;
    }
    else if(memcmp(data , "#OBSVMCMPA" , 10) == 0)
    {
        return MSG_OBSVMCMPA;
    }
    else
    {
        return MSG_UNKNOW;
    }
}
/*
@fn:对OBSVMCMPA报文进行解析，这里先尝试解析第一条数据
该报文第一部分为消息头，截至到;号
*/
void OBSVMCMPA_MSG_Parse(uint8_t *data , CMPA_Info * CMPA)
{

    //将指针定位到分号后第一个','的数据部分
    uint8_t *star = (memchr((const char*)data , ';' , strlen((const char*)data)))+1;      //定位到 ；号
    size_t log_count;
    sscanf((const char*)star , "%zu" , &log_count);                                     //获取报文内完整数据的数量
    star = (memchr((const char*)star , ',' , strlen((const char*)star)))+1;             //单个检测数据数据头
    uint8_t *end = memchr((const char*)star , ',' , strlen((const char*)star));         //单个检测数据尾部','号
    printf("log count : %d\r\n" , log_count);
    /*实现对整个报文的数据解析并收集打印*/
    for(int i = 0 ; i < log_count-1 ; i++)
    {
        if(i == log_count -2)                   //最后一段完整的数据处理方式不同
        {
            star=end+1;
            end = memchr((const char*)star , '*' , strlen((const char*)star));  //最后的结尾是以*做标志的
            Single_Data_Parse(star , end , CMPA);
            break;
        }
        Single_Data_Parse(star , end , CMPA);
        star=end+1;
        end = memchr((const char*)star , ',' , strlen((const char*)star));
    }
}

//static void Head_Data_Parse(uint8_t *star , uint8_t *end , CMPA_Data_Head * )

/*
@fn:负责找到单个的48字符数据，分割号为','
*/
static void Single_Data_Parse(uint8_t *star , uint8_t *end , CMPA_Info *CMPA)
{
    uint8_t buf[25];
    Decode_Hex_Record((const char*)star , 48 , buf);
    CMPA->tracking_raw   = Get_Bits_LSB(buf , 0, 32);      //获取通道跟踪状态 起始bit = 0，位长32bit
    CMPA->doppler_raw    = Get_Bits_LSB(buf , 32 , 28);    //获取多普勒原始数据，起始位32 位长28bit
    CMPA->psr_raw        = Get_Bits_LSB(buf , 60 , 36);    //获取伪距PSR原始数据   起始位60 位长36
    CMPA->adr_raw        = Get_Bits_LSB(buf , 96 , 32);    //获取载波相位ADR       起始位96 位长32
    CMPA->psr_std_index  = Get_Bits_LSB(buf, 128, 4);      //获取伪距标准差索引
    CMPA->adr_std_index  = Get_Bits_LSB(buf, 132, 4);      //获取载波相位标准差索引
    CMPA->prn            = Get_Bits_LSB(buf, 136, 8);      //获取卫星PRN
    CMPA->lock_time_raw  = Get_Bits_LSB(buf, 144, 21);     //获取锁定时间
    CMPA->cne_raw        = Get_Bits_LSB(buf, 165, 5);      //获取C/N0
    printf("%llu    %llu    %llu    %llu    %llu    %llu    %llu    %llu    %llu\r\n", CMPA->tracking_raw,
    CMPA->doppler_raw , CMPA->psr_raw , CMPA->adr_raw , CMPA->psr_std_index , CMPA->adr_std_index , CMPA->prn,
    CMPA->lock_time_raw , CMPA->cne_raw);
    //printf("%.*s\r\n" , 8 , star);
}

/*
@fn:将十六进制字符数据转换为整数 如 1 -> 1 ; 9 -> 9; a -> 10; f -> 15...
*/
static int HEX_To_Byte(char c)
{
    if(c >= '0' && c <= '9')
        return c - '0';
    if(c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if(c >= 'A' && c <= 'F')
        return c - 'A' + 10;    
    return -1;
}
/*
@fn:将48个十六进制字符数据全部转换为对应的十六数值，分高低位存储到一个字节空间中，这里主要是将两个十六进制数据存储到一个字节空间中去，形式变化在HEX_To_Byte函数中
 hex 指向48个字符数据的起始位置， hex_len 为字符数据长度 但必须为偶数 奇数会导致溢出，此类型报文数据都为偶数 uint8_t out[24]预期指向一个能存放24字节的缓冲区
*/
static bool Decode_Hex_Record(const char *hex , size_t hex_len , uint8_t out[24])   //out 接收一个数组名其实也相当于指针 函数形参中的数组声明会自动调整为指针声明
{
    if (hex == NULL || out == NULL || hex_len != 48)
    {
        printf("hex == NULL || out == NULL || hex_len != 48 error\r\n");
        return false;
    }
    for(size_t i = 0; i < 24; i++)
    {   

        int high = HEX_To_Byte(hex[i * 2]);
        int low  = HEX_To_Byte(hex[i * 2 +1]);      //★★★★  循环中同时处理两个数据方法
        if (high < 0 || low < 0)                    //避免为负数
        {
            printf("high < 0 || low < 0 \r\n");
            return false;
        }
        out[i] = (uint8_t)((high << 4) | low);      //high在高四位，low在低四位     
    }
    return true;
}
/*
@fn:从一段uint8_t 空间中 按照起始位和长度，提取出二进制数据并将其转化为整数
    data 空间的头指针， start_bit 从第几位开始提取 ， width 一共提取多少位 返回值为提取出的二进制而组成的整数
*/
static uint64_t Get_Bits_LSB(const uint8_t *data , unsigned start_bit , unsigned width)
{
    uint64_t value = 0;
    for (unsigned i = 0; i < width; i++)                            /*假设data[0] = 0Xb2,start_bit = 3 , width = 4*/
    {
        unsigned bit_pos = start_bit + i;                           //i = 3,                        1u的作用是保留最低位，其他清零 相当于掩码最低位为1
        if ((data[bit_pos / 8] >> (bit_pos % 8)) & 1u)              //3/8 = 0 ,(data[0] >> 3) = 1011 0010 >> 3 = 0001 0110  & 1u = 0000 0000
        {
            value |= (uint64_t)1 << i;                              //找到目标位，判断是 1 还是 0，是 1 则 0000 0000 |= 0000 0001 不是则跳过，一直遍历到i < width
        }
    }
    return value;
}

/*
目前打算先将帧数据分包，一包代表一个完整的单次检测数据，头尾指针分别指向单次数据的头尾部，
然后将这个48个十六进制数据转换为24字节数据，这个24字节数据就作为单次检测的一个数据
此时将这次的检测数据放入Get_Bits_LSB单独取出不同数据段的数据
*/




void RMC_MSG_Parse(uint8_t *data , size_t len , RMC_Info * RMC)
{
    uint8_t *star = memchr(data , ',' , strlen((const char*)data));
    star++;
    uint8_t *end  = memchr(star , ',' , strlen((const char*)star));
    UTC_Time_Parse(star , end , &RMC->time);
    star = (end+1);
    end  = memchr(star , ',' , strlen((const char*)star));
    sscanf((const char*)star , "%c" , &RMC->status);   //结构体存储了status
    printf("status:%c\r\n",RMC->status);             
    star = (end+1);
    end  = (memchr(star , ',' , strlen((const char*)star)))+2;
    Latitude_Parse(star , end , &RMC->latitude);
    star = (end+1);
    end  = (memchr(star , ',' , strlen((const char*)star)))+2;
    Longitude_Parse(star , end , &RMC->longitude);
    star = (end+1);
    end  = (memchr(star , ',' , strlen((const char*)star)))+1;
    end  = memchr(end , ',' , strlen((const char*)end));
    sscanf((const char*)star , "%lf,%lf" , &RMC->ground_speed , &RMC->ground_angle);
    printf("speed:%.3lfKm\\h angle:%.1lf deg\r\n" , RMC->ground_speed*1.852 , RMC->ground_angle);              //★★★
    star = (end+1);
    end  = (memchr(star , ',' , strlen((const char*)star)));
    ddmmyy_Data_Parse(star , end , &RMC->date);
    star = (end+1);
    end  = (memchr(star , ',' , strlen((const char*)star)))+6;
    Magnetic_Yaw_Parse(star , end , &RMC->yaw);
    printf("\r\n");
}   

/*
@fn:解析GGA报文数据，可以返回一个结构体，这里先正常解析所有数据,返回结构体无法应对多种情况，这里将结构体指针作为参数传入
$GNGGA,075924.00,2309.44452648,N,11329.68284448,E,1,28,0.6,51.5391,M,-6.4446,M,,*51
每隔一个 ‘ ，‘号就是一段数据，
*/
void GGA_MSG_Parse(uint8_t *data , size_t len , GGA_Info * GGA)
{
    uint8_t *star = memchr(data , ',' , strlen((const char*)data));//将指针移向第一块数据前的 ’,‘号
    star++;  // 指向第一个逗号后面的数据
    //size_t remaining = (size_t)len - (size_t)(star - data);     //避免搜索长度大于数据长度
    uint8_t *end = memchr(star, ',', strlen((const char*)data));
    UTC_Time_Parse(star , end , &GGA->time);
    /*这次解析完毕后还需要更新star 和 end 指针的位置，方便下次解析*/
    star = end+1;
    end  = (memchr(star , ',' , strlen((const char*)star)))+2; //获取到北纬或者南纬字符
    Latitude_Parse(star , end , &GGA->latitude);
    star = end+1;
    end  = (memchr(star , ',' , strlen((const char*)star)))+2;//获取到东经或者西经字符
    Longitude_Parse(star , end , &GGA->longitude);
    star = end+1;
    end  = memchr(star , ',' , strlen((const char*)star));
    end+=7;
    GGA_Base_Parse(star , end , GGA);
    star = end+1;
    end  = memchr(star , ',' , strlen((const char*)star));
    end+=12; //获取海拔高度单位字符
    Altitude_Parse(star , end , GGA);
}
/*
@fn:获取报文的UTC时间
*arg ：为报文UTC数据部分的 头尾 ’，‘符号，数据部分长度 
数据实例：010353.00
*/
void UTC_Time_Parse(uint8_t *star , uint8_t *end , UTC_Time *time)
{
    size_t len = end - star;
    char data_buf[len+1];
    for(int i = 0;i < len ; i++)
    {
        data_buf[i] = *star;
        star++;
    }
    data_buf[len] = '\0';        //末尾添加\0
    int ret = sscanf(data_buf , "%2d%2d%2d.%2d" , &time->hour , &time->minute , &time->second , &time->millisecond);
    time->hour+=8;//转换为北京时间 UCT+8
    if(ret == 4)
    {
        printf("%02d.%02d:%02d.%02d\r\n" , time->hour , time->minute , time->second , time->millisecond);
        //printf("%s\r\n" , data_buf);
    }
    else
    {
        printf("ret = %d\r\n" , ret);
    }
}
/*
@fn:获取报文的纬度
*arg ：为报文数据部分的 头部数据，末尾 ’ ，‘号下标，
数据实例：2309.44452648,N       前两位为纬度的度数，后两位和小数部分一起/60作为纬度的分数， N代表北纬
*/
void Latitude_Parse(uint8_t *star , uint8_t *end , Latitude_Info *latitude)
{
    size_t len = end - star;
    char data_buf[len+1];
    for(int i = 0;i < len ; i++)
    {
        data_buf[i] = *star;
        star++;
    }
    data_buf[len] = '\0'; 
    /* 维度格式 ： 2309.44454175,N   */
    sscanf(data_buf , "%2d%lf,%c" , &latitude->degree , &latitude->minute  , &latitude->direction);
    latitude->decimal_degree = latitude->degree + latitude->minute / 60.00000000;
    printf("%lf deg %c\r\n",latitude->decimal_degree, latitude->direction);
}
/*
@fn:获取报文的经度数据
*arg ：为报文数据部分的 头部数据，末尾 ’ ，‘号下标，
数据示例为：11329.68317841,E     //前三位为经度的度数 ，后两位加上小数部分/60 为经度的分数 ， E代表东经，
*/
void Longitude_Parse(uint8_t *star , uint8_t *end , Longitude_Info *longitude)
{
    size_t len = end - star;
    char data_buf[len+1];
    for(int i = 0;i < len ; i++)
    {
        data_buf[i] = *star;
        star++;
    }
    data_buf[len] = '\0'; 
    sscanf( data_buf,"%3d%lf,%c" , &longitude->degree , &longitude->minute , &longitude->direction);
    longitude->decimal_degree = longitude->degree + longitude->minute/60.000000;
    printf("%lf deg %c\r\n",longitude->decimal_degree, longitude->direction);
}
void Altitude_Parse(uint8_t *star , uint8_t *end , GGA_Info *GGA)
{
    size_t len = end - star;
    char data_buf[len+1];
    for(int i = 0;i < len ; i++)
    {
        data_buf[i] = *star;
        star++;
    }
    data_buf[len] = '\0';
    sscanf(data_buf,"%lf,%c,%lf,%c",&GGA->Altitude.Altitude,&GGA->Altitude.Altitude_char,&GGA->Altitude.ellipsoid_height,&GGA->Altitude.ellipsoid_height_char);
    printf("Altitude:%.2f%c  ellipsoid:%.3f%c\r\n",GGA->Altitude.Altitude , GGA->Altitude.Altitude_char , GGA->Altitude.ellipsoid_height , GGA->Altitude.ellipsoid_height_char);
    printf("\r\n");
}
/*
@fn:获取报文的 定位质量 ， 卫星数量 ， 定位因子 数据
*arg ：为报文数据部分的 头部数据，末尾 ’ ，‘号下标，
数据示例为：1,28,0.6    分别为 (定位质量)：1，(卫星数量)28，(定位因子)0.6
*/
void GGA_Base_Parse(uint8_t *star , uint8_t *end , GGA_Info *GGA)
{
    size_t len = end - star;
    char data_buf[len+1];
    for(int i = 0;i < len ; i++)
    {
        data_buf[i] = *star;
        star++;
    }
    data_buf[len] = '\0'; 
    sscanf(data_buf , "%1d,%2d,%lf" , &GGA->position_quality , &GGA->satellite_count , &GGA->Hdop);
    printf("position_quality:%d satellite_count:%d Hdop:%.1f \r\n",GGA->position_quality,GGA->satellite_count,GGA->Hdop);
}
/*
@fn:获取报文的日期信息， 格式为 ddmmyy  d代表日，m代表月，y代表年
*arg ：为报文数据部分的 头部数据，末尾 ’ ，‘号下标，
数据示例为：271026
*/
void ddmmyy_Data_Parse(uint8_t *star , uint8_t *end , Date *date)
{
    size_t len = end - star;
    char data_buf[len+1];
    for(int i = 0;i < len ; i++)
    {
        data_buf[i] = *star;
        star++;
    }
    data_buf[len] = '\0'; 
    sscanf(data_buf , "%2d%2d%2d" , &date->day , &date->month , &date->year);
    printf("20%d/%d/%d\r\n" , date->year , date->month , date->day);
}
/*
@fn:获取报文的日期信息， 格式为 ddmmyy  d代表日，m代表月，y代表年
*arg ：为报文数据部分的 头部数据，末尾 ’ ，‘号下标，
数据示例为：271026
*/
void Magnetic_Yaw_Parse(uint8_t *star , uint8_t *end , Magnetic_Yaw *yaw)
{
    size_t len = end - star;
    char data_buf[len+1];
    for(int i = 0;i < len ; i++)
    {
        data_buf[i] = *star;
        star++;
    }
    data_buf[len] = '\0'; 
    sscanf(data_buf , "%lf,%c,%c,%c" ,&yaw->angle ,&yaw->direction , &yaw->mode , &yaw->status);
    printf("yaw:%.1f %c   mode:%c   status:%c\r\n" , yaw->angle ,yaw->direction , yaw->mode , yaw->status);
}



#if !UM980_LOG_CHECK_IS
void gnss_uart3_to_mcu_task(void *arg)
{
    uint8_t data[128];
    //接收完整报文缓冲区
    uint8_t log_buf[1024];
    size_t  buf_len = 0;
    while(1)
    {
        uart_event_t event;
        if (xQueueReceive(gnss_uart3_queue , &event, portMAX_DELAY) != pdTRUE) 
        {
            continue;
        }
        if(event.type == UART_DATA)
        {
            uint16_t len = event.size;  //缓冲区存在阈值，目前看来可能是128，导致最多读取到120字节就会返回一次事件
            while(len > 0)
            {
                uint16_t real_len = len > sizeof(data) ? sizeof(data) : len;
                int data_count = uart_read_bytes(UART3 , data , real_len , portMAX_DELAY);
                //判断是否有\r\n 有则代表输出的包已满 可以换行分割
                char *star = memchr(data , '\r' , real_len);
                char *next = memchr(data , '\n' , real_len);
                if( star != NULL && next != NULL && (next - star == 1))//找到了报文结束,一次性打印报文,增加报文类型判断
                {
                    if(real_len != 120)
                    {
                        data[real_len] = '\0';
                        real_len++;
                        memcpy(log_buf + buf_len , data , real_len);//前面加过\0，这里不用在加一次
                        buf_len += real_len;
                        printf("%s" , log_buf);
                        /*-----------类型检测--------------*/
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
#endif
#if UM980_LOG_CHECK_IS
/*
 * 处理一条完整的 GNSS NMEA 报文
 * uart_num = 2 表示你当前的 COM3 接收通道
 */
static void process_com3_nmea_line(char *line,uint32_t *checked_count,uint32_t *pass_count,uint32_t *fail_count,uint32_t *invalid_count)
{
    // 统计 GGA、RMC、ZDA 报文数量
    //check_log_format(line, 2);

    // 检查报文格式及异或校验
    int result = log_check(line);

    if (result == 1)
    {
        // 格式正确且校验通过
        (*checked_count)++;
        (*pass_count)++;
    }
    else if (result == 0)
    {
        // 格式正确，但校验失败
        (*checked_count)++;
        (*fail_count)++;

        printf("GNSS COM3 checksum fail: %s\r\n", line);
    }
    else
    {
        // 格式错误
        (*invalid_count)++;

        printf("GNSS COM3 invalid message: %s\r\n", line);
    }
}
void gnss_uart3_to_mcu_task(void *arg)
{
    uint32_t checked_count = 0;
    uint32_t pass_count = 0;
    uint32_t fail_count = 0;
    uint32_t invalid_count = 0;

    TickType_t start_tick = xTaskGetTickCount();
    TickType_t last_report_tick = start_tick;

    const TickType_t report_period = pdMS_TO_TICKS(60000);

    uart_event_t event;

    // UART 数据分块读取缓冲区，不需要添加 '\0'
    uint8_t data[128];

    /*
     * NMEA 完整报文缓存区。
     * 必须放在 while 循环外，才能保留跨多次 UART 接收的数据。
     */
    char nmea_line[256];
    size_t nmea_len = 0;

    /*
     * 1 表示当前报文超长或含有非法的 NUL 字节。
     * 此时丢弃后续字节，直到遇到 '\n' 再恢复。
     */
    int discard_line = 0;

    while (1)
    {
        /*
         * 最多等待 1 秒。
         * 即使超时，也不能直接 continue，
         * 否则会跳过下面的 60 秒统计检查。
         */
        BaseType_t queue_result = xQueueReceive(gnss_uart3_queue,&event,pdMS_TO_TICKS(1000));
        if (queue_result == pdTRUE)
        {
            switch (event.type)
            {
                case UART_DATA:
                {
                    /*
                     * event.size 可能大于 data 数组。
                     * 因此必须分块读取，并使用 size_t 保存长度。
                     */
                    size_t remaining = (size_t)event.size;

                    while (remaining > 0)
                    {
                        size_t want =
                            remaining > sizeof(data)
                            ? sizeof(data)
                            : remaining;

                        int read_len = uart_read_bytes(
                            UART3,
                            data,
                            (uint32_t)want,
                            pdMS_TO_TICKS(50)
                        );

                        if (read_len <= 0)
                        {
                            printf(
                                "GNSS COM3 UART read failed: %d\r\n",
                                read_len
                            );
                            break;
                        }

                        // 按实际读取长度更新剩余数据
                        remaining -= (size_t)read_len;

                        /*
                         * 逐字节解析。
                         * 一次读取可能包含半条、一条或多条 NMEA 报文。
                         */
                        for (int i = 0; i < read_len; i++)
                        {
                            char ch = (char)data[i];

                            // 当前报文已损坏或超长，丢弃直到行结束
                            if (discard_line)
                            {
                                if (ch == '\n')
                                {
                                    discard_line = 0;
                                    nmea_len = 0;
                                }

                                continue;
                            }

                            // 收到换行符，表示一条 NMEA 报文结束
                            if (ch == '\n')
                            {
                                // 去掉结尾的 '\r'
                                if (nmea_len > 0 &&
                                    nmea_line[nmea_len - 1] == '\r')
                                {
                                    nmea_len--;
                                }

                                // 添加 C 字符串结束符
                                nmea_line[nmea_len] = '\0';

                                // 忽略空行
                                if (nmea_len > 0)
                                {
                                    process_com3_nmea_line(
                                        nmea_line,
                                        &checked_count,
                                        &pass_count,
                                        &fail_count,
                                        &invalid_count
                                    );
                                }

                                // 当前报文处理完毕，准备下一条
                                nmea_len = 0;
                                continue;
                            }

                            // NMEA 文本正常情况下不应包含 NUL 字节
                            if (ch == '\0')
                            {
                                nmea_len = 0;
                                discard_line = 1;
                                invalid_count++;

                                printf(
                                    "GNSS COM3 invalid NUL byte\r\n"
                                );
                                continue;
                            }

                            // 检查缓存区容量，预留一个字节给 '\0'
                            if (nmea_len < sizeof(nmea_line) - 1)
                            {
                                nmea_line[nmea_len++] = ch;
                            }
                            else
                            {
                                // 报文超过缓存容量，丢弃当前报文
                                nmea_len = 0;
                                discard_line = 1;
                                invalid_count++;

                                printf(
                                    "GNSS COM3 message too long\r\n"
                                );
                            }
                        }
                    }

                    break;
                }

                case UART_FIFO_OVF:
                {
                    printf("GNSS COM3 UART FIFO overflow\r\n");

                    // 数据已经可能丢失，清理接收缓冲区并重置解析状态
                    uart_flush_input(UART3);
                    xQueueReset(gnss_uart3_queue);

                    nmea_len = 0;
                    discard_line = 0;
                    break;
                }

                case UART_BUFFER_FULL:
                {
                    printf("GNSS COM3 RX buffer full\r\n");

                    uart_flush_input(UART3);
                    xQueueReset(gnss_uart3_queue);

                    nmea_len = 0;
                    discard_line = 0;
                    break;
                }

                case UART_FRAME_ERR:
                    printf("GNSS COM3 UART frame error\r\n");
                    break;

                case UART_PARITY_ERR:
                    printf("GNSS COM3 UART parity error\r\n");
                    break;

                default:
                    break;
            }
        }

        /*
         * 定时检查放在事件处理之外。
         * 无论这一轮收到事件还是等待超时，都有机会执行。
         */
        TickType_t now = xTaskGetTickCount();

        if ((TickType_t)(now - last_report_tick) >= report_period)
        {
            uint32_t fail_rate_x100 = 0;

            if (checked_count > 0)
            {
                fail_rate_x100 =
                    (uint32_t)(
                        ((uint64_t)fail_count * 10000U)
                        / checked_count
                    );
            }

            uint32_t elapsed_seconds =
                (uint32_t)(
                    (now - start_tick) / configTICK_RATE_HZ
                );

            /*
             * 尽量避免等待打印互斥锁而长时间阻塞 UART 接收任务。
             * 如果锁正在使用，下次循环会再次尝试打印统计信息。
             */
            BaseType_t can_print = pdTRUE;

            if (uart_printf_mutex != NULL)
            {
                can_print = xSemaphoreTake(
                    uart_printf_mutex,
                    pdMS_TO_TICKS(10)
                );
            }

            if (can_print == pdTRUE)
            {
                printf(
                    "\r\n===== GNSS COM3 CHECKSUM STATISTICS =====\r\n"
                    "Elapsed       : %lu seconds\r\n"
                    "Checked       : %lu\r\n"
                    "Pass          : %lu\r\n"
                    "Fail          : %lu\r\n"
                    "Failure rate  : %lu.%02lu%%\r\n"
                    "Invalid format: %lu\r\n"
                    "=========================================\r\n",

                    (unsigned long)elapsed_seconds,
                    (unsigned long)checked_count,
                    (unsigned long)pass_count,
                    (unsigned long)fail_count,
                    (unsigned long)(fail_rate_x100 / 100),
                    (unsigned long)(fail_rate_x100 % 100),
                    (unsigned long)invalid_count

                    // 当前任务接收的是 COM3，因此使用 UART3 对应计数器
                );

                if (uart_printf_mutex != NULL)
                {
                    xSemaphoreGive(uart_printf_mutex);
                }

                last_report_tick = now;
            }
        }
    }
}

void check_log_format(const char *recv_data , int uart_num)
{
    switch(uart_num)
    {
        case 1:
            if (strncmp(recv_data, "$GPGGA", 6) == 0 || strncmp(recv_data, "$GNGGA", 6) == 0)
            {
                uart2_gga_count++;
            }
            else if (strncmp(recv_data, "$GPRMC", 6) == 0 ||strncmp(recv_data, "$GNRMC", 6) == 0)
            {
                uart2_rmc_count++;
            }
            else if (strncmp(recv_data, "$GPZDA", 6) == 0 ||strncmp(recv_data, "$GNZDA", 6) == 0)
            {
                uart2_zda_count++;
            }
            break;
        case 2:
            if (strncmp(recv_data, "$GPGGA", 6) == 0 || strncmp(recv_data, "$GNGGA", 6) == 0)
            {
                uart3_gga_count++;
            }
            else if (strncmp(recv_data, "$GPRMC", 6) == 0 ||strncmp(recv_data, "$GNRMC", 6) == 0)
            {
                uart3_rmc_count++;
            }
            else if (strncmp(recv_data, "$GPZDA", 6) == 0 ||strncmp(recv_data, "$GNZDA", 6) == 0)
            {
                uart3_zda_count++;
            }
            break;
        default:
            break;
    }

}


static int hex_to_value(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10;
    }
    return -1;
}
/*
 *  返回值：
 *  1：校验通过
 *  0：校验失败
 * -1：格式不正确或没有校验字段
 */
static int log_check(const char *log)
{
    //检查开头不为  $   则认为报文格式不正确
    if(log == NULL || log[0] != '$')
        return -1;
    //找到 * 第一次出现的位置，返回指针
    const char *star = strchr(log , '*');
    if (star == NULL || star[1] == '\0' || star[2] == '\0')
        return -1;
    /* 本函数要求 *HH 后面直接结束 */
    if (star[3] != '\0')
    {
        if (!(star[3] == '\r' && star[4] == '\n' && star[5] == '\0'))
        return -1;
    }
    int high = hex_to_value(star[1]);
    int low  = hex_to_value(star[2]);
    //说明不是16进制 0-F
    if (high < 0 || low < 0)
        return -1;
    uint8_t calculated = 0;
    /* 只计算 $ 和 * 之间的字符 */
    //line第一个字节为$,舍去，str指向最后校验值前的一个字符
    for (const char *p = log + 1; p < star; p++)
    {
        calculated ^= (uint8_t)*p;
    }
    //high为高位，左移后 或 操作相当于两者相加
    uint8_t received = (uint8_t)((high << 4) | low);

    return (calculated == received) ? 1 : 0;
}
#endif
/*
bool log_32CRC_check(const char *data , uint8_t len)
{
    static bool IS_Find_Star = 0;
    static bool IS_Find_End  = 0;
    static char * star;
    static char * end ;
    static uint32_t crc;
   //先找到 # 字符开头，strchr()不需要传入数据长度，但是他是以\0为结尾进行遍历搜寻，memchr()则是指定长度进行搜寻
   if( !IS_Find_Star && !IS_Find_End)
   {
        star = memchr(data , '#' , len);//报文不会当前末尾连接着上个报文开头，所以当前len一般跟随当前star指针为1
        if( star != NULL)   //能够找到报文开头 , 接着就是修改标志位，然后继续向下一边遍历一边校验，直到遍历到末尾或者遍历到 * 字符 , 找到 * 字符后完成crc计算 将 crc 清0；
        {
            IS_Find_Star = 1;
            //遍历一边，看看能不能找到 * 符号，能找到则进行CRC校验， 找不到正常遍历获取crc值
            if((end = memchr(data , '*' , len)) != NULL)//能找到
            {
                IS_Find_End = 1;
                 //计算报文开头到 * 字符的长度
                const char * get_len  = ++star;   //开头跳过 # 字符
                uint8_t log_len = 0; 
                while( *end != *get_len)
                {
                    get_len++;
                    log_len++;
                }
                crc = crc32_calculate( (const uint8_t *)star , log_len , crc);
                //将 end 后八位作为整数输出
                end++;
                uint32_t received_crc = char_to_uint32(end , 8);
                if(crc == received_crc)
                {
                    IS_Find_Star = 0;IS_Find_End = 0;star = NULL;end = NULL;crc = 0;
                    return true;
                }
                else
                {
                    IS_Find_Star = 0;IS_Find_End = 0;star = NULL;end = NULL;crc = 0;
                    return false;
                }
            }
            else
            {
                data++;     //跳过#符号
                crc = crc32_calculate( (const uint8_t *)data , len-1 , crc);//
            }
        }
   }
   else//已经找到报文开头，接下来判断报文是不是末尾即可
   {
        if( (end = memchr(data , '*' , len)) != NULL)  //找到末尾，则修改标志位，计算到 * 之前的数据长度，将数据放入校验，取出校验值和 * 后的数据到\r\n的数据对比，根据对比结果返回值
        {
            IS_Find_End = 1;
            //计算报文开头到 * 字符的长度
            const char * get_len  = data;
            uint8_t log_len = 0; 
            while( *end != *get_len)
            {
                get_len++;
                log_len++;
            }
            crc = crc32_calculate( (const uint8_t *)data , log_len , crc);
            //先取出报文自带CRC , 将所有crc值和 * 后的数据 8位 作比较
            end++;
            //CRC 比较
            uint32_t received_crc = char_to_uint32(end , 8);
            if(crc == received_crc)
            {
                IS_Find_Star = 0;IS_Find_End = 0;star = NULL;end = NULL;crc = 0;
                return true;
            }
            else
            {
                IS_Find_Star = 0;IS_Find_End = 0;star = NULL;end = NULL;crc = 0;
                return false;
            }
        }
        else                                    //找不到末尾，处理数据进行生成 CRC 校验码即可
        {
            crc = crc32_calculate( (const uint8_t *)data , len , crc);
        }
   }
}
*/
uint32_t char_to_uint32(char *data , uint8_t len)
{
    uint32_t received_crc = 0;
    const char *p = data + 1;

    for (int i = 0; i < 8; i++)
    {
        uint8_t value;

        if (p[i] >= '0' && p[i] <= '9')
        {
            value = p[i] - '0';
        }
        else if (p[i] >= 'A' && p[i] <= 'F')
        {
            value = p[i] - 'A' + 10;
        }
        else if (p[i] >= 'a' && p[i] <= 'f')
        {
            value = p[i] - 'a' + 10;
        }
        else
        {
            // 出现非十六进制字符，校验字段无效
            break;
        }
        received_crc = (received_crc << 4) | value;
    }
    return received_crc;
}

void uart_printf_mutex_init(void)
{
    uart_printf_mutex = xSemaphoreCreateMutex();

    if(uart_printf_mutex == NULL)
    {
        printf("uart_printf_mutex_init fail!\r\n");
    }
}

/*
 * 初始化 CRC-32 查找表
 * 程序启动时调用一次即可
 */
void crc32_init_table(void)
{
    for (uint32_t i = 0; i < 256; i++)
    {
        uint32_t crc = i;

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1;
            }
        }
        crc32_table[i] = crc;
    }
}
/*
 * 计算 CRC-32
 * data：待校验数据
 * len ：数据长度
 */
uint32_t crc32_calculate(const uint8_t *data, size_t len , uint32_t crc)
{
    for (size_t i = 0; i < len; i++)
    {
        uint8_t index = (uint8_t)((crc ^ data[i]) & 0xFF);

        crc = crc32_table[index] ^ (crc >> 8);
    }
    return crc;
}