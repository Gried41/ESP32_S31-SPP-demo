/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "User_Uart.h"
#include "mcu_to_gnss.h"
#include "mcu_to_im.h"
#include "esp_timer.h"
#include "time.h"
#include "sys/time.h"
#define CPU_MONITOR_MAX_TASKS 64

static bool is_idle_task(const char *name);
static bool get_idle_runtime_delta(const TaskStatus_t *previous,UBaseType_t previous_count,const TaskStatus_t *current,UBaseType_t current_count,uint64_t *idle_delta);
void cpu_monitor_task(void *arg);

#if 0 // SPP/GAP handlers are kept for later, but Bluetooth is disabled for now.
#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define SPP_SHOW_DATA 1
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/

static const char local_device_name[] = CONFIG_EXAMPLE_LOCAL_DEVICE_NAME;
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) 
    {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static void print_speed(void)
{
    float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;
    float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;
    float time_interval = time_new_s - time_old_s;
    float speed = data_num * 8 / time_interval / 1000.0;
    ESP_LOGI(SPP_TAG, "speed(%fs ~ %fs): %f kbit/s" , time_old_s, time_new_s, speed);
    data_num = 0;
    time_old.tv_sec = time_new.tv_sec;
    time_old.tv_usec = time_new.tv_usec;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) 
    {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) 
        {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } 
        else 
        {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        uart_spp_set_connection(param->close.handle, false);
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) 
        {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%"PRIu32" sec_id:%d scn:%d", param->start.handle, param->start.sec_id,param->start.scn);
            esp_bt_gap_set_device_name(local_device_name);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } 
        else 
        {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)    //在蓝牙协议栈相关的处理函数中，打印操作比较耗时，推荐交给其他低优先级任务去完成
         /*                               打印耗时过长，可能阻塞蓝牙协议栈的处理流程，并影响数据传输吞吐量。
         * We only show the data in which the data length is less than 128 here. If you want to print the data and
         * the data rate is high, it is strongly recommended to process them in other lower priority application task
         * rather than in this callback directly. Since the printing takes too much time, it may stuck the Bluetooth
         * stack and also have a effect on the throughput!
         */
        //ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%"PRIu32,param->data_ind.len, param->data_ind.handle);
        if (param->data_ind.len < 128) 
        {
            char buf[128];
            //ESP_LOG_BUFFER_HEX("", param->data_ind.data, param->data_ind.len);
            memcpy(buf , param->data_ind.data , param->data_ind.len);
            buf[param->data_ind.len] = '\0';
            printf("recv data : %s\r\n",buf);
        }
#else
        gettimeofday(&time_new, NULL);
        data_num += param->data_ind.len;
        if (time_new.tv_sec - time_old.tv_sec >= 3) 
        {
            print_speed();
        }
#endif
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        //ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%"PRIu32", rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        if (param->srv_open.status == ESP_SPP_SUCCESS) {
            uart_spp_set_connection(param->srv_open.handle, true);
        }
        gettimeofday(&time_old, NULL);
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) 
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) 
        {
            ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                     bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        } 
        else 
        {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) 
        {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } 
        else 
        {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %06"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%06"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
                 bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
        break;

    default: 
    {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}
#endif
/*
 * CPU 负载测试任务
 *
 * 每个周期：
 *   忙循环约 100 ms
 *   休眠约 900 ms
 *
 * 绑定到 Core 1，避免不必要地占用另一个核心。
 */
static TaskHandle_t cpu_load_test_handle = NULL;
static void cpu_load_test_task(void *arg)
{
    const int64_t busy_time_us = 100000;  // 100 ms

    while (1)
    {
        // 记录忙循环的起始时间
        int64_t start_time = esp_timer_get_time();

        // 持续占用 CPU，直到经过约 100 ms
        while ((esp_timer_get_time() - start_time) < busy_time_us)
        {
            __asm__ __volatile__("nop");
        }

        // 休眠约 900 ms，使整体周期接近 1 秒
        vTaskDelay(pdMS_TO_TICKS(900));
    }
}

/*
 * 启动 CPU 负载测试
 */
esp_err_t cpu_load_test_start(void)
{
    if (cpu_load_test_handle != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(cpu_load_test_task,"cpu_load_test",3072,NULL,2,&cpu_load_test_handle,1);

    if (ret != pdPASS)
    {
        cpu_load_test_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/*
 * 停止 CPU 负载测试
 */
void cpu_load_test_stop(void)
{
    if (cpu_load_test_handle != NULL)
    {
        TaskHandle_t task = cpu_load_test_handle;
        cpu_load_test_handle = NULL;

        vTaskDelete(task);
    }
}




//蓝牙控制器负责底层无线通信，Bluedroid负责更上层的蓝牙协议处理
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

#if 0 // Bluetooth controller, Bluedroid, GAP, and SPP startup are intentionally disabled.
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));                    //释放ESP_BT_MODE_BLE的内存，释放完成后通常不再能使用BLE控制器蓝牙功能

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();            //从蓝牙控制器获取默认的配置，返回一个结构体
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK)                              //配置初始化蓝牙控制器
    {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)             //启用蓝牙控制器，指定运行模式 ESP_BT_MODE_CLASSIC_BT 
    {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
#if (CONFIG_EXAMPLE_SSP_ENABLED == false)
    bluedroid_cfg.ssp_en = false;
#endif
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK)                  //使用指定配置初始化 bluedroid 蓝牙协议栈
    {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK)                                       //启用 bluedroid 蓝牙协议栈，使得上层蓝牙功能可以运行
    {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK)                  //注册 gap 事件回调函数，处理设备可发现性，配对，安全认证等相关事件
    {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK)                        //注册 spp 事件回调函数，用来处理串口仿真协议，比如初始化完成，连接建立，数据接收和断开连接等
    {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_spp_cfg_t bt_spp_cfg = 
    {
        .mode = esp_spp_mode,
        .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
        .tx_buffer_size = 0, /* Only used for ESP_SPP_MODE_VFS mode */
    };
    if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK)                           //初始化增强版 spp 功能
    {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
#endif
/*-----------------------------------User Box-------------------------------------------*/
    uart1_init();
    uart_printf_mutex_init();
    mcu_uart_init();
    uart3_init();
    uart2_init();
    rx_to_gnss_task_init();
    //BaseType_t task_created = xTaskCreate(cpu_monitor_task,"cpu_monitor",4096,NULL,1,NULL);
    //ESP_ERROR_CHECK(cpu_load_test_start());
/*-----------------------------------User Box-------------------------------------------*/
}








/*
 * 判断是不是 FreeRTOS 空闲任务。
 * ESP-IDF 多核下通常命名为 IDLE0、IDLE1 等。
 */
static bool is_idle_task(const char *name)
{
    return (name != NULL && strncmp(name, "IDLE", 4) == 0);
}

/*
 * 计算这 10 秒内所有 IDLE 任务的累计运行时间。
 * 通过任务句柄匹配前后两次快照，避免任务数组顺序发生变化。
 */
static bool get_idle_runtime_delta(const TaskStatus_t *previous,UBaseType_t previous_count,const TaskStatus_t *current,UBaseType_t current_count,uint64_t *idle_delta)
{
    uint64_t idle_sum = 0;
    UBaseType_t matched_idle_tasks = 0;

    for (UBaseType_t i = 0; i < current_count; i++)
    {
        if (!is_idle_task(current[i].pcTaskName))
        {
            continue;
        }

        for (UBaseType_t j = 0; j < previous_count; j++)
        {
            if (current[i].xHandle == previous[j].xHandle)
            {
                /*
                 * 使用无符号减法，支持运行计数器在两次采样间回绕。
                 */
                configRUN_TIME_COUNTER_TYPE delta =(configRUN_TIME_COUNTER_TYPE)(current[i].ulRunTimeCounter -previous[j].ulRunTimeCounter);

                idle_sum += (uint64_t)delta;
                matched_idle_tasks++;
                break;
            }
        }
    }

    if (matched_idle_tasks == 0)
    {
        return false;
    }

    *idle_delta = idle_sum;
    return true;
}

void cpu_monitor_task(void *arg)
{
    TaskStatus_t *previous =
        pvPortMalloc(sizeof(TaskStatus_t) * CPU_MONITOR_MAX_TASKS);

    TaskStatus_t *current =
        pvPortMalloc(sizeof(TaskStatus_t) * CPU_MONITOR_MAX_TASKS);

    if (previous == NULL || current == NULL)
    {
        printf("CPU monitor: failed to allocate memory\r\n");

        if (previous != NULL)
        {
            vPortFree(previous);
        }

        if (current != NULL)
        {
            vPortFree(current);
        }

        vTaskDelete(NULL);
        return;
    }

    configRUN_TIME_COUNTER_TYPE previous_total_runtime = 0;

    UBaseType_t previous_count = uxTaskGetSystemState(
        previous,
        CPU_MONITOR_MAX_TASKS,
        &previous_total_runtime
    );

    if (previous_count == 0)
    {
        printf("CPU monitor: initial snapshot failed. ""Check task capacity and FreeRTOS configuration.\r\n");
        vPortFree(previous);
        vPortFree(current);
        vTaskDelete(NULL);
        return;
    }

    const TickType_t period = pdMS_TO_TICKS(10000);
    TickType_t last_wake_tick = xTaskGetTickCount();

    while (1)
    {
        /* 固定周期运行，避免处理耗时不断累加到采样周期。 */
        vTaskDelayUntil(&last_wake_tick, period);

        configRUN_TIME_COUNTER_TYPE current_total_runtime = 0;

        UBaseType_t current_count = uxTaskGetSystemState(current,CPU_MONITOR_MAX_TASKS,&current_total_runtime
        );

        if (current_count == 0)
        {
            printf("CPU monitor: snapshot failed; ""task count may exceed buffer capacity (%d)\r\n",CPU_MONITOR_MAX_TASKS);
            continue;
        }

        /*
         * 总运行时间计数器和每个任务的运行时间计数器，
         * 都来自 FreeRTOS 的运行时间统计时钟。
         */
        configRUN_TIME_COUNTER_TYPE elapsed_counter =(configRUN_TIME_COUNTER_TYPE)(current_total_runtime - previous_total_runtime);

        uint64_t elapsed_runtime = (uint64_t)elapsed_counter;

        uint64_t idle_runtime_delta = 0;

        bool idle_valid = get_idle_runtime_delta(previous,previous_count,current,current_count,&idle_runtime_delta);

        /*
         * 多核系统的总 CPU 容量为：
         * 统计时间 × 核心数量。
         *
         * CPU 占用率 = 1 - 空闲时间 / 总 CPU 容量。
         */
        uint64_t total_cpu_capacity =elapsed_runtime * (uint64_t)configNUMBER_OF_CORES;

        if (total_cpu_capacity > 0 && idle_valid)
        {
            /* 计算百分比，保留两位小数。 */
            if (idle_runtime_delta > total_cpu_capacity)
            {
                idle_runtime_delta = total_cpu_capacity;
            }
            uint64_t busy_x100 =((total_cpu_capacity - idle_runtime_delta) * 10000ULL+ total_cpu_capacity / 2ULL)/ total_cpu_capacity;

            uint64_t idle_x100 =(idle_runtime_delta * 10000ULL+ total_cpu_capacity / 2ULL)/ total_cpu_capacity;

            printf(
                "\r\n========== CPU USAGE ==========\r\n"
                "Interval : 10 seconds\r\n"
                "Cores    : %d\r\n"
                "CPU Busy : %llu.%02llu%%\r\n"
                "CPU Idle : %llu.%02llu%%\r\n"
                "===============================\r\n",

                (int)configNUMBER_OF_CORES,

                (unsigned long long)(busy_x100 / 100ULL),
                (unsigned long long)(busy_x100 % 100ULL),

                (unsigned long long)(idle_x100 / 100ULL),
                (unsigned long long)(idle_x100 % 100ULL)
            );
        }
        else
        {
            printf("CPU monitor: invalid runtime statistics sample\r\n");
        }
        /*
         * 保存本次快照，作为下一轮的起点。
         */
        memcpy(previous,current,current_count * sizeof(TaskStatus_t));
        previous_count = current_count;
        previous_total_runtime = current_total_runtime;
    }
}