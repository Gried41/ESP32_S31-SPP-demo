#ifndef _MCU_TO_GNSS_H_
#define _MCU_TO_GNSS_H_

#include <stdint.h>

typedef enum
{
    MSG_UNKNOW,
    MSG_GGA,
    MSG_RMC,
    MSG_ZDA,
    MSG_OBSVMCMPA
    
}GNSS_MSG_TYPE;
typedef struct
{
    int hour;
    int minute;
    int second;
    int millisecond;
}UTC_Time;
typedef struct
{
    int     degree;          // 纬度的度，例如 23
    double  minute;           // 纬度的分，例如 9.44454175
    char    direction;          // 纬度方向：'N' 或 'S'
    double  decimal_degree;   // 转换后的十进制度数
} Latitude_Info;
typedef struct
{
    int     degree;         
    double  minute;          
    char    direction;          
    double  decimal_degree;   
} Longitude_Info;
typedef struct
{
    double  Altitude;
    char    Altitude_char;
    double  ellipsoid_height;
    char    ellipsoid_height_char;
}GGA_Height;
typedef struct
{
    int     day;
    int     month;
    int     year;
}Date;
typedef struct 
{
    double  angle;
    char    direction;
    char    mode;
    char    status;
}Magnetic_Yaw;


typedef struct 
{
    UTC_Time        time;
    Latitude_Info   latitude;
    Longitude_Info  longitude;
    int             position_quality;
    int             satellite_count;
    double          Hdop;
    GGA_Height      Altitude;
}GGA_Info;


typedef struct
{
    UTC_Time        time;
    char            status;
    Latitude_Info   latitude;
    Longitude_Info  longitude;
    Date            date;
    double          ground_speed;
    double          ground_angle;
    Magnetic_Yaw    yaw;
}RMC_Info;

typedef struct
{
    double          cpu_idle_rate;
    char            time_system[4];
    char            time_status[5];
    int             time_weeks;
    int             weeks_ms;
}CMPA_Data_Head;
typedef struct
{
    CMPA_Data_Head  head_data;
    uint64_t        tracking_raw;
    uint64_t        doppler_raw; 
    uint64_t        psr_raw;         
    uint64_t        adr_raw;       
    uint64_t        psr_std_index;  
    uint64_t        adr_std_index;  
    uint64_t        prn;            
    uint64_t        lock_time_raw;  
    uint64_t        cne_raw;        
}CMPA_Info;


#define UM980_LOG_CHECK_IS      0

void mcu_uart_init(void);
void rx_to_gnss_task(void *arg);
void rx_to_gnss_task_init(void);
void Get_cmd_go(uint8_t *data , uint8_t len);
void uart2_init(void);
void uart3_init(void);
void gnss_uart2_to_mcu_task(void *arg);
void gnss_uart3_to_mcu_task(void *arg);

#if UM980_LOG_CHECK_IS
static void process_com3_nmea_line(char *line,uint32_t *checked_count,uint32_t *pass_count,uint32_t *fail_count,uint32_t *invalid_count);
static void process_com2_nmea_line(char *line,uint32_t *checked_count,uint32_t *pass_count,uint32_t *fail_count,uint32_t *invalid_count);
static int log_check(const char *log);
static int hex_to_value(char c);
void check_log_format(const char *recv_data , int uart_num);
#endif
uint32_t char_to_uint32(char *data , uint8_t len);
void uart_printf_mutex_init(void);
//bool log_32CRC_check(const char *data , uint8_t len);
void crc32_init_table(void);
uint32_t crc32_calculate(const uint8_t *data, size_t len , uint32_t crc);

void Get_Log_Type(uint8_t *data , size_t len);
GNSS_MSG_TYPE Log_Type(uint8_t *data , size_t len);

void GGA_MSG_Parse(uint8_t *data , size_t len , GGA_Info * GGA);
void RMC_MSG_Parse(uint8_t *data , size_t len , RMC_Info * RMC);
void OBSVMCMPA_MSG_Parse(uint8_t *data , CMPA_Info * CMPA);

static int HEX_To_Byte(char c);
static bool Decode_Hex_Record(const char *hex , size_t hex_len , uint8_t out[24]);
static uint64_t Get_Bits_LSB(const uint8_t *data , unsigned start_bit , unsigned width);


static void Single_Data_Parse(uint8_t *star , uint8_t *end , CMPA_Info *CMPA);

void UTC_Time_Parse(uint8_t *star , uint8_t *end , UTC_Time *time);
void Latitude_Parse(uint8_t *star , uint8_t *end , Latitude_Info *latitude);
void Longitude_Parse(uint8_t *star , uint8_t *end , Longitude_Info *longitude);
void GGA_Base_Parse(uint8_t *star , uint8_t *end , GGA_Info *GGA);
void Altitude_Parse(uint8_t *star , uint8_t *end , GGA_Info *GGA);
void ddmmyy_Data_Parse(uint8_t *star , uint8_t *end , Date *date);
void Magnetic_Yaw_Parse(uint8_t *star , uint8_t *end , Magnetic_Yaw *yaw);
#endif
