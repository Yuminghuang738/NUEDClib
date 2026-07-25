#include "jy61p_port.h"
#include "ti_msp_dl_config.h"
#include "delay.h"
#include <stdbool.h>

#define JY61P_HEADER          0x55
#define JY61P_PACKET_SIZE     11

#define WRITE_HEADER_1        0xFF
#define WRITE_HEADER_2        0xAA

static const uint8_t unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
static const uint8_t save_cmd[]   = {0xFF, 0xAA, 0x00, 0x00, 0x00};

static volatile JY61P_Angle jy61p_angle;
static volatile bool synced = false;

static bool is_valid_type(uint8_t b)
{
    return (b >= 0x50 && b <= 0x5A);
}

static void send_write_cmd(uint8_t addr, uint16_t data)
{
    uint8_t dl = (uint8_t)(data & 0xFF);
    uint8_t dh = (uint8_t)((data >> 8) & 0xFF);

    DL_UART_Main_transmitDataBlocking(JY61P_UART, WRITE_HEADER_1);
    DL_UART_Main_transmitDataBlocking(JY61P_UART, WRITE_HEADER_2);
    DL_UART_Main_transmitDataBlocking(JY61P_UART, addr);
    DL_UART_Main_transmitDataBlocking(JY61P_UART, dl);
    DL_UART_Main_transmitDataBlocking(JY61P_UART, dh);
}

static void send_bytes(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(JY61P_UART, data[i]);
    }
}

int JY61P_Init(void)
{
    synced = false;
    jy61p_angle.roll  = 0.0f;
    jy61p_angle.pitch = 0.0f;
    jy61p_angle.yaw   = 0.0f;
    return 0;
}

void JY61P_SetBaud(uint32_t baud)
{
    uint32_t ibrd, fbrd;
    switch (baud) {
        case 4800:   ibrd = 520; fbrd = 53; break;
        case 9600:   ibrd = 260; fbrd = 27; break;
        case 19200:  ibrd = 130; fbrd = 13; break;
        case 38400:  ibrd =  65; fbrd =  7; break;
        case 57600:  ibrd =  43; fbrd = 26; break;
        case 115200: ibrd =  21; fbrd = 45; break;
        case 230400: ibrd =  10; fbrd = 54; break;
        case 460800: ibrd =   5; fbrd = 27; break;
        case 921600: ibrd =   2; fbrd = 46; break;
        default:     return;
    }
    DL_UART_Main_disable(JY61P_UART);
    DL_UART_Main_setBaudRateDivisor(JY61P_UART, ibrd, fbrd);
    while (!DL_UART_Main_isRXFIFOEmpty(JY61P_UART)) {
        DL_UART_Main_receiveData(JY61P_UART);
    }
    DL_UART_Main_enable(JY61P_UART);
    synced = false;
}

static bool parse_packet(void)
{
    uint8_t buf[JY61P_PACKET_SIZE];

    if (DL_UART_Main_isRXFIFOEmpty(JY61P_UART)) {
        return false;
    }

    if (!synced) {
        uint8_t b = DL_UART_Main_receiveData(JY61P_UART);
        while (b != JY61P_HEADER) {
            if (DL_UART_Main_isRXFIFOEmpty(JY61P_UART)) return false;
            b = DL_UART_Main_receiveData(JY61P_UART);
        }
        if (DL_UART_Main_isRXFIFOEmpty(JY61P_UART)) return false;
        buf[1] = DL_UART_Main_receiveData(JY61P_UART);
        if (!is_valid_type(buf[1])) return false;
        buf[0] = JY61P_HEADER;
        synced = true;
    } else {
        buf[0] = DL_UART_Main_receiveData(JY61P_UART);
        if (buf[0] != JY61P_HEADER) {
            /* false sync: search for next 0x55 header instead of discarding */
            while (buf[0] != JY61P_HEADER) {
                if (DL_UART_Main_isRXFIFOEmpty(JY61P_UART)) {
                    synced = false;
                    return false;
                }
                buf[0] = DL_UART_Main_receiveData(JY61P_UART);
            }
            /* found 0x55, fall through to read type byte below */
        }
        if (DL_UART_Main_isRXFIFOEmpty(JY61P_UART)) return false;
        buf[1] = DL_UART_Main_receiveData(JY61P_UART);
        if (!is_valid_type(buf[1])) {
            synced = false;
            return false;
        }
    }

    for (int i = 2; i < JY61P_PACKET_SIZE; i++) {
        uint32_t timeout = 5000;
        while (DL_UART_Main_isRXFIFOEmpty(JY61P_UART)) {
            if (--timeout == 0) {
                synced = false;
                return false;
            }
        }
        buf[i] = DL_UART_Main_receiveData(JY61P_UART);
    }

    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) sum += buf[i];
    if (sum != buf[10]) {
        synced = false;
        return false;
    }

    if (buf[1] == JY61P_TYPE_ANGLE) {
        int16_t raw_roll  = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
        int16_t raw_pitch = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
        int16_t raw_yaw   = (int16_t)(((uint16_t)buf[7] << 8) | buf[6]);

        jy61p_angle.roll  = (float)raw_roll  / 32768.0f * 180.0f;
        jy61p_angle.pitch = (float)raw_pitch / 32768.0f * 180.0f;
        jy61p_angle.yaw   = (float)raw_yaw   / 32768.0f * 180.0f;
    }

    return true;
}

int JY61P_Read_Angle(JY61P_Angle *angle)
{
    /* 关中断保护软浮点重入 (M0+ 无硬件 FPU, parse_packet 浮点转换
     * 与定时 ISR 中 calculate_speed/motor_PID 浮点运算不可并发)
     * 保存/恢复 PRIMASK 以支持嵌套调用 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (int i = 0; i < 64; i++) {
        if (DL_UART_Main_isRXFIFOEmpty(JY61P_UART)) break;
        parse_packet();
    }

    if (angle) *angle = jy61p_angle;
    if (!primask) __enable_irq();
    return 0;
}

float JY61P_Get_Yaw(void)
{
    JY61P_Read_Angle(NULL);
    return jy61p_angle.yaw;
}

void JY61P_WriteReg(uint8_t addr, uint16_t value)
{
    send_bytes(unlock_cmd, 5);
    delay_ms(200);
    send_write_cmd(addr, value);
    delay_ms(100);
    send_bytes(save_cmd, 5);
    delay_ms(100);
}

void JY61P_ConfigDefaults(void)
{
    JY61P_WriteReg(JY61P_REG_RSW, 0x000E);
}

int JY61P_DetectBaud(const uint32_t *baud_list, int count)
{
    for (int i = 0; i < count; i++) {
        JY61P_SetBaud(baud_list[i]);
        delay_ms(300);

        int valid_packets = 0;
        for (int retry = 0; retry < 50; retry++) {
            if (!DL_UART_Main_isRXFIFOEmpty(JY61P_UART)) {
                if (parse_packet()) valid_packets++;
            }
            delay_ms(10);
        }

        if (valid_packets >= 3) {
            return (int)baud_list[i];
        }
    }
    return 0;
}
