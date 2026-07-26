#ifndef _JY61P_PORT_H_
#define _JY61P_PORT_H_

#include <stdint.h>

/* JY61P UART1: PA8=TX, PA9=RX */
#define JY61P_UART             UART1

typedef struct {
    float roll;
    float pitch;
    float yaw;
} JY61P_Angle;

#define JY61P_TYPE_TIME       0x50
#define JY61P_TYPE_ACCEL      0x51
#define JY61P_TYPE_GYRO       0x52
#define JY61P_TYPE_ANGLE      0x53
#define JY61P_TYPE_MAG        0x54
#define JY61P_TYPE_PORT       0x55
#define JY61P_TYPE_PRESS      0x56
#define JY61P_TYPE_GPS        0x57
#define JY61P_TYPE_GPSVEL     0x58
#define JY61P_TYPE_QUAT       0x59
#define JY61P_TYPE_GPSPREC    0x5A

#define JY61P_REG_SAVE        0x00
#define JY61P_REG_CALSW       0x01
#define JY61P_REG_RSW         0x02
#define JY61P_REG_RRATE       0x03
#define JY61P_REG_BAUD        0x04
#define JY61P_REG_READADDR    0x27
#define JY61P_REG_KEY         0x69

int  JY61P_Init(void);
void JY61P_SetBaud(uint32_t baud);
int  JY61P_DetectBaud(const uint32_t *baud_list, int count);
int  JY61P_Read_Angle(JY61P_Angle *angle);
float JY61P_Get_Yaw(void);
float JY61P_Get_Yaw_Cached(void);
void JY61P_WriteReg(uint8_t addr, uint16_t value);
void JY61P_ConfigDefaults(void);

#endif
