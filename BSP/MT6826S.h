#ifndef MT6826S_H
#define MT6826S_H

#include "spi.h"
#include "gpio.h"
#include "crc.h"
#include "System.h"
#include "FOC_SVPWM.h"

/*==============================================================================
 * MT6826S 磁编码器 (SPI + DMA + 硬件 CRC)
 *
 *   TIM1 CC4 (载波峰值) 触发: 发送读角度命令 → DMA 接收 4 字节帧 → CRC 校验
 *   → 更新机械角 (0~360°) 与电角度 (0~2π)。另提供阻塞式寄存器读写,
 *   仅用于零点校准等非实时流程。
 *============================================================================*/

/*==============================================================================
 * 电机与角度参数
 *============================================================================*/
#define MOTOR_POLE_PAIRS         14.0f                   /* 电机极对数 */
#define DEG_TO_RAD               0.017453292519943295f   /* π/180: 度转弧度 */
#define MT_2PI                   6.28318530717958647692f /* 2π */
#define MT_INV_2PI               0.15915494309189533577f /* 1/(2π): 角度归约 */

/*==============================================================================
 * 通信命令字节
 *============================================================================*/
#define MT6826S_CMD_READ_REG     0x30U                   /* 0011: 单字节读寄存器 */
#define MT6826S_CMD_WRITE_REG    0x60U                   /* 0110: 单字节写寄存器 */
#define MT6826S_CMD_READ_ANGLE   0xA0U                   /* 1010: 连续读角度 */
#define MT6826S_CMD_BURN_EEPROM  0xC0U                   /* 1100: 烧录 EEPROM */
#define MT6826S_EEPROM_ACK       0x55U                   /* 烧录命令接收成功标志 */

/*==============================================================================
 * 寄存器地址
 *   角度:  0x003 ANGLE[14:7] / 0x004 ANGLE[6:0], 只读 (低字节左对齐于 bit7:1)
 *   状态:  0x005 STATUS[2:0] 只读 / 0x112 EE_DONE[5] 烧录状态
 *   零点:  0x009 ZERO_POS[11:4] / 0x00A ZERO_POS[3:0] + Z_PUL_WID[3:0], EEPROM
 *============================================================================*/
#define MT6826S_REG_ANGLE_H      0x003U
#define MT6826S_REG_ANGLE_L      0x004U
#define MT6826S_REG_STATUS       0x005U
#define MT6826S_REG_ZERO_POS_H   0x009U
#define MT6826S_REG_ZERO_POS_L   0x00AU
#define MT6826S_REG_EE_STATUS    0x112U

#define MT6826S_BIT_EE_DONE      (1U << 5)               /* EE_STATUS 的 EE_DONE 位 */
#define MT6826S_ZERO_POS_HI_MASK 0xF0U                   /* 0x00A[7:4] = ZERO_POS[3:0] */
#define MT6826S_ZERO_POS_LO_MASK 0x0FU                   /* 0x00A[3:0] = Z_PUL_WID[3:0] */

/*==============================================================================
 * 超时与错误上限
 *============================================================================*/
#define MT6826S_SPI_TIMEOUT      10U                     /* SPI 通信超时 (ms) */
#define MT6826S_EEPROM_TIMEOUT   500U                    /* EEPROM 烧录等待超时 (ms) */
#define MT6826S_CRC_ERROR_MAX    10U                     /* CRC 连续错误上限 (暂未使用) */

/* MT6826S_GetData() 通道选择位 */
#define MT6826S_CMD_ANGLE        0x01
#define MT6826S_CMD_ELANGLE      0x02

/*==============================================================================
 * 类型定义
 *============================================================================*/
/* 角度状态: Angle 为机械角 (0~360°), ElAngle 为电角度 (0~2π rad) */
typedef struct {
    uint8_t MODE;       /* 运行模式: 0=空闲, 1=运行, 2=校准 */
    float   Angle;      /* 机械角 */
    float   ElAngle;    /* 电角度 */
} MT6826S_State_t;

/*==============================================================================
 * 接口函数
 *============================================================================*/
void  MT6826S_Init(void);                 /* 初始化角度读取触发源 (TIM1 CC4) */
void  MT6826S_Start(void);                /* 开始连续读取角度 */
void  MT6826S_Stop(void);                 /* 停止连续读取角度 */
float MT6826S_GetData(uint8_t cmd);       /* 读取角度, cmd 见 MT6826S_CMD_xxx */
void  MT6826S_Continue_ReadAngle(void);   /* 发起一次角度读取 (TIM1 CC4 中断调用) */
void  ElAngle_Calibration(void);          /* 电角度零点校准 (阻塞, 含 EEPROM 烧录) */

#endif /* MT6826S_H */