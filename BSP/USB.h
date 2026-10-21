#ifndef USB_H
#define USB_H

#include <stdbool.h>
#include <stdint.h>
#include "Ringbuffer.h"

#define USB_TX_BUFFER_SIZE 4096
#define USB_RX_BUFFER_SIZE 512

/*==============================================================================
 * 上位机命令解析参数
 *
 * 架构:
 *   USB 中断 (CDC_Receive_FS) 只调用 USB_RxPush() 把整包字节写入
 *   USB_RxBuffer; 主循环空闲时调用 USB_Command_Process() 做行缓存、
 *   命令解析和 PID 参数写入。中断内不做任何解析或 PID 操作。
 *============================================================================*/
#define USB_CMD_LINE_MAX        64u         /* 单行缓存大小 (含结尾 '\0') */
#define USB_CMD_PROCESS_BUDGET  64u         /* 每次空闲调用最多消费的字节数 */
#define USB_CMD_GAIN_MAX        1000000.0f  /* 增益上限 (角度环/角速度环) */
#define USB_CMD_SPEED_MAX       10000.0f    /* Speed_Ref 绝对值上限 (RPM) */
#define USB_CMD_ANGLE_MAX       6283.1853f  /* Angle_Ref 绝对值上限 (1000*2π, rad) */

void USB_Init(void);

/**
 * @brief USB 中断接收入口: 只把整包数据写入 USB_RxBuffer
 * @param data 接收数据
 * @param len  字节数 (<= APP_RX_DATA_SIZE)
 * @note 仅允许在 USB 中断 (CDC_Receive_FS) 中调用; 环形缓冲区满时整包丢弃。
 */
void USB_RxPush(const uint8_t *data, uint16_t len);

/**
 * @brief USB 复位/重新枚举时请求丢弃未完成的半行
 * @note 由 CDC_Init_FS 调用; 具体清理在主循环解析时完成。
 */
void USB_RxAbort(void);

/**
 * @brief 空闲命令解析 (主循环调用, 非阻塞, 单次有字节预算)
 */
void USB_Command_Process(void);

extern RingBuffer_t USB_TxBuffer; // USB 发送环形缓冲区
extern RingBuffer_t USB_RxBuffer; // USB 接收环形缓冲区

/* 调试计数: 成功命令数 / 失败行数 / 丢弃的 USB 包数 */
extern volatile uint32_t USB_Cmd_OkCount;
extern volatile uint32_t USB_Cmd_ErrCount;
extern volatile uint32_t USB_Rx_OverflowCount;

#endif /* USB_H */