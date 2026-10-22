#ifndef USB_H
#define USB_H

#include <stdbool.h>
#include <stdint.h>
#include "usbd_cdc_if.h"
#include "Ringbuffer.h"

/*==============================================================================
 * USB CDC 命令通道
 *
 *   接收: CDC_Receive_FS (USB 中断) 只把整包数据压入 USB_RxBuffer;
 *   解析: 主循环空闲时调用 USB_Command_Process(), 做行缓存、命令解析与
 *         PID 参数写入。中断内不做解析或 PID 操作。
 *   发送: 遥测等数据写入 USB_TxBuffer, 由 USB_TxProcess() 在空闲时出队。
 *============================================================================*/

#define USB_TX_BUFFER_SIZE      4096        /* 发送环形缓冲区 (2 的幂) */
#define USB_RX_BUFFER_SIZE      512         /* 接收环形缓冲区 (2 的幂) */

/*==============================================================================
 * 上位机命令解析参数
 *============================================================================*/
#define USB_CMD_LINE_MAX        64u         /* 单行缓存大小 (含结尾 '\0') */
#define USB_CMD_PROCESS_BUDGET  64u         /* 每次空闲调用最多消费的字节数 */
#define USB_CMD_GAIN_MAX        1000000.0f  /* 增益上限 (三个环) */
#define USB_CMD_SPEED_MAX       10000.0f    /* Speed_Ref 绝对值上限 (RPM) */
#define USB_CMD_ANGLE_MAX       6283.1853f  /* Angle_Ref 绝对值上限 (1000×2π, rad) */
#define USB_CMD_CURR_MAX        1.9f        /* Curr/Id 绝对值上限 (A), 与相电流量程一致 */

/*==============================================================================
 * 接口函数
 *============================================================================*/
void USB_Init(void);

/**
 * @brief  USB 中断接收入口: 把整包数据写入 USB_RxBuffer
 * @param  data 接收数据
 * @param  len  字节数 (<= APP_RX_DATA_SIZE)
 * @note   仅允许在 USB 中断 (CDC_Receive_FS) 中调用; 缓冲区满时整包丢弃。
 */
void USB_RxPush(const uint8_t *data, uint16_t len);

/**
 * @brief  USB 复位/重新枚举时请求丢弃未完成的半行
 * @note   由 CDC_Init_FS 调用; 实际清理在 USB_Command_Process() 中完成。
 */
void USB_RxAbort(void);

/**
 * @brief  空闲命令解析 (主循环调用, 非阻塞, 单次有字节预算)
 */
void USB_Command_Process(void);

/**
 * @brief  发送任务 (主循环调用, 非阻塞): 上一包发完后取一帧交给 CDC
 * @note   先确认 CDC 空闲再 Peek/发送, 发送成功后才 Advance 精确消费;
 *         避免 BUSY 时数据已出队, 以及 Read 在折返处少消费导致重复发送。
 */
void USB_TxProcess(void);

extern RingBuffer_t USB_TxBuffer;   /* USB 发送环形缓冲区 */
extern RingBuffer_t USB_RxBuffer;   /* USB 接收环形缓冲区 */

/* 调试计数: 成功命令数 / 失败行数 / 丢弃的 USB 包数 */
extern volatile uint32_t USB_Cmd_OkCount;
extern volatile uint32_t USB_Cmd_ErrCount;
extern volatile uint32_t USB_Rx_OverflowCount;

#endif /* USB_H */