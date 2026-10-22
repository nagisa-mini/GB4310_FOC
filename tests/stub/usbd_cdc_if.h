/*
 * 主机侧测试用 usbd_cdc_if.h 桩
 *
 * 用途: 编译 tests/usb_cmd_test.c 时覆盖 USB_Device/App/usbd_cdc_if.h,
 *       为 BSP/USB.c 的 USB_TxProcess() 提供可控的 CDC 发送接口,
 *       避免把 STM32 USB 设备库依赖带入主机测试。
 * 注意: 两个函数由 usb_cmd_test.c 实现, 测试可控制"是否就绪/发送结果"。
 */

#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#include <stdint.h>

/* 与固件 USBD_OK 数值一致 (0) */
#define USBD_OK 0u

uint8_t CDC_Transmit_Ready(void);
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

#endif /* __USBD_CDC_IF_H__ */