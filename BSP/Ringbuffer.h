#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 环形缓冲区控制结构体
 * @note 适用于单生产者(写方)-单消费者(读方)模型：head 仅由写方修改，
 *       tail 仅由读方修改。多对一/一对多场景需调用方自行加锁。
 */
typedef struct {
    uint8_t *buffer;           // 底层缓冲数组指针
    uint32_t size;             // 缓冲区容量 (必须为 2 的整数次幂)
    uint32_t mask;             // 掩码，固定为 (size - 1)
    volatile uint32_t head;    // 写入指针 (单调递增，自动溢出)
    volatile uint32_t tail;    // 读取指针 (单调递增，自动溢出)
} RingBuffer_t;

/**
 * @brief 初始化环形缓冲区
 * @param rb 缓冲区结构体句柄
 * @param pool 用户静态分配的内存数组
 * @param size 数组长度 (必须是 2 的幂次方，如 64, 128, 512, 1024)
 * @return true 初始化成功; false 尺寸不合法
 */
bool RingBuffer_Init(RingBuffer_t *rb, uint8_t *pool, uint32_t size);

/**
 * @brief 批量写入数据 (全有或全无：空间不足时不写入任何数据)
 * @return 写入的字节数；剩余空间不足时返回 0
 */
uint32_t RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint32_t len);

/**
 * @brief 批量读取数据并移出缓冲区 (零拷贝：不复制数据，直接返回缓冲池内部地址)
 * @param data [输出] 数据起始地址 (指向缓冲池内部，调用方须在下次写入前消费)
 * @return 实际读取并消费的字节数，即 *data 处连续可用的长度
 *         (数据折返时可能小于请求长度)
 * @note 返回值才是实际消费长度，调用方必须按它处理，不能假设等于 len。
 *       需要"发送/处理成功后再出队"时 (如 USB、DMA)，请改用
 *       RingBuffer_Peek() + RingBuffer_Advance()，否则中途失败会丢数据。
 */
uint32_t RingBuffer_Read(RingBuffer_t *rb, uint8_t **data, uint32_t len);

/**
 * @brief 查看缓冲区数据 (不移动读指针)
 * @return 实际查看的字节数
 */
uint32_t RingBuffer_Peek(const RingBuffer_t *rb, uint8_t *data, uint32_t len);

/**
 * @brief 精确消费(丢弃)指定长度的数据, 只移动读指针, 不复制数据
 * @param rb  缓冲区句柄
 * @param len 要消费的字节数; 超过当前有效数据时按有效数据量处理
 * @note 与 RingBuffer_Peek 配对使用: Peek 查看并发送成功后, 用本函数
 *       精确消费同样长度。不要改用 RingBuffer_Read 消费, 因为数据折返
 *       时 Read 只返回连续的一段, 消费长度会小于 Peek 返回的长度,
 *       导致折返处的数据被重复发送。
 */
void RingBuffer_Advance(RingBuffer_t *rb, uint32_t len);

/**
 * @brief 写入单字节
 */
bool RingBuffer_WriteByte(RingBuffer_t *rb, uint8_t byte);

/**
 * @brief 读取单字节
 */
bool RingBuffer_ReadByte(RingBuffer_t *rb, uint8_t *byte);

/**
 * @brief 获取当前已存放的有效数据长度
 */
uint32_t RingBuffer_GetUsed(const RingBuffer_t *rb);

/**
 * @brief 获取当前剩余可用空间
 */
uint32_t RingBuffer_GetFree(const RingBuffer_t *rb);

/**
 * @brief 判断缓冲区是否已满
 */
bool RingBuffer_IsFull(const RingBuffer_t *rb);
/**
 * @brief 判断缓冲区是否为空
 */
bool RingBuffer_IsEmpty(const RingBuffer_t *rb);

/**
 * @brief 清空缓冲区
 * @note 须在生产者与消费者均停止访问时调用 (如关闭相关中断之后)
 */
void RingBuffer_Flush(RingBuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* __RINGBUFFER_H */