#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 环形缓冲区 (单生产者 / 单消费者)
 *
 *   head 仅由生产者修改, tail 仅由消费者修改; 多对一/一对多场景需调用方加锁。
 *   容量必须是 2 的幂, 下标用掩码取模; head/tail 单调递增, 无符号相减即为
 *   有效数据长度, 读取过程中无需判断回绕。
 *============================================================================*/

/* 控制结构 (由 RingBuffer_Init 填写) */
typedef struct {
    uint8_t *buffer;            /* 缓冲池 */
    uint32_t size;              /* 容量 (2 的幂) */
    uint32_t mask;              /* 掩码: size - 1 */
    volatile uint32_t head;     /* 写指针 (仅生产者修改) */
    volatile uint32_t tail;     /* 读指针 (仅消费者修改) */
} RingBuffer_t;

/**
 * @brief  初始化缓冲区; size 必须是 2 的幂, 否则返回 false
 */
bool RingBuffer_Init(RingBuffer_t *rb, uint8_t *pool, uint32_t size);

/**
 * @brief  批量写入 (全有或全无: 空间不足时不写入任何数据)
 * @retval 实际写入字节数; 空间不足返回 0
 */
uint32_t RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint32_t len);

/**
 * @brief  批量读取并移出 (零拷贝: 返回缓冲池内部地址)
 * @param  data [输出] 连续可读数据起始地址, 下次写入前须消费完
 * @retval 实际消费字节数; 数据折返时可能小于请求长度
 * @note   只返回连续的一段, 不能假设返回值等于 len; 需要"处理成功后再出队"
 *         时改用 RingBuffer_Peek() + RingBuffer_Advance()。
 */
uint32_t RingBuffer_Read(RingBuffer_t *rb, uint8_t **data, uint32_t len);

/**
 * @brief  查看数据 (拷贝到 data, 不移动读指针); 可跨回绕返回完整长度
 * @retval 实际查看字节数
 */
uint32_t RingBuffer_Peek(const RingBuffer_t *rb, uint8_t *data, uint32_t len);

/**
 * @brief  精确消费 len 字节 (只推进读指针, 不拷贝); 超出有效数据时按实际长度处理
 * @note   与 Peek 配对: 发送成功后再用本函数消费同样长度, 避免 Read 在折返处
 *         只能消费一部分而造成重复发送。
 */
void RingBuffer_Advance(RingBuffer_t *rb, uint32_t len);

bool     RingBuffer_WriteByte(RingBuffer_t *rb, uint8_t byte); /* 写入单字节, 满则 false */
bool     RingBuffer_ReadByte(RingBuffer_t *rb, uint8_t *byte); /* 读取单字节, 空则 false */
uint32_t RingBuffer_GetUsed(const RingBuffer_t *rb);           /* 有效数据长度 */
uint32_t RingBuffer_GetFree(const RingBuffer_t *rb);           /* 剩余空间 */
bool     RingBuffer_IsFull(const RingBuffer_t *rb);
bool     RingBuffer_IsEmpty(const RingBuffer_t *rb);

/**
 * @brief  清空缓冲区 (须在生产者/消费者均停止访问时调用)
 */
void RingBuffer_Flush(RingBuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* RINGBUFFER_H */