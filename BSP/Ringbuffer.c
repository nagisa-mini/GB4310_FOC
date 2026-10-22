#include "Ringbuffer.h"

#if defined(__ARMCC_VERSION) || defined(__GNUC__)
#include "cmsis_compiler.h"     /* __DMB() 内存屏障 */
#else
#define __DMB()
#endif

/*==============================================================================
 * 内部函数
 *============================================================================*/

static inline bool IsPowerOfTwo(uint32_t val)
{
    return (val > 0) && ((val & (val - 1)) == 0);
}

/*==============================================================================
 * 接口函数
 *============================================================================*/

bool RingBuffer_Init(RingBuffer_t *rb, uint8_t *pool, uint32_t size)
{
    if (rb == NULL || pool == NULL || !IsPowerOfTwo(size))
    {
        return false;
    }

    rb->buffer = pool;
    rb->size   = size;
    rb->mask   = size - 1;
    rb->head   = 0;
    rb->tail   = 0;

    return true;
}

uint32_t RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint32_t len)
{
    if (rb == NULL || data == NULL || len == 0)
    {
        return 0;
    }

    uint32_t head = rb->head;
    uint32_t tail = rb->tail;

    /* head/tail 单调递增并自然回绕, 无符号相减即为有效数据量 */
    uint32_t free_space = rb->size - (head - tail);

    /* 空间不足时不写入任何数据 (全有或全无) */
    if (len > free_space)
    {
        return 0;
    }

    /* 分段拷贝: 先写数组尾部, 折返后再从头部续写 */
    uint32_t offset = head & rb->mask;
    uint32_t chunk  = (len < (rb->size - offset)) ? len : (rb->size - offset);

    memcpy(rb->buffer + offset, data, chunk);
    if (len > chunk)
    {
        memcpy(rb->buffer, data + chunk, len - chunk);
    }

    /* 内存屏障: 数据落入 RAM 后再发布 head */
    __DMB();
    rb->head = head + len;

    return len;
}

uint32_t RingBuffer_Read(RingBuffer_t *rb, uint8_t **data, uint32_t len)
{
    if (rb == NULL || len == 0)
    {
        return 0;
    }

    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    uint32_t used = head - tail;    /* 无符号回绕相减, 同 Write */

    if (len > used)
    {
        len = used;
    }
    if (len == 0)
    {
        return 0;
    }

    /* 零拷贝: 返回缓冲池内部地址, 折返时只返回连续的一段 */
    uint32_t offset     = tail & rb->mask;
    uint32_t contiguous = rb->size - offset;    /* tail 到数组尾部的连续长度 */
    if (len > contiguous)
    {
        len = contiguous;
    }

    *data = rb->buffer + offset;

    /* 内存屏障: 数据读完后推进 tail, 把空间还给生产者 */
    __DMB();
    rb->tail = tail + len;

    return len;
}

uint32_t RingBuffer_Peek(const RingBuffer_t *rb, uint8_t *data, uint32_t len)
{
    if (rb == NULL || data == NULL || len == 0)
    {
        return 0;
    }

    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    uint32_t used = head - tail;

    if (len > used)
    {
        len = used;
    }
    if (len == 0)
    {
        return 0;
    }

    uint32_t offset = tail & rb->mask;
    uint32_t chunk  = (len < (rb->size - offset)) ? len : (rb->size - offset);

    memcpy(data, rb->buffer + offset, chunk);
    if (len > chunk)
    {
        memcpy(data + chunk, rb->buffer, len - chunk);
    }

    return len;
}

bool RingBuffer_WriteByte(RingBuffer_t *rb, uint8_t byte)
{
    if (RingBuffer_IsFull(rb))
    {
        return false;
    }

    uint32_t head = rb->head;
    rb->buffer[head & rb->mask] = byte;

    __DMB();
    rb->head = head + 1;
    return true;
}

bool RingBuffer_ReadByte(RingBuffer_t *rb, uint8_t *byte)
{
    if (RingBuffer_IsEmpty(rb))
    {
        return false;
    }

    uint32_t tail = rb->tail;
    *byte = rb->buffer[tail & rb->mask];

    __DMB();
    rb->tail = tail + 1;
    return true;
}

void RingBuffer_Flush(RingBuffer_t *rb)
{
    /* 须在生产者/消费者均停止访问时调用 (如关闭相关中断之后) */
    if (rb != NULL)
    {
        rb->head = 0;
        rb->tail = 0;
    }
}

void RingBuffer_Advance(RingBuffer_t *rb, uint32_t len)
{
    if ((rb == NULL) || (len == 0u))
    {
        return;
    }

    /* 消费量不能超过当前有效数据量 */
    uint32_t used = rb->head - rb->tail;
    if (len > used)
    {
        len = used;
    }
    if (len == 0u)
    {
        return;
    }

    /* 与 Write/Read 一致: 先完成数据访问, 再推进 tail 释放空间 */
    __DMB();
    rb->tail += len;
}

uint32_t RingBuffer_GetUsed(const RingBuffer_t *rb)
{
    return (rb->head - rb->tail);
}

uint32_t RingBuffer_GetFree(const RingBuffer_t *rb)
{
    return (rb->size - (rb->head - rb->tail));
}

bool RingBuffer_IsFull(const RingBuffer_t *rb)
{
    return RingBuffer_GetFree(rb) == 0;
}

bool RingBuffer_IsEmpty(const RingBuffer_t *rb)
{
    return rb->head == rb->tail;
}