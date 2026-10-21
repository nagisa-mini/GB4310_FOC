/*
 * RingBuffer 主机侧回归测试 (不参与固件构建)
 *
 * 运行方式 (Windows / MinGW):
 *   cd 软件代码\GB4310_FOC
 *   gcc -std=c11 -O2 -Wall -Wextra tests\ringbuffer_test.c BSP\Ringbuffer.c ^
 *       -I BSP -I tests\stub -o build\ringbuffer_test.exe
 *   .\build\ringbuffer_test.exe
 *
 * 重点验证:
 *   1. 环形缓冲区回绕时 RingBuffer_Read 只返回连续段 (这是 API 设计);
 *   2. RingBuffer_Peek 跨回绕返回完整数据;
 *   3. 新增的 RingBuffer_Advance 能精确消费 Peek 返回的长度, 不重复不丢失;
 *   4. 写满时 Write 全有或全无。
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Ringbuffer.h"

static unsigned g_checks = 0u;
static unsigned g_fails  = 0u;

#define CHECK(cond)                                                           \
    do {                                                                      \
        g_checks++;                                                           \
        if (!(cond)) {                                                        \
            g_fails++;                                                        \
            printf("[FAIL] %s:%d %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                     \
    } while (0)

/* 把 rb 推到"tail 位于缓冲区尾部附近、下一次 Peek 跨回绕"的状态 */
static void FillWrapState(RingBuffer_t *rb, uint8_t base)
{
    uint8_t data[64];

    CHECK(RingBuffer_Init(rb, NULL, 0u) == false);   /* 参数非法 */

    /* 用同一个 pool 重新初始化 */
    static uint8_t pool[64];
    CHECK(RingBuffer_Init(rb, pool, sizeof(pool)) == true);
    CHECK(RingBuffer_IsEmpty(rb));

    for (uint32_t i = 0; i < 60u; i++) {
        data[i] = (uint8_t)(base + i);
    }
    CHECK(RingBuffer_Write(rb, data, 60u) == 60u);
    CHECK(RingBuffer_GetUsed(rb) == 60u);

    /* tail=60，head 仍为 60；下次写入将折返 */
    uint8_t *p = NULL;
    CHECK(RingBuffer_Read(rb, &p, 60u) == 60u);
    CHECK(RingBuffer_GetUsed(rb) == 0u);

    CHECK(RingBuffer_Write(rb, data, 60u) == 60u);
    CHECK(RingBuffer_GetUsed(rb) == 60u);
}

int main(void)
{
    RingBuffer_t rb;
    uint8_t src[64];
    uint8_t peek[64];
    uint8_t *p = NULL;

    printf("== RingBuffer 测试 ==\n");

    /* 1. 基础行为与"写满全有或全无" */
    static uint8_t pool[64];
    CHECK(RingBuffer_Init(&rb, pool, sizeof(pool)) == true);
    CHECK(RingBuffer_GetUsed(&rb) == 0u);
    CHECK(RingBuffer_GetFree(&rb) == sizeof(pool));
    CHECK(RingBuffer_IsEmpty(&rb));

    for (uint32_t i = 0; i < 60u; i++) {
        src[i] = (uint8_t)i;
    }
    CHECK(RingBuffer_Write(&rb, src, 60u) == 60u);
    CHECK(RingBuffer_GetUsed(&rb) == 60u);
    CHECK(RingBuffer_Write(&rb, src, 5u) == 0u);       /* 空间不足, 整批拒绝 */
    CHECK(RingBuffer_GetUsed(&rb) == 60u);

    /* 2. tail 在末尾时 Read 只返回连续段 (4 字节) */
    CHECK(RingBuffer_Read(&rb, &p, 60u) == 60u);
    CHECK(RingBuffer_GetUsed(&rb) == 0u);
    for (uint32_t i = 0; i < 60u; i++) {
        src[i] = (uint8_t)(0xA0u + i);
    }
    CHECK(RingBuffer_Write(&rb, src, 60u) == 60u);
    CHECK(RingBuffer_Peek(&rb, peek, 32u) == 32u);
    for (uint32_t i = 0u; i < 32u; i++) {
        CHECK(peek[i] == (uint8_t)(0xA0u + i));        /* Peek 跨回绕, 内容完整 */
    }
    /* 此时 Read(32) 只能消费连续段 4 字节 —— 这就是不能拿 Read 消费 Peek 长度的原因 */
    CHECK(RingBuffer_Read(&rb, &p, 32u) == 4u);
    CHECK(RingBuffer_GetUsed(&rb) == 56u);

    /* 3. Peek + Advance: 精确消费 32 字节, 后续数据不重复 */
    FillWrapState(&rb, 0xA0u);
    CHECK(RingBuffer_Peek(&rb, peek, 32u) == 32u);
    RingBuffer_Advance(&rb, 32u);
    CHECK(RingBuffer_GetUsed(&rb) == 28u);
    CHECK(RingBuffer_Peek(&rb, peek, 8u) == 8u);
    for (uint32_t i = 0u; i < 8u; i++) {
        CHECK(peek[i] == (uint8_t)(0xA0u + 32u + i));  /* 应接着第 33 字节, 不是从头发 */
    }
    RingBuffer_Advance(&rb, 1000u);                    /* 超过有效数据: 全部消费 */
    CHECK(RingBuffer_GetUsed(&rb) == 0u);
    CHECK(RingBuffer_IsEmpty(&rb));

    /* 4. 非法参数 */
    RingBuffer_Advance(NULL, 10u);                     /* 不应崩溃 */
    RingBuffer_Advance(&rb, 0u);
    CHECK(RingBuffer_GetUsed(&rb) == 0u);

    printf("\n检查项: %u, 失败: %u\n", g_checks, g_fails);
    if (g_fails != 0u) {
        printf("测试失败\n");
        return 1;
    }
    printf("全部通过\n");
    return 0;
}
