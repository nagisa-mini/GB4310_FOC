#ifndef FMT_H
#define FMT_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 轻量 ASCII 浮点格式化模块 —— 替代 snprintf 的浮点打印
 *
 * 特点:
 *   - 纯整数运算 + IEEE-754 位操作, 不调用 snprintf/printf,
 *     不使用 double, 不使用 malloc;
 *   - 无全局状态, 可重入, 可在中断上下文中调用;
 *   - 舍入遵循 C 标准 printf("%.Nf") 的"正确舍入 + 四舍六入五取偶";
 *   - 输出为 8 位数组(ASCII 字符), 可直接交给 RingBuffer_Write / DMA。
 *
 * 典型用法 (TIM7 1kHz 遥测):
 *   const float telemetry[3] = {500.0f, FOC_State.Speed, PID_Speed_Output};
 *   uint8_t line[48];
 *   size_t n = Fmt_LineU8(line, sizeof(line), telemetry, 3u, 2u);
 *   if (n != 0u) {
 *       (void)RingBuffer_Write(&USB_TxBuffer, line, (uint32_t)n);
 *   }
 *   // line 内容: "500.00,1234.50,-0.12\r\n"
 *
 * 注意:
 *   - 推荐"先在本模块拼好整行, 再一次性 RingBuffer_Write";
 *     逐字节调用 RingBuffer_WriteByte 每字节都有 IsFull/DMB 开销,
 *     会明显变慢。
 *============================================================================*/

#define FMT_DECIMALS_MAX 6u   /* 支持的小数位数上限 (超出自动钳位) */

/**
 * @brief 单个浮点数 → ASCII 字符串, 例如 "123.45" (不含逗号/换行)
 * @param buf       输出缓冲区 (8 位数组)
 * @param cap       buf 容量 (字节, 含结尾 '\0')
 * @param value     待格式化的浮点数
 * @param decimals  小数位数 0 ~ FMT_DECIMALS_MAX
 * @return 实际写入的字符数 (不含 '\0'); 参数非法返回 0
 * @note 结果始终以 '\0' 结尾 (cap > 0 时), 空间不足时安全截断。
 */
size_t Fmt_FloatU8(uint8_t *buf, size_t cap, float value, uint8_t decimals);

/**
 * @brief 浮点数组 → ASCII 行 "v0,v1,...,vN\r\n"
 * @param buf       输出缓冲区
 * @param cap       容量 (含结尾 '\0')
 * @param values    浮点数组 (不可为 NULL)
 * @param count     元素个数 (可为 0, 此时输出 "\r\n")
 * @param decimals  小数位数 0 ~ FMT_DECIMALS_MAX
 * @return 实际写入长度 (不含 '\0'); 参数非法返回 0
 */
size_t Fmt_LineU8(uint8_t *buf, size_t cap, const float *values,
                  uint8_t count, uint8_t decimals);

#ifdef __cplusplus
}
#endif

#endif /* FMT_H */
