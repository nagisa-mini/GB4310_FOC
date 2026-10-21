/*
 * Fmt 模块主机侧测试 (不参与固件构建, 只验证算法正确性)
 *
 * 运行方式 (Windows / MinGW):
 *   cd 软件代码\GB4310_FOC
 *   gcc -std=c11 -O2 -Wall -Wextra tests\fmt_test.c BSP\Fmt.c -I BSP -o build\fmt_test.exe -lm
 *   .\build\fmt_test.exe
 *
 * 测试内容:
 *   1. 单浮点输出与 80 位 long double 精确参考(五取偶)逐字符比较;
 *   2. 随机 float 位模式模糊测试, 覆盖 0~6 位小数;
 *   3. 整行输出 (逗号 + \r\n), 长度、'\0'、字节值;
 *   4. 各种缓冲区长度下的截断/越界检查;
 *   5. NaN/Inf/ovf/负零等边界;
 *   6. 与 snprintf 的主机侧耗时对比 (仅供参考)。
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#include "Fmt.h"

static unsigned g_checks = 0u;
static unsigned g_fails  = 0u;

static void CheckFail(const char *file, int line, const char *fmt, ...)
{
    g_fails++;
    if (g_fails <= 20u) {
        va_list ap;
        fprintf(stderr, "[FAIL] %s:%d: ", file, line);
        va_start(ap, fmt);
        vfprintf(stderr, fmt, ap);
        va_end(ap);
        fputc('\n', stderr);
    }
}

#define CHECK_STR(got, expect)                                                    \
    do {                                                                          \
        g_checks++;                                                               \
        const char *g_ = (got);                                                   \
        const char *e_ = (expect);                                                \
        if (strcmp(g_, e_) != 0) {                                                \
            CheckFail(__FILE__, __LINE__, "got \"%s\" expect \"%s\"", g_, e_);    \
        }                                                                         \
    } while (0)

#define CHECK_INT(got, expect)                                                    \
    do {                                                                          \
        g_checks++;                                                               \
        long long g_ = (long long)(got);                                          \
        long long e_ = (long long)(expect);                                       \
        if (g_ != e_) {                                                           \
            CheckFail(__FILE__, __LINE__, "got %lld expect %lld", g_, e_);        \
        }                                                                         \
    } while (0)

#define CHECK_TRUE(cond)                                                          \
    do {                                                                          \
        g_checks++;                                                               \
        if (!(cond)) {                                                            \
            CheckFail(__FILE__, __LINE__, "condition false: %s", #cond);          \
        }                                                                         \
    } while (0)

/*------------------------------------------------------------------------------
 * 精确参考实现: long double 对 float 无精度损失, 平局五取偶
 *---------------------------------------------------------------------------*/
static void RefFloatHalfEven(char *out, size_t cap, float value, int decimals)
{
    static const unsigned long long s_p10[7] = {
        1ULL, 10ULL, 100ULL, 1000ULL, 10000ULL, 100000ULL, 1000000ULL
    };

    const bool neg = signbit(value) != 0;
    long double scaled = fabsl((long double)value);
    for (int i = 0; i < decimals; i++) {
        scaled *= 10.0L;
    }

    unsigned long long fixed = (unsigned long long)scaled;
    const long double frac = scaled - (long double)fixed;
    if ((frac > 0.5L) || ((frac == 0.5L) && ((fixed & 1ULL) != 0ULL))) {
        fixed++;
    }

    if (decimals == 0) {
        snprintf(out, cap, "%s%llu", neg ? "-" : "", fixed);
    } else {
        const unsigned long long p10 = s_p10[decimals];
        snprintf(out, cap, "%s%llu.%0*llu", neg ? "-" : "",
                 fixed / p10, decimals, fixed % p10);
    }
}

static bool IsRoundingTie(float value, int decimals)
{
    long double scaled = fabsl((long double)value);
    for (int i = 0; i < decimals; i++) {
        scaled *= 10.0L;
    }
    const long double ip = floorl(scaled);
    return (scaled - ip) == 0.5L;
}

/*------------------------------------------------------------------------------
 * 1. 单值 vs 精确参考
 *---------------------------------------------------------------------------*/
static void TestSingle(void)
{
    static const float values[] = {
        0.0f, -0.0f, 1.0f, -1.0f, 0.5f, -0.5f, 1.5f, 2.5f, -2.5f, 3.5f,
        0.05f, -0.05f, 0.005f, 0.0005f, 0.00005f, 0.000005f,
        0.1f, 0.2f, 0.3f, 1.0f / 3.0f, 2.0f / 3.0f,
        0.125f, -0.125f, 0.375f, 2.675f, 9.999f, 99.995f, 99.999f,
        999.999f, 1234.5678f, 65535.999f, 500.0f, 1234.5f,
        1e6f, -1e6f, 1e9f, -1e9f, 1e12f, 16777216.0f, 8388608.5f,
        2147483647.0f, -2147483648.0f, 4294967296.0f,
    };
    static const int decimals_list[] = {0, 1, 2, 3, 4, 5, 6};

    for (size_t d = 0u; d < sizeof(decimals_list) / sizeof(decimals_list[0]); d++) {
        const int dec = decimals_list[d];
        for (size_t i = 0u; i < sizeof(values) / sizeof(values[0]); i++) {
            char got[64];
            char expect[64];
            const size_t n = Fmt_FloatU8((uint8_t *)got, sizeof(got), values[i], (uint8_t)dec);
            RefFloatHalfEven(expect, sizeof(expect), values[i], dec);
            CHECK_STR(got, expect);
            CHECK_INT(n, strlen(got));
        }
    }

    /* 特殊值 */
    {
        char got[64];
        char expect[64];

        Fmt_FloatU8((uint8_t *)got, sizeof(got), INFINITY, 2u);
        snprintf(expect, sizeof(expect), "%.2f", (double)INFINITY);
        CHECK_STR(got, expect);

        Fmt_FloatU8((uint8_t *)got, sizeof(got), -INFINITY, 2u);
        snprintf(expect, sizeof(expect), "%.2f", (double)-INFINITY);
        CHECK_STR(got, expect);

        Fmt_FloatU8((uint8_t *)got, sizeof(got), NAN, 2u);
        CHECK_STR(got, "nan");
        Fmt_FloatU8((uint8_t *)got, sizeof(got), -NAN, 2u);
        CHECK_STR(got, "-nan");

        Fmt_FloatU8((uint8_t *)got, sizeof(got), 1e30f, 2u);
        CHECK_STR(got, "ovf");
        Fmt_FloatU8((uint8_t *)got, sizeof(got), -3.5e20f, 0u);
        CHECK_STR(got, "-ovf");
    }

    /* 小数位超上限钳位 */
    {
        char a[64];
        char b[64];
        Fmt_FloatU8((uint8_t *)a, sizeof(a), 1.23456789f, FMT_DECIMALS_MAX);
        Fmt_FloatU8((uint8_t *)b, sizeof(b), 1.23456789f, 200u);
        CHECK_STR(b, a);
    }
}

/*------------------------------------------------------------------------------
 * 2. 模糊测试
 *---------------------------------------------------------------------------*/
static uint32_t g_rng = 0x12345678u;

static uint32_t Rand32(void)
{
    uint32_t x = g_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_rng = x;
    return x;
}

static float RandFloat(void)
{
    uint32_t bits;
    switch (Rand32() % 3u) {
    case 0u:
        bits = Rand32();
        break;
    case 1u:
        bits = (Rand32() & 0x80000000u)
             | (((110u + (Rand32() % 71u)) & 0xFFu) << 23)
             | (Rand32() & 0x7FFFFFu);
        break;
    default:
        bits = (Rand32() & 0x80000000u)
             | (((1u + (Rand32() % 126u)) & 0xFFu) << 23)
             | (Rand32() & 0x7FFFFFu);
        break;
    }
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void TestFuzzSingle(void)
{
    const int rounds = 40000;

    for (int dec = 0; dec <= (int)FMT_DECIMALS_MAX; dec++) {
        const double limit = 9.0e18 / pow(10.0, dec);

        for (int i = 0; i < rounds; i++) {
            const float value = RandFloat();
            if (!isfinite(value) || (fabs((double)value) >= limit)) {
                continue;
            }

            char got[64];
            char expect[64];
            const size_t n = Fmt_FloatU8((uint8_t *)got, sizeof(got), value, (uint8_t)dec);
            RefFloatHalfEven(expect, sizeof(expect), value, dec);

            if (strcmp(got, expect) != 0) {
                uint32_t bits;
                memcpy(&bits, &value, sizeof(bits));
                CheckFail(__FILE__, __LINE__,
                          "bits=0x%08lX value=%.9g dec=%d got=\"%s\" expect=\"%s\"",
                          (unsigned long)bits, (double)value, dec, got, expect);
            }
            g_checks++;
            CHECK_INT(n, strlen(got));

            /* 非平局时与平台 snprintf 交叉验证 */
            if (!IsRoundingTie(value, dec)) {
                char snp[64];
                snprintf(snp, sizeof(snp), "%.*f", dec, (double)value);
                if (strcmp(got, snp) != 0) {
                    CheckFail(__FILE__, __LINE__,
                              "non-tie mismatch: value=%.9g dec=%d got=\"%s\" snprintf=\"%s\"",
                              (double)value, dec, got, snp);
                }
            }
        }
    }
}

/*------------------------------------------------------------------------------
 * 3. 整行输出
 *---------------------------------------------------------------------------*/
static void TestLine(void)
{
    uint8_t buf[64];
    const float v3[3] = {500.0f, 1234.5f, -0.125f};

    const size_t n = Fmt_LineU8(buf, sizeof(buf), v3, 3u, 2u);
    CHECK_STR((char *)buf, "500.00,1234.50,-0.12\r\n");
    CHECK_INT(n, 22u);
    CHECK_INT(buf[n], 0);
    CHECK_INT(((char *)buf)[6], ',');
    CHECK_INT(((char *)buf)[20], '\r');
    CHECK_INT(((char *)buf)[21], '\n');

    /* count = 0 → 只有 \r\n */
    CHECK_INT(Fmt_LineU8(buf, sizeof(buf), v3, 0u, 2u), 2u);
    CHECK_STR((char *)buf, "\r\n");

    /* 单元素 / 多元素拼接与参考一致 */
    const float values[6] = {1.0f, -2.25f, 0.0f, 1e5f, 0.0001f, -0.0f};
    for (uint8_t count = 1u; count <= 6u; count++) {
        char expect[128];
        size_t pos = 0u;
        for (uint8_t i = 0u; i < count; i++) {
            char one[64];
            RefFloatHalfEven(one, sizeof(one), values[i], 3);
            const size_t len = strlen(one);
            if (i != 0u) {
                expect[pos++] = ',';
            }
            memcpy(&expect[pos], one, len);
            pos += len;
        }
        expect[pos++] = '\r';
        expect[pos++] = '\n';
        expect[pos]   = '\0';

        CHECK_INT(Fmt_LineU8(buf, sizeof(buf), values, count, 3u), pos);
        CHECK_STR((char *)buf, expect);
    }

    /* 参数非法 */
    CHECK_INT(Fmt_LineU8(NULL, 32u, v3, 3u, 2u), 0u);
    CHECK_INT(Fmt_LineU8(buf, 0u, v3, 3u, 2u), 0u);
    CHECK_INT(Fmt_LineU8(buf, sizeof(buf), NULL, 3u, 2u), 0u);
    CHECK_INT(Fmt_FloatU8(NULL, 8u, 1.0f, 2u), 0u);
    CHECK_INT(Fmt_FloatU8(buf, 0u, 1.0f, 2u), 0u);

    /* 小数位超上限钳位 */
    {
        uint8_t a[64];
        uint8_t b[64];
        Fmt_LineU8(a, sizeof(a), v3, 3u, FMT_DECIMALS_MAX);
        Fmt_LineU8(b, sizeof(b), v3, 3u, 255u);
        CHECK_STR((char *)b, (char *)a);
    }
}

/*------------------------------------------------------------------------------
 * 4. 截断与越界检查
 *---------------------------------------------------------------------------*/
static void TestBounds(void)
{
    const float values[3] = {1234.5678f, -98765.4321f, 0.0001234f};

    for (size_t cap = 1u; cap <= 48u; cap++) {
        unsigned char storage[96];
        memset(storage, 0xA5, sizeof(storage));
        uint8_t *buf = &storage[8];

        const size_t n = Fmt_LineU8(buf, cap, values, 3u, 6u);
        CHECK_TRUE(n <= (cap - 1u));
        CHECK_INT(strlen((char *)buf), n);
        CHECK_INT(buf[n], 0u);

        for (size_t i = 0u; i < 8u; i++) {
            CHECK_INT(storage[i], 0xA5);
        }
        for (size_t i = 8u + cap; i < sizeof(storage); i++) {
            CHECK_INT(storage[i], 0xA5);
        }
    }

    for (size_t cap = 1u; cap <= 40u; cap++) {
        unsigned char storage[96];
        memset(storage, 0xA5, sizeof(storage));
        uint8_t *buf = &storage[8];

        Fmt_FloatU8(buf, cap, -123456.789f, 6u);
        CHECK_TRUE(strlen((char *)buf) <= (cap - 1u));
        for (size_t i = 0u; i < 8u; i++) {
            CHECK_INT(storage[i], 0xA5);
        }
        for (size_t i = 8u + cap; i < sizeof(storage); i++) {
            CHECK_INT(storage[i], 0xA5);
        }
    }
}

/*------------------------------------------------------------------------------
 * 5. 随机数组整行 & 长度一致性
 *---------------------------------------------------------------------------*/
static void TestFuzzLines(void)
{
    for (int round = 0; round < 5000; round++) {
        float values[6];
        const uint8_t count = (uint8_t)(1u + (Rand32() % 6u));
        char expect[160];
        size_t epos = 0u;

        for (uint8_t i = 0u; i < count; i++) {
            /* 限定在遥测量级, 避免触发 ovf/inf 分支 (上面单值测试已覆盖) */
            values[i] = (float)((int32_t)(Rand32() % 200000001u) - 100000000) / 1000.0f;
            char one[64];
            RefFloatHalfEven(one, sizeof(one), values[i], 2);
            const size_t len = strlen(one);
            if (i != 0u) {
                expect[epos++] = ',';
            }
            memcpy(&expect[epos], one, len);
            epos += len;
        }
        expect[epos++] = '\r';
        expect[epos++] = '\n';
        expect[epos]   = '\0';

        uint8_t buf[160];
        const size_t n = Fmt_LineU8(buf, sizeof(buf), values, count, 2u);
        if ((n != epos) || (strcmp((char *)buf, expect) != 0)) {
            CheckFail(__FILE__, __LINE__,
                      "line mismatch round=%d count=%u n=%u expect_len=%u got=\"%s\"",
                      round, (unsigned)count, (unsigned)n, (unsigned)epos,
                      (char *)buf);
        }
        g_checks++;
    }
}

/*------------------------------------------------------------------------------
 * 6. 主机侧性能参考
 *---------------------------------------------------------------------------*/
static void Benchmark(void)
{
    enum { ITERATIONS = 300000 };
    uint8_t buf[64];
    const float values[3] = {500.0f, 1234.5f, -0.125f};
    volatile size_t sink = 0u;

    const clock_t t0 = clock();
    for (int i = 0; i < ITERATIONS; i++) {
        sink += Fmt_LineU8(buf, sizeof(buf), values, 3u, 2u);
    }
    const clock_t t1 = clock();

    for (int i = 0; i < ITERATIONS; i++) {
        const int n = snprintf((char *)buf, sizeof(buf), "%.2f,%.2f,%.2f\r\n",
                               (double)values[0], (double)values[1], (double)values[2]);
        if (n > 0) {
            sink += (size_t)n;
        }
    }
    const clock_t t2 = clock();

    const double fmt_us = 1e6 * (double)(t1 - t0) / CLOCKS_PER_SEC / ITERATIONS;
    const double snp_us = 1e6 * (double)(t2 - t1) / CLOCKS_PER_SEC / ITERATIONS;
    printf("  Fmt_LineU8 : %.3f us/次\n", fmt_us);
    printf("  snprintf   : %.3f us/次  (%.1fx)\n", snp_us, snp_us / fmt_us);
    (void)sink;
}

int main(void)
{
    printf("== Fmt 模块测试 ==\n");

    printf("[1] 单值 vs 精确参考 ...\n");
    TestSingle();

    printf("[2] 随机位模式模糊测试 ...\n");
    TestFuzzSingle();

    printf("[3] 整行输出 ...\n");
    TestLine();

    printf("[4] 截断/越界检查 ...\n");
    TestBounds();

    printf("[5] 随机数组整行 ...\n");
    TestFuzzLines();

    printf("[6] 主机性能参考 ...\n");
    Benchmark();

    printf("\n检查项: %u, 失败: %u\n", g_checks, g_fails);
    if (g_fails != 0u) {
        printf("测试失败\n");
        return 1;
    }
    printf("全部通过\n");
    return 0;
}
