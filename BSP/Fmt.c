#include "Fmt.h"

/*==============================================================================
 * 实现要点
 *   1. 按 IEEE-754 binary32 位域拆出符号 / 指数 / 尾数;
 *   2. 用整数精确计算 x = mantissa * 10^decimals * 2^(exp-150), 再按
 *      "四舍六入五取偶"舍入成整数 (全程不经过 double);
 *   3. 整数转十进制用 /100 乘法 + 两位数字表, 单个数值先写入本地 token;
 *   4. 再把 token / 分隔符一次性拷入用户缓冲区, 每个 token 只做一次边界检查。
 *
 * 全部为整数运算, 无 stdio / malloc / 全局状态, 可在 1 kHz 中断内调用。
 *============================================================================*/

static const uint32_t s_pow10[FMT_DECIMALS_MAX + 1u] = {
    1u, 10u, 100u, 1000u, 10000u, 100000u, 1000000u
};

/* 两位十进制数字表: 下标 2*n / 2*n+1 即数字 n 的十位/个位 */
static const char s_two_digits[] =
    "00010203040506070809"
    "10111213141516171819"
    "20212223242526272829"
    "30313233343536373839"
    "40414243444546474849"
    "50515253545556575859"
    "60616263646566676869"
    "70717273747576777879"
    "80818283848586878889"
    "90919293949596979899";

typedef enum {
    FMT_NUM_OK = 0,     /* 正常值 */
    FMT_NUM_NAN,        /* NaN */
    FMT_NUM_INF,        /* ±Inf */
    FMT_NUM_OVF         /* 有限值, 但放大约 10^decimals 倍后超出 int64 范围 */
} FmtNumStatus_t;

/* 带越界保护的输出游标 (预留末尾 '\0') */
typedef struct {
    uint8_t *buf;
    size_t   cap;
    size_t   pos;
} Fmt_Writer_t;

/*==============================================================================
 * 内部函数
 *============================================================================*/

/* 追加一段字符, 每次调用只做一次边界判断 */
static void Fmt_Append(Fmt_Writer_t *w, const char *src, size_t len)
{
    if (w->pos >= (w->cap - 1u))
    {
        return;
    }
    size_t space = (w->cap - 1u) - w->pos;
    if (len > space)
    {
        len = space;
    }
    for (size_t i = 0u; i < len; i++)
    {
        w->buf[w->pos + i] = (uint8_t)src[i];
    }
    w->pos += len;
}

/**
 * @brief  把 float 精确舍入成"放大 10^decimals 倍的整数"
 * @param  value  输入浮点值
 * @param  dec    小数位数 0 ~ FMT_DECIMALS_MAX
 * @param  scaled [输出] 舍入后的绝对值 (|value| * 10^dec)
 * @param  neg    [输出] 符号位 (含 -0.0)
 * @retval FMT_NUM_OK / NAN / INF / OVF
 */
static FmtNumStatus_t Fmt_ScaleFloat(float value, uint8_t dec,
                                     uint64_t *scaled, bool *neg)
{
    union {
        float    f;
        uint32_t u;
    } bits;
    bits.f = value;

    const uint32_t sign_bit = bits.u >> 31;
    const uint32_t exp_bits = (bits.u >> 23) & 0xFFu;
    const uint32_t frac_bits = bits.u & 0x7FFFFFu;

    *neg = (sign_bit != 0u);

    /* Inf / NaN: 指数全 1 */
    if (exp_bits == 0xFFu)
    {
        return (frac_bits != 0u) ? FMT_NUM_NAN : FMT_NUM_INF;
    }

    uint32_t mant;
    int32_t  exp2;
    if (exp_bits == 0u)
    {
        if (frac_bits == 0u)                /* ±0.0 */
        {
            *scaled = 0u;
            return FMT_NUM_OK;
        }
        mant = frac_bits;                   /* 非规格化数 */
        exp2 = -149;
    }
    else
    {
        mant = frac_bits | 0x800000u;
        exp2 = (int32_t)exp_bits - 150;     /* exp_bits - 127 - 23 */
    }

    /* 快路径: dec <= 2 时 mant * 10^dec <= 1.68e9 < 2^31,
     * 结果落在 uint32 范围时用 32 位运算 (遥测主路径)。 */
    if (dec <= 2u)
    {
        uint32_t x = mant * s_pow10[dec];
        if (exp2 >= 0)
        {
            if ((exp2 < 31) && (x <= (UINT32_MAX >> (uint32_t)exp2)))
            {
                *scaled = (uint64_t)(x << (uint32_t)exp2);
                return FMT_NUM_OK;
            }
            /* 超出 uint32, 继续走 64 位路径 */
        }
        else
        {
            const uint32_t shift = (uint32_t)(-exp2);
            if (shift >= 32u)
            {
                /* x < 2^31 <= half, 舍入结果为 0 */
                *scaled = 0u;
                return FMT_NUM_OK;
            }
            const uint32_t rem  = x & ((UINT32_C(1) << shift) - 1u);
            const uint32_t half = UINT32_C(1) << (shift - 1u);
            x >>= shift;
            /* 四舍六入五取偶: 与 printf 默认舍入一致 */
            if ((rem > half) || ((rem == half) && ((x & 1u) != 0u)))
            {
                x++;
            }
            *scaled = x;
            return FMT_NUM_OK;
        }
    }

    /* 64 位路径: decimals >= 3, 或数值大到超过 uint32 */
    uint64_t x = (uint64_t)mant * (uint64_t)s_pow10[dec];

    if (exp2 >= 0)
    {
        if ((exp2 >= 64) || (x > ((uint64_t)INT64_MAX >> (uint32_t)exp2)))
        {
            return FMT_NUM_OVF;
        }
        x <<= (uint32_t)exp2;
    }
    else
    {
        const uint32_t shift = (uint32_t)(-exp2);
        if (shift >= 63u)
        {
            x = 0u;
        }
        else
        {
            const uint64_t rem  = x & ((UINT64_C(1) << shift) - 1u);
            const uint64_t half = UINT64_C(1) << (shift - 1u);
            x >>= shift;
            if ((rem > half) || ((rem == half) && ((x & 1u) != 0u)))
            {
                x++;
            }
        }
    }

    if (x > (uint64_t)INT64_MAX)
    {
        return FMT_NUM_OVF;
    }

    *scaled = x;
    return FMT_NUM_OK;
}

/**
 * @brief  uint64 → 十进制数字串 (不含 '\0'), out 至少 20 字节
 * @retval 数字个数
 */
static size_t Fmt_FormatDigits(uint64_t value, char *out)
{
    if (value <= (uint64_t)UINT32_MAX)
    {
        /* 常用路径: 32 位 /100 乘法 + 两位数字表 */
        uint32_t v = (uint32_t)value;
        char     tmp[12];
        char    *p = tmp + sizeof(tmp);

        while (v >= 100u)
        {
            const uint32_t q = v / 100u;
            const uint32_t r = v - (q * 100u);
            p -= 2;
            p[0] = s_two_digits[(2u * r)];
            p[1] = s_two_digits[(2u * r) + 1u];
            v = q;
        }
        if (v >= 10u)
        {
            p -= 2;
            p[0] = s_two_digits[(2u * v)];
            p[1] = s_two_digits[(2u * v) + 1u];
        }
        else
        {
            p--;
            p[0] = (char)('0' + (char)v);
        }

        const size_t len = (size_t)((tmp + sizeof(tmp)) - p);
        for (size_t i = 0u; i < len; i++)
        {
            out[i] = p[i];
        }
        return len;
    }

    /* 极大值路径 (遥测不会触发): 64 位逐位除法, 极少执行 */
    char  tmp[20];
    char *p = tmp + sizeof(tmp);
    while (value >= 10u)
    {
        const uint32_t digit = (uint32_t)(value % 10u);
        value /= 10u;
        p--;
        p[0] = (char)('0' + (char)digit);
    }
    p--;
    p[0] = (char)('0' + (char)value);

    const size_t len = (size_t)((tmp + sizeof(tmp)) - p);
    for (size_t i = 0u; i < len; i++)
    {
        out[i] = p[i];
    }
    return len;
}

/**
 * @brief  把一个 float 写成本地 token (含符号/小数点, 不含分隔符)
 * @param  tok   输出缓冲区, 至少 32 字节
 * @retval token 长度
 */
static size_t Fmt_EmitToken(char *tok, float value, uint8_t dec)
{
    uint64_t scaled = 0u;
    bool     neg    = false;
    const FmtNumStatus_t status = Fmt_ScaleFloat(value, dec, &scaled, &neg);

    size_t n = 0u;
    if (status != FMT_NUM_OK)
    {
        if (neg)
        {
            tok[n++] = '-';
        }
        if (status == FMT_NUM_NAN)
        {
            tok[n++] = 'n'; tok[n++] = 'a'; tok[n++] = 'n';
        }
        else if (status == FMT_NUM_INF)
        {
            tok[n++] = 'i'; tok[n++] = 'n'; tok[n++] = 'f';
        }
        else
        {
            tok[n++] = 'o'; tok[n++] = 'v'; tok[n++] = 'f';
        }
        return n;
    }

    char   digits[20];
    const size_t digits_len = Fmt_FormatDigits(scaled, digits);

    if (neg)
    {
        tok[n++] = '-';
    }

    if (dec == 0u)
    {
        for (size_t i = 0u; i < digits_len; i++)
        {
            tok[n++] = digits[i];
        }
    }
    else if (digits_len <= dec)
    {
        /* |value| < 1: 补前导零, 如 "0.00" / "0.05" */
        tok[n++] = '0';
        tok[n++] = '.';
        for (size_t i = digits_len; i < dec; i++)
        {
            tok[n++] = '0';
        }
        for (size_t i = 0u; i < digits_len; i++)
        {
            tok[n++] = digits[i];
        }
    }
    else
    {
        /* |value| >= 1: 从右往左数 dec 位作为小数部分 */
        const size_t int_len = digits_len - dec;
        for (size_t i = 0u; i < int_len; i++)
        {
            tok[n++] = digits[i];
        }
        tok[n++] = '.';
        for (size_t i = int_len; i < digits_len; i++)
        {
            tok[n++] = digits[i];
        }
    }
    return n;
}

/*==============================================================================
 * 对外接口
 *============================================================================*/

size_t Fmt_FloatU8(uint8_t *buf, size_t cap, float value, uint8_t decimals)
{
    if ((buf == NULL) || (cap == 0u))
    {
        return 0u;
    }

    if (decimals > FMT_DECIMALS_MAX)
    {
        decimals = FMT_DECIMALS_MAX;
    }

    char tok[32];
    size_t len = Fmt_EmitToken(tok, value, decimals);

    const size_t max_len = cap - 1u;
    if (len > max_len)
    {
        len = max_len;
    }
    for (size_t i = 0u; i < len; i++)
    {
        buf[i] = (uint8_t)tok[i];
    }
    buf[len] = '\0';
    return len;
}

size_t Fmt_LineU8(uint8_t *buf, size_t cap, const float *values,
                  uint8_t count, uint8_t decimals)
{
    if ((buf == NULL) || (cap == 0u) || (values == NULL))
    {
        return 0u;
    }

    if (decimals > FMT_DECIMALS_MAX)
    {
        decimals = FMT_DECIMALS_MAX;
    }

    Fmt_Writer_t w;
    w.buf = buf;
    w.cap = cap;
    w.pos = 0u;

    char tok[32];
    for (uint8_t i = 0u; i < count; i++)
    {
        const size_t len = Fmt_EmitToken(tok, values[i], decimals);
        Fmt_Append(&w, tok, len);
        if ((uint8_t)(i + 1u) < count)
        {
            Fmt_Append(&w, ",", 1u);
        }
    }
    Fmt_Append(&w, "\r\n", 2u);

    buf[w.pos] = '\0';
    return w.pos;
}