#include "FOC_SVPWM.h"

/* 全局 FOC 状态 (extern 声明见 FOC_SVPWM.h) */
FOC_State_t FOC_State;

/*==============================================================================
 * 内部常量
 *============================================================================*/

/* π 常量 (不依赖 math.h 的 M_PI, 部分工具链不含此宏) */
#define FOC_PI      3.14159265358979323846f
#define FOC_INV_PI  0.31830988618379067154f   /* 1/π */
#define FOC_2PI     6.28318530717958647692f   /* 2π */

/*==============================================================================
 * 查找表 (LUT): 用查表替代运行时分支
 * 表索引为扇区号 0~6 (扇区 1~6 有效, 索引 0 为占位)
 *============================================================================*/

/* 扇区判断: 权重 N (0~7) → 扇区号 (0~6, 0=无效) */
static const uint8_t sector_lut[8] = {
    0,  /* N=0: 无效 */
    2,  /* N=1: 扇区 2 */
    6,  /* N=2: 扇区 6 */
    1,  /* N=3: 扇区 1 */
    4,  /* N=4: 扇区 4 */
    3,  /* N=5: 扇区 3 */
    5,  /* N=6: 扇区 5 */
    0,  /* N=7: 无效 */
};

/* 矢量时间系数矩阵: T1 = Σ coeff_T1[sector][i] × {X,Y,Z}[i] */
static const float coeff_T1[7][3] = {
    { 0.0f,  0.0f,  0.0f},  /* 0: 无效 */
    { 0.0f,  0.0f, -1.0f},  /* 1: T1 = -Z */
    { 0.0f,  0.0f,  1.0f},  /* 2: T1 = +Z */
    { 1.0f,  0.0f,  0.0f},  /* 3: T1 = +X */
    {-1.0f,  0.0f,  0.0f},  /* 4: T1 = -X */
    { 0.0f, -1.0f,  0.0f},  /* 5: T1 = -Y */
    { 0.0f,  1.0f,  0.0f},  /* 6: T1 = +Y */
};

static const float coeff_T2[7][3] = {
    { 0.0f,  0.0f,  0.0f},  /* 0: 无效 */
    { 1.0f,  0.0f,  0.0f},  /* 1: T2 = +X */
    { 0.0f,  1.0f,  0.0f},  /* 2: T2 = +Y */
    { 0.0f, -1.0f,  0.0f},  /* 3: T2 = -Y */
    { 0.0f,  0.0f,  1.0f},  /* 4: T2 = +Z */
    { 0.0f,  0.0f, -1.0f},  /* 5: T2 = -Z */
    {-1.0f,  0.0f,  0.0f},  /* 6: T2 = -X */
};

/* 七段式 SVPWM 相位排列: phase_map[sector][phase] → {Tabc_0.a, b, c} 下标 */
static const uint8_t phase_map[7][3] = {
    {0, 0, 0},  /* 0: 无效 */
    {0, 1, 2},  /* 1: a→Tabc_0.a, b→Tabc_0.b, c→Tabc_0.c */
    {1, 0, 2},  /* 2: a→Tabc_0.b, b→Tabc_0.a, c→Tabc_0.c */
    {2, 0, 1},  /* 3: a→Tabc_0.c, b→Tabc_0.a, c→Tabc_0.b */
    {2, 1, 0},  /* 4: a→Tabc_0.c, b→Tabc_0.b, c→Tabc_0.a */
    {1, 2, 0},  /* 5: a→Tabc_0.b, b→Tabc_0.c, c→Tabc_0.a */
    {0, 2, 1},  /* 6: a→Tabc_0.a, b→Tabc_0.c, c→Tabc_0.b */
};

/*==============================================================================
 * CORDIC 硬件加速 sin/cos (直接寄存器访问, 无 HAL 开销)
 *============================================================================*/

/**
 * @brief  配置 CORDIC 为余弦模式 (FOC_Init 中调用一次)
 * @note   输入 angle/π (q1.31), 输出 {cos(θ), sin(θ)} (q1.31);
 *         精度 10 次迭代 (~0.1% 误差)。
 */
static void cordic_config_cosine(void)
{
    CORDIC->CSR = LL_CORDIC_FUNCTION_COSINE
                | LL_CORDIC_PRECISION_10CYCLES   /* 10 次迭代, ~0.1% 误差 */
                | LL_CORDIC_SCALE_0
                | LL_CORDIC_NBWRITE_1            /* 1 次 32 位写入: 角度 */
                | LL_CORDIC_NBREAD_2             /* 2 次 32 位读取: cos, sin */
                | LL_CORDIC_INSIZE_32BITS
                | LL_CORDIC_OUTSIZE_32BITS;
}

/**
 * @brief  用硬件 CORDIC 一次计算 sin(θ) 与 cos(θ)
 * @param  angle 弧度制角度 [0, 2π)
 * @note   硬件约 10 个 CORDIC 时钟完成, 软件 cosf+sinf 需约 400 周期。
 */
static void cordic_sin_cos(float angle,
                           float *p_sin,
                           float *p_cos)
{
    float norm;
    int32_t q31_angle;

    /* 角度归一化: [0, 2π) → [-π, π), 使 angle/π ∈ [-1, 1) */
    norm = angle * FOC_INV_PI;
    if (norm >= 1.0f)
    {
        norm -= 2.0f;  /* [π, 2π) → [-π, 0) */
    }
    /* 转换为 q1.31: int32_t(norm × 2^31) */
    q31_angle = (int32_t)(norm * Q31_SCALE);

    /* 写入 WDATA 后 CORDIC 硬件自动开始计算 */
    LL_CORDIC_WriteData(CORDIC, (uint32_t)q31_angle);

    /* 等待结果就绪 (10 次迭代 ≈ 10 个 CORDIC 时钟) */
    while (LL_CORDIC_IsActiveFlag_RRDY(CORDIC) == 0U)
    {
        /* 通常 1~2 次循环即就绪 */
    }

    /* 第 1 次读出 cos(θ), 第 2 次读出 sin(θ), 均为 q1.31 */
    *p_cos = (float)((int32_t)LL_CORDIC_ReadData(CORDIC)) * Q31_INV_SCALE;
    *p_sin = (float)((int32_t)LL_CORDIC_ReadData(CORDIC)) * Q31_INV_SCALE;
}

/*==============================================================================
 * 帕克逆变换 (CORDIC 加速)
 *============================================================================*/

/**
 * @brief  帕克逆变换: Uαβ = Park⁻¹(Udq, θ)
 * @note   用 cordic_sin_cos() 同时取得 sin/cos, 约 15 周期完成;
 *         软件 cosf()+sinf() 约需 400 周期。
 */
static void FOC_Rev_Park(FOC_State_t *FOC_State)
{
    float sin_theta, cos_theta;
    float Ud, Uq;

    /* 缓存成员到局部变量 (FPU 寄存器) */
    Ud = FOC_State->Uqd.d;
    Uq = FOC_State->Uqd.q;

    cordic_sin_cos(FOC_State->ElAngle, &sin_theta, &cos_theta);

    /* Uα = Ud×cosθ - Uq×sinθ, Uβ = Ud×sinθ + Uq×cosθ */
    FOC_State->Ualpha_beta.alpha = Ud * cos_theta - Uq * sin_theta;
    FOC_State->Ualpha_beta.beta  = Ud * sin_theta + Uq * cos_theta;
}

/*==============================================================================
 * 克拉克 + 帕克正变换 (Iabc → Idq)
 *============================================================================*/

/**
 * @brief  电流反馈变换: Iabc → Iαβ → Idq
 * @note   Clarke (等幅值): Iα = Ia, Iβ = (Ia + 2×Ib) × INV_SQRT_3
 *         Park:  Id =  Iα×cosθ + Iβ×sinθ
 *                Iq = -Iα×sinθ + Iβ×cosθ
 */
static void FOC_ClarkePark_Currents(FOC_State_t *FOC_State)
{
    float sin_theta, cos_theta;
    float Ia, Ib;
    float Ialpha, Ibeta;

    /* 缓存三相电流到局部变量 */
    Ia = FOC_State->Iabc.a;
    Ib = FOC_State->Iabc.b;

    /* Clarke: Iabc → Iαβ */
    Ialpha = Ia;
    Ibeta  = (Ia + 2.0f * Ib) * INV_SQRT_3;

    /* Park: Iαβ → Idq (CORDIC 一次给出 sin/cos) */
    cordic_sin_cos(FOC_State->ElAngle, &sin_theta, &cos_theta);

    FOC_State->Iqd.d = Ialpha * cos_theta + Ibeta * sin_theta;
    FOC_State->Iqd.q = Ibeta * cos_theta - Ialpha * sin_theta;
}

/*==============================================================================
 * 扇区判断 (LUT, 零分支)
 *============================================================================*/

/**
 * @brief  SVPWM 扇区判断
 * @retval 扇区号 1~6
 * @note   权重计算 + 查表, 全程无分支; 原实现为 3 个 if + 6-way switch。
 */
static uint8_t SectorJudgment(FOC_State_t *FOC_State)
{
    float A, B, C;
    uint8_t N;

    /* 缓存 αβ 电压 */
    const float alpha = FOC_State->Ualpha_beta.alpha;
    const float beta  = FOC_State->Ualpha_beta.beta;

    /* A = Uβ, B = √3×Uα - Uβ, C = -√3×Uα - Uβ */
    A = beta;
    B = SQRT_3 * alpha - beta;
    C = -SQRT_3 * alpha - beta;

    /* N = (A>0)×1 + (B>0)×2 + (C>0)×4, 无分支 */
    N = ((A > 0.0f) ? 1U : 0U)
      | ((B > 0.0f) ? 2U : 0U)
      | ((C > 0.0f) ? 4U : 0U);

    return sector_lut[N];
}

/*==============================================================================
 * 矢量作用时间计算 (LUT, 零分支)
 *============================================================================*/

/**
 * @brief  SVPWM 矢量时间计算并更新 PWM 比较值
 * @note   1. T1/T2 由系数矩阵乘加得到, 无 6-way switch;
 *         2. 七段式分配由排列矩阵得到, 无 6-way switch;
 *         3. 除法改乘法: /4→×0.25, /2→×0.5, /(Tpwm/2)→×预计算倒数;
 *         4. 公共子表达式提前计算。
 */
static void VectorActionTime(FOC_State_t *FOC_State)
{
    float X, Y, Z;
    float T0, T1, T2;
    float T1_plus_T2;
    float Tabc_0a, Tabc_0b, Tabc_0c;
    float t0_arr[3];
    const uint8_t *map;
    float scale;
    uint8_t sec;
    uint32_t tpwm_f;
    float power;

    /* ---- 阶段 1: 缓存成员并预计算公因子 ---- */
    sec    = FOC_State->sector;
    tpwm_f = FOC_State->Tpwm;
    power  = FOC_State->power;

    /* tmp = Tpwm × √3 / power; 用倒数替代多次除法 */
    {
        const float inv_power = 1.0f / power;
        const float tmp = (float)tpwm_f * SQRT_3 * inv_power;

        const float alpha = FOC_State->Ualpha_beta.alpha;
        const float beta  = FOC_State->Ualpha_beta.beta;

        X = tmp * beta;
        Y = tmp * (alpha * SQRT_3_DIV_2 + beta * 0.5f);
        Z = tmp * (beta * 0.5f - alpha * SQRT_3_DIV_2);
    }

    /* ---- 阶段 2: 矢量时间分配 (系数矩阵乘加) ---- */
    T1 = coeff_T1[sec][0] * X + coeff_T1[sec][1] * Y + coeff_T1[sec][2] * Z;
    T2 = coeff_T2[sec][0] * X + coeff_T2[sec][1] * Y + coeff_T2[sec][2] * Z;

    /* ---- 阶段 3: 过调制处理: T1/T2 等比例缩放 ---- */
    T1_plus_T2 = T1 + T2;
    if (T1_plus_T2 > (float)tpwm_f)
    {
        /* T1 = T1/(T1+T2)×Tpwm, 先求倒数再乘 */
        const float inv_sum = 1.0f / T1_plus_T2;
        const float scale_overmod = (float)tpwm_f * inv_sum;
        T1 *= scale_overmod;
        T2 *= scale_overmod;
    }

    /* 零矢量时间 */
    T0 = (float)tpwm_f - T1 - T2;

    /* ---- 阶段 4: 计算 Tabc_0 (七段式中间值) ---- */
    Tabc_0a = T0 * 0.25f;
    Tabc_0b = Tabc_0a + T1 * 0.5f;
    Tabc_0c = Tabc_0b + T2 * 0.5f;

    /* ---- 阶段 5: 七段式相位分配并写比较寄存器 ---- */
    t0_arr[0] = Tabc_0a;
    t0_arr[1] = Tabc_0b;
    t0_arr[2] = Tabc_0c;
    map = phase_map[sec];

    {
        const float Ta = t0_arr[map[0]];
        const float Tb = t0_arr[map[1]];
        const float Tc = t0_arr[map[2]];

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)Tc);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)Tb);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)Ta);

        /* ---- 阶段 6: 反算三相电压 (调试/监控用) ---- */
        scale = power * 2.0f / (float)tpwm_f;
        FOC_State->Uabc.a = Ta * scale;
        FOC_State->Uabc.b = Tb * scale;
        FOC_State->Uabc.c = Tc * scale;
    }
}

/*==============================================================================
 * 顶层 FOC SVPWM 调度
 *============================================================================*/

/**
 * @brief  一次完整 SVPWM 计算周期
 * @note   FOC_Rev_Park → SectorJudgment → VectorActionTime
 */
static void FOC_RUN_SVPWM(FOC_State_t *FOC_State)
{
    /* 帕克逆变换: Udq → Uαβ (CORDIC 加速) */
    FOC_Rev_Park(FOC_State);

    /* 相电压取反 (适配硬件 PWM 极性) */
    FOC_State->Ualpha_beta.alpha = -FOC_State->Ualpha_beta.alpha;
    FOC_State->Ualpha_beta.beta  = -FOC_State->Ualpha_beta.beta;

    /* 扇区判断 + 矢量时间计算 */
    FOC_State->sector = SectorJudgment(FOC_State);
    VectorActionTime(FOC_State);
}

/*==============================================================================
 * 初始化 & PWM 启停
 *============================================================================*/

/**
 * @brief  FOC 初始化: 配置 CORDIC, 清零状态, 启动 TIM1 更新中断
 */
void FOC_Init(void)
{
    /* CORDIC 硬件配置 (一次配置, 运行时零开销) */
    cordic_config_cosine();

    FOC_State.sector = 0;

    FOC_State.Uqd.q = 0.0f;
    FOC_State.Uqd.d = 0.0f;
    FOC_State.Iqd.q = 0.0f;
    FOC_State.Iqd.d = 0.0f;

    FOC_State.Uabc.a = 0.0f;
    FOC_State.Uabc.b = 0.0f;
    FOC_State.Uabc.c = 0.0f;

    FOC_State.Iabc.a = 0.0f;
    FOC_State.Iabc.b = 0.0f;
    FOC_State.Iabc.c = 0.0f;

    FOC_State.Ualpha_beta.alpha = 0.0f;
    FOC_State.Ualpha_beta.beta  = 0.0f;

    FOC_State.ElAngle = 0.0f;

    FOC_State.power = 0.0f;     /* 母线电压 (V) */
    FOC_State.Tpwm  = 8500;     /* PWM 周期计数值 (170MHz / 20kHz) */

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_Delay(1000);
}

/**
 * @brief  启动 TIM1 三相 PWM 输出 (含互补通道)
 */
void FOC_Start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    FOC_State.State = true;
}

/**
 * @brief  停止 TIM1 三相 PWM 输出 (含互补通道)
 */
void FOC_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    FOC_State.State = false;
}

/**
 * @brief  TIM 更新中断回调 (全工程唯一实现)
 * @note   TIM1 (20 kHz): FOC 采样与 SVPWM 调度
 *         TIM7 (1 kHz):   角速度环每拍执行, 角度环 5 分频 (200 Hz);
 *                         每拍输出一行 12 路 PID 遥测。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        if (FOC_State.State)
        {
            FOC_State.power  = ADC_GetData(ADC_CMD_VBUS);
            ADC_GetCurrents(&FOC_State.Iabc.a, &FOC_State.Iabc.b, &FOC_State.Iabc.c);
            FOC_State.ElAngle = MT6826S_GetData(MT6826S_CMD_ELANGLE);

            /* Iabc → Idq (CORDIC 加速) */
            FOC_ClarkePark_Currents(&FOC_State);

            /* 电流环: 输出 Udq 电压指令 */
            PID_Current_Loop_Isr(FOC_State.Iqd.d, FOC_State.Iqd.q,
                                 &FOC_State.Uqd.d, &FOC_State.Uqd.q);

            FOC_RUN_SVPWM(&FOC_State);
        }
    }
    else if (htim == &htim7)
    {
        static uint8_t counter = 0;

        PID_Speed_Loop_Isr();
        if (++counter >= 5)     /* 5 分频 → 200 Hz 角度环 */
        {
            counter = 0;
            PID_Angle_Loop_Isr();
        }

        /* 1 kHz 遥测: 12 路 PID 数据 (顺序见 PID_TELEM_xxx)
         * 用 Fmt_LineU8 做纯整数 ASCII 格式化, 再一次性写入环形缓冲区;
         * 不使用 snprintf / 浮点 printf, 避免 1 kHz 中断里的高开销。
         * 按 12 路最坏情况预留: 每路约 10 字符 + 逗号 + CRLF。 */
        float telemetry[PID_TELEM_COUNT];
        PID_GetTelemetry(telemetry);
        uint8_t line[192];
        const size_t n = Fmt_LineU8(line, sizeof(line), telemetry,
                                    (uint8_t)PID_TELEM_COUNT, 4u);
        if (n != 0u && RingBuffer_GetFree(&USB_TxBuffer) > n)
        {
            (void)RingBuffer_Write(&USB_TxBuffer, line, (uint32_t)n);
        }
    }
}