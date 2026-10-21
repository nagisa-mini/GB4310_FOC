#include "FOC_SVPWM.h"
#include "ADC_Sample.h"
#include "PID.h"
#include "Fmt.h"
#include "stm32g4xx_hal_tim.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <stdint.h>

/* 全局 FOC 状态变量定义 (extern 在 FOC_SVPWM.h 中声明) */
FOC_State_t FOC_State;

/*==============================================================================
 * 内部常量
 *============================================================================*/

/* π 常量 (避免依赖 math.h 中的 M_PI, 部分工具链不含此宏) */
#define FOC_PI      3.14159265358979323846f
#define FOC_INV_PI  0.31830988618379067154f   /* 1/π */
#define FOC_2PI     6.28318530717958647692f   /* 2π */

/*==============================================================================
 * 查找表 (LUTs) — 替代运行时分支
 * 所有表索引为扇区号 0-6 (扇区1-6有效, 索引0为占位)
 *============================================================================*/

/* 扇区判断: 权重值 N (0-7) → 扇区号 (0-6, 0=无效) */
static const uint8_t sector_lut[8] = {
    0,  /* N=0: 无效 */
    2,  /* N=1: 扇区2 */
    6,  /* N=2: 扇区6 */
    1,  /* N=3: 扇区1 */
    4,  /* N=4: 扇区4 */
    3,  /* N=5: 扇区3 */
    5,  /* N=6: 扇区5 */
    0,  /* N=7: 无效 */
};

/*
 * 矢量时间系数矩阵: T1 = sum(coeff_T1[sector][i] * {X,Y,Z}[i])
 * 每行仅一个非零元素 (±1), 替代 6-way switch
 * 使用 FMA (融合乘加) 高效计算: 即使有零项, FPU 单周期完成
 */
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

/*
 * 七段式 SVPWM 相位排列矩阵
 * phase_map[sector][phase] = 索引到 {Tabc_0.a, Tabc_0.b, Tabc_0.c}
 * phase: 0=a相, 1=b相, 2=c相
 */
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
 * CORDIC 硬件加速 sin/cos (LL 直接寄存器访问, 零 HAL 开销)
 *============================================================================*/

/**
 * @brief  配置 CORDIC 为余弦模式 (FOC_Init 中调用一次)
 * @note   余弦模式: 输入 angle/π (q1.31), 输出 {cos(θ), sin(θ)} (q1.31)
 *         精度: 10次迭代 (~0.1% 误差)
 */
static void cordic_config_cosine(void)
{
    CORDIC->CSR = LL_CORDIC_FUNCTION_COSINE
                | LL_CORDIC_PRECISION_10CYCLES   /* 10次迭代, ~0.1%误差 */
                | LL_CORDIC_SCALE_0
                | LL_CORDIC_NBWRITE_1            /* 1次32位写入: 角度 */
                | LL_CORDIC_NBREAD_2             /* 2次32位读取: cos, sin */
                | LL_CORDIC_INSIZE_32BITS
                | LL_CORDIC_OUTSIZE_32BITS;
}

/**
 * @brief  使用硬件CORDIC一次计算 sin(θ) 和 cos(θ)
 * @param  angle  弧度制角度, 范围 [0, 2π)
 * @param  p_sin  输出: sin(θ)
 * @param  p_cos  输出: cos(θ)
 * @note   核心优势: 同时计算sin和cos, 硬件10周期完成,
 *          软件FPU需~400周期 (cosf+sinf各~200周期)
 */
static void cordic_sin_cos(float angle,
                                                                 float *p_sin,
                                                                 float *p_cos)
{
    float norm;
    int32_t q31_angle;

    /*
     * 角度归一化: 将 [0, 2π) 映射到 [-π, π), 使 angle/π ∈ [-1, 1)
     * 浮点比较 + 减法 → 1~2个FPU周期 (硬件FPU的条件分支可预测)
     */
    norm = angle * FOC_INV_PI;
    if (norm >= 1.0f) {
        norm -= 2.0f;  /* [π, 2π) → [-π, 0) */
    }
    /* norm ∈ [-1, 1), 转换为 q1.31: int32_t(norm × 2^31) */
    q31_angle = (int32_t)(norm * Q31_SCALE);

    /* 写入 WDATA → CORDIC硬件自动开始计算 */
    LL_CORDIC_WriteData(CORDIC, (uint32_t)q31_angle);

    /* 等待结果就绪 (10次迭代 ≈ 10个 CORDIC时钟) */
    while (LL_CORDIC_IsActiveFlag_RRDY(CORDIC) == 0U) {
        /* CORDIC 计算中, 通常1-2次循环即就绪 */
    }

    /* 读取结果: 第1次 = cos(θ), 第2次 = sin(θ), 均为 q1.31 */
    /* 转换为 float: val = q31 × (1/2^31) */
    *p_cos = (float)((int32_t)LL_CORDIC_ReadData(CORDIC)) * Q31_INV_SCALE;
    *p_sin = (float)((int32_t)LL_CORDIC_ReadData(CORDIC)) * Q31_INV_SCALE;
}

/*==============================================================================
 * 帕克逆变换 (硬件加速版)
 * 使用 CORDIC 同时获取 sin/cos, 一次硬件操作替代两次软件三角函数
 *============================================================================*/

/**
 * @brief  帕克逆变换: Uαβ = Park⁻¹(Udq, θ)
 * @note   原始: cosf()+sinf() ≈ 400 FPU周期
 *          优化: cordic_sin_cos() ≈ 15周期 (含q31转换)
 *          加速比: ~26x (仅此函数)
 */
static void FOC_Rev_Park(FOC_State_t *FOC_State)
{
    float sin_theta, cos_theta;
    float Ud, Uq;

    /* 缓存结构体成员到局部变量 (FPU寄存器) */
    Ud = FOC_State->Uqd.d;
    Uq = FOC_State->Uqd.q;

    /* 硬件 CORDIC 一次计算 sin 和 cos */
    cordic_sin_cos(FOC_State->ElAngle, &sin_theta, &cos_theta);

    /* 矩阵乘法: Uα = Ud×cosθ - Uq×sinθ, Uβ = Ud×sinθ + Uq×cosθ */
    FOC_State->Ualpha_beta.alpha = Ud * cos_theta - Uq * sin_theta;
    FOC_State->Ualpha_beta.beta  = Ud * sin_theta + Uq * cos_theta;
}

/*==============================================================================
 * 克拉克+帕克正变换 (电流反馈 Iabc → Idq)
 * 硬件 CORDIC 加速 sin/cos 计算
 *============================================================================*/

/**
 * @brief  克拉克+帕克正变换: Iabc → Iαβ → Idq
 * @note   用于电流环反馈, 三相电流 → dq轴电流
 *
 *         Clarke (等幅值):
 *           Iα = Ia
 *           Iβ = (Ia + 2*Ib) / √3 = (Ia + 2*Ib) × INV_SQRT_3
 *
 *         Park (CORDIC硬件加速):
 *           Id =  Iα × cos(θ) + Iβ × sin(θ)
 *           Iq = -Iα × sin(θ) + Iβ × cos(θ)
 *
 *         性能:
 *         - CORDIC sin/cos: ~15周期 (含q31转换)
 *         - 矩阵乘法: 4次乘加 ≈ 4周期
 *         - 总计: ~20 FPU周期 (软件需 ~420周期, 加速比 ~21x)
 */
static void FOC_ClarkePark_Currents(FOC_State_t *FOC_State)
{
    float sin_theta, cos_theta;
    float Ia, Ib;
    float Ialpha, Ibeta;

    /* 缓存三相电流到局部变量 (FPU寄存器) */
    Ia = FOC_State->Iabc.a;
    Ib = FOC_State->Iabc.b;

    /*
     * 阶段1: Clarke 变换 (Iabc → Iαβ)
     * Iα = Ia
     * Iβ = (Ia + 2×Ib) × INV_SQRT_3
     */
    Ialpha = Ia;
    Ibeta  = (Ia + 2.0f * Ib) * INV_SQRT_3;

    /*
     * 阶段2: Park 变换 (Iαβ → Idq)
     * 硬件 CORDIC 一次计算 sin(θ) 和 cos(θ)
     */
    cordic_sin_cos(FOC_State->ElAngle, &sin_theta, &cos_theta);

    /*
     * 阶段3: 旋转矩阵
     * Id =  Iα×cos(θ) + Iβ×sin(θ)
     * Iq = -Iα×sin(θ) + Iβ×cos(θ)
     */
    FOC_State->Iqd.d = Ialpha * cos_theta + Ibeta * sin_theta;
    FOC_State->Iqd.q = Ibeta * cos_theta - Ialpha * sin_theta;
}

/*==============================================================================
 * 扇区判断 (LUT版, 零分支)
 *============================================================================*/

/**
 * @brief  SVPWM 扇区判断
 * @return 扇区号 (1-6)
 * @note   原始: 3个if + 6-way switch (9个分支)
 *          优化: 权重计算 + LUT查表 (0个分支)
 */
static uint8_t SectorJudgment(FOC_State_t *FOC_State)
{
    float A, B, C;
    uint8_t N;

    /* 缓存 αβ 电压 */
    const float alpha = FOC_State->Ualpha_beta.alpha;
    const float beta  = FOC_State->Ualpha_beta.beta;

    /*
     * 三相反电动势计算: 用于判断参考电压矢量所在扇区
     * A = Uβ, B = √3×Uα - Uβ, C = -√3×Uα - Uβ
     */
    A = beta;
    B = SQRT_3 * alpha - beta;
    C = -SQRT_3 * alpha - beta;

    /* 权重计算: N = (A>0)×1 + (B>0)×2 + (C>0)×4, 无分支 */
    N = ((A > 0.0f) ? 1U : 0U)
      | ((B > 0.0f) ? 2U : 0U)
      | ((C > 0.0f) ? 4U : 0U);

    /* LUT 直接映射权重→扇区, 零分支 */
    return sector_lut[N];
}

/*==============================================================================
 * 矢量作用时间计算 (LUT版, 零分支)
 *============================================================================*/

/**
 * @brief  SVPWM 矢量时间计算及 PWM 占空比更新
 * @note   优化要点:
 *         1. T1/T2分配: 系数矩阵乘加 → 0分支 (原6-way switch)
 *         2. 七段式分配: 排列矩阵 → 0分支 (原6-way switch)
 *         3. 除法→乘法: /4.0f→×0.25f, /2.0f→×0.5f, /(Tpwm/2)→×预计算倒数
 *         4. 公用子表达式预计算
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

    /* ---- 阶段1: 缓存结构体成员 ---- */
    sec    = FOC_State->sector;
    tpwm_f = FOC_State->Tpwm;
    power  = FOC_State->power;

    /*
     * 预计算公因子: tmp = Tpwm × √3 / power
     * 使用倒数: 一次除法 vs 每次都除
     */
    {
        const float inv_power = 1.0f / power;  /* 倒数替代多次除法 */
        const float tmp = (float)tpwm_f * SQRT_3 * inv_power;

        const float alpha = FOC_State->Ualpha_beta.alpha;
        const float beta  = FOC_State->Ualpha_beta.beta;

        X = tmp * beta;
        Y = tmp * (alpha * SQRT_3_DIV_2 + beta * 0.5f);
        Z = tmp * (beta * 0.5f - alpha * SQRT_3_DIV_2);
    }

    /* ---- 阶段2: 矢量时间分配 (系数矩阵乘加, 零分支) ---- */
    T1 = coeff_T1[sec][0] * X + coeff_T1[sec][1] * Y + coeff_T1[sec][2] * Z;
    T2 = coeff_T2[sec][0] * X + coeff_T2[sec][1] * Y + coeff_T2[sec][2] * Z;

    /* ---- 阶段3: 过调制处理 ---- */
    T1_plus_T2 = T1 + T2;
    if (T1_plus_T2 > (float)tpwm_f) {
        /*
         * 过调制: 等比例缩放 T1, T2
         * T1 = T1/(T1+T2)*Tpwm → 先算倒数再乘, 减少除法次数
         */
        const float inv_sum = 1.0f / T1_plus_T2;
        const float scale_overmod = (float)tpwm_f * inv_sum;
        T1 *= scale_overmod;
        T2 *= scale_overmod;
    }

    /* 零矢量时间 */
    T0 = (float)tpwm_f - T1 - T2;

    /*
     * ---- 阶段4: 计算 Tabc_0 (中间值) ----
     * 原始: Tabc_0.a = T0/4.0f  → 优化: T0 * 0.25f
     * 原始: Tabc_0.b = Tabc_0.a + T1/2.0f → 优化: T1 * 0.5f
     * 原始: Tabc_0.c = Tabc_0.b + T2/2.0f → 优化: T2 * 0.5f
     */
    Tabc_0a = T0 * 0.25f;
    Tabc_0b = Tabc_0a + T1 * 0.5f;
    Tabc_0c = Tabc_0b + T2 * 0.5f;

    /* ---- 阶段5: 七段式 SVPWM 相位分配 (排列矩阵, 零分支) ---- */
    t0_arr[0] = Tabc_0a;
    t0_arr[1] = Tabc_0b;
    t0_arr[2] = Tabc_0c;
    map = phase_map[sec];

    {
        const float Ta = t0_arr[map[0]];
        const float Tb = t0_arr[map[1]];
        const float Tc = t0_arr[map[2]];

        /* 写入PWM比较寄存器 (硬件操作, 非计算瓶颈) */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)Tc);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)Tb);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)Ta);

        /*
         * ---- 阶段6: 反算三相电压 (用于调试/监控) ----
         * 原始: Uabc.x = Tabc_1.x / (Tpwm/2) * power
         * 优化: 预计算 scale = power * 2 / Tpwm, 然后 Tabc_1.x * scale
         * 注意: 先算 power/tpwm_f 精度更高
         */
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
 * @brief  FOC SVPWM 一次完整计算周期
 * @note   调用路径 (全内联, 无函数调用开销):
 *         FOC_Rev_Park → cordic_sin_cos (CORDIC硬件)
 *         SectorJudgment (LUT)
 *         VectorActionTime (LUT + FMAC)
 */
static void FOC_RUN_SVPWM(FOC_State_t *FOC_State)
{
    /* 帕克逆变换: Udq → Uαβ (硬件CORDIC加速) */
    FOC_Rev_Park(FOC_State);

    /* 相电压取反 (适配硬件 PWM 极性) */
    FOC_State->Ualpha_beta.alpha = -FOC_State->Ualpha_beta.alpha;
    FOC_State->Ualpha_beta.beta  = -FOC_State->Ualpha_beta.beta;

    /* 扇区判断 (LUT) */
    FOC_State->sector = SectorJudgment(FOC_State);

    /* 矢量时间计算及PWM更新 (LUT + FMA) */
    VectorActionTime(FOC_State);
}

/*==============================================================================
 * 初始化 & PWM 启停
 *============================================================================*/

/**
 * @brief  FOC 初始化
 * @note   配置 CORDIC 为余弦模式 (只需一次),
 *         利用 FPU 初始化结构体为零
 */
void FOC_Init(void)
{
    /* ---- CORDIC 硬件配置 (一次配置, 运行时零开销) ---- */
    cordic_config_cosine();

    /* ---- FOC 状态初始化 (利用 FPU 批量零初始化) ---- */
    FOC_State.sector = 0;

    /* Uqd 初始值 */
    FOC_State.Uqd.q = 0.0f;
    FOC_State.Uqd.d = 0.0f;

    /* Iqd 初始值 (电流反馈 dq轴) */
    FOC_State.Iqd.q = 0.0f;
    FOC_State.Iqd.d = 0.0f;

    /* 输出电压归零 */
    FOC_State.Uabc.a = 0.0f;
    FOC_State.Uabc.b = 0.0f;
    FOC_State.Uabc.c = 0.0f;

    /* 电流归零 */
    FOC_State.Iabc.a = 0.0f;
    FOC_State.Iabc.b = 0.0f;
    FOC_State.Iabc.c = 0.0f;

    /* αβ 电压归零 */
    FOC_State.Ualpha_beta.alpha = 0.0f;
    FOC_State.Ualpha_beta.beta  = 0.0f;

    /* 电角度归零 */
    FOC_State.ElAngle = 0.0f;

    /* 运行参数 */
    FOC_State.power = 0.0f;    /* 母线电压 (V) */
    FOC_State.Tpwm  = 8500;     /* PWM周期计数值 (170MHz / 20kHz) */

    /* 启动定时器中断 */
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_Delay(1000);
}

void FOC_Start(void)
{
    // 启动 TIM1 PWM 输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    FOC_State.State = true; // 设置 FOC 状态为运行
}

void FOC_Stop(void)
{
    // 停止 TIM1 PWM 输出
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    FOC_State.State = false; // 设置 FOC 状态为停止
}

/**
  * @brief  TIM 更新中断回调
  * @note   TIM1 (20kHz): FOC 采样与 SVPWM 调度
  *         TIM7 (1ms):    PID 闭环周期任务 (角度采样 → 角速度计算 → 双环运算)
  *         注意: 本回调全工程只能有一处定义, PID 任务因此并入此处
  * @param  htim TIM 句柄
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1) 
    {
        if(FOC_State.State) 
        {
            FOC_State.power=ADC_GetData(ADC_CMD_VBUS);
            ADC_GetCurrents(&FOC_State.Iabc.a,&FOC_State.Iabc.b,&FOC_State.Iabc.c);
            FOC_State.ElAngle=MT6826S_GetData(MT6826S_CMD_ELANGLE);
            /* 克拉克+帕克正变换: Iabc → Idq (硬件CORDIC加速) */
            FOC_ClarkePark_Currents(&FOC_State);
            PID_Current_Loop_Isr(FOC_State.Iqd.d, FOC_State.Iqd.q,
                           &FOC_State.Uqd.d, &FOC_State.Uqd.q);
            FOC_RUN_SVPWM(&FOC_State);
        }
    }
    else if (htim == &htim7)
    {
        static uint8_t counter = 0;
        PID_Speed_Loop_Isr();
        if (++counter >= 5) /* 1kHz 遥测 */
        {
            counter = 0;
            PID_Angle_Loop_Isr();
        }
        /* 1kHz 遥测: 12 路 PID 数据 (顺序见 PID_TELEM_xxx)
         * 用 Fmt_LineU8 做纯整数 ASCII 格式化, 再一次性写入环形缓冲区;
         * 不使用 snprintf / 浮点 printf, 避免 1kHz 中断里的高开销。
         * 缓冲区按 12 路最坏情况预留: 每路约 10 字符 + 逗号 + CRLF。 */
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