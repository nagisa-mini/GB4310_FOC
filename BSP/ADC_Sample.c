#include "ADC_Sample.h"

/*==============================================================================
 * 模块内部状态
 *============================================================================*/

/* 工作模式 (ADC_State.MODE) */
enum
{
    ADC_MODE_IDLE  = 0,                 /* 空闲: 中断回调不更新数据 */
    ADC_MODE_CALIB = 1,                 /* 零点校准 */
    ADC_MODE_RUN   = 2,                 /* 正常采样 */
};

/* 采样状态 (仅本模块使用) */
static ADC_State_t ADC_State =
{
    .MODE = ADC_MODE_IDLE,
};

static uint16_t ADC_Buf[ADC_BUF_SIZE];

/*==============================================================================
 * 接口函数
 *============================================================================*/

/**
 * @brief  电流零点校准 (阻塞)
 * @note   注入组累计 ZERO_CALIBRATE_SIZE 次采样后求平均并回到空闲模式;
 *         250 次 @20kHz PWM 约 12.5 ms, 故等待 100 ms 留足余量。
 *         校准结束后需重新调用 ADC_Sample_Start() 才能继续采样。
 */
void ADC_CalibrateZero(void)
{
    ADC_State.MODE = ADC_MODE_CALIB;
    HAL_Delay(100);
}

/**
 * @brief  初始化 ADC 采样
 * @note   顺序: ADC 自校准 → 规则组 DMA (Vbus/VREFINT) → 注入组中断 (Ia/Ic)
 *         → 电流零点校准; 先空跑一次规则采样取得实测 Vunit 供电流换算。
 */
void ADC_Sample_Init(void)
{
    ADC_State.MODE = ADC_MODE_IDLE;

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_Buf[ADC_BUF_VBUS], 2U);
    HAL_TIM_Base_Start_IT(&htim7);                  /* TIM7 TRGO 触发规则组 */

    /* 空跑一次规则采样, 取得 VDDA 实测值 Vunit */
    ADC_State.MODE = ADC_MODE_RUN;
    HAL_Delay(2);
    ADC_State.MODE = ADC_MODE_IDLE;

    HAL_ADCEx_InjectedStart_IT(&hadc1);             /* TIM1 CC4 触发注入组 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, INJ_TRIG_CC4);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

    ADC_CalibrateZero();
}

/**
 * @brief  进入正常采样模式
 */
void ADC_Sample_Start(void)
{
    ADC_State.MODE = ADC_MODE_RUN;
}

/**
 * @brief  停止采样, 进入空闲模式 (中断回调不再更新任何数据)
 */
void ADC_Sample_Stop(void)
{
    ADC_State.MODE = ADC_MODE_IDLE;
}

/*==============================================================================
 * 中断回调
 *============================================================================*/

/**
 * @brief  规则组转换完成回调 (TIM7 TRGO 触发, 1 kHz)
 * @note   由 VREFINT 实测值反推单位 ADC 码电压 Vunit, 再换算母线电压 Vbus,
 *         并输出 1/Vunit 到 DAC 供外部电路做增益补偿。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1 || ADC_State.MODE != ADC_MODE_RUN)
    {
        return;
    }

    /* Vunit = 校准电压 × VREFINT_CAL / 实测码 / 满量程 */
    ADC_State.Vunit = VDDA_CAL_VOLTAGE * (*VREFINT_CAL_ADDR) / ADC_Buf[ADC_BUF_VREFINT] / ADC_FULL_SCALE;
    ADC_State.Vbus  = ADC_Buf[ADC_BUF_VBUS] * ADC_State.Vunit * VBUS_DIVIDER;

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(1.0f / ADC_State.Vunit));
}

/**
 * @brief  注入组转换完成回调 (TIM1 CC4 触发, 20 kHz)
 * @note   校准模式: 累计 ZERO_CALIBRATE_SIZE 次采样求电流零点;
 *         运行模式: 换算 Ia/Ic, 三相无中线由 Ib = -(Ia + Ic) 得到 B 相电流。
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static uint32_t Calibrate_SumA = 0, Calibrate_SumC = 0;
    static float    Calibrate_OffsetA = 0, Calibrate_OffsetC = 0;   /* 电流零点 */
    static uint8_t  Calibrate_Count = 0;

    if (hadc != &hadc1)
    {
        return;
    }

    if (ADC_State.MODE == ADC_MODE_CALIB)
    {
        Calibrate_SumA += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        Calibrate_SumC += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

        if (++Calibrate_Count >= ZERO_CALIBRATE_SIZE)
        {
            Calibrate_OffsetA = (float)Calibrate_SumA / (float)ZERO_CALIBRATE_SIZE;
            Calibrate_OffsetC = (float)Calibrate_SumC / (float)ZERO_CALIBRATE_SIZE;
            Calibrate_Count   = 0;
            Calibrate_SumA    = 0;
            Calibrate_SumC    = 0;
            ADC_State.MODE    = ADC_MODE_IDLE;      /* 校准完成, 需重新 Start */
        }
    }
    else if (ADC_State.MODE == ADC_MODE_RUN)
    {
        ADC_Buf[ADC_BUF_IA] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        ADC_Buf[ADC_BUF_IC] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

        /* I = (码值 - 零点) × 单位码电压 / 运放增益 / 采样电阻 */
        ADC_State.Ia = (ADC_Buf[ADC_BUF_IA] - Calibrate_OffsetA) * ADC_State.Vunit
                       / CURR_AMP_GAIN / CURR_SHUNT;
        ADC_State.Ic = (ADC_Buf[ADC_BUF_IC] - Calibrate_OffsetC) * ADC_State.Vunit
                       / CURR_AMP_GAIN / CURR_SHUNT;
        ADC_State.Ib = -(ADC_State.Ia + ADC_State.Ic);  /* 三相无中线: Ib = -(Ia + Ic) */
    }
}

/*==============================================================================
 * 数据读取
 *============================================================================*/

/**
 * @brief  一次读取三相电流 (三相值由同一帧注入采样更新)
 */
void ADC_GetCurrents(float *Ia, float *Ib, float *Ic)
{
    *Ia = ADC_State.Ia;
    *Ib = ADC_State.Ib;
    *Ic = ADC_State.Ic;
}

/**
 * @brief  读取单通道采样数据
 * @param  cmd 通道选择位: ADC_CMD_IA / IB / IC / VBUS / VUNIT
 * @retval 通道数值 (电流 A, 电压 V); cmd 无效时返回 0
 * @note   三相电流建议改用 ADC_GetCurrents(), 一次取齐。
 */
float ADC_GetData(uint8_t cmd)
{
    float value = 0.0f;

    if (cmd & (ADC_CMD_IA | ADC_CMD_IB | ADC_CMD_IC))
    {
        if      (cmd == ADC_CMD_IA) value = ADC_State.Ia;
        else if (cmd == ADC_CMD_IB) value = ADC_State.Ib;
        else if (cmd == ADC_CMD_IC) value = ADC_State.Ic;
    }
    else if (cmd & (ADC_CMD_VBUS | ADC_CMD_VUNIT))
    {
        if      (cmd == ADC_CMD_VBUS)  value = ADC_State.Vbus;
        else if (cmd == ADC_CMD_VUNIT) value = ADC_State.Vunit;
    }

    return value;
}