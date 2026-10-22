#include "MT6826S.h"

/*==============================================================================
 * 模块内部状态
 *============================================================================*/

/* 工作模式 (MT6826S_State.MODE) */
enum
{
    MT_MODE_IDLE  = 0,                  /* 空闲: 不启动角度读取 */
    MT_MODE_RUN   = 1,                  /* 运行: 由 TIM1 CC4 触发连续读取 */
    MT_MODE_CALIB = 2,                  /* 校准: 暂停读取, 供 ElAngle_Calibration 使用 */
};

/* 连续读取状态机 (Continue_ReadAngle_State) */
enum
{
    READ_STATE_IDLE = 0,                /* 空闲, 可发起新一帧 */
    READ_STATE_CMD  = 1,                /* 读命令已发出, 等待发送完成 */
    READ_STATE_DATA = 2,                /* 已启动接收, 等待角度数据 */
};

/* 角度帧格式 */
#define ANGLE_FRAME_LEN     4U          /* 角度帧长: 2 字节角度 + 1 字节保留 + 1 字节 CRC */
#define ANGLE_CRC_LEN       3U          /* CRC 校验字节数 (不含 CRC 本身) */
#define ANGLE_RAW_FULL      32768.0f    /* 15 位角度满量程 */
#define ANGLE_DEG_FULL      360.0f      /* 一圈角度 (度) */

#define MT_TRIG_CC4         4245U       /* TIM1 CC4 比较值: 载波峰值处触发 (与 ADC 注入组一致) */

/* 角度状态 (仅本模块使用) */
static MT6826S_State_t MT6826S_State =
{
    .MODE = MT_MODE_IDLE,
};

static uint8_t MT6826S_Data[ANGLE_FRAME_LEN];           /* 角度接收缓冲 (DMA 目标) */
static uint8_t MT6826S_TxBuf[2];                        /* 读命令发送缓冲 (DMA 源, 不可放在栈上) */
static uint8_t Continue_ReadAngle_State = READ_STATE_IDLE;

/*==============================================================================
 * 内部函数
 *============================================================================*/

/**
 * @brief  组装 16 位寄存器地址命令
 * @note   帧格式: [0] = 命令字 | 地址[11:8], [1] = 地址[7:0]
 */
static void MT6826S_BuildCmd(uint8_t *tx_buf, uint8_t cmd, uint16_t addr)
{
    tx_buf[0] = (uint8_t)(cmd | ((addr >> 8) & 0x0FU));
    tx_buf[1] = (uint8_t)(addr & 0xFFU);
}

/**
 * @brief  将角度归约到 [0, 2π)
 * @note   用乘法 + 截断代替 fmodf; 末尾保险减法用于吸收浮点舍入误差。
 */
static float NormalizeAngle(float angle)
{
    if (angle >= MT_2PI)
    {
        int32_t n = (int32_t)(angle * MT_INV_2PI);          /* 截断取整: angle/2π 的整数部分 */
        angle -= (float)n * MT_2PI;
        if (angle >= MT_2PI)                                /* 浮点舍入保险 */
        {
            angle -= MT_2PI;
        }
    }
    else if (angle < 0.0f)
    {
        int32_t n = (int32_t)(-angle * MT_INV_2PI) + 1;     /* ceil(-angle/2π) */
        angle += (float)n * MT_2PI;
        if (angle < 0.0f)
        {
            angle += MT_2PI;
        }
    }

    return angle;
}

/*==============================================================================
 * 接口函数
 *============================================================================*/

/**
 * @brief  初始化角度读取触发源 (TIM1 CC4)
 * @note   ADC_Sample_Init() 已完成同样的启动, 正常流程下无需重复调用。
 */
void MT6826S_Init(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MT_TRIG_CC4);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief  开始连续读取角度 (由 TIM1 CC4 中断按 PWM 周期触发)
 */
void MT6826S_Start(void)
{
    MT6826S_State.MODE = MT_MODE_RUN;
    Continue_ReadAngle_State = READ_STATE_IDLE;
}

/**
 * @brief  停止连续读取角度
 */
void MT6826S_Stop(void)
{
    MT6826S_State.MODE = MT_MODE_IDLE;
    Continue_ReadAngle_State = READ_STATE_IDLE;
}

/**
 * @brief  发起一次角度读取 (拉低片选并发送读命令)
 * @note   仅在上一次读取结束 (状态机空闲) 时发起; 发送完成后由
 *         HAL_SPI_TxCpltCallback 接力启动接收。发送缓冲为模块内静态变量,
 *         供 DMA 在函数返回后继续读取。
 */
void MT6826S_Continue_ReadAngle(void)
{
    if (Continue_ReadAngle_State != READ_STATE_IDLE)
    {
        return;
    }

    MT6826S_BuildCmd(MT6826S_TxBuf, MT6826S_CMD_READ_ANGLE, MT6826S_REG_ANGLE_H);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(&hspi3, MT6826S_TxBuf, 2);
    Continue_ReadAngle_State = READ_STATE_CMD;
}

/**
 * @brief  读取角度数据
 * @param  cmd MT6826S_CMD_ANGLE (机械角 0~360°) / MT6826S_CMD_ELANGLE (电角度 0~2π)
 * @retval 对应角度值; cmd 无效时返回 0
 */
float MT6826S_GetData(uint8_t cmd)
{
    float value = 0.0f;

    if (cmd == MT6826S_CMD_ANGLE)
    {
        value = MT6826S_State.Angle;
    }
    else if (cmd == MT6826S_CMD_ELANGLE)
    {
        value = MT6826S_State.ElAngle;
    }

    return value;
}

/*==============================================================================
 * 中断回调
 *============================================================================*/

/**
 * @brief  TIM1 CC4 比较中断回调 (载波峰值处)
 * @note   仅在计数器递减 (峰值刚过) 且处于运行模式时发起角度读取,
 *         采样点与 ADC 注入组保持一致。
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_4)
    {
        return;
    }

    if ((TIM1->CR1 & TIM_CR1_DIR) != 0 && MT6826S_State.MODE == MT_MODE_RUN)
    {
        MT6826S_Continue_ReadAngle();
    }
}

/**
 * @brief  SPI 发送完成回调 (读命令已发出) → 启动 DMA 接收
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi3 || MT6826S_State.MODE != MT_MODE_RUN || Continue_ReadAngle_State != READ_STATE_CMD)
    {
        return;
    }

    HAL_SPI_Receive_DMA(&hspi3, MT6826S_Data, ANGLE_FRAME_LEN);
    Continue_ReadAngle_State = READ_STATE_DATA;
}

/**
 * @brief  SPI 接收完成回调: 校验 CRC 并换算机械角/电角度
 * @note   校验通过才更新角度; 无论是否通过都释放片选并回到空闲状态。
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    float angle;

    if (hspi != &hspi3 || MT6826S_State.MODE != MT_MODE_RUN || Continue_ReadAngle_State != READ_STATE_DATA)
    {
        return;
    }

    if ((uint8_t)HAL_CRC_Calculate(&hcrc, (uint32_t *)MT6826S_Data, ANGLE_CRC_LEN) == MT6826S_Data[3])
    {
        /* 机械角 = ANGLE[14:0] / 32768 × 360° */
        angle = ((uint16_t)MT6826S_Data[0] << 7 | (uint16_t)MT6826S_Data[1] >> 1)
                / ANGLE_RAW_FULL * ANGLE_DEG_FULL;
        MT6826S_State.Angle   = angle;
        MT6826S_State.ElAngle = NormalizeAngle(angle * MOTOR_POLE_PAIRS * DEG_TO_RAD);
    }

    Continue_ReadAngle_State = READ_STATE_IDLE;
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}

/*==============================================================================
 * 寄存器访问 (阻塞)
 *============================================================================*/

/**
 * @brief  读单个寄存器
 */
static uint8_t MT6826S_ReadReg(uint16_t addr)
{
    uint8_t tx_cmd[2];
    uint8_t rx_data;

    MT6826S_BuildCmd(tx_cmd, MT6826S_CMD_READ_REG, addr);

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx_cmd, 2, MT6826S_SPI_TIMEOUT);
    HAL_SPI_Receive(&hspi3, &rx_data, 1, MT6826S_SPI_TIMEOUT);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

    return rx_data;
}

/**
 * @brief  写单个寄存器
 */
static void MT6826S_WriteReg(uint16_t addr, uint8_t data)
{
    uint8_t tx_buf[3];

    MT6826S_BuildCmd(tx_buf, MT6826S_CMD_WRITE_REG, addr);
    tx_buf[2] = data;

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx_buf, 3, MT6826S_SPI_TIMEOUT);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}

/**
 * @brief  读取 15 位原始机械角 (阻塞)
 * @note   分两次读寄存器: 0x003 取 ANGLE[14:7], 0x004 取左对齐的 ANGLE[6:0],
 *         合并为 15 位原始角 (0~32767)。
 */
static uint16_t MT6826S_ReadAngle(void)
{
    uint8_t tx_cmd[2];
    uint8_t rx_data;
    uint16_t angle = 0U;

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);

    MT6826S_BuildCmd(tx_cmd, MT6826S_CMD_READ_REG, MT6826S_REG_ANGLE_H);
    HAL_SPI_Transmit(&hspi3, tx_cmd, 2, MT6826S_SPI_TIMEOUT);
    HAL_SPI_Receive(&hspi3, &rx_data, 1, MT6826S_SPI_TIMEOUT);
    angle = (uint16_t)rx_data << 7;

    MT6826S_BuildCmd(tx_cmd, MT6826S_CMD_READ_REG, MT6826S_REG_ANGLE_L);
    HAL_SPI_Transmit(&hspi3, tx_cmd, 2, MT6826S_SPI_TIMEOUT);
    HAL_SPI_Receive(&hspi3, &rx_data, 1, MT6826S_SPI_TIMEOUT);
    angle |= (uint16_t)rx_data >> 1;

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

    return angle;
}

/**
 * @brief  烧录 EEPROM 并等待完成
 * @note   命令地址位全 0; 芯片以 MT6826S_EEPROM_ACK 应答后,
 *         轮询 EE_STATUS 的 EE_DONE 位直至超时。
 * @retval 0=成功, 1=芯片未确认烧录命令, 2=等待 EE_DONE 超时
 */
static uint8_t MT6826S_BurnEEPROM(void)
{
    uint8_t tx_cmd[2];
    uint8_t ack;
    uint32_t tick_start;

    tx_cmd[0] = MT6826S_CMD_BURN_EEPROM;
    tx_cmd[1] = 0x00U;

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx_cmd, 2, MT6826S_SPI_TIMEOUT);
    HAL_SPI_Receive(&hspi3, &ack, 1, MT6826S_SPI_TIMEOUT);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

    if (ack != MT6826S_EEPROM_ACK)
    {
        return 1;                       /* 芯片未确认烧录命令 */
    }

    tick_start = HAL_GetTick();
    do
    {
        if (MT6826S_ReadReg(MT6826S_REG_EE_STATUS) & MT6826S_BIT_EE_DONE)
        {
            return 0;                   /* 烧录成功 */
        }
        HAL_Delay(1);
    } while ((HAL_GetTick() - tick_start) < MT6826S_EEPROM_TIMEOUT);

    return 2;                           /* 超时 */
}

/*==============================================================================
 * 电角度校准
 *============================================================================*/

/**
 * @brief  写入 12 位零点位置并烧录
 * @note   ZERO_POS 与 Z_PUL_WID 共用一个字节, 写入前先读回并保留 Z_PUL_WID;
 *         写 0x00A 时 ZERO_POS[3:0] 位于高 4 位, 故需先左移 4 位。
 * @retval 同 MT6826S_BurnEEPROM(): 0=成功, 1=未确认, 2=超时
 */
static uint8_t MT6826S_SaveZeroPos(uint16_t raw_angle_15bit)
{
    uint16_t zero_pos_12bit;
    uint8_t  reg_00A_old;

    zero_pos_12bit = (uint16_t)(raw_angle_15bit >> 3);      /* 15 位 → 12 位: 截断低 3 位 */

    /* Step 1: 清零 ZERO_POS, 避免旧值干扰 (Z_PUL_WID 保持不变) */
    MT6826S_WriteReg(MT6826S_REG_ZERO_POS_H, 0x00U);
    reg_00A_old = MT6826S_ReadReg(MT6826S_REG_ZERO_POS_L);
    MT6826S_WriteReg(MT6826S_REG_ZERO_POS_L, reg_00A_old & MT6826S_ZERO_POS_LO_MASK);

    /* Step 2: 写入目标零点值 (读-改-写, 保留 Z_PUL_WID) */
    MT6826S_WriteReg(MT6826S_REG_ZERO_POS_H, (uint8_t)((zero_pos_12bit >> 4) & 0xFFU));
    reg_00A_old = MT6826S_ReadReg(MT6826S_REG_ZERO_POS_L);
    reg_00A_old = (uint8_t)(((zero_pos_12bit << 4) & MT6826S_ZERO_POS_HI_MASK)
                            | (reg_00A_old & MT6826S_ZERO_POS_LO_MASK));
    MT6826S_WriteReg(MT6826S_REG_ZERO_POS_L, reg_00A_old);

    /* Step 3: 烧录 EEPROM */
    return MT6826S_BurnEEPROM();
}

/**
 * @brief  电角度零点校准 (阻塞)
 * @note   流程: d 轴电压把转子对齐到电角度 0 → 清零 ZERO_POS → 等待芯片稳定
 *         (>100 ms) → 读回当前角度写入零点并烧录。全程暂停连续角度读取。
 */
void ElAngle_Calibration(void)
{
    uint16_t raw_angle = 0U;
    uint8_t  reg_00A;

    MT6826S_State.MODE = MT_MODE_CALIB;

    /* ---- 电机对齐: d 轴给电压, 转子被拉到电角度 0 ---- */
    FOC_Start();
    FOC_State.Uqd.q = 0.0f;
    FOC_State.Uqd.d = 6.0f;
    HAL_Delay(500);
    FOC_State.Uqd.q = 0.0f;
    FOC_State.Uqd.d = 0.0f;
    FOC_Stop();

    MT6826S_WriteReg(MT6826S_REG_ZERO_POS_H, 0x00U);
    reg_00A = MT6826S_ReadReg(MT6826S_REG_ZERO_POS_L);
    MT6826S_WriteReg(MT6826S_REG_ZERO_POS_L, reg_00A & MT6826S_ZERO_POS_LO_MASK);

    HAL_Delay(500);                     /* 等待芯片稳定 (datasheet: >100ms) */
    raw_angle = MT6826S_ReadAngle();
    (void)MT6826S_SaveZeroPos(raw_angle);

    MT6826S_State.MODE = MT_MODE_IDLE;
}