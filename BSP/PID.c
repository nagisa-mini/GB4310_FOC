#include "PID.h"

#include <stddef.h>
#include <string.h>

/*==============================================================================
 * 三环增益 (单位与各环输入输出一致, 按电机实测整定)
 *   角度环:   输入 rad,     输出 RPM
 *   角速度环: 输入 RPM, 输出 A
 *   电流环:   输入 A,       输出 V
 *============================================================================*/
/* 角度环 (200Hz) */
#define LOOP_ANGLE_KP        250.0f
#define LOOP_ANGLE_KI        0.0f
#define LOOP_ANGLE_KD        0.0f
#define LOOP_ANGLE_I_LIMIT   504.0f     /* 积分限幅 */
#define LOOP_ANGLE_OUT_LIMIT 504.0f     /* 输出限幅: RPM (角速度给定上限) */

/* 角速度环 (1kHz)
 * 注意: 15 位编码器 @1ms 测速的量化约为 1.83 RPM/LSB。
 *       速度环 Kp 过大 (实测 0.017 起) 会在 Speed=0 附近形成 1kHz 极限环:
 *       每个编码器 LSB 被放大成约 Kp×1.83 A 的电流脉冲, 零速时又没有
 *       反电动势阻尼, 表现为电机高频抖动/异响、Speed_Fdb 在 0 附近跳变。
 *       建议 Kp <= 0.01, Ki=0 起步; 需要更高刚度时先做测速滤波/加长测速窗口。
 */
#define LOOP_SPEED_KP        0.0165f
#define LOOP_SPEED_KI        1.0f
#define LOOP_SPEED_KD        0.0f
#define LOOP_SPEED_I_LIMIT   1.27f       /* 积分限幅 */
#define LOOP_SPEED_OUT_LIMIT 1.27f       /* 输出限幅: A (q轴电流给定上限) */

/* 电流环 (20kHz), d/q 两轴共用同一组增益 */
#define LOOP_CURR_KP         7.17392f
#define LOOP_CURR_KI         0.77768f
#define LOOP_CURR_KD         0.0f
#define LOOP_CURR_I_LIMIT    24.0f      /* 积分限幅 */
#define LOOP_CURR_OUT_LIMIT  24.0f      /* 输出限幅: V (电压给定上限) */

/*==============================================================================
 * 三环控制器实例 (仅本模块使用)
 *============================================================================*/
PID_t PID_Angle;     /* 角度环 */
PID_t PID_Speed;     /* 角速度环 */
PID_t PID_Curr_D;    /* 电流环 d 轴 */
PID_t PID_Curr_Q;    /* 电流环 q 轴 */
PID_RUN_t PID_Run;       /* 三环运行状态 (给定/反馈/模式) */

/*==============================================================================
 * 内部函数
 *============================================================================*/

/**
  * @brief  按环编号取控制器句柄
  * @param  loop 环编号 (仅角度环/角速度环支持在线整定)
  * @retval 控制器指针; 参数非法返回 NULL
  */
static PID_t *PID_GetLoop(PID_Loop_t loop)
{
    if (loop == PID_LOOP_ANGLE)
    {
        return &PID_Angle;
    }
    if (loop == PID_LOOP_SPEED)
    {
        return &PID_Speed;
    }
    return NULL;
}

/**
  * @brief  限幅
  * @param  val 输入值
  * @param  limit 绝对值上限 (要求 >= 0)
  * @retval 限幅后的值
  */
static float PID_Clamp(float val, float limit)
{
    if (val > limit)
    {
        return limit;
    }
    if (val < -limit)
    {
        return -limit;
    }
    return val;
}

/*==============================================================================
 * 公共函数
 *============================================================================*/

/**
  * @brief  初始化 PID 控制器
  * @param  pid 控制器句柄
  * @param  Kp/Ki/Kd 比例/积分/微分系数
  * @param  I_limit 积分限幅 (绝对值)
  * @param  Out_limit 输出限幅 (绝对值)
  * @retval None
  */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float I_limit, float Out_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->I_limit = I_limit;
    pid->Out_limit = Out_limit;
    PID_Reset(pid);
}

/**
  * @brief  清空控制器状态 (误差/积分/输出归零)
  * @param  pid 控制器句柄
  * @retval None
  */
void PID_Reset(PID_t *pid)
{
    pid->Ref = 0.0f;
    pid->Fdb = 0.0f;
    pid->Err = 0.0f;
    pid->Err_last = 0.0f;
    pid->Integral = 0.0f;
    pid->Out = 0.0f;
}

/**
  * @brief  角度误差计算 (取跨零点最短路径)
  * @note   |ref − fdb| > π 时沿另一方向旋转
  * @param  ref 给定角度 (rad)
  * @param  fdb 反馈角度 (rad)
  * @retval 误差, 范围 (−π, π]
  */
float PID_Angle_Err(float ref, float fdb)
{
    float err = ref - fdb;

    if (err > PID_PI)
    {
        err -= PID_2PI;
    }
    else if (err < -PID_PI)
    {
        err += PID_2PI;
    }

    return err;
}

/**
  * @brief  单环 PID 运算 (位置式)
  * @note   积分项限幅, 输出限幅; ts 为本次与上次运算的间隔 (s)
  * @param  pid 控制器句柄
  * @param  ref 给定值
  * @param  fdb 反馈值
  * @param  ts  运算周期 (s)
  * @retval 输出值
  */
float PID_Calc(PID_t *pid, float ref, float fdb, float ts)
{
    pid->Ref = ref;
    pid->Fdb = fdb;
    pid->Err = ref - fdb;

    /* 积分累加并限幅 */
    pid->Integral += pid->Ki * pid->Err * ts;
    pid->Integral = PID_Clamp(pid->Integral, pid->I_limit);

    /* 位置式输出: 比例 + 积分 + 微分 */
    pid->Out = pid->Kp * pid->Err
             + pid->Integral
             + pid->Kd * (pid->Err - pid->Err_last) * (1.0f / ts);

    /* 输出限幅 */
    pid->Out = PID_Clamp(pid->Out, pid->Out_limit);

    pid->Err_last = pid->Err;

    return pid->Out;
}

/**
  * @brief  三环闭环参数初始化
  * @note   上电或模式切换时调用一次, 各环状态清零
  * @retval None
  */
void PID_3Loop_Init(void)
{
    /* 角度环: rad → rad/min, 200Hz */
    PID_Init(&PID_Angle, LOOP_ANGLE_KP, LOOP_ANGLE_KI, LOOP_ANGLE_KD,
             LOOP_ANGLE_I_LIMIT, LOOP_ANGLE_OUT_LIMIT);

    /* 角速度环: rad/min → A, 1kHz */
    PID_Init(&PID_Speed, LOOP_SPEED_KP, LOOP_SPEED_KI, LOOP_SPEED_KD,
             LOOP_SPEED_I_LIMIT, LOOP_SPEED_OUT_LIMIT);

    /* 电流环 d 轴: A → V, 20kHz, d 轴给定恒为 0 */
    PID_Init(&PID_Curr_D, LOOP_CURR_KP, LOOP_CURR_KI, LOOP_CURR_KD,
             LOOP_CURR_I_LIMIT, LOOP_CURR_OUT_LIMIT);

    /* 电流环 q 轴: A → V, 20kHz */
    PID_Init(&PID_Curr_Q, LOOP_CURR_KP, LOOP_CURR_KI, LOOP_CURR_KD,
             LOOP_CURR_I_LIMIT, LOOP_CURR_OUT_LIMIT);

    PID_Run.mode = PID_MODE_STOP; /* 默认模式 */
    PID_Run.Angle_Ref = 0.0f;
    PID_Run.Angle_Fdb = 0.0f;
    PID_Run.Speed_Ref = 0.0f;
    PID_Run.Speed_Fdb = 0.0f;
    PID_Run.Curr_Q_Ref = 0.0f;

}

/*==============================================================================
 * 上位机在线整定接口
 *============================================================================*/

/**
  * @brief  写一组 PID 增益
  * @note   三个 float 在同一短临界区内写入, 避免 1kHz 速度环看到半套新增益;
  *         不修改积分/误差状态, 因此在线改参不会造成积分跳变。
  * @param  loop 环编号
  * @param  Kp/Ki/Kd 比例/积分/微分系数
  * @retval true 成功; false 环编号非法
  */
bool PID_SetGains(PID_Loop_t loop, float Kp, float Ki, float Kd)
{
    PID_t *pid = PID_GetLoop(loop);

    if (pid == NULL)
    {
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    __set_PRIMASK(primask);

    return true;
}

/**
  * @brief  读一组 PID 增益
  * @param  loop 环编号
  * @param  Kp/Ki/Kd [输出] 比例/积分/微分系数
  * @retval true 成功; false 参数非法
  */
bool PID_GetGains(PID_Loop_t loop, float *Kp, float *Ki, float *Kd)
{
    PID_t *pid = PID_GetLoop(loop);

    if ((pid == NULL) || (Kp == NULL) || (Ki == NULL) || (Kd == NULL))
    {
        return false;
    }

    *Kp = pid->Kp;
    *Ki = pid->Ki;
    *Kd = pid->Kd;

    return true;
}

/**
  * @brief  清空四个 PID 的积分项
  * @note   上位机重新设定 Angle/Speed 给定后调用, 避免旧的积分量
  *         继续抵抗新的给定 (包括 20kHz 电流环 d/q 积分)。
  *         只清 Integral, 不清 Kp/Ki/Kd、Err_last 和 Out。
  *         在短临界区内完成, 防止与 1kHz/20kHz 中断竞争。
  * @param  None
  * @retval None
  */
void PID_ResetIntegralAll(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    PID_Angle.Integral = 0.0f;
    PID_Speed.Integral = 0.0f;
    PID_Curr_D.Integral = 0.0f;
    PID_Curr_Q.Integral = 0.0f;

    __set_PRIMASK(primask);
}

/**
  * @brief  切换三环运行模式
  * @note   模式变化时先在临界区内停环并清空各 PID 状态, 再写入新模式,
  *         避免切换瞬间带着旧的积分/输出继续运算。
  *         切到角度环时把 Speed_Ref 清零, 防止角度环 5ms 更新前
  *         速度环先按旧给定动作。
  * @param  mode PID_MODE_STOP / PID_MODE_ANGLE / PID_MODE_SPEED
  * @retval true 成功; false 模式非法
  */
bool PID_SetMode(uint8_t mode)
{
    if (mode > PID_MODE_SPEED)
    {
        return false;
    }
    if (mode == PID_Run.mode)
    {
        return true;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* 先停环, 让三个中断服务程序不再访问 PID 状态, 再安全复位 */
    PID_Run.mode = PID_MODE_STOP;
    PID_Reset(&PID_Angle);
    PID_Reset(&PID_Speed);
    PID_Reset(&PID_Curr_D);
    PID_Reset(&PID_Curr_Q);
    PID_Run.Curr_Q_Ref = 0.0f;
    if (mode == PID_MODE_ANGLE)
    {
        PID_Run.Speed_Ref = 0.0f;
    }
    PID_Run.mode = mode;

    __set_PRIMASK(primask);

    return true;
}

/*==============================================================================
 * 角度环 (外环) —— TIM7 中断, 每 5 拍执行一次 (200Hz)
 *============================================================================*/

/**
  * @brief  角度环运算
  * @note   调用周期 5ms; 结果缓存为角速度环给定, 1kHz 的角速度环取用
  *         TIM7 中断里示例: if (++cnt >= PID_ANGLE_DIV) { cnt = 0; PID_Angle_Loop_Isr(...); }
  * @param  angle_ref 角度给定 (rad)
  * @param  angle_fdb 角度反馈 (rad, 机械角)
  * @retval None
  */
void PID_Angle_Loop_Isr(void)
{
  if (PID_Run.mode == PID_MODE_ANGLE)
  {
    PID_Run.Angle_Fdb = MT6826S_GetData(MT6826S_CMD_ANGLE)*DEG_TO_RAD; /* 读取机械角, 供角度环运算使用 */
    /* 角度误差取跨零点最短路径, 再按普通单环运算 */
    float err = PID_Angle_Err(PID_Run.Angle_Ref, PID_Run.Angle_Fdb);
    PID_Run.Speed_Ref = PID_Calc(&PID_Angle, err, 0.0f, PID_ANGLE_TS);
  }
}

/*==============================================================================
 * 角速度环 (中环) —— TIM7 中断, 每拍执行 (1kHz)
 *============================================================================*/

/**
  * @brief  由机械角差分计算机械轴转速
  * @note   1kHz 周期调用; 差分取跨零点最短路径, 输出单位 rpm (转/分)
  *         rpm = delta(rad) / PID_SPEED_TS(s) * PID_RAD2RPM
  * @param  None
  * @retval 机械轴转速 (rpm)
  */
float AngleSpeed_Get(void)
{
    static float angle_fdb_last,angle_fdb = 0.0f;
    float speed_fdb;
    angle_fdb_last=angle_fdb;
    angle_fdb=MT6826S_GetData(MT6826S_CMD_ANGLE)*DEG_TO_RAD;
    speed_fdb=PID_Angle_Err(angle_fdb,angle_fdb_last)*PID_RAD2RPM/PID_SPEED_TS; /* 由机械角差分计算轴转速 (rpm) */
    return speed_fdb;
}

/**
  * @brief  角速度环运算
  * @note   调用周期 1ms; 给定取自角度环 (5ms 更新一次, 中间 4 拍保持不变)
  * @param  speed_fdb 角速度反馈 (RPM)
  * @retval q 轴电流给定 (A)
  */
void PID_Speed_Loop_Isr(void)
{
  PID_Run.Speed_Fdb = AngleSpeed_Get();
  if (PID_Run.mode != PID_MODE_STOP)
  {
    PID_Run.Curr_Q_Ref = PID_Calc(&PID_Speed, PID_Run.Speed_Ref, PID_Run.Speed_Fdb, PID_SPEED_TS);
  }
}

/*==============================================================================
 * 电流环 (内环) —— TIM1 中断, 与 PWM/电流采样同频 (20kHz)
 *   时序: ADC 电流采样 → Clarke/Park 得 Idq → 本函数算 Udq → SVPWM 输出
 *============================================================================*/

/**
  * @brief  电流环运算 (d/q 双轴)
  * @note   d 轴给定固定为 0; 应在 SVPWM 之前调用, 反馈用本次采样刚算出的 Idq
  * @param  iq_ref q 轴电流给定 (A), 来自角速度环
  * @param  id_fdb d 轴电流反馈 (A)
  * @param  iq_fdb q 轴电流反馈 (A)
  * @param  vd_out 输出: d 轴电压给定 (V)
  * @param  vq_out 输出: q 轴电压给定 (V)
  * @retval None
  */
void PID_Current_Loop_Isr(float id_fdb, float iq_fdb,
                          float *vd_out, float *vq_out)
{
  if (PID_Run.mode != PID_MODE_STOP)
  {
    *vd_out = PID_Calc(&PID_Curr_D, 0.0f, id_fdb, PID_CURR_TS);
    *vq_out = PID_Calc(&PID_Curr_Q, PID_Run.Curr_Q_Ref, iq_fdb, PID_CURR_TS);
  }
  else
  {
    /* 停止模式必须显式输出零电压, 否则 SVPWM 会保持上一次的 Udq */
    *vd_out = 0.0f;
    *vq_out = 0.0f;
  }
}

/*==============================================================================
 * 参数掉电保存 —— Flash 最后一页
 *
 * 存储布局 (40 字节, 5 个 doubleword):
 *   magic + version + size + 6 个增益 + crc32 + reserved
 *
 * 链接脚本已把 FLASH 长度改为 126K, 0x0801F800 这一页不参与程序链接。
 * 只保存角度环/角速度环的 Kp/Ki/Kd; 模式和给定不保存。
 *============================================================================*/
#define PID_CONFIG_FLASH_ADDR   (FLASH_BASE + 0x1F800UL)
#define PID_CONFIG_FLASH_PAGE   ((PID_CONFIG_FLASH_ADDR - FLASH_BASE) / 0x800UL)
#define PID_CONFIG_MAGIC        0x50494431UL      /* "PID1" */
#define PID_CONFIG_VERSION      1U
#define PID_CONFIG_GAIN_MAX     1000000.0f

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float angle_kp;
    float angle_ki;
    float angle_kd;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    uint32_t crc;
    uint32_t reserved;
} PID_Config_t;

_Static_assert((sizeof(PID_Config_t) % 8U) == 0U,
               "PID_Config_t size must be a multiple of 8 bytes");

static bool s_pid_config_loaded = false;

/**
  * @brief  CRC32 (IEEE 802.3, 反射多项式 0xEDB88320)
  */
static uint32_t PID_Config_Crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint32_t i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (uint32_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1UL) != 0UL)
            {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

/**
  * @brief  增益合法性检查 (同时排除 NaN / Inf)
  */
static bool PID_Config_GainValid(float value)
{
    return ((value >= 0.0f) && (value <= PID_CONFIG_GAIN_MAX)) ? true : false;
}

/**
  * @brief  从 Flash 读取并校验一条记录
  * @retval true 记录有效
  */
static bool PID_Config_Read(PID_Config_t *cfg)
{
    if (cfg == NULL)
    {
        return false;
    }

    memcpy(cfg, (const void *)PID_CONFIG_FLASH_ADDR, sizeof(*cfg));

    if ((cfg->magic != PID_CONFIG_MAGIC) ||
        (cfg->version != PID_CONFIG_VERSION) ||
        (cfg->size != (uint16_t)sizeof(PID_Config_t)))
    {
        return false;
    }

    if (cfg->crc != PID_Config_Crc32((const uint8_t *)cfg,
                                     (uint32_t)offsetof(PID_Config_t, crc)))
    {
        return false;
    }

    return (PID_Config_GainValid(cfg->angle_kp) &&
            PID_Config_GainValid(cfg->angle_ki) &&
            PID_Config_GainValid(cfg->angle_kd) &&
            PID_Config_GainValid(cfg->speed_kp) &&
            PID_Config_GainValid(cfg->speed_ki) &&
            PID_Config_GainValid(cfg->speed_kd));
}

/**
  * @brief  把当前 6 个增益写入 Flash 最后一页
  * @note   页面擦除期间 CPU 取指会被 Flash 忙状态阻塞约几十毫秒,
  *         因此只允许在 STOP 模式 (零电压矢量) 下保存。
  * @retval true 写入并回读校验成功
  */
bool PID_Config_Save(void)
{
    if (PID_Run.mode != PID_MODE_STOP)
    {
        return false;
    }

    PID_Config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = PID_CONFIG_MAGIC;
    cfg.version = PID_CONFIG_VERSION;
    cfg.size = (uint16_t)sizeof(cfg);
    cfg.reserved = 0xFFFFFFFFUL;

    if (!PID_GetGains(PID_LOOP_ANGLE, &cfg.angle_kp, &cfg.angle_ki, &cfg.angle_kd) ||
        !PID_GetGains(PID_LOOP_SPEED, &cfg.speed_kp, &cfg.speed_ki, &cfg.speed_kd))
    {
        return false;
    }

    if (!PID_Config_GainValid(cfg.angle_kp) ||
        !PID_Config_GainValid(cfg.angle_ki) ||
        !PID_Config_GainValid(cfg.angle_kd) ||
        !PID_Config_GainValid(cfg.speed_kp) ||
        !PID_Config_GainValid(cfg.speed_ki) ||
        !PID_Config_GainValid(cfg.speed_kd))
    {
        return false;
    }

    cfg.crc = PID_Config_Crc32((const uint8_t *)&cfg,
                               (uint32_t)offsetof(PID_Config_t, crc));

    uint64_t words[(sizeof(PID_Config_t) + 7U) / 8U];
    memcpy(words, &cfg, sizeof(cfg));

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef erase;
    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = PID_CONFIG_FLASH_PAGE;
    erase.NbPages = 1U;

    uint32_t page_error = 0U;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_error);
    if (status == HAL_OK)
    {
        for (uint32_t i = 0U; i < (sizeof(words) / sizeof(words[0])); i++)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                  PID_CONFIG_FLASH_ADDR + (i * 8U),
                                  words[i]) != HAL_OK)
            {
                status = HAL_ERROR;
                break;
            }
        }
    }

    (void)HAL_FLASH_Lock();

    /* HAL 在擦除/编程结束后已调用 FLASH_FlushCaches(), 直接回读校验 */
    if (status != HAL_OK)
    {
        return false;
    }

    PID_Config_t check;
    if (!PID_Config_Read(&check))
    {
        return false;
    }
    if (memcmp(&check, &cfg, sizeof(cfg)) != 0)
    {
        return false;
    }

    s_pid_config_loaded = true;
    return true;
}

/**
  * @brief  上电读取保存的增益并应用
  * @note   放在 PID_3Loop_Init() 之后调用; 没有有效记录时保留默认增益。
  * @retval true 成功加载
  */
bool PID_Config_Load(void)
{
    PID_Config_t cfg;

    s_pid_config_loaded = false;

    if (!PID_Config_Read(&cfg))
    {
        return false;
    }
    if (!PID_SetGains(PID_LOOP_ANGLE, cfg.angle_kp, cfg.angle_ki, cfg.angle_kd))
    {
        return false;
    }
    if (!PID_SetGains(PID_LOOP_SPEED, cfg.speed_kp, cfg.speed_ki, cfg.speed_kd))
    {
        return false;
    }

    s_pid_config_loaded = true;
    return true;
}

/**
  * @brief  本次上电是否成功加载了 Flash 中的参数
  */
bool PID_Config_IsLoaded(void)
{
    return s_pid_config_loaded;
}
