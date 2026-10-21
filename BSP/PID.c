#include "PID.h"

#include "MT6826S.h"    /* DEG_TO_RAD / MT6826S_GetData */

#include <stddef.h>
#include <string.h>

/*==============================================================================
 * 三环默认增益 (按电机实测整定, 单位与各环输入输出一致)
 *   角度环:   rad → RPM
 *   角速度环: RPM → A
 *   电流环:   A   → V
 *============================================================================*/
/* 角度环 (200 Hz) */
#define LOOP_ANGLE_KP        250.0f
#define LOOP_ANGLE_KI        0.0f
#define LOOP_ANGLE_KD        0.0f
#define LOOP_ANGLE_I_LIMIT   504.0f       /* 积分限幅 */
#define LOOP_ANGLE_OUT_LIMIT 504.0f       /* 输出限幅 (RPM, 角速度给定上限) */

/* 角速度环 (1 kHz)
 * 注意: 15 位编码器 @1ms 测速的量化约为 1.83 RPM/LSB。Kp 过大 (实测 0.017 起)
 *       会在零速附近形成 1 kHz 极限环: 每个编码器 LSB 被放大成约 Kp×1.83 A
 *       的电流脉冲, 零速时又没有反电动势阻尼, 表现为高频抖动/异响。
 *       建议 Kp <= 0.01、Ki=0 起步; 需要更高刚度时先做测速滤波/加长测速窗口。
 */
#define LOOP_SPEED_KP        0.0165f
#define LOOP_SPEED_KI        1.0f
#define LOOP_SPEED_KD        0.0f
#define LOOP_SPEED_I_LIMIT   1.27f        /* 积分限幅 */
#define LOOP_SPEED_OUT_LIMIT 1.27f        /* 输出限幅 (A, q 轴电流给定上限) */

/* 电流环 (20 kHz): d/q 两轴共用同一组增益 */
#define LOOP_CURR_KP         7.17392f
#define LOOP_CURR_KI         0.77768f
#define LOOP_CURR_KD         0.0f
#define LOOP_CURR_I_LIMIT    24.0f        /* 积分限幅 */
#define LOOP_CURR_OUT_LIMIT  24.0f        /* 输出限幅 (V, 电压给定上限) */

/*==============================================================================
 * 模块内部状态 (均不对外暴露)
 *============================================================================*/

/* 通用位置式 PID 控制器 */
typedef struct {
    float Kp, Ki, Kd;       /* 比例/积分/微分系数 */
    float I_limit;          /* 积分限幅 (绝对值) */
    float Out_limit;        /* 输出限幅 (绝对值) */

    float Ref, Fdb;         /* 给定/反馈 */
    float Err, Err_last;    /* 本次/上次误差 (上次误差用于微分) */
    float Integral;         /* 积分累加项 */
    float Out;              /* 输出 */
} PID_t;

/* 三环运行状态 (给定/反馈/模式) */
typedef struct {
    float Angle_Ref;        /* 角度给定 (rad) */
    float Angle_Fdb;        /* 角度反馈 (rad) */
    float Speed_Ref;        /* 角速度给定 (RPM) */
    float Speed_Fdb;        /* 角速度反馈 (RPM) */
    float Curr_Q_Ref;       /* q 轴电流给定 (A) */
    float Curr_D_Ref;       /* d 轴电流给定 (A) */
    uint8_t mode;           /* 运行模式 (PID_MODE_xxx) */
} PID_RUN_t;

static PID_t PID_Angle;     /* 角度环 */
static PID_t PID_Speed;     /* 角速度环 */
static PID_t PID_Curr_D;    /* 电流环 d 轴 */
static PID_t PID_Curr_Q;    /* 电流环 q 轴 */
static PID_RUN_t PID_Run;   /* 三环运行状态 */

/*==============================================================================
 * 内部函数
 *============================================================================*/

/**
 * @brief  限幅 (绝对值上限, 要求 limit >= 0)
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

/**
 * @brief  按环编号取控制器句柄
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
    if (loop == PID_LOOP_CURRENT)
    {
        /* d/q 两轴共用同一组增益, 以 q 轴控制器作为读写代表 */
        return &PID_Curr_Q;
    }
    return NULL;
}

/**
 * @brief  初始化控制器: 写增益与限幅, 并清空状态
 */
static void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
                     float I_limit, float Out_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->I_limit = I_limit;
    pid->Out_limit = Out_limit;

    pid->Ref = 0.0f;
    pid->Fdb = 0.0f;
    pid->Err = 0.0f;
    pid->Err_last = 0.0f;
    pid->Integral = 0.0f;
    pid->Out = 0.0f;
}

/**
 * @brief  清空控制器状态 (误差/积分/输出归零, 保留增益)
 */
static void PID_Reset(PID_t *pid)
{
    pid->Ref = 0.0f;
    pid->Fdb = 0.0f;
    pid->Err = 0.0f;
    pid->Err_last = 0.0f;
    pid->Integral = 0.0f;
    pid->Out = 0.0f;
}

/**
 * @brief  清四个积分项 (调用者需自行关中断)
 */
static void PID_ClearIntegralAll(void)
{
    PID_Angle.Integral = 0.0f;
    PID_Speed.Integral = 0.0f;
    PID_Curr_D.Integral = 0.0f;
    PID_Curr_Q.Integral = 0.0f;
}

/**
 * @brief  角度误差: 跨零点取最短路径
 * @retval 误差, 范围 (−π, π]
 */
static float PID_Angle_Err(float ref, float fdb)
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
 * @brief  单环位置式 PID 运算 (积分限幅 + 输出限幅)
 * @param  ts 运算周期 (s)
 * @retval 限幅后的输出
 */
static float PID_Calc(PID_t *pid, float ref, float fdb, float ts)
{
    pid->Ref = ref;
    pid->Fdb = fdb;
    pid->Err = ref - fdb;

    pid->Integral += pid->Ki * pid->Err * ts;
    pid->Integral = PID_Clamp(pid->Integral, pid->I_limit);

    pid->Out = pid->Kp * pid->Err
             + pid->Integral
             + pid->Kd * (pid->Err - pid->Err_last) * (1.0f / ts);
    pid->Out = PID_Clamp(pid->Out, pid->Out_limit);

    pid->Err_last = pid->Err;
    return pid->Out;
}

/**
 * @brief  由机械角差分计算转速, 1 kHz 调用
 * @retval 机械轴转速 (RPM)
 */
static float PID_AngleSpeed_Get(void)
{
    static float angle_last = 0.0f;
    static float angle_now = 0.0f;

    angle_last = angle_now;
    angle_now = MT6826S_GetData(MT6826S_CMD_ANGLE) * DEG_TO_RAD;

    return PID_Angle_Err(angle_now, angle_last) * PID_RAD2RPM / PID_SPEED_TS;
}

/*==============================================================================
 * 初始化与运行状态接口
 *============================================================================*/

void PID_3Loop_Init(void)
{
    PID_Init(&PID_Angle, LOOP_ANGLE_KP, LOOP_ANGLE_KI, LOOP_ANGLE_KD,
             LOOP_ANGLE_I_LIMIT, LOOP_ANGLE_OUT_LIMIT);
    PID_Init(&PID_Speed, LOOP_SPEED_KP, LOOP_SPEED_KI, LOOP_SPEED_KD,
             LOOP_SPEED_I_LIMIT, LOOP_SPEED_OUT_LIMIT);
    PID_Init(&PID_Curr_D, LOOP_CURR_KP, LOOP_CURR_KI, LOOP_CURR_KD,
             LOOP_CURR_I_LIMIT, LOOP_CURR_OUT_LIMIT);
    PID_Init(&PID_Curr_Q, LOOP_CURR_KP, LOOP_CURR_KI, LOOP_CURR_KD,
             LOOP_CURR_I_LIMIT, LOOP_CURR_OUT_LIMIT);

    PID_Run.mode = PID_MODE_STOP;
    PID_Run.Angle_Ref = 0.0f;
    PID_Run.Angle_Fdb = 0.0f;
    PID_Run.Speed_Ref = 0.0f;
    PID_Run.Speed_Fdb = 0.0f;
    PID_Run.Curr_Q_Ref = 0.0f;
    PID_Run.Curr_D_Ref = 0.0f;
}

uint8_t PID_GetMode(void)
{
    return PID_Run.mode;
}

/**
 * @brief  写角度给定; 给定值实际变化时清四个积分项
 * @param  rad 角度给定 (rad); 调用者负责折算到 (−π, π]
 * @note   写入与清积分在同一临界区内完成, 避免中断看到新给定配旧积分
 */
void PID_SetAngleRef(float rad)
{
    if (PID_Run.Angle_Ref == rad)
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    PID_Run.Angle_Ref = rad;
    PID_ClearIntegralAll();
    __set_PRIMASK(primask);
}

/**
 * @brief  写角速度给定; 给定值实际变化时清四个积分项
 * @param  rpm 角速度给定 (RPM)
 */
void PID_SetSpeedRef(float rpm)
{
    if (PID_Run.Speed_Ref == rpm)
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    PID_Run.Speed_Ref = rpm;
    PID_ClearIntegralAll();
    __set_PRIMASK(primask);
}

/**
 * @brief  写 q 轴电流给定; 仅电流模式有效
 * @param  amp q 轴电流给定 (A)
 * @retval true 成功; false 当前不是电流模式或数值非法
 * @note   给定值实际变化时清空四个积分项; 重复发送相同值不清零。
 *         电流模式由 PID_SetMode(PID_MODE_CURRENT) 进入, 进入时 d/q 给定清 0。
 */
bool PID_SetCurrentRef(float amp)
{
    if (PID_Run.mode != PID_MODE_CURRENT)
    {
        return false;
    }
    if (!(amp >= -1.0e30f) || !(amp <= 1.0e30f))
    {
        return false;      /* 拒绝 NaN / Inf */
    }
    if (PID_Run.Curr_Q_Ref == amp)
    {
        return true;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    PID_Run.Curr_Q_Ref = amp;
    PID_ClearIntegralAll();
    __set_PRIMASK(primask);

    return true;
}

/**
 * @brief  写 d 轴电流给定; 仅电流模式有效
 * @param  amp d 轴电流给定 (A)
 * @retval true 成功; false 当前不是电流模式或数值非法
 * @note   给定值实际变化时清空四个积分项; 重复发送相同值不清零。
 *         电流模式由 PID_SetMode(PID_MODE_CURRENT) 进入, 进入时 d/q 给定清 0。
 */
bool PID_SetCurrentDRef(float amp)
{
    if (PID_Run.mode != PID_MODE_CURRENT)
    {
        return false;
    }
    if (!(amp >= -1.0e30f) || !(amp <= 1.0e30f))
    {
        return false;      /* 拒绝 NaN / Inf */
    }
    if (PID_Run.Curr_D_Ref == amp)
    {
        return true;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    PID_Run.Curr_D_Ref = amp;
    PID_ClearIntegralAll();
    __set_PRIMASK(primask);

    return true;
}

/**
 * @brief  取遥测快照, 按 PID_TELEM_xxx 顺序写入 values[0..PID_TELEM_COUNT-1]
 */
void PID_GetTelemetry(float *values)
{
    if (values == NULL)
    {
        return;
    }

    values[PID_TELEM_ANGLE_REF] = PID_Run.Angle_Ref;
    values[PID_TELEM_ANGLE_FDB] = PID_Run.Angle_Fdb;
    values[PID_TELEM_ANGLE_OUT] = PID_Angle.Out;
    values[PID_TELEM_SPEED_REF] = PID_Run.Speed_Ref;
    values[PID_TELEM_SPEED_FDB] = PID_Run.Speed_Fdb;
    values[PID_TELEM_SPEED_OUT] = PID_Speed.Out;
    values[PID_TELEM_CURR_Q_REF] = PID_Run.Curr_Q_Ref;
    values[PID_TELEM_CURR_Q_FDB] = PID_Curr_Q.Fdb;
    values[PID_TELEM_CURR_Q_OUT] = PID_Curr_Q.Out;
    values[PID_TELEM_CURR_D_REF] = PID_Run.Curr_D_Ref;
    values[PID_TELEM_CURR_D_FDB] = PID_Curr_D.Fdb;
    values[PID_TELEM_CURR_D_OUT] = PID_Curr_D.Out;
}

/*==============================================================================
 * 三环中断运算
 *============================================================================*/

/**
 * @brief  角度环 (200 Hz): 机械角反馈 → 角速度给定
 * @note   结果写入 Speed_Ref, 由 1 kHz 角速度环零阶保持取用
 */
void PID_Angle_Loop_Isr(void)
{
    PID_Run.Angle_Fdb = MT6826S_GetData(MT6826S_CMD_ANGLE) * DEG_TO_RAD;
    if (PID_Run.mode == PID_MODE_ANGLE)
    {
        /* 误差先取跨零点最短路径, 再按普通单环运算 */
        float err = PID_Angle_Err(PID_Run.Angle_Ref, PID_Run.Angle_Fdb);
        PID_Run.Speed_Ref = PID_Calc(&PID_Angle, err, 0.0f, PID_ANGLE_TS);
    }
}

/**
 * @brief  角速度环 (1 kHz): 角度差分测速 → q 轴电流给定
 * @note   给定取自角度环 (5 ms 更新, 中间 4 拍保持不变)
 */
void PID_Speed_Loop_Isr(void)
{
    PID_Run.Speed_Fdb = PID_AngleSpeed_Get();

    /* 角度/角速度模式才由速度环生成 q 轴电流给定;
     * 电流模式直接使用主机下发的 Curr 给定, 不能再被速度环覆盖。 */
    if ((PID_Run.mode == PID_MODE_ANGLE) || (PID_Run.mode == PID_MODE_SPEED))
    {
        PID_Run.Curr_Q_Ref = PID_Calc(&PID_Speed, PID_Run.Speed_Ref,
                                      PID_Run.Speed_Fdb, PID_SPEED_TS);
    }
}

/**
 * @brief  电流环 (20 kHz): Idq 反馈 → Udq 电压给定
 * @note   非电流模式 d 轴给定固定为 0; 电流模式 d/q 给定分别来自
 *         Curr_D_Ref/Curr_Q_Ref (主机 Id/Curr 命令)
 *         应在 ADC 采样与 Clarke/Park 变换之后、SVPWM 之前调用
 * @param  id_fdb d 轴电流反馈 (A)
 * @param  iq_fdb q 轴电流反馈 (A)
 * @param  vd_out d 轴电压给定 (V)
 * @param  vq_out q 轴电压给定 (V)
 */
void PID_Current_Loop_Isr(float id_fdb, float iq_fdb,
                          float *vd_out, float *vq_out)
{
    if (PID_Run.mode != PID_MODE_STOP)
    {
        *vd_out = PID_Calc(&PID_Curr_D, PID_Run.Curr_D_Ref, id_fdb, PID_CURR_TS);
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
 * 上位机在线整定接口
 *============================================================================*/

/**
 * @brief  写一组增益
 * @note   三个 float 在同一临界区内写入, 避免中断看到半套新增益;
 *         不修改积分/误差状态, 在线改参不会造成积分跳变。
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
    if (loop == PID_LOOP_CURRENT)
    {
        /* 电流环 d/q 必须保持同一组增益, 否则上位机只改到一个轴 */
        PID_Curr_D.Kp = Kp;
        PID_Curr_D.Ki = Ki;
        PID_Curr_D.Kd = Kd;
    }
    __set_PRIMASK(primask);

    return true;
}

/**
 * @brief  读一组增益
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
 * @brief  清空四个积分项 (临界区内完成, 不影响增益/误差/输出)
 * @note   上位机重新设定给定后由 PID_SetAngleRef/PID_SetSpeedRef 内部调用
 */
void PID_ResetIntegralAll(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    PID_ClearIntegralAll();
    __set_PRIMASK(primask);
}

/**
 * @brief  切换运行模式
 * @note   模式变化时先在临界区内停环并复位各环状态, 再写入新模式,
 *         避免切换瞬间带着旧积分/输出继续运算; 切到角度环时清 Speed_Ref,
 *         切到电流环时先清 Curr_Q_Ref/Curr_D_Ref, 由主机随后下发
 *         Curr/Id 给定。
 * @retval true 成功; false 模式非法
 */
bool PID_SetMode(uint8_t mode)
{
    if (mode > PID_MODE_CURRENT)
    {
        return false;
    }
    if (mode == PID_Run.mode)
    {
        return true;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* 先停环, 让中断服务程序不再访问 PID 状态, 再安全复位 */
    PID_Run.mode = PID_MODE_STOP;
    PID_Reset(&PID_Angle);
    PID_Reset(&PID_Speed);
    PID_Reset(&PID_Curr_D);
    PID_Reset(&PID_Curr_Q);
    PID_Run.Curr_Q_Ref = 0.0f;
    PID_Run.Curr_D_Ref = 0.0f;
    if (mode == PID_MODE_ANGLE)
    {
        PID_Run.Speed_Ref = 0.0f;
    }
    PID_Run.mode = mode;

    __set_PRIMASK(primask);

    return true;
}

/*==============================================================================
 * 参数掉电保存 —— Flash 最后一页
 *
 * 存储布局 (56 字节, 7 个 doubleword):
 *   magic + version + size + 9 个增益 + crc32 + reserved + padding
 *
 * 链接脚本已把 FLASH 长度改为 126K, 0x0801F800 这一页不参与程序链接。
 * 保存角度环/角速度环/电流环的 Kp/Ki/Kd; 模式和给定不保存。
 *============================================================================*/
#define PID_CONFIG_FLASH_ADDR   (FLASH_BASE + 0x1F800UL)
#define PID_CONFIG_FLASH_PAGE   ((PID_CONFIG_FLASH_ADDR - FLASH_BASE) / 0x800UL)
#define PID_CONFIG_MAGIC        0x50494431UL      /* "PID1" */
#define PID_CONFIG_VERSION      2U                /* v2: 增加电流环 3 个增益 */
#define PID_CONFIG_GAIN_MAX     1000000.0f

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float angle_kp;
    float angle_ki;
    float angle_kd;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float curr_kp;
    float curr_ki;
    float curr_kd;
    uint32_t crc;
    uint32_t reserved;
    uint32_t padding;
} PID_Config_t;

_Static_assert((sizeof(PID_Config_t) % 8U) == 0U,
               "PID_Config_t size must be a multiple of 8 bytes");
_Static_assert(sizeof(PID_Config_t) == 56U,
               "PID_Config_t size must be 56 bytes (v2)");
_Static_assert(offsetof(PID_Config_t, crc) == 44U,
               "PID_Config_t crc must be at offset 44");

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
            PID_Config_GainValid(cfg->speed_kd) &&
            PID_Config_GainValid(cfg->curr_kp) &&
            PID_Config_GainValid(cfg->curr_ki) &&
            PID_Config_GainValid(cfg->curr_kd));
}

/**
 * @brief  把当前 9 个增益写入 Flash 最后一页
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
    /* padding 保持 0: 避免最后一个 doubleword 全为 0xFFFFFFFF (擦除态) */
    cfg.padding = 0x00000000UL;

    if (!PID_GetGains(PID_LOOP_ANGLE, &cfg.angle_kp, &cfg.angle_ki, &cfg.angle_kd) ||
        !PID_GetGains(PID_LOOP_SPEED, &cfg.speed_kp, &cfg.speed_ki, &cfg.speed_kd) ||
        !PID_GetGains(PID_LOOP_CURRENT, &cfg.curr_kp, &cfg.curr_ki, &cfg.curr_kd))
    {
        return false;
    }

    if (!PID_Config_GainValid(cfg.angle_kp) ||
        !PID_Config_GainValid(cfg.angle_ki) ||
        !PID_Config_GainValid(cfg.angle_kd) ||
        !PID_Config_GainValid(cfg.speed_kp) ||
        !PID_Config_GainValid(cfg.speed_ki) ||
        !PID_Config_GainValid(cfg.speed_kd) ||
        !PID_Config_GainValid(cfg.curr_kp) ||
        !PID_Config_GainValid(cfg.curr_ki) ||
        !PID_Config_GainValid(cfg.curr_kd))
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
 * @brief  读取 Flash 保存的 9 个增益并应用
 * @note   上电时放在 PID_3Loop_Init() 之后调用; 没有有效记录时保留默认增益。
 *         也可由上位机 Load 命令在任意模式下调用。
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
    if (!PID_SetGains(PID_LOOP_CURRENT, cfg.curr_kp, cfg.curr_ki, cfg.curr_kd))
    {
        return false;
    }

    s_pid_config_loaded = true;
    return true;
}

/**
 * @brief  最近一次保存/加载是否成功
 */
bool PID_Config_IsLoaded(void)
{
    return s_pid_config_loaded;
}