#include "USB.h"

#include <PID.h>

RingBuffer_t USB_TxBuffer; // USB 发送环形缓冲区
RingBuffer_t USB_RxBuffer; // USB 接收环形缓冲区
static uint8_t USB_TxPool[USB_TX_BUFFER_SIZE]; // USB 发送缓冲池
static uint8_t USB_RxPool[USB_RX_BUFFER_SIZE]; // USB 接收缓冲池

/* 调试计数 */
volatile uint32_t USB_Cmd_OkCount = 0u;        // 成功执行的命令行数
volatile uint32_t USB_Cmd_ErrCount = 0u;       // 解析失败/超长/超范围的行数
volatile uint32_t USB_Rx_OverflowCount = 0u;   // USB_RxBuffer 满而丢弃的包数

/*==============================================================================
 * 命令解析状态 (仅主循环访问)
 *============================================================================*/
static char s_line[USB_CMD_LINE_MAX];          // 当前行缓存
static uint8_t s_line_len = 0u;                // 当前行长度 (不含 '\0')
static bool s_line_overflow = false;           // 当前行超长
static bool s_discard = false;                 // 丢弃到下一个换行 (溢出/复位后重新同步)
static volatile bool s_rx_overflow = false;    // 中断置位: 有包因缓冲区满被丢弃
static volatile bool s_rx_abort = false;       // USB 复位: 丢弃未完成的半行

/*==============================================================================
 * 命令表
 *============================================================================*/
typedef enum {
    USB_CMD_ID_ANGLE_KP = 0,
    USB_CMD_ID_ANGLE_KI,
    USB_CMD_ID_ANGLE_KD,
    USB_CMD_ID_SPEED_KP,
    USB_CMD_ID_SPEED_KI,
    USB_CMD_ID_SPEED_KD,
    USB_CMD_ID_CURR_KP,
    USB_CMD_ID_CURR_KI,
    USB_CMD_ID_CURR_KD,
    USB_CMD_ID_ANGLE_REF,
    USB_CMD_ID_SPEED_REF,
    USB_CMD_ID_CURR_REF,
    USB_CMD_ID_CURR_D_REF,
    USB_CMD_ID_MODE,
    USB_CMD_ID_SAVE,
    USB_CMD_ID_LOAD
} USB_CmdId_t;

typedef struct {
    const char *name;       // 小写键名 (输入键会先转小写)
    USB_CmdId_t id;
} USB_CmdEntry_t;

static const USB_CmdEntry_t s_cmd_table[] = {
    { "angle_kp",   USB_CMD_ID_ANGLE_KP  },
    { "angle_ki",   USB_CMD_ID_ANGLE_KI  },
    { "angle_kd",   USB_CMD_ID_ANGLE_KD  },
    { "speed_kp",   USB_CMD_ID_SPEED_KP  },
    { "speed_ki",   USB_CMD_ID_SPEED_KI  },
    { "speed_kd",   USB_CMD_ID_SPEED_KD  },
    { "curr_kp",    USB_CMD_ID_CURR_KP   },  // 电流环 d/q 共用
    { "curr_ki",    USB_CMD_ID_CURR_KI   },
    { "curr_kd",    USB_CMD_ID_CURR_KD   },
    { "current_kp", USB_CMD_ID_CURR_KP   },  // 别名
    { "current_ki", USB_CMD_ID_CURR_KI   },
    { "current_kd", USB_CMD_ID_CURR_KD   },
    { "angle",      USB_CMD_ID_ANGLE_REF },  // 兼容 Angle_Ref 的短名
    { "angle_ref",  USB_CMD_ID_ANGLE_REF },
    { "speed",      USB_CMD_ID_SPEED_REF },  // 上位机示例 "Speed=1.0"
    { "speed_ref",  USB_CMD_ID_SPEED_REF },
    { "curr",       USB_CMD_ID_CURR_REF  },  // 电流模式 q 轴电流给定
    { "curr_ref",   USB_CMD_ID_CURR_REF  },
    { "current",    USB_CMD_ID_CURR_REF  },
    { "current_ref", USB_CMD_ID_CURR_REF },
    { "iq",         USB_CMD_ID_CURR_REF  },
    { "iq_ref",     USB_CMD_ID_CURR_REF  },
    { "id",         USB_CMD_ID_CURR_D_REF },  // 电流模式 d 轴电流给定
    { "id_ref",     USB_CMD_ID_CURR_D_REF },
    { "curr_d",     USB_CMD_ID_CURR_D_REF },
    { "curr_d_ref", USB_CMD_ID_CURR_D_REF },
    { "current_d",  USB_CMD_ID_CURR_D_REF },
    { "current_d_ref", USB_CMD_ID_CURR_D_REF },
    { "d_curr",     USB_CMD_ID_CURR_D_REF },
    { "d_curr_ref", USB_CMD_ID_CURR_D_REF },
    { "dcurr",      USB_CMD_ID_CURR_D_REF },
    { "mode",       USB_CMD_ID_MODE      },
    { "save",       USB_CMD_ID_SAVE      },  // 无值命令: Save
    { "load",       USB_CMD_ID_LOAD      },  // 无值命令: Load
};

/*==============================================================================
 * 内部工具
 *============================================================================*/

static bool USB_IsSpace(char c)
{
    return ((c == ' ') || (c == '\t')) ? true : false;
}

/**
  * @brief 比较两个已转小写的 C 字符串
  */
static bool USB_KeyEqual(const char *key, const char *name)
{
    while ((*key != '\0') && (*name != '\0'))
    {
        if (*key != *name)
        {
            return false;
        }
        key++;
        name++;
    }
    return ((*key == '\0') && (*name == '\0')) ? true : false;
}

/**
  * @brief 解析十进制浮点 (不使用 strtof/sscanf)
  * @note  支持可选 +/-、整数/小数、前后空格; 不支持指数;
  *        总有效数字限制 9 位, 超范围/溢出/多余字符返回 false。
  */
static bool USB_ParseFloat(const char *s, float *out)
{
    if ((s == NULL) || (out == NULL))
    {
        return false;
    }

    while (USB_IsSpace(*s))
    {
        s++;
    }

    bool neg = false;
    if ((*s == '+') || (*s == '-'))
    {
        neg = (*s == '-') ? true : false;
        s++;
    }

    float value = 0.0f;
    uint32_t digits = 0u;

    while ((*s >= '0') && (*s <= '9'))
    {
        if (digits >= 9u)
        {
            return false;
        }
        value = value * 10.0f + (float)(*s - '0');
        digits++;
        s++;
    }

    if (*s == '.')
    {
        float scale = 0.1f;
        s++;
        while ((*s >= '0') && (*s <= '9'))
        {
            if (digits >= 9u)
            {
                return false;
            }
            value += (float)(*s - '0') * scale;
            scale *= 0.1f;
            digits++;
            s++;
        }
    }

    if (digits == 0u)
    {
        return false;
    }

    while (USB_IsSpace(*s))
    {
        s++;
    }
    if (*s != '\0')
    {
        return false;
    }

    /* 溢出成 Inf 时拒绝; 数字解析不会产生 NaN */
    if (!(value <= 1.0e30f))
    {
        return false;
    }

    *out = neg ? -value : value;
    return true;
}

/**
  * @brief 解析十进制非负整数 (用于 Mode)
  * @note  仅数字, 前后允许空格; 最多 9 位; 多余字符返回 false。
  */
static bool USB_ParseUint(const char *s, uint32_t *out)
{
    if ((s == NULL) || (out == NULL))
    {
        return false;
    }

    while (USB_IsSpace(*s))
    {
        s++;
    }

    if ((*s < '0') || (*s > '9'))
    {
        return false;
    }

    uint32_t value = 0u;
    uint32_t digits = 0u;
    while ((*s >= '0') && (*s <= '9'))
    {
        if (digits >= 9u)
        {
            return false;
        }
        value = value * 10u + (uint32_t)(*s - '0');
        digits++;
        s++;
    }

    while (USB_IsSpace(*s))
    {
        s++;
    }
    if (*s != '\0')
    {
        return false;
    }

    *out = value;
    return true;
}

/**
  * @brief 设置单个增益, 保留同组另外两个参数
  */
static bool USB_ApplyGain(PID_Loop_t loop, uint8_t gain_index, const char *value)
{
    float v;
    float kp;
    float ki;
    float kd;

    if (!USB_ParseFloat(value, &v))
    {
        return false;
    }
    if ((v < 0.0f) || (v > USB_CMD_GAIN_MAX))
    {
        return false;
    }
    if (!PID_GetGains(loop, &kp, &ki, &kd))
    {
        return false;
    }

    switch (gain_index)
    {
        case 0u: kp = v; break;
        case 1u: ki = v; break;
        case 2u: kd = v; break;
        default: return false;
    }

    return PID_SetGains(loop, kp, ki, kd);
}

/**
  * @brief 设置角度给定并归一化到 (-π, π]
  * @note  归一化后交给 PID_SetAngleRef(); 该接口在给定值实际变化时
  *        清空角度/角速度/电流 d/q 四个积分项, 重复发送相同值不清零。
  */
static bool USB_ApplyAngleRef(const char *value)
{
    float v;

    if (!USB_ParseFloat(value, &v))
    {
        return false;
    }
    if ((v < -USB_CMD_ANGLE_MAX) || (v > USB_CMD_ANGLE_MAX))
    {
        return false;
    }

    while (v > PID_PI)
    {
        v -= PID_2PI;
    }
    while (v <= -PID_PI)
    {
        v += PID_2PI;
    }

    PID_SetAngleRef(v);
    return true;
}

/**
  * @brief 设置角速度给定 (RPM)
  * @note  交给 PID_SetSpeedRef(); 给定值实际变化时清空四个积分项,
  *        重复发送相同值不清零。
  */
static bool USB_ApplySpeedRef(const char *value)
{
    float v;

    if (!USB_ParseFloat(value, &v))
    {
        return false;
    }
    if ((v < -USB_CMD_SPEED_MAX) || (v > USB_CMD_SPEED_MAX))
    {
        return false;
    }

    PID_SetSpeedRef(v);
    return true;
}

/**
  * @brief 设置运行模式
  */
static bool USB_ApplyMode(const char *value)
{
    uint32_t mode;

    if (!USB_ParseUint(value, &mode))
    {
        return false;
    }
    if (mode > PID_MODE_CURRENT)
    {
        return false;
    }

    return PID_SetMode((uint8_t)mode);
}

/**
  * @brief 电流给定范围检查 (d/q 共用, 留 0.1mA 浮点解析容差)
  */
static bool USB_CurrentInRange(float v)
{
    const float limit = USB_CMD_CURR_MAX + 1.0e-4f;

    return ((v >= -limit) && (v <= limit)) ? true : false;
}

/**
  * @brief 设置 q 轴电流给定 (仅电流模式)
  * @note  电流模式用于调试 20kHz 电流环; q 轴对应转矩。
  */
static bool USB_ApplyCurrentRef(const char *value)
{
    float v;

    if (!USB_ParseFloat(value, &v))
    {
        return false;
    }
    if (!USB_CurrentInRange(v))
    {
        return false;
    }
    if (PID_GetMode() != PID_MODE_CURRENT)
    {
        return false;
    }

    return PID_SetCurrentRef(v);
}

/**
  * @brief 设置 d 轴电流给定 (仅电流模式)
  * @note  电流模式用于调试 20kHz 电流环; d 轴对应励磁分量。
  */
static bool USB_ApplyCurrentDRef(const char *value)
{
    float v;

    if (!USB_ParseFloat(value, &v))
    {
        return false;
    }
    if (!USB_CurrentInRange(v))
    {
        return false;
    }
    if (PID_GetMode() != PID_MODE_CURRENT)
    {
        return false;
    }

    return PID_SetCurrentDRef(v);
}

/**
  * @brief 保存当前 PID 增益到 Flash
  * @note  新协议为无值命令 "save"; 为兼容旧上位机仍接受 "save=1"。
  *        要求当前为 Mode=0 (STOP), 避免 Flash 擦除阻塞 20kHz 电流环时
  *        电机仍被驱动。
  */
static bool USB_ApplySave(const char *value)
{
    if (value != NULL)
    {
        uint32_t save;

        if (!USB_ParseUint(value, &save) || (save != 1u))
        {
            return false;
        }
    }
    if (PID_GetMode() != PID_MODE_STOP)
    {
        return false;
    }

    return PID_Config_Save();
}

/**
  * @brief 从 Flash 读取并应用保存的 9 个增益
  * @note  无值命令 "load"; 兼容 "load=1"。读取不擦写 Flash, 可在任意模式执行。
  *        没有有效记录或校验失败时返回 false。
  */
static bool USB_ApplyLoad(const char *value)
{
    if (value != NULL)
    {
        uint32_t load;

        if (!USB_ParseUint(value, &load) || (load != 1u))
        {
            return false;
        }
    }

    return PID_Config_Load();
}

/**
  * @brief 执行一条已去掉换行的命令
  * @param line 以 '\0' 结尾的命令行
  * @retval true 命令合法且已生效; false 忽略本行
  * @note  支持 "Key=Value" 和少量无值命令 "save"/"load"; 无值命令后面
  *        不允许再跟其他字符 (空格除外)。
  */
static bool USB_Command_Execute(const char *line)
{
    const char *key_start = line;
    const char *key_end;
    const char *p;
    const char *value = NULL;
    const USB_CmdEntry_t *entry = NULL;
    char key[16];
    uint32_t key_len;

    while (USB_IsSpace(*key_start))
    {
        key_start++;
    }

    key_end = key_start;
    while ((*key_end != '\0') && (*key_end != '=') && !USB_IsSpace(*key_end))
    {
        key_end++;
    }

    key_len = (uint32_t)(key_end - key_start);
    if ((key_len == 0u) || (key_len >= sizeof(key)))
    {
        return false;
    }

    for (uint32_t i = 0u; i < key_len; i++)
    {
        char c = key_start[i];
        if ((c >= 'A') && (c <= 'Z'))
        {
            c = (char)(c - 'A' + 'a');
        }
        key[i] = c;
    }
    key[key_len] = '\0';

    p = key_end;
    while (USB_IsSpace(*p))
    {
        p++;
    }
    if (*p == '=')
    {
        value = p + 1;
        while (USB_IsSpace(*value))
        {
            value++;
        }
    }
    else if (*p != '\0')
    {
        return false;      /* 无值命令后跟了其他字符 */
    }

    for (uint32_t i = 0u; i < (sizeof(s_cmd_table) / sizeof(s_cmd_table[0])); i++)
    {
        if (USB_KeyEqual(key, s_cmd_table[i].name))
        {
            entry = &s_cmd_table[i];
            break;
        }
    }
    if (entry == NULL)
    {
        return false;
    }

    switch (entry->id)
    {
        case USB_CMD_ID_ANGLE_KP: return USB_ApplyGain(PID_LOOP_ANGLE, 0u, value);
        case USB_CMD_ID_ANGLE_KI: return USB_ApplyGain(PID_LOOP_ANGLE, 1u, value);
        case USB_CMD_ID_ANGLE_KD: return USB_ApplyGain(PID_LOOP_ANGLE, 2u, value);
        case USB_CMD_ID_SPEED_KP: return USB_ApplyGain(PID_LOOP_SPEED, 0u, value);
        case USB_CMD_ID_SPEED_KI: return USB_ApplyGain(PID_LOOP_SPEED, 1u, value);
        case USB_CMD_ID_SPEED_KD: return USB_ApplyGain(PID_LOOP_SPEED, 2u, value);
        case USB_CMD_ID_CURR_KP: return USB_ApplyGain(PID_LOOP_CURRENT, 0u, value);
        case USB_CMD_ID_CURR_KI: return USB_ApplyGain(PID_LOOP_CURRENT, 1u, value);
        case USB_CMD_ID_CURR_KD: return USB_ApplyGain(PID_LOOP_CURRENT, 2u, value);
        case USB_CMD_ID_ANGLE_REF: return USB_ApplyAngleRef(value);
        case USB_CMD_ID_SPEED_REF: return USB_ApplySpeedRef(value);
        case USB_CMD_ID_CURR_REF:
            return (value != NULL) ? USB_ApplyCurrentRef(value) : false;
        case USB_CMD_ID_CURR_D_REF:
            return (value != NULL) ? USB_ApplyCurrentDRef(value) : false;
        case USB_CMD_ID_MODE: return USB_ApplyMode(value);
        case USB_CMD_ID_SAVE: return USB_ApplySave(value);
        case USB_CMD_ID_LOAD: return USB_ApplyLoad(value);
        default: return false;
    }
}

/**
  * @brief 向行状态机喂一个字节
  */
static void USB_Command_FeedByte(uint8_t byte)
{
    char c = (char)byte;

    if (s_discard)
    {
        if ((c == '\r') || (c == '\n'))
        {
            s_discard = false;
        }
        return;
    }

    if ((c == '\r') || (c == '\n') || (c == '\0'))
    {
        if (s_line_overflow)
        {
            USB_Cmd_ErrCount++;
        }
        else if (s_line_len != 0u)
        {
            s_line[s_line_len] = '\0';
            if (USB_Command_Execute(s_line))
            {
                USB_Cmd_OkCount++;
            }
            else
            {
                USB_Cmd_ErrCount++;
            }
        }
        else
        {
            /* 空行: 忽略 */
        }

        s_line_len = 0u;
        s_line_overflow = false;
        return;
    }

    if (s_line_len < (uint8_t)(USB_CMD_LINE_MAX - 1u))
    {
        s_line[s_line_len] = c;
        s_line_len++;
    }
    else
    {
        s_line_overflow = true;
    }
}

/*==============================================================================
 * 对外接口
 *============================================================================*/

void USB_Init(void)
{
    (void)RingBuffer_Init(&USB_TxBuffer, USB_TxPool, USB_TX_BUFFER_SIZE);
    (void)RingBuffer_Init(&USB_RxBuffer, USB_RxPool, USB_RX_BUFFER_SIZE);

    s_line_len = 0u;
    s_line_overflow = false;
    s_discard = false;
    s_rx_overflow = false;
    s_rx_abort = false;

    USB_Cmd_OkCount = 0u;
    USB_Cmd_ErrCount = 0u;
    USB_Rx_OverflowCount = 0u;
}

void USB_RxPush(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0u))
    {
        return;
    }

    /* RingBuffer_Write 全有或全无: 空间不足时整包丢弃, 不写入半包 */
    if (RingBuffer_Write(&USB_RxBuffer, data, (uint32_t)len) != (uint32_t)len)
    {
        USB_Rx_OverflowCount++;
        s_rx_overflow = true;
    }
}

void USB_RxAbort(void)
{
    s_rx_abort = true;
}

void USB_Command_Process(void)
{
    /* USB 复位: 丢弃未完成的半行, 防止与重连后的新命令拼接 */
    if (s_rx_abort)
    {
        s_rx_abort = false;
        if ((s_line_len != 0u) || s_line_overflow)
        {
            s_line_len = 0u;
            s_line_overflow = false;
            s_discard = true;
        }
    }

    /* 有整包被丢弃: 当前行可能缺字节, 丢弃到下一个换行以重新同步 */
    if (s_rx_overflow)
    {
        s_rx_overflow = false;
        s_line_len = 0u;
        s_line_overflow = false;
        s_discard = true;
    }

    uint32_t budget = USB_CMD_PROCESS_BUDGET;
    uint8_t chunk[32];

    while ((budget != 0u) && !RingBuffer_IsEmpty(&USB_RxBuffer))
    {
        uint32_t n = RingBuffer_GetUsed(&USB_RxBuffer);
        if (n > (uint32_t)sizeof(chunk))
        {
            n = (uint32_t)sizeof(chunk);
        }
        if (n > budget)
        {
            n = budget;
        }

        n = RingBuffer_Peek(&USB_RxBuffer, chunk, n);
        if (n == 0u)
        {
            break;
        }
        RingBuffer_Advance(&USB_RxBuffer, n);

        for (uint32_t i = 0u; i < n; i++)
        {
            USB_Command_FeedByte(chunk[i]);
        }
        budget -= n;
    }
}