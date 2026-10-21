# PID 三环闭环控制模块 设计文档

| 项目 | 内容 |
|---|---|
| 源文件 | `BSP/PID.h`、`BSP/PID.c` |
| 平台 | STM32G431CB，Cortex-M4F，170 MHz |
| 状态 | 三环均已接入：角度环/角速度环在 TIM7 中断，电流环在 TIM1 中断 |

---

## 1. 总体框架

三环级联，外环输出作为内环给定；内环频率逐级放大 5~20 倍。

```
                 角度给定 (rad)
                       │
                       ▼
              ┌─────────────────┐
   角度反馈 ─▶│ 角度环 PID_Angle │──▶ 角速度给定 (RPM)
    (rad)     │     @200 Hz      │
              └─────────────────┘
                       │ 模块内缓存 Speed_Ref
                       ▼
              ┌─────────────────┐
 角速度反馈 ─▶│ 角速度环 PID_Speed│──▶ q 轴电流给定 (A)
   (RPM)      │     @1 kHz       │
              └─────────────────┘
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
   ┌─────────────┐           ┌─────────────┐
   │ 电流环 d 轴  │           │ 电流环 q 轴  │
   │ ref = Id*   │           │ ref = Iq*   │
   │ (非电流模式0)│           │(速度环/主机)│
   └──────┬──────┘           └──────┬──────┘
          │ Vd (V)                  │ Vq (V)
          └────────────┬────────────┘
                       ▼
              Udq ─▶ FOC / SVPWM（模块外）
```

### 运行位置与频率

| 环 | 周期 | 频率 | 运行位置 |
|---|---|---|---|
| 角度环 | 5 ms | 200 Hz | TIM7 中断，每 5 拍一次 |
| 角速度环 | 1 ms | 1 kHz | TIM7 中断，每拍 |
| 电流环 d/q | 50 µs | 20 kHz | TIM1 中断（FOC 中断） |

### 单位约定

| 物理量 | 单位 |
|---|---|
| 角度 | rad（机械角） |
| 角速度 | RPM（转/分） |
| 电流 | A |
| 电压 | V |
| 周期 | s |

---

## 2. 功能

| 编号 | 功能 |
|---|---|
| F1 | 通用位置式 PID 运算，含**积分限幅**与**输出限幅** |
| F2 | 角度误差跨零点处理，始终走 (−π, π] 最短路径 |
| F3 | 角度增量计算，供编码器差分测速使用 |
| F4 | 角度环 / 角速度环 / 电流环三环级联运算 |
| F5 | 电流环 d/q 双轴独立控制；非电流模式 d 轴给定固定为 0，电流模式 d/q 给定均可由主机设置；两轴共用一组在线增益 |
| F6 | 三环参数集中初始化、状态清零 |
| F7 | 环间给定缓存（5 ms 角度环 → 1 ms 速度环，零阶保持） |
| F8 | 四种运行模式：STOP / ANGLE / SPEED / CURRENT；电流模式直接给定 d/q 轴电流 |

不含任何额外保护：无过流、堵转、断线检测，无死区补偿，无积分分离与回退式抗饱和。

---

## 3. 函数接口

### 3.1 封装方式

四个控制器实例（`PID_Angle`/`PID_Speed`/`PID_Curr_D`/`PID_Curr_Q`）和三环运行状态（给定/反馈/模式）都是 `PID.c` 内部 `static` 变量，`PID_t`、`PID_RUN_t` 等内部结构不再出现在头文件中。外部只能通过下列接口读写，避免绕过临界区直接修改状态。

### 3.2 状态与给定接口

| 函数原型 | 功能 | 说明 |
|---|---|---|
| `void PID_3Loop_Init(void)` | 四环初始化 | 写默认增益/限幅，状态清零，模式 = STOP |
| `uint8_t PID_GetMode(void)` | 读当前模式 | 返回 `PID_MODE_xxx` |
| `void PID_SetAngleRef(float rad)` | 写角度给定 | 给定实际变化时清角度/角速度/电流 d/q 四个积分项；输入应在 (−π, π] |
| `void PID_SetSpeedRef(float rpm)` | 写角速度给定 | 同上，单位 RPM |
| `bool PID_SetCurrentRef(float amp)` | 写 q 轴电流给定 | 仅 `PID_MODE_CURRENT` 有效；给定变化时清四个积分项 |
| `bool PID_SetCurrentDRef(float amp)` | 写 d 轴电流给定 | 仅 `PID_MODE_CURRENT` 有效；给定变化时清四个积分项 |
| `void PID_GetTelemetry(float *values)` | 取遥测快照 | 按 `PID_TELEM_*` 顺序写 12 个 float |
| `bool PID_SetGains(PID_Loop_t loop, float Kp, float Ki, float Kd)` | 在线写增益 | 短临界区；写 `PID_LOOP_CURRENT` 时同时写 d/q 两轴 |
| `bool PID_GetGains(PID_Loop_t loop, float *Kp, float *Ki, float *Kd)` | 读增益 | 参数非法返回 false |
| `bool PID_SetMode(uint8_t mode)` | 切换模式 | 模式变化时复位四环状态 |
| `void PID_ResetIntegralAll(void)` | 清四个积分项 | 短临界区；给定接口已内部调用 |
| `bool PID_Config_Save(void)` | 保存 9 个增益 | 仅 STOP 模式允许 |
| `bool PID_Config_Load(void)` | 上电/命令加载 | 无有效记录时保留默认值 |
| `bool PID_Config_IsLoaded(void)` | 是否加载成功 | 最近一次加载的结果 |

### 3.3 遥测通道

`PID_GetTelemetry(values)` 按 `PID_Telem_t` 的顺序填充，可直接交给 `Fmt_LineU8()`：

| 序号 | 宏 | 内容 | 单位 |
|---|---|---|---|
| 0 | `PID_TELEM_ANGLE_REF` | 角度给定 | rad |
| 1 | `PID_TELEM_ANGLE_FDB` | 角度反馈 | rad |
| 2 | `PID_TELEM_ANGLE_OUT` | 角度环输出 | RPM |
| 3 | `PID_TELEM_SPEED_REF` | 角速度给定 | RPM |
| 4 | `PID_TELEM_SPEED_FDB` | 角速度反馈 | RPM |
| 5 | `PID_TELEM_SPEED_OUT` | 角速度环输出 | A |
| 6 | `PID_TELEM_CURR_Q_REF` | q 轴电流给定 | A |
| 7 | `PID_TELEM_CURR_Q_FDB` | q 轴电流反馈 | A |
| 8 | `PID_TELEM_CURR_Q_OUT` | 电流环 q 轴输出 | V |
| 9 | `PID_TELEM_CURR_D_REF` | d 轴电流给定 | A |
| 10 | `PID_TELEM_CURR_D_FDB` | d 轴电流反馈 | A |
| 11 | `PID_TELEM_CURR_D_OUT` | 电流环 d 轴输出 | V |

### 3.4 三环函数

| 函数原型 | 功能 | 频率 |
|---|---|---|
| `void PID_Angle_Loop_Isr(void)` | 读机械角 → 角度误差 → 角速度给定 | 200 Hz |
| `void PID_Speed_Loop_Isr(void)` | 角度差分测速 → q 轴电流给定 | 1 kHz |
| `void PID_Current_Loop_Isr(float id_fdb, float iq_fdb, float *vd_out, float *vq_out)` | d/q 双轴 PID → 电压给定；STOP 模式输出 0 | 20 kHz |

### 3.5 周期与换算宏

| 宏 | 值 | 含义 |
|---|---|---|
| `PID_ANGLE_TS` | 0.005f | 角度环周期 (s) |
| `PID_SPEED_TS` | 0.001f | 角速度环周期 (s) |
| `PID_CURR_TS` | 0.00005f | 电流环周期 (s) |
| `PID_ANGLE_DIV` | 5U | 角度环在 TIM7 中的分频 |
| `PID_RAD2RPM` | 9.549296586f | rad/s → RPM |
| `PID_PI` / `PID_2PI` | 3.14159265f / 6.28318531f | π / 2π |

### 3.6 调用关系

```c
PID_3Loop_Init();                        /* 上电一次 */
(void)PID_Config_Load();                 /* 读取 Flash 保存的增益 (可选) */

/* TIM7 @1kHz —— 每拍速度环, 每 5 拍角度环 */
PID_Speed_Loop_Isr();
if (++cnt >= PID_ANGLE_DIV) { cnt = 0; PID_Angle_Loop_Isr(); }

/* TIM1 @20kHz —— 电流采样与 Clarke/Park 之后、SVPWM 之前 */
PID_Current_Loop_Isr(FOC_State.Iqd.d, FOC_State.Iqd.q,
                     &FOC_State.Uqd.d, &FOC_State.Uqd.q);

/* 上位机命令 (主循环) */
PID_SetSpeedRef(400.0f);
PID_SetMode(PID_MODE_SPEED);

/* 电流模式: 进入后设置 d/q 轴电流给定, 速度环不再覆盖 Curr_Q_Ref */
PID_SetMode(PID_MODE_CURRENT);
if (!PID_SetCurrentRef(0.5f)) { /* 当前不是电流模式 */ }
if (!PID_SetCurrentDRef(0.1f)) { /* 当前不是电流模式 */ }

/* 1kHz 遥测: 12 路最坏情况约 130+ 字符, 缓冲区别小于 160 */
float telemetry[PID_TELEM_COUNT];
PID_GetTelemetry(telemetry);
uint8_t line[192];
Fmt_LineU8(line, sizeof(line), telemetry, (uint8_t)PID_TELEM_COUNT, 4u);
```

### 3.7 测速说明

模块内部 `PID_AngleSpeed_Get()` 在 1 kHz 速度环里用相邻两次机械角差分测速：

```c
rpm = PID_Angle_Err(angle_now, angle_last) * PID_RAD2RPM / PID_SPEED_TS;
```

差分取跨零点最短路径，结果为机械轴转速（RPM）。该函数为 `PID.c` 内部 static，不对外暴露。
