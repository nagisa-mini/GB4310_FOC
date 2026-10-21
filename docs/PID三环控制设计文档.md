# PID 三环闭环控制模块 设计文档

| 项目 | 内容 |
|---|---|
| 源文件 | `BSP/PID.h`、`BSP/PID.c` |
| 平台 | STM32G431CB，Cortex-M4F，170 MHz |
| 状态 | 电流环已接入 TIM1 中断（测试给定 0.2 A）；角度环、角速度环尚未接入 |

---

## 1. 总体框架

三环级联，外环输出作为内环给定；内环频率逐级放大 5~20 倍。

```
                 角度给定 (rad)
                       │
                       ▼
              ┌─────────────────┐
   角度反馈 ─▶│ 角度环 PID_Angle │──▶ 角速度给定 (rad/min)
    (rad)     │     @200 Hz      │
              └─────────────────┘
                       │ 模块内缓存 PID_Speed_Ref
                       ▼
              ┌─────────────────┐
 角速度反馈 ─▶│ 角速度环 PID_Speed│──▶ q 轴电流给定 (A)
  (rad/min)   │     @1 kHz       │
              └─────────────────┘
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
   ┌─────────────┐           ┌─────────────┐
   │ 电流环 d 轴  │           │ 电流环 q 轴  │
   │ ref = 0     │           │ ref = iq_ref│
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
| 角速度 | rad/min |
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
| F5 | 电流环 d/q 双轴独立控制，d 轴给定恒为 0 |
| F6 | 三环参数集中初始化、状态清零 |
| F7 | 环间给定缓存（5 ms 角度环 → 1 ms 速度环，零阶保持） |

不含任何额外保护：无过流、堵转、断线检测，无死区补偿，无积分分离与回退式抗饱和。

---

## 3. 函数接口

### 3.1 数据结构

```c
typedef struct {
    float Kp, Ki, Kd;              /* 比例 / 积分 / 微分系数 */
    float I_limit;                 /* 积分限幅（绝对值，单位同输出） */
    float Out_limit;               /* 输出限幅（绝对值，单位同输出） */
    float Ref, Fdb, Err, Err_last; /* 给定 / 反馈 / 误差 / 上次误差 */
    float Integral, Out;           /* 积分项 / 输出 */
} PID_t;
```

### 3.2 通用函数

| 函数原型 | 功能 | 参数 | 返回 |
|---|---|---|---|
| `void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float I_limit, float Out_limit)` | 写增益与限幅，并清空状态 | `pid` 句柄；增益；积分/输出限幅 | void |
| `void PID_Reset(PID_t *pid)` | 清空误差、积分、输出（保留增益） | `pid` 句柄 | void |
| `float PID_Calc(PID_t *pid, float ref, float fdb, float ts)` | 单环 PID 运算 | `ref` 给定；`fdb` 反馈；`ts` 周期(s) | 限幅后输出 |
| `float PID_Angle_Err(float ref, float fdb)` | 角度误差，跨零点取最短路径 | 给定角度、反馈角度（rad） | 误差 rad，(−π, π] |
| `float PID_Angle_Delta(float ang_new, float ang_old)` | 角度增量，供测速 | 本次、上次机械角（rad） | 增量 rad，(−π, π] |

> 测速（`AngleSpeed_Get`）：`rpm = PID_Angle_Delta(now, last) * PID_RAD2RPM / dt`
>
> 注意：测速输出已改为 rpm，而速度环其余链路（给定、限幅、增益）仍按 rad/min 约定；
> 若直接把 `AngleSpeed_Get()` 的结果送 `PID_Speed_Loop_Isr` 作反馈，需将其除以 2π，
> 或把速度给定一并换成 rpm（800 rad/min ≈ 127.32 rpm）。

### 3.3 三环函数

| 函数原型 | 功能 | 参数 | 返回 | 频率 |
|---|---|---|---|---|
| `void PID_3Loop_Init(void)` | 初始化四个控制器实例并清零环间缓存 | 无 | void | 上电一次 |
| `void PID_Angle_Loop_Isr(float angle_ref, float angle_fdb)` | 角度环：误差 → 角速度给定 | 目标、当前机械角（rad） | void | 200 Hz |
| `float PID_Speed_Loop_Isr(float speed_fdb)` | 角速度环 → q 轴电流给定 | 角速度反馈（rad/min） | 电流给定 A | 1 kHz |
| `void PID_Current_Loop_Isr(float iq_ref, float id_fdb, float iq_fdb, float *vd_out, float *vq_out)` | 电流环 d/q 双轴 → 电压给定 | `iq_ref` A；`id_fdb`/`iq_fdb` A；两个输出指针 | void，经指针返回 Vd/Vq | 20 kHz |
| `float PID_Current_Loop_Q_Isr(float iq_ref, float iq_fdb)` | 电流环仅 q 轴 | `iq_ref`/`iq_fdb`（A） | Vq (V) | 20 kHz |

### 3.4 周期与换算宏

| 宏 | 值 | 含义 |
|---|---|---|
| `PID_ANGLE_TS` | 0.005f | 角度环周期 (s) |
| `PID_SPEED_TS` | 0.001f | 角速度环周期 (s) |
| `PID_CURR_TS` | 0.00005f | 电流环周期 (s) |
| `PID_ANGLE_DIV` | 5U | 角度环在 TIM7 中的分频 |
| `PID_RAD2MIN` / `PID_MIN2RAD` | 60.0f / 0.016666667f | rad/s ↔ rad/min |
| `PID_PI` / `PID_2PI` | 3.14159265f / 6.28318531f | π / 2π |

### 3.5 调用关系

```c
PID_3Loop_Init();                       /* 上电一次 */

/* TIM7 @1kHz —— 先角度环（每 5 拍），再速度环（每拍） */
if (++cnt >= PID_ANGLE_DIV) { cnt = 0; PID_Angle_Loop_Isr(ang_ref, ang_fdb); }
iq_ref = PID_Speed_Loop_Isr(speed_fdb);

/* TIM1 @20kHz —— 电流采样之后、SVPWM 之前 */
PID_Current_Loop_Isr(iq_ref, Iqd.d, Iqd.q, &Uqd.d, &Uqd.q);
```
