# USB 上位机命令协议与实现说明

| 项目 | 内容 |
|---|---|
| 源文件 | `BSP/USB.h`、`BSP/USB.c` |
| 中断入口 | `USB_Device/App/usbd_cdc_if.c` → `CDC_Receive_FS()` |
| 空闲解析 | `Core/Src/main.c` 的 `while(1)` → `USB_Command_Process()` |
| 传输层 | USB CDC（虚拟串口），无波特率/校验位概念 |
| 目标 | 在线设置角度环/角速度环 Kp、Ki、Kd 及 `PID_Run` 的给定与模式 |

---

## 1. 总体数据流

```text
上位机
  │  "Speed=1.0\r\n"
  ▼
USB OUT 中断 → USB_LP_IRQHandler()
  │
  ▼
CDC_Receive_FS()                  ← 只做一件事: USB_RxPush()
  │
  ▼
USB_RxBuffer (512 B, SPSC 环形缓冲区)
  │
  ▼
main while(1) → USB_Command_Process()
  │   Peek/Advance + 行缓存 + 逐字节状态机
  ▼
USB_Command_Execute()
  │
  ├─ PID_SetGains()  → 角度环 / 角速度环 Kp、Ki、Kd
  ├─ PID_Run.Angle_Ref = ... ─┐
  ├─ PID_Run.Speed_Ref = ... ─┴→ PID_ResetIntegralAll() 清四个积分项
  └─ PID_SetMode()   → mode 0/1/2 (变化时复位各环状态)
```

设计约束：

- USB 中断内**不解析字符串、不调用 PID、不写发送缓冲区**，只把整包数据写入 `USB_RxBuffer`。
- 命令解析在主循环空闲时执行，单次最多消费 `USB_CMD_PROCESS_BUDGET`（64）字节，非阻塞。
- `USB_RxBuffer` 是单生产者（USB 中断）/单消费者（主循环）环形缓冲区，不需要关中断。
- 解析过程不使用 `strtof` / `sscanf` / `printf`，无动态内存，可在裸机主循环安全运行。

---

## 2. 帧格式

一条命令一行：

```text
Key=Value
```

- 行结尾：`\r`、`\n` 或 `\r\n`；三种都支持。
- Key 不区分大小写；`Key`、`=`、`Value` 两侧允许空格或制表符。
- Value 为十进制数，可带 `+` / `-` 号和一个小数点；**不支持指数**（`1e3` 会被拒绝）。
- 空行忽略；一行只能有一条命令。
- 非法命令、未知 Key、非法数值、超范围数值：整行丢弃，不修改任何 PID 状态，`USB_Cmd_ErrCount` 加一。
- 不返回 ACK/NAK，遥测数据流保持原格式（`Speed_Ref,Speed_Fdb\r\n`）。

---

## 3. 命令表

| Key | 目标 | 单位 | 允许范围 |
|---|---|---|---|
| `Angle_Kp` | 角度环 Kp | — | 0 ~ 1,000,000 |
| `Angle_Ki` | 角度环 Ki | — | 0 ~ 1,000,000 |
| `Angle_Kd` | 角度环 Kd | — | 0 ~ 1,000,000 |
| `Speed_Kp` | 角速度环 Kp | — | 0 ~ 1,000,000 |
| `Speed_Ki` | 角速度环 Ki | — | 0 ~ 1,000,000 |
| `Speed_Kd` | 角速度环 Kd | — | 0 ~ 1,000,000 |
| `Angle` / `Angle_Ref` | `PID_Run.Angle_Ref` | rad（机械角） | \|v\| ≤ 1000·2π，自动归一化到 (−π, π] |
| `Speed` / `Speed_Ref` | `PID_Run.Speed_Ref` | RPM | \|v\| ≤ 10,000 |
| `Mode` | `PID_Run.mode` | — | 0 / 1 / 2 |
| `Save` | 保存 6 个增益到 Flash | — | 仅 `1`；要求当前 `Mode=0` |

增益不允许为负；角度给定会自动折算到最短角度区间；速度给定沿用 `PID_Run.Speed_Ref` 现有的 RPM 单位，不做 rad/s 换算。

### 模式定义

| Mode | 含义 | 行为 |
|---|---|---|
| 0 | 停止 | 复位四个 PID 状态，`Curr_Q_Ref=0`，电流环输出 Vd=Vq=0（零电压矢量，无驱动转矩）；PWM 仍运行，不是断开 MOSFET 的高阻自由滑行 |
| 1 | 角度环 | 复位各环；`Speed_Ref` 先清 0，之后由 200 Hz 角度环生成 |
| 2 | 角速度环 | 复位各环；直接使用主机设置的 `Speed_Ref` |

模式切换时 `PID_SetMode()` 会清空角度环/角速度环/电流环 d/q 的积分、误差和输出，避免切换瞬间带着旧积分动作。重复发送相同模式不会复位。

> 注意：`mode=1` 时 `Speed_Ref` 由角度环每 5 ms 重写，主机通过 `Speed=...` 设置的值会被覆盖，这是三环级联结构决定的。

---

## 4. 示例

```text
Speed=1.0
Angle=0.785
Angle_Kp=8.0
Angle_Ki=0.2
Angle_Kd=0.0
Speed_Kp=0.01
Speed_Ki=0.0
Speed_Kd=0.0
Mode=2
```

每条实际发送时应带 `\r\n`，例如：

```text
Speed=1.0\r\n
```

批量发送也可以，解析器按行切分：

```text
Speed=400\r\nMode=2\r\nAngle_Kp=8.0\r\n
```

命令可以跨 USB 包：例如先发 `Spe`，再发 `ed=1.0\r\n`，仍然能正确解析。

### 推荐下发顺序

```text
速度模式:  Mode=2  →  Speed=...
角度模式:  Mode=1  →  Angle=...
停止:      Mode=0
增益:      Angle_Kp/...、Speed_Kp/... 可随时下发
保存:      Mode=0  →  Save=1
```

先切模式再给给定，原因是 `mode=1` 时角度环每 5 ms 会重写 `Speed_Ref`；如果先发 `Speed=...` 再切 `Mode=1`，该值会被角度环覆盖。切到 `Mode=2` 后角度环停止写 `Speed_Ref`，此时再发 `Speed=...` 才能稳定保持。

### 给定变化时清空积分

`Angle` / `Speed` 的给定值**实际发生变化**时，命令层会调用 `PID_ResetIntegralAll()`，在短临界区内清空：

- 角度环积分
- 角速度环积分
- 电流环 d 轴积分
- 电流环 q 轴积分（20 kHz 电流环）

这样旧的积分量不会继续抵抗新的阶跃给定。该操作只清 `Integral`，不修改 Kp/Ki/Kd、`Err_last` 和 `Out`。

注意：

- 重复发送相同给定值**不会**清积分，避免上位机周期刷新时把积分一直按住。
- 修改增益不会清积分。
- `Mode` 切换仍按原有逻辑复位四个 PID 的完整状态（误差/积分/输出）。

### 参数保存与上电加载

`Save=1` 把当前角度环/角速度环的 6 个增益写入 Flash 最后一页：

```text
Mode=0          ← 必须先停止
Speed_Kp=0.0165
Speed_Ki=1.0
Speed_Kd=0
Save=1          ← 保存；约几十毫秒
```

行为说明：

- **只保存 6 个增益**（`Angle_Kp/Ki/Kd`、`Speed_Kp/Ki/Kd`），不保存 `Mode`、`Angle_Ref`、`Speed_Ref`；上电后模式和给定仍按 `PID_3Loop_Init()` 的默认值。
- 保存前必须处于 `Mode=0`。Flash 页擦除期间 CPU 取指会被阻塞约几十毫秒，运行中保存会让 20 kHz 电流环丢拍，因此命令层直接拒绝非停止模式。
- 记录包含 magic、版本号、长度和 CRC32；上电时 `main()` 在 `PID_3Loop_Init()` 之后调用 `PID_Config_Load()`，校验通过才覆盖默认增益，否则保留默认值。
- 存储地址为 `0x0801F800`（最后 2 KB 页）；链接脚本已把 `FLASH` 长度改为 `126K`，该页不参与程序链接。
- 无 ACK：成功时 `USB_Cmd_OkCount++`，校验失败/模式不对/写入失败时 `USB_Cmd_ErrCount++`。
- 使用 ST-Link/CubeProgrammer 做**整片擦除**会同时擦掉保存的参数；只擦除程序扇区则不受影响。
- 参数页按“整页擦除再写”更新，不要周期性自动保存，避免无谓磨损。

### Speed=0 高频震荡的排查

若 `Speed=0` 时电机高频抖动/异响，同时遥测 `Speed_Fdb` 在 0 附近跳变，优先检查速度环 `Speed_Kp`：

- 15 位编码器在 1 ms 测速周期下的量化约为 **1.83 RPM/LSB**。
- `Speed_Kp=0.02` 时，一个编码器 LSB 会被放大成约 `0.02×1.83≈0.037 A` 的电流脉冲；零速附近没有反电动势阻尼，容易形成 1 kHz 极限环。
- 实测 `Speed_Kp=0.02` 会出现该现象，降低到 `0.0165` 左右后明显改善；建议从 `0.005~0.01`、`Speed_Ki=0` 起步，再按负载逐步整定。
- 如果确实需要更高刚度，应先给速度反馈加低通滤波或加长测速窗口（4~5 ms），再逐步提高 Kp。
- 发送 `Mode=0` 可快速验证：若震荡立即消失，说明是速度环在零速附近的极限环，而不是电流环/硬件问题。

---

## 5. 错误处理与恢复

| 场景 | 行为 |
|---|---|
| 缺 `=`、空 Key、空 Value | 丢弃整行，错误计数 +1 |
| 未知 Key | 丢弃整行，错误计数 +1 |
| 非数字、指数、尾部多余字符 | 丢弃整行，错误计数 +1 |
| 数值超范围 | 丢弃整行，错误计数 +1 |
| 单行超过 63 字符 | 整行丢弃，错误计数 +1，等下一行恢复 |
| `USB_RxBuffer` 满（512 B） | 新到的整包丢弃，`USB_Rx_OverflowCount` +1；主循环清空半行并丢弃到下一个换行后恢复 |
| USB 重新枚举 | `CDC_Init_FS()` 调用 `USB_RxAbort()`，丢弃未完成半行，避免与新命令拼接 |
| 空行 | 忽略，不计错误 |

调试计数（可用调试器查看，不占用遥测流）：

```c
extern volatile uint32_t USB_Cmd_OkCount;       /* 成功命令数 */
extern volatile uint32_t USB_Cmd_ErrCount;      /* 失败/超长行数 */
extern volatile uint32_t USB_Rx_OverflowCount;  /* 丢弃的 USB 包数 */
```

---

## 6. 上电默认值

`PID_3Loop_Init()` 的现有行为未改变：

- `PID_Run.mode = PID_MODE_SPEED`（2，角速度环）
- `PID_Run.Speed_Ref = 400.0f` RPM
- 角度/速度/电流环使用 `PID.c` 中的编译期默认增益

如果希望上电后保持停止，可在初始化完成后调用 `PID_SetMode(PID_MODE_STOP)`，或在 `main()` 的 `USER CODE BEGIN 2` 中显式设置；本次实现未改变该默认行为。

---

## 7. 测试

主机侧单元测试（不参与固件构建）：

```bat
cd 软件代码\GB4310_FOC
gcc -std=c11 -O2 -Wall -Wextra tests\usb_cmd_test.c BSP\USB.c BSP\Ringbuffer.c ^
    -I tests\stub -I BSP -o build\usb_cmd_test.exe
build\usb_cmd_test.exe
```

覆盖内容：单命令、浮点解析、跨包分片、CR/LF/CRLF、一包多行、大小写/空格、别名、非法输入、范围检查、角度归一化、给定变化清积分、超长行、缓冲区溢出、环形回绕、USB 复位半行丢弃、解析字节预算。

固件构建（STM32CubeIDE 或本机 ninja）：

```bat
C:\Users\15483\AppData\Local\stm32cube\bundles\ninja\1.13.2+st.1\bin\ninja.exe -C build\Debug
```

---

## 8. 已知限制与扩展点

- 不支持无换行的“裸帧”；主机必须发送 `\r`/`\n`。
- 不提供查询命令、ACK/NAK、CRC 或鉴权；USB CDC 链路本身可靠，默认信任上位机。
- 不开放电流环增益；如需扩展，在 `BSP/USB.c` 的 `s_cmd_table` 增加 Key，并在 `PID.h` 增加对应 `PID_Loop_t` 或独立 setter。
- 参数不写入 Flash，掉电恢复为编译期默认值。
- USB 重新枚举时只清半行，已进入环形缓冲区的完整旧命令仍可能被解析。
- 上位机实际使用 rad/s 或其他单位时，在 `USB_ApplySpeedRef()` 中加换算即可，协议与解析器无需改动。