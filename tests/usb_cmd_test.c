/*
 * USB 上位机命令解析模块主机侧测试 (不参与固件构建)
 *
 * 运行方式 (Windows / MinGW):
 *   cd 软件代码\GB4310_FOC
 *   gcc -std=c11 -O2 -Wall -Wextra tests\usb_cmd_test.c BSP\USB.c BSP\Ringbuffer.c ^
 *       -I tests\stub -I BSP -o build\usb_cmd_test.exe
 *   .\build\usb_cmd_test.exe
 *
 * 测试内容:
 *   1. 单条命令与浮点解析 (Speed / Angle / Curr / Kp / Ki / Kd / Mode);
 *   2. 跨 USB 包分片、一包多行、CR/LF/CRLF、空行;
 *   3. 大小写、Key/Value 两侧空格、短名与 *_Ref/Current 别名;
 *   4. 非法 Key/Value、缺 '='、空值、指数、负数增益、超范围、未知模式;
 *   5. 角度归一化到 (-π, π];
 *   6. 电流模式/电流增益/电流给定范围, 以及无值 save/load 命令;
 *   7. 超长行、USB_RxBuffer 溢出、环形回绕后的重新同步;
 *   8. USB_RxAbort 半行丢弃、单次解析字节预算、空缓冲区调用。
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "USB.h"
#include "PID.h"

/*------------------------------------------------------------------------------
 * PID 测试桩
 *---------------------------------------------------------------------------*/
static float g_angle_ref;
static float g_speed_ref;
static float g_curr_ref;
static float g_curr_d_ref;
static uint8_t g_mode;

static float g_gain[3][3];
static unsigned g_set_gain_calls = 0u;
static unsigned g_set_mode_calls = 0u;
static unsigned g_set_curr_calls = 0u;
static unsigned g_set_curr_d_calls = 0u;
static unsigned g_reset_integral_calls = 0u;
static unsigned g_config_save_calls = 0u;
static bool g_config_save_result = true;
static unsigned g_config_load_calls = 0u;
static bool g_config_load_result = true;
static uint8_t g_last_mode = 0u;

bool PID_GetGains(PID_Loop_t loop, float *Kp, float *Ki, float *Kd)
{
    if (((unsigned)loop > 2u) || (Kp == NULL) || (Ki == NULL) || (Kd == NULL))
    {
        return false;
    }
    *Kp = g_gain[loop][0];
    *Ki = g_gain[loop][1];
    *Kd = g_gain[loop][2];
    return true;
}

bool PID_SetGains(PID_Loop_t loop, float Kp, float Ki, float Kd)
{
    if ((unsigned)loop > 2u)
    {
        return false;
    }
    g_gain[loop][0] = Kp;
    g_gain[loop][1] = Ki;
    g_gain[loop][2] = Kd;
    g_set_gain_calls++;
    return true;
}

bool PID_SetMode(uint8_t mode)
{
    if (mode > PID_MODE_CURRENT)
    {
        return false;
    }
    g_mode = mode;
    g_last_mode = mode;
    g_set_mode_calls++;
    if (mode == PID_MODE_STOP || mode == PID_MODE_CURRENT)
    {
        g_curr_ref = 0.0f;
        g_curr_d_ref = 0.0f;
    }
    return true;
}

uint8_t PID_GetMode(void)
{
    return g_mode;
}

/* 与真实实现约定一致: 仅当给定值实际变化时清积分 */
void PID_SetAngleRef(float rad)
{
    if (g_angle_ref != rad)
    {
        g_angle_ref = rad;
        g_reset_integral_calls++;
    }
}

void PID_SetSpeedRef(float rpm)
{
    if (g_speed_ref != rpm)
    {
        g_speed_ref = rpm;
        g_reset_integral_calls++;
    }
}

bool PID_SetCurrentRef(float amp)
{
    if (g_mode != PID_MODE_CURRENT)
    {
        return false;
    }
    if (!(amp >= -1.0e30f) || !(amp <= 1.0e30f))
    {
        return false;
    }
    if (g_curr_ref != amp)
    {
        g_curr_ref = amp;
        g_reset_integral_calls++;
    }
    g_set_curr_calls++;
    return true;
}

bool PID_SetCurrentDRef(float amp)
{
    if (g_mode != PID_MODE_CURRENT)
    {
        return false;
    }
    if (!(amp >= -1.0e30f) || !(amp <= 1.0e30f))
    {
        return false;
    }
    if (g_curr_d_ref != amp)
    {
        g_curr_d_ref = amp;
        g_reset_integral_calls++;
    }
    g_set_curr_d_calls++;
    return true;
}

bool PID_Config_Save(void)
{
    g_config_save_calls++;
    return g_config_save_result;
}

bool PID_Config_Load(void)
{
    g_config_load_calls++;
    return g_config_load_result;
}

bool PID_Config_IsLoaded(void)
{
    return false;
}

/*------------------------------------------------------------------------------
 * 测试辅助
 *---------------------------------------------------------------------------*/
static unsigned g_checks = 0u;
static unsigned g_fails = 0u;

#define CHECK(cond)                                                           \
    do {                                                                      \
        g_checks++;                                                           \
        if (!(cond)) {                                                        \
            g_fails++;                                                        \
            printf("[FAIL] %s:%d %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                     \
    } while (0)

#define CHECK_INT(got, expect)                                                \
    do {                                                                      \
        g_checks++;                                                           \
        long long g_ = (long long)(got);                                      \
        long long e_ = (long long)(expect);                                   \
        if (g_ != e_) {                                                       \
            g_fails++;                                                        \
            printf("[FAIL] %s:%d got %lld expect %lld\n",                     \
                   __FILE__, __LINE__, g_, e_);                              \
        }                                                                     \
    } while (0)

#define CHECK_NEAR(got, expect, eps)                                          \
    do {                                                                      \
        g_checks++;                                                           \
        float g_ = (float)(got);                                              \
        float e_ = (float)(expect);                                           \
        float d_ = g_ - e_;                                                   \
        if (d_ < 0.0f) { d_ = -d_; }                                          \
        if (!(d_ <= (float)(eps))) {                                          \
            g_fails++;                                                        \
            printf("[FAIL] %s:%d got %.9g expect %.9g eps %.9g\n",            \
                   __FILE__, __LINE__, (double)g_, (double)e_,                \
                   (double)(eps));                                           \
        }                                                                     \
    } while (0)

static void Drain(void)
{
    uint32_t guard = 0u;
    while (!RingBuffer_IsEmpty(&USB_RxBuffer))
    {
        USB_Command_Process();
        if (++guard > 10000u)
        {
            CHECK(0);
            break;
        }
    }
}

static void FeedBytes(const void *data, size_t len)
{
    USB_RxPush((const uint8_t *)data, (uint16_t)len);
    Drain();
}

static void FeedLine(const char *line)
{
    FeedBytes(line, strlen(line));
}

static void ResetAll(void)
{
    USB_Init();

    g_angle_ref = 0.0f;
    g_speed_ref = 0.0f;
    g_curr_ref = 0.0f;
    g_curr_d_ref = 0.0f;
    g_mode = PID_MODE_SPEED;

    g_gain[PID_LOOP_ANGLE][0] = 8.0f;
    g_gain[PID_LOOP_ANGLE][1] = 0.2f;
    g_gain[PID_LOOP_ANGLE][2] = 0.0f;
    g_gain[PID_LOOP_SPEED][0] = 0.01f;
    g_gain[PID_LOOP_SPEED][1] = 0.0f;
    g_gain[PID_LOOP_SPEED][2] = 0.0f;
    g_gain[PID_LOOP_CURRENT][0] = 7.0f;
    g_gain[PID_LOOP_CURRENT][1] = 0.5f;
    g_gain[PID_LOOP_CURRENT][2] = 0.0f;

    g_set_gain_calls = 0u;
    g_set_mode_calls = 0u;
    g_set_curr_calls = 0u;
    g_set_curr_d_calls = 0u;
    g_reset_integral_calls = 0u;
    g_config_save_calls = 0u;
    g_config_save_result = true;
    g_config_load_calls = 0u;
    g_config_load_result = true;
    g_last_mode = 0u;
}

/*------------------------------------------------------------------------------
 * 测试用例
 *---------------------------------------------------------------------------*/
static void TestBasic(void)
{
    uint32_t ok;

    ResetAll();

    FeedLine("Speed=1.0\r\n");
    CHECK_NEAR(g_speed_ref, 1.0f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 1);
    CHECK_INT(g_reset_integral_calls, 1);

    FeedLine("Angle=0.785\r\n");
    CHECK_NEAR(g_angle_ref, 0.785f, 1e-5f);
    CHECK_INT(g_reset_integral_calls, 2);

    /* 给定不变时不重复清积分, 避免上位机周期刷新把积分一直按住 */
    FeedLine("Speed=1.0\r\n");
    FeedLine("Angle=0.785\r\n");
    CHECK_INT(g_reset_integral_calls, 2);

    FeedLine("Angle_Kp=9.5\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][0], 9.5f, 1e-6f);
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][1], 0.2f, 1e-6f);
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][2], 0.0f, 1e-6f);
    CHECK_INT(g_set_gain_calls, 1);

    FeedLine("Angle_Ki=0.15\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][0], 9.5f, 1e-6f);
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][1], 0.15f, 1e-6f);

    FeedLine("Angle_Kd=0.05\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][2], 0.05f, 1e-6f);

    FeedLine("Speed_Kp=0.02\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_SPEED][0], 0.02f, 1e-6f);
    CHECK_NEAR(g_gain[PID_LOOP_SPEED][1], 0.0f, 1e-6f);

    FeedLine("Speed_Ki=0.25\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_SPEED][0], 0.02f, 1e-6f);
    CHECK_NEAR(g_gain[PID_LOOP_SPEED][1], 0.25f, 1e-6f);

    FeedLine("Speed_Kd=0.125\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_SPEED][2], 0.125f, 1e-6f);

    FeedLine("Curr_Kp=7.5\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_CURRENT][0], 7.5f, 1e-6f);
    CHECK_NEAR(g_gain[PID_LOOP_CURRENT][1], 0.5f, 1e-6f);

    FeedLine("Curr_Ki=0.75\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_CURRENT][1], 0.75f, 1e-6f);

    FeedLine("Curr_Kd=0.01\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_CURRENT][2], 0.01f, 1e-6f);

    /* 改增益、切模式不应额外调用积分清零 (模式切换在 PID_SetMode 内部复位) */
    CHECK_INT(g_reset_integral_calls, 2);

    ok = USB_Cmd_OkCount;
    FeedLine("Mode=1\r\n");
    CHECK_INT(g_mode, PID_MODE_ANGLE);
    CHECK_INT(g_last_mode, PID_MODE_ANGLE);
    CHECK_INT(g_set_mode_calls, 1);
    CHECK_INT(USB_Cmd_OkCount, ok + 1);

    ok = USB_Cmd_OkCount;
    FeedLine("Mode=0\r\n");
    CHECK_INT(g_mode, PID_MODE_STOP);
    CHECK_INT(USB_Cmd_OkCount, ok + 1);

    ok = USB_Cmd_OkCount;
    FeedLine("Mode=2\r\n");
    CHECK_INT(g_mode, PID_MODE_SPEED);
    CHECK_INT(g_set_mode_calls, 3);
    CHECK_INT(USB_Cmd_OkCount, ok + 1);
    CHECK_INT(g_reset_integral_calls, 2);

    ok = USB_Cmd_OkCount;
    FeedLine("Mode=3\r\n");
    CHECK_INT(g_mode, PID_MODE_CURRENT);
    CHECK_INT(g_last_mode, PID_MODE_CURRENT);
    CHECK_INT(g_set_mode_calls, 4);
    CHECK_INT(USB_Cmd_OkCount, ok + 1);
    CHECK_INT(g_reset_integral_calls, 2);

    /* 电流模式直接给 q 轴电流; 给定变化时清积分 */
    ok = USB_Cmd_OkCount;
    FeedLine("Curr=1.25\r\n");
    CHECK_NEAR(g_curr_ref, 1.25f, 1e-6f);
    CHECK_INT(g_set_curr_calls, 1);
    CHECK_INT(g_reset_integral_calls, 3);
    CHECK_INT(USB_Cmd_OkCount, ok + 1);

    FeedLine("Curr=1.25\r\n");
    CHECK_INT(g_set_curr_calls, 2);
    CHECK_INT(g_reset_integral_calls, 3);

    CHECK_INT(USB_Cmd_ErrCount, 0);
}

static void TestFragmentation(void)
{
    ResetAll();

    FeedBytes("Spe", 3u);
    CHECK_INT(USB_Cmd_OkCount, 0);
    FeedBytes("ed=2.5", 6u);
    CHECK_INT(USB_Cmd_OkCount, 0);
    FeedBytes("\r", 1u);
    CHECK_NEAR(g_speed_ref, 2.5f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 1);
    FeedBytes("\n", 1u);
    CHECK_INT(USB_Cmd_OkCount, 1);
    CHECK_INT(USB_Cmd_ErrCount, 0);
}

static void TestFraming(void)
{
    ResetAll();

    FeedLine("Speed=3.0\nAngle=1.0\r\nSpeed=4.0\r");
    CHECK_NEAR(g_speed_ref, 4.0f, 1e-6f);
    CHECK_NEAR(g_angle_ref, 1.0f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 3);

    FeedLine("\r\n\n\r");
    CHECK_INT(USB_Cmd_OkCount, 3);
    CHECK_INT(USB_Cmd_ErrCount, 0);
}

static void TestWhitespaceAndAliases(void)
{
    ResetAll();

    FeedLine("  sPeEd = 1.25  \r\n");
    CHECK_NEAR(g_speed_ref, 1.25f, 1e-6f);

    FeedLine("ANGLE_KP=10\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][0], 10.0f, 1e-6f);

    FeedLine("angle_ref=2.0\r\n");
    CHECK_NEAR(g_angle_ref, 2.0f, 1e-6f);

    FeedLine("Speed_Ref=-3.5\r\n");
    CHECK_NEAR(g_speed_ref, -3.5f, 1e-6f);

    FeedLine("Current_Kp=7.25\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_CURRENT][0], 7.25f, 1e-6f);

    CHECK_INT(USB_Cmd_OkCount, 5);
    CHECK_INT(USB_Cmd_ErrCount, 0);
    CHECK_INT(g_reset_integral_calls, 3); /* Speed、Angle_Ref、Speed_Ref 各清一次 */
}

static void TestInvalid(void)
{
    static const char *bad[] = {
        "Speed\r\n",
        "=1.0\r\n",
        "Speed=\r\n",
        "Foo=1\r\n",
        "Speed=abc\r\n",
        "Speed=1.0x\r\n",
        "Speed=1e3\r\n",
        "Speed=+. \r\n",
        "Speed=1 2\r\n",
        "Angle_Kp=-1\r\n",
        "Angle_Kp=1000001\r\n",
        "Speed=10001\r\n",
        "Angle=10000\r\n",
        "Mode=4\r\n",
        "Mode=-1\r\n",
        "Mode=1.5\r\n",
        "Mode=2x\r\n",
    };

    ResetAll();

    for (unsigned i = 0u; i < (sizeof(bad) / sizeof(bad[0])); i++)
    {
        FeedLine(bad[i]);
    }

    CHECK_INT(USB_Cmd_OkCount, 0);
    CHECK_INT(USB_Cmd_ErrCount, (long long)(sizeof(bad) / sizeof(bad[0])));
    CHECK_NEAR(g_speed_ref, 0.0f, 1e-6f);
    CHECK_NEAR(g_angle_ref, 0.0f, 1e-6f);
    CHECK_INT(g_mode, PID_MODE_SPEED);
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][0], 8.0f, 1e-6f);
    CHECK_NEAR(g_gain[PID_LOOP_SPEED][0], 0.01f, 1e-6f);
    CHECK_INT(g_set_gain_calls, 0);
    CHECK_INT(g_set_mode_calls, 0);
    CHECK_INT(g_reset_integral_calls, 0);
}

static void TestRangesAndAngleWrap(void)
{
    ResetAll();

    FeedLine("Angle=4.0\r\n");
    CHECK_NEAR(g_angle_ref, 4.0f - 6.2831853f, 1e-5f);

    FeedLine("Angle=-4.0\r\n");
    CHECK_NEAR(g_angle_ref, -4.0f + 6.2831853f, 1e-5f);

    FeedLine("Angle=0\r\n");
    CHECK_NEAR(g_angle_ref, 0.0f, 1e-6f);

    FeedLine("Speed=-100.5\r\n");
    CHECK_NEAR(g_speed_ref, -100.5f, 1e-5f);

    FeedLine("Angle_Kp=0\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][0], 0.0f, 1e-6f);

    FeedLine("Angle_Kp=1000000\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][0], 1000000.0f, 1.0f);

    FeedLine("Speed=10000\r\n");
    CHECK_NEAR(g_speed_ref, 10000.0f, 0.01f);

    CHECK_INT(USB_Cmd_ErrCount, 0);
}

static void TestCurrentMode(void)
{
    ResetAll();

    /* 非电流模式拒绝 q/d 轴电流给定 */
    FeedLine("Curr=1.0\r\n");
    FeedLine("Id=0.2\r\n");
    CHECK_INT(g_set_curr_calls, 0);
    CHECK_INT(g_set_curr_d_calls, 0);
    CHECK_INT(USB_Cmd_ErrCount, 2);

    FeedLine("Mode=3\r\n");
    CHECK_INT(g_mode, PID_MODE_CURRENT);
    CHECK_INT(USB_Cmd_OkCount, 1);

    /* 电流环增益别名 */
    FeedLine("Current_Kp=7.2\r\n");
    FeedLine("Current_Ki=0.8\r\n");
    FeedLine("Current_Kd=0.02\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_CURRENT][0], 7.2f, 1e-6f);
    CHECK_NEAR(g_gain[PID_LOOP_CURRENT][1], 0.8f, 1e-6f);
    CHECK_NEAR(g_gain[PID_LOOP_CURRENT][2], 0.02f, 1e-6f);

    /* q 轴: 三种别名; 每次实际变化清一次积分 */
    FeedLine("Curr=-0.5\r\n");
    CHECK_NEAR(g_curr_ref, -0.5f, 1e-6f);
    FeedLine("Iq=0.8\r\n");
    CHECK_NEAR(g_curr_ref, 0.8f, 1e-6f);
    FeedLine("Curr_Ref=-0.25\r\n");
    CHECK_NEAR(g_curr_ref, -0.25f, 1e-6f);
    CHECK_INT(g_reset_integral_calls, 3);

    /* d 轴: 三种别名; 与 q 轴给定互不影响 */
    FeedLine("Id=0.3\r\n");
    CHECK_NEAR(g_curr_d_ref, 0.3f, 1e-6f);
    FeedLine("Curr_D=-0.6\r\n");
    CHECK_NEAR(g_curr_d_ref, -0.6f, 1e-6f);
    FeedLine("Id_Ref=0.2\r\n");
    CHECK_NEAR(g_curr_d_ref, 0.2f, 1e-6f);
    CHECK_NEAR(g_curr_ref, -0.25f, 1e-6f);
    CHECK_INT(g_reset_integral_calls, 6);

    /* q 轴边界值 (相电流量程 ±1.9A) 与超范围 */
    FeedLine("Curr=1.9\r\n");
    CHECK_NEAR(g_curr_ref, 1.9f, 1e-6f);
    FeedLine("Curr=-1.9\r\n");
    CHECK_NEAR(g_curr_ref, -1.9f, 1e-6f);

    /* d 轴边界值与超范围 */
    FeedLine("Id=1.9\r\n");
    CHECK_NEAR(g_curr_d_ref, 1.9f, 1e-6f);
    FeedLine("Id=-1.9\r\n");
    CHECK_NEAR(g_curr_d_ref, -1.9f, 1e-6f);

    FeedLine("Curr=1.91\r\n");
    FeedLine("Curr=-1.91\r\n");
    FeedLine("Id=1.91\r\n");
    FeedLine("Id=-1.91\r\n");
    FeedLine("Curr=abc\r\n");
    FeedLine("Id=abc\r\n");
    FeedLine("Mode=4\r\n");

    CHECK_NEAR(g_curr_ref, -1.9f, 1e-6f);
    CHECK_NEAR(g_curr_d_ref, -1.9f, 1e-6f);
    CHECK_INT(g_set_curr_calls, 5);
    CHECK_INT(g_set_curr_d_calls, 5);
    CHECK_INT(g_reset_integral_calls, 10);
    CHECK_INT(USB_Cmd_OkCount, 14);
    CHECK_INT(USB_Cmd_ErrCount, 9);
}

static void TestOverlongLine(void)
{
    char long_line[80];

    ResetAll();

    memset(long_line, 'A', 70u);
    long_line[70] = '\n';
    long_line[71] = '\0';
    FeedLine(long_line);
    CHECK_INT(USB_Cmd_OkCount, 0);
    CHECK_INT(USB_Cmd_ErrCount, 1);

    FeedLine("Speed=5.0\r\n");
    CHECK_NEAR(g_speed_ref, 5.0f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 1);
}

static void TestRxOverflow(void)
{
    uint8_t junk[USB_RX_BUFFER_SIZE];

    ResetAll();

    memset(junk, 'A', sizeof(junk));
    USB_RxPush(junk, (uint16_t)sizeof(junk));
    CHECK_INT(USB_Rx_OverflowCount, 0);

    USB_RxPush((const uint8_t *)"B", 1u);
    CHECK_INT(USB_Rx_OverflowCount, 1);

    Drain();

    /* 溢出后处于丢弃到换行状态: 这一行整行丢弃 */
    FeedLine("junk\n");
    CHECK_INT(USB_Cmd_OkCount, 0);

    FeedLine("Speed=6.0\r\n");
    CHECK_NEAR(g_speed_ref, 6.0f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 1);
}

static void TestRxAbort(void)
{
    ResetAll();

    FeedLine("Spe");
    CHECK_INT(USB_Cmd_OkCount, 0);

    USB_RxAbort();
    FeedLine("ed=7.0\r\n");
    CHECK_INT(USB_Cmd_OkCount, 0);
    CHECK_INT(USB_Cmd_ErrCount, 0);

    FeedLine("Speed=8.0\r\n");
    CHECK_NEAR(g_speed_ref, 8.0f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 1);

    /* 没有半行时 abort 不应丢弃下一条完整命令 (上电枚举场景) */
    USB_RxAbort();
    FeedLine("Speed=4.0\r\n");
    CHECK_NEAR(g_speed_ref, 4.0f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 2);
}

static void TestSaveCommand(void)
{
    ResetAll();

    /* 运行中拒绝保存, 避免 Flash 擦除阻塞电流环 */
    g_mode = PID_MODE_SPEED;
    FeedLine("save\r\n");
    CHECK_INT(g_config_save_calls, 0);
    CHECK_INT(USB_Cmd_ErrCount, 1);

    /* 停止模式允许无值 save */
    g_mode = PID_MODE_STOP;
    FeedLine("save\r\n");
    CHECK_INT(g_config_save_calls, 1);
    CHECK_INT(USB_Cmd_OkCount, 1);
    CHECK_INT(USB_Cmd_ErrCount, 1);

    /* 大小写/前后空格, 以及兼容旧 save=1 */
    FeedLine("  SaVe  \r\n");
    FeedLine("save=1\r\n");
    CHECK_INT(g_config_save_calls, 3);
    CHECK_INT(USB_Cmd_OkCount, 3);

    /* 无值 save 不接受数值 0/2/非数字/空值 */
    FeedLine("save=0\r\n");
    FeedLine("save=2\r\n");
    FeedLine("save=abc\r\n");
    FeedLine("save=\r\n");
    CHECK_INT(g_config_save_calls, 3);
    CHECK_INT(USB_Cmd_ErrCount, 5);

    /* Flash 写入失败: 命令合法但执行失败, 计入错误 */
    g_config_save_result = false;
    FeedLine("save\r\n");
    CHECK_INT(g_config_save_calls, 4);
    CHECK_INT(USB_Cmd_ErrCount, 6);
    CHECK_INT(USB_Cmd_OkCount, 3);

    /* load 无值, 读取/应用 Flash 中的 9 个增益; 不要求停止模式 */
    g_config_load_result = true;
    FeedLine("load\r\n");
    FeedLine("  LoAd  \r\n");
    FeedLine("load=1\r\n");
    CHECK_INT(g_config_load_calls, 3);
    CHECK_INT(USB_Cmd_OkCount, 6);

    FeedLine("load=0\r\n");
    FeedLine("load=abc\r\n");
    CHECK_INT(g_config_load_calls, 3);
    CHECK_INT(USB_Cmd_ErrCount, 8);

    g_config_load_result = false;
    FeedLine("load\r\n");
    CHECK_INT(g_config_load_calls, 4);
    CHECK_INT(USB_Cmd_ErrCount, 9);

    /* 无值命令后面不能再跟参数 */
    FeedLine("save 1\r\n");
    FeedLine("load 1\r\n");
    CHECK_INT(g_config_save_calls, 4);
    CHECK_INT(g_config_load_calls, 4);
    CHECK_INT(USB_Cmd_ErrCount, 11);
}

static void TestProcessBudget(void)
{
    char batch[256];
    size_t pos = 0u;

    ResetAll();

    for (unsigned i = 0u; i < 20u; i++)
    {
        memcpy(&batch[pos], "Speed=1.0\n", 10u);
        pos += 10u;
    }
    CHECK_INT(pos, 200u);

    USB_RxPush((const uint8_t *)batch, (uint16_t)pos);
    CHECK_INT(RingBuffer_GetUsed(&USB_RxBuffer), 200u);

    USB_Command_Process();
    CHECK_INT(RingBuffer_GetUsed(&USB_RxBuffer), 200u - USB_CMD_PROCESS_BUDGET);

    Drain();
    CHECK_INT(USB_Cmd_OkCount, 20);
    CHECK_NEAR(g_speed_ref, 1.0f, 1e-6f);
}

static void TestRingWrap(void)
{
    static uint8_t block[510];

    ResetAll();

    memset(block, 'A', sizeof(block));

    USB_RxPush(block, (uint16_t)sizeof(block));
    Drain();

    FeedLine("\n");
    CHECK_INT(USB_Cmd_ErrCount, 1);

    USB_RxPush(block, (uint16_t)sizeof(block));
    Drain();

    FeedLine("\n");
    CHECK_INT(USB_Cmd_ErrCount, 2);

    /* 此时 tail 位于缓冲区尾部附近, 下一条命令的 Peek 会跨回绕 */
    FeedLine("Speed=9.0\r\n");
    CHECK_NEAR(g_speed_ref, 9.0f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 1);
}

static void TestEmptyProcess(void)
{
    ResetAll();

    USB_Command_Process();
    USB_Command_Process();

    CHECK_INT(USB_Cmd_OkCount, 0);
    CHECK_INT(USB_Cmd_ErrCount, 0);
}

int main(void)
{
    printf("== USB 命令解析测试 ==\n");

    TestBasic();
    TestFragmentation();
    TestFraming();
    TestWhitespaceAndAliases();
    TestInvalid();
    TestRangesAndAngleWrap();
    TestCurrentMode();
    TestOverlongLine();
    TestRxOverflow();
    TestRxAbort();
    TestSaveCommand();
    TestProcessBudget();
    TestRingWrap();
    TestEmptyProcess();

    printf("\n检查项: %u, 失败: %u\n", g_checks, g_fails);
    if (g_fails != 0u)
    {
        printf("测试失败\n");
        return 1;
    }
    printf("全部通过\n");
    return 0;
}