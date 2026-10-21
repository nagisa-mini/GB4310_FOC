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
 *   1. 单条命令与浮点解析 (Speed / Angle / Kp / Ki / Kd / Mode);
 *   2. 跨 USB 包分片、一包多行、CR/LF/CRLF、空行;
 *   3. 大小写、Key/Value 两侧空格、短名与 *_Ref 别名;
 *   4. 非法 Key/Value、缺 '='、空值、指数、负数增益、超范围、未知模式;
 *   5. 角度归一化到 (-π, π];
 *   6. 超长行、USB_RxBuffer 溢出、环形回绕后的重新同步;
 *   7. USB_RxAbort 半行丢弃、单次解析字节预算、空缓冲区调用。
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
PID_RUN_t PID_Run;

static float g_gain[2][3];
static unsigned g_set_gain_calls = 0u;
static unsigned g_set_mode_calls = 0u;
static unsigned g_reset_integral_calls = 0u;
static unsigned g_config_save_calls = 0u;
static bool g_config_save_result = true;
static uint8_t g_last_mode = 0u;

bool PID_GetGains(PID_Loop_t loop, float *Kp, float *Ki, float *Kd)
{
    if (((unsigned)loop > 1u) || (Kp == NULL) || (Ki == NULL) || (Kd == NULL))
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
    if ((unsigned)loop > 1u)
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
    if (mode > PID_MODE_SPEED)
    {
        return false;
    }
    PID_Run.mode = mode;
    g_last_mode = mode;
    g_set_mode_calls++;
    return true;
}

void PID_ResetIntegralAll(void)
{
    g_reset_integral_calls++;
}

bool PID_Config_Save(void)
{
    g_config_save_calls++;
    return g_config_save_result;
}

bool PID_Config_Load(void)
{
    return false;
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

    memset(&PID_Run, 0, sizeof(PID_Run));
    PID_Run.mode = PID_MODE_SPEED;

    g_gain[PID_LOOP_ANGLE][0] = 8.0f;
    g_gain[PID_LOOP_ANGLE][1] = 0.2f;
    g_gain[PID_LOOP_ANGLE][2] = 0.0f;
    g_gain[PID_LOOP_SPEED][0] = 0.01f;
    g_gain[PID_LOOP_SPEED][1] = 0.0f;
    g_gain[PID_LOOP_SPEED][2] = 0.0f;

    g_set_gain_calls = 0u;
    g_set_mode_calls = 0u;
    g_reset_integral_calls = 0u;
    g_config_save_calls = 0u;
    g_config_save_result = true;
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
    CHECK_NEAR(PID_Run.Speed_Ref, 1.0f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 1);
    CHECK_INT(g_reset_integral_calls, 1);

    FeedLine("Angle=0.785\r\n");
    CHECK_NEAR(PID_Run.Angle_Ref, 0.785f, 1e-5f);
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

    /* 改增益、切模式不应额外调用积分清零 (模式切换在 PID_SetMode 内部复位) */
    CHECK_INT(g_reset_integral_calls, 2);

    ok = USB_Cmd_OkCount;
    FeedLine("Mode=1\r\n");
    CHECK_INT(PID_Run.mode, PID_MODE_ANGLE);
    CHECK_INT(g_last_mode, PID_MODE_ANGLE);
    CHECK_INT(g_set_mode_calls, 1);
    CHECK_INT(USB_Cmd_OkCount, ok + 1);

    ok = USB_Cmd_OkCount;
    FeedLine("Mode=0\r\n");
    CHECK_INT(PID_Run.mode, PID_MODE_STOP);
    CHECK_INT(USB_Cmd_OkCount, ok + 1);

    ok = USB_Cmd_OkCount;
    FeedLine("Mode=2\r\n");
    CHECK_INT(PID_Run.mode, PID_MODE_SPEED);
    CHECK_INT(g_set_mode_calls, 3);
    CHECK_INT(USB_Cmd_OkCount, ok + 1);
    CHECK_INT(g_reset_integral_calls, 2);

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
    CHECK_NEAR(PID_Run.Speed_Ref, 2.5f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 1);
    FeedBytes("\n", 1u);
    CHECK_INT(USB_Cmd_OkCount, 1);
    CHECK_INT(USB_Cmd_ErrCount, 0);
}

static void TestFraming(void)
{
    ResetAll();

    FeedLine("Speed=3.0\nAngle=1.0\r\nSpeed=4.0\r");
    CHECK_NEAR(PID_Run.Speed_Ref, 4.0f, 1e-6f);
    CHECK_NEAR(PID_Run.Angle_Ref, 1.0f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 3);

    FeedLine("\r\n\n\r");
    CHECK_INT(USB_Cmd_OkCount, 3);
    CHECK_INT(USB_Cmd_ErrCount, 0);
}

static void TestWhitespaceAndAliases(void)
{
    ResetAll();

    FeedLine("  sPeEd = 1.25  \r\n");
    CHECK_NEAR(PID_Run.Speed_Ref, 1.25f, 1e-6f);

    FeedLine("ANGLE_KP=10\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][0], 10.0f, 1e-6f);

    FeedLine("angle_ref=2.0\r\n");
    CHECK_NEAR(PID_Run.Angle_Ref, 2.0f, 1e-6f);

    FeedLine("Speed_Ref=-3.5\r\n");
    CHECK_NEAR(PID_Run.Speed_Ref, -3.5f, 1e-6f);

    CHECK_INT(USB_Cmd_OkCount, 4);
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
        "Mode=3\r\n",
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
    CHECK_NEAR(PID_Run.Speed_Ref, 0.0f, 1e-6f);
    CHECK_NEAR(PID_Run.Angle_Ref, 0.0f, 1e-6f);
    CHECK_INT(PID_Run.mode, PID_MODE_SPEED);
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
    CHECK_NEAR(PID_Run.Angle_Ref, 4.0f - 6.2831853f, 1e-5f);

    FeedLine("Angle=-4.0\r\n");
    CHECK_NEAR(PID_Run.Angle_Ref, -4.0f + 6.2831853f, 1e-5f);

    FeedLine("Angle=0\r\n");
    CHECK_NEAR(PID_Run.Angle_Ref, 0.0f, 1e-6f);

    FeedLine("Speed=-100.5\r\n");
    CHECK_NEAR(PID_Run.Speed_Ref, -100.5f, 1e-5f);

    FeedLine("Angle_Kp=0\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][0], 0.0f, 1e-6f);

    FeedLine("Angle_Kp=1000000\r\n");
    CHECK_NEAR(g_gain[PID_LOOP_ANGLE][0], 1000000.0f, 1.0f);

    FeedLine("Speed=10000\r\n");
    CHECK_NEAR(PID_Run.Speed_Ref, 10000.0f, 0.01f);

    CHECK_INT(USB_Cmd_ErrCount, 0);
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
    CHECK_NEAR(PID_Run.Speed_Ref, 5.0f, 1e-6f);
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
    CHECK_NEAR(PID_Run.Speed_Ref, 6.0f, 1e-6f);
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
    CHECK_NEAR(PID_Run.Speed_Ref, 8.0f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 1);

    /* 没有半行时 abort 不应丢弃下一条完整命令 (上电枚举场景) */
    USB_RxAbort();
    FeedLine("Speed=4.0\r\n");
    CHECK_NEAR(PID_Run.Speed_Ref, 4.0f, 1e-6f);
    CHECK_INT(USB_Cmd_OkCount, 2);
}

static void TestSaveCommand(void)
{
    ResetAll();

    /* 运行中拒绝保存, 避免 Flash 擦除阻塞电流环 */
    PID_Run.mode = PID_MODE_SPEED;
    FeedLine("Save=1\r\n");
    CHECK_INT(g_config_save_calls, 0);
    CHECK_INT(USB_Cmd_ErrCount, 1);

    /* 停止模式允许保存 */
    PID_Run.mode = PID_MODE_STOP;
    FeedLine("Save=1\r\n");
    CHECK_INT(g_config_save_calls, 1);
    CHECK_INT(USB_Cmd_OkCount, 1);
    CHECK_INT(USB_Cmd_ErrCount, 1);

    /* 只接受 1; 大小写/空格不影响 Key 解析 */
    FeedLine("Save=0\r\n");
    FeedLine("Save=2\r\n");
    FeedLine("Save=abc\r\n");
    CHECK_INT(g_config_save_calls, 1);
    CHECK_INT(USB_Cmd_ErrCount, 4);

    /* Flash 写入失败: 命令合法但执行失败, 计入错误 */
    g_config_save_result = false;
    FeedLine("  SaVe = 1  \r\n");
    CHECK_INT(g_config_save_calls, 2);
    CHECK_INT(USB_Cmd_ErrCount, 5);
    CHECK_INT(USB_Cmd_OkCount, 1);
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
    CHECK_NEAR(PID_Run.Speed_Ref, 1.0f, 1e-6f);
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
    CHECK_NEAR(PID_Run.Speed_Ref, 9.0f, 1e-6f);
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