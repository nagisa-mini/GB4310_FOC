#ifndef CMSIS_COMPILER_HOST_STUB_H
#define CMSIS_COMPILER_HOST_STUB_H

/* 主机侧测试用的 CMSIS 编译器封装替身: 只提供 Ringbuffer.c 需要的 __DMB */
#define __DMB() ((void)0)

#endif /* CMSIS_COMPILER_HOST_STUB_H */
