#ifndef SYSTEM_H
#define SYSTEM_H

#define ENTER_CRITICAL()   uint32_t __primask = __get_PRIMASK(); __disable_irq()
#define EXIT_CRITICAL()    __set_PRIMASK(__primask)

#endif