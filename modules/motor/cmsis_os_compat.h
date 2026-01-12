#ifndef CMSIS_OS_COMPAT_H
#define CMSIS_OS_COMPAT_H

#include <stdint.h>

typedef void *osThreadId_t;
typedef void *osThreadId;
#define osPriorityNormal 0

/* Minimal stubs/macros so source compiles on non-CMSIS hosts.
   These are no-ops and intended only to satisfy declarations in files. */
#define osThreadDef(name, thread, priority, instances, stack) \
    /* stub: no-op */

static inline osThreadId osThreadCreate(void *def, void *arg)
{
    (void)def; (void)arg;
    return (osThreadId)0;
}

static inline void osDelay(uint32_t ms)
{
    (void)ms;
    /* no-op for host build; embed targets should provide real CMSIS-RTOS */
}

#endif /* CMSIS_OS_COMPAT_H */