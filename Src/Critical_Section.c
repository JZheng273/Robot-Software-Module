#include "Critical_Section.h"
#include "libXHU_RRC_LIB_Conf.h"

#if (!(libUSE_FREERTOS))
#include "main.h"
#else
#include "FreeRTOS.h"
#include "task.h"
#endif

void Critical_Enter(void)
{
#if (!(libUSE_FREERTOS))
__disable_irq();
#else
taskENTER_CRITICAL();
#endif
}

void Critical_Exit(void)
{
#if (!(libUSE_FREERTOS))
__enable_irq();
#else
taskEXIT_CRITICAL();
#endif
}
