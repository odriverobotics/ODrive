#ifndef __MYAPP_HPP
#define __MYAPP_HPP

#ifdef __cplusplus


extern "C" {
#endif

#include <cmsis_os.h>

extern TaskHandle_t MYAPPTaskHandle;

void start_MYAPP(void);
void resume_MYAPP(void* arg);


#ifdef __cplusplus
}
#endif

#endif 