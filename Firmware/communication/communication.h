#ifndef COMMANDS_H
#define COMMANDS_H

// TODO: resolve assert
#define assert(expr)

#ifdef __cplusplus

#include <functional>
#include <limits>

extern "C" {
#endif

#include <cmsis_os.h>

extern osThreadId comm_thread;
extern const uint32_t stack_size_comm_thread;

void init_communication(void);
void initTree();
void communication_task(void * ctx);

#ifdef __cplusplus
}
#endif

#endif /* COMMANDS_H */
