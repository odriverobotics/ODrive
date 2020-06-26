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

void init_communication(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMANDS_H */
