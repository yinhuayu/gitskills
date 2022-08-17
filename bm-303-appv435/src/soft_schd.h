
#include <stdint.h>

typedef void (*task_func)(uint32_t);

int task_func_register(task_func f, uint32_t interval);
void soft_schdule(void);