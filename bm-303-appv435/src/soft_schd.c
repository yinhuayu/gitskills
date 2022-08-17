
#include "systick.h"
#include "soft_schd.h"

#define TASK_CNT_MAX (10)
#define SCH_NULL_PTR 0

typedef struct
{

    char *task_name[16];
    task_func func;
    uint32_t task_interval;
    uint32_t task_last_tick; // record by systick or timer.

} soft_task_t;

soft_task_t tasks[TASK_CNT_MAX];

int task_func_register(task_func f, uint32_t interval)
{
    for (int i = 0; i < TASK_CNT_MAX; ++i)
    {
        if (0 == tasks[i].func)
        {
            tasks[i].func = f;
            tasks[i].task_interval = interval;
            return i;
        }
    }
    return -1; // fail
}

void soft_schdule()
{
    for (int i = 0; i < TASK_CNT_MAX; ++i)
    {
         if (HAL_GetTick() - tasks[i].task_last_tick >= tasks[i].task_interval)
        {
            tasks[i].task_last_tick = HAL_GetTick();
            // do task work.
            if (tasks[i].func)
                tasks[i].func(tasks[i].task_last_tick);
        }
    }
}


void SCH_delete_Task(task_func f)
{
    uint32_t id_counter;
    for (id_counter = 0; id_counter < TASK_CNT_MAX;)
    {
        if (tasks[id_counter].func != f)
            id_counter++;
        else
        {
           // __disable_irq();
            tasks[id_counter].func = SCH_NULL_PTR;
           // __enable_irq();
            id_counter = SCH_NULL_PTR + 1;
        }
    }
}
