#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <native/task.h>
#include <native/timer.h>

#define PERIOD_NS 1000000000 // 1 second

RT_TASK my_task;

void task_function(void *arg)
{
    int count = 0;
    while(1)
    {
        printf("Task count: %d\n", count++);
        rt_task_wait_period(NULL);
    }
}

int main(int argc, char** argv)
{
    int ret;

    /* create Xenomai task */
    ret = rt_task_create(&my_task, "MyTask", 0, 50, 0);
    if(ret < 0)
    {
        printf("Failed to create task: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    /* make Xenomai task periodic */
    ret = rt_task_set_periodic(&my_task, TM_NOW, PERIOD_NS);
    if(ret < 0)
    {
        printf("Failed to set task periodic: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    /* start Xenomai task */
    ret = rt_task_start(&my_task, &task_function, NULL);
    if(ret < 0)
    {
        printf("Failed to start task: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    /* wait for user input to exit */
    printf("Press enter to exit...\n");
    getchar();

    /* clean up and exit */
    ret = rt_task_delete(&my_task);
    if(ret < 0)
    {
        printf("Failed to delete task: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
