#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>

#define PERIOD_NS 1000000000 // 1 second
#define ITER 100

RT_TASK rt_timing_task;
int period_counter = 0;

void task_function(void *arg)
{
    float computation_load;

    while(1){ 
        rt_task_wait_period(NULL);
        computation_load += 69 * 420 / sqrt(2);
        printf("Task iter: %d\n", period_counter++);
    }
}

int main(int argc, char** argv)
{
    int ret;
    struct timespec thread_time_start;
    struct timespec thread_time_end;
    double execution_time;
    double diff_s, diff_ns;

    /* create Xenomai task */
    ret = rt_task_create(&rt_timing_task, "rt_timing_task", 0, 50, 0);
    if(ret < 0)
    {
        printf("Failed to create task: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    /* make Xenomai task periodic */
    ret = rt_task_set_periodic(&rt_timing_task, TM_NOW, PERIOD_NS);
    if(ret < 0)
    {
        printf("Failed to set task periodic: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    // Get start time 
    clock_gettime(CLOCK_MONOTONIC, &thread_time_start);

    /* start Xenomai task */
    ret = rt_task_start(&rt_timing_task, &task_function, NULL);
    if(ret < 0)
    {
        printf("Failed to start task: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    /* wait for user input to exit */
    printf("Press enter to exit...\n");
    getchar();

    // Get end time
    clock_gettime(CLOCK_MONOTONIC, &thread_time_end); 

    // Calculate the time difference in ms 
    diff_s = ((double)thread_time_end.tv_sec - (double)thread_time_start.tv_sec)*1e3;
    diff_ns = ((double)thread_time_end.tv_nsec - (double)thread_time_start.tv_nsec)/1e6;
    if (diff_ns < 0 ) {
        diff_ns += 1e3;
        diff_s -= 1e3;
    }
    execution_time = diff_s - period_counter*PERIOD_NS/1000000 + diff_ns; 

    // Print the difference in msec
    printf("The avg. period execution time over: %d periods was: %lf ms, excluding the period wait\n", period_counter, execution_time);


    /* clean up and exit */
    ret = rt_task_delete(&rt_timing_task);
    if(ret < 0)
    {
        printf("Failed to delete task: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
