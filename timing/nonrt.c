#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

void *periodic_timer(void *iterations) {
    printf("Starting job\n");
    // Cast the input argument to int
    int *msec;
    msec = (int*) iterations;

    // Define the relative wait time used in clock_nanosleep
    struct timespec deadline;
    deadline.tv_sec = 0; 
    deadline.tv_nsec = 1e6; // 1 msec = 1e6 nsecs

    // Loop every msec
    int i, j;
    float computation_load;
    for (i=0; i < *msec; i++) { 
        // Wait deadline.tv_nsec relative to the monotonic clock 
        for (j=0; j < 1e6; j++) {
            computation_load += j*i/sqrt(2);
        }
        clock_nanosleep(CLOCK_MONOTONIC, 0, &deadline, NULL);
    }
    printf("Iterations: %d, 1 ms per iterations\n", i);
}

int main() {
    pthread_t thread;
    int amount_of_loop_iterations = 100; // Loop for # ms
    int iret;
    struct timespec thread_time_start;
    struct timespec thread_time_end;
    double execution_time;
    double diff_s, diff_ns;

    // Start function periodic_timer on a thread 
    iret = pthread_create( &thread, NULL, periodic_timer, (void*) &amount_of_loop_iterations);

    // Get start time 
    clock_gettime(CLOCK_MONOTONIC, &thread_time_start);

    // Wait for the thread to finish
    pthread_join(thread, NULL); 

    // Get end time
    clock_gettime(CLOCK_MONOTONIC, &thread_time_end); 

    // Calculate the time difference in ms 
    diff_s = ((double)thread_time_end.tv_sec - (double)thread_time_start.tv_sec)*1e3;
    diff_ns = ((double)thread_time_end.tv_nsec - (double)thread_time_start.tv_nsec)/1e6;
    if (diff_ns < 0 ) {
        diff_ns += 1e3;
        diff_s -= 1e3;
    }
    execution_time = diff_s + diff_ns; 

    // Print the difference in msec
    printf("The expected execution time was %d ms, the actual execution time was: %lf ms\n", amount_of_loop_iterations, execution_time);

    //printf("Thread returns: %d\n",iret);
    exit(0);
}

