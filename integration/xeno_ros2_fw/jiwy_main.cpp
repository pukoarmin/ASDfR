#include <stdio.h>
#include <unistd.h>
#include <iterator>
#include <signal.h>
#include <vector>
#include <sys/syscall.h>

#include "framework/multiCommClass.h"
#include "framework/runnableClass.h"
#include "framework/superThread.h"
#include "framework/icoCommClass.h"

#include "Controllers/ControllerPan/ControllerPan.h"
// #include <Your 20-sim-code-generated h-file?> Don't forget to compile the cpp file by adding it to CMakeLists.txt

#define PERIOD_20sim_ns 1000000 //1ms

RT_TASK rt_20sim;

volatile bool exitbool = false;

void exe_20sim(void *arg)
{
    20sim_o = new ControllerPan; 
    int input_length = 12;
    int output_length = 8;

    int u = { 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
    int y = { 0, -1, -1, -1, -1, -1, -1, -1 }; 

    IcoComm* icoComm = new IcoComm(y,u);
    frameworkComm *controller_uPorts[] = {
    new IDDPComm(u),
    icoComm};
    frameworkComm *controller_yPorts[] = {
    new IDDPComm(y),
    icoComm};

   runnable *wrapper = new wrapper(20sim_o, controller_uPorts, controller_yPorts, input_length, output_length); 
    wrapper->setSize(input_length, output_length);
    wrapper->prerun();

    while(1) {
        wrapper->step();
    }
}

void exit_handler(int s)
{
    printf("Caught signal %d\n", s);
    exitbool = true;
}

int main()
{
    //CREATE CNTRL-C HANDLER
    signal(SIGINT, exit_handler);

    printf("Press Ctrl-C to stop program\n"); // Note: this will 
        // not kill the program; just jump out of the wait loop. Hence,
        // you can still do proper clean-up. You are free to alter the
        // way of determining when to stop (e.g., run for a fixed time).

    // CONFIGURE, CREATE AND START THREADS HERE
    
    /* create Xenomai task */
    int ret;

    ret = rt_task_create(&rt_20sim, "rt_20sim", 0, 50, 0);
    if(ret < 0)
    {
        printf("Failed to create task: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    /* make Xenomai task periodic */
    ret = rt_task_set_periodic(&rt_20sim, TM_NOW, PERIOD_20sim_ns);
    if(ret < 0)
    {
        printf("Failed to set task periodic: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    /* start Xenomai task */
    ret = rt_task_start(&rt_20sim, &exe_20sim, NULL);
    if(ret < 0)
    {
        printf("Failed to start task: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }

    // WAIT FOR CNTRL-C
    timespec t = {.tv_sec=0, .tv_nsec=100000000}; // 1/10 second

    while (!exitbool)
    {
        // Let the threads do the real work
        nanosleep(&t, NULL);
        // Wait for Ctrl-C to exit
    }
    printf("Ctrl-C was pressed: Stopping gracefully...\n");

    //CLEANUP HERE 
    /* clean up and exit */
    ret = rt_task_delete(&t_20sim);
    if(ret < 0)
    {
        printf("Failed to delete task: %s\n", strerror(-ret));
        return EXIT_FAILURE;
    }


    return 0;
}