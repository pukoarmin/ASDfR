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

#include "ControllerPan/ControllerPan.h"
// #include <Your 20-sim-code-generated h-file?> Don't forget to compile the cpp file by adding it to CMakeLists.txt

//#define PERIOD_20sim_ns 1000000 //1ms

//RT_TASK rt_20sim;

volatile bool exitbool = false;

void ReadConvert(const double* src, double* dst)
{
    static double lastKnownGoodValue = 0;S
    /* We assume that src is a 3 element double array, which has:
    * src[0] = temperature reading in degrees Fahrenheit (scaling)
    * -> We want the output in degrees Celcius
    * src[1] = some measurement with unreliable communication; once
    * in a while a (faulty) 0 is received; this should be
    * filtered and the previous value should be passed (filtering)
    * src[2] = some good measurement which does not need
    * scaling neither filtering
    */
    dst[0] = (src[0]-32.0) / 1.8; // scaling
    //if ( src[1]!=0) // filtering
    //{
    //    dst[1] = src[1];
    //    lastKnownGoodValue = src[1];
    //}
    //else
    //{
    //    dst[1] = lastKnownGoodValue;
    //}
    //    dst[2] = src[2]; // passing

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
    ControllerPan* sim20_o = new ControllerPan; 
    int input_length = 2;
    int output_length = 1;

    int u[] = { 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
    int y[] = { 0, -1, -1, -1, -1, -1, -1, -1 }; 

    IcoComm* icoComm = new IcoComm(y,u);
    frameworkComm *controller_uPorts[] = {
    new XDDPComm(10, -1, 1, 0),
    icoComm};
    frameworkComm *controller_yPorts[] = {
    icoComm};

   runnable *controller_runnable = new wrapper<ControllerPan>(sim20_o, controller_uPorts, controller_yPorts, input_length, output_length); 
    controller_runnable->setSize(1, 0);
    /* create Xenomai task */

    xenoThread controllerClass(controller_runnable);
    controllerClass.init(1000000,98,0);

    //controllerClass.enableLogging(true, 26);
    //Start threads gcc jiwy_main.cpp -o test $(/usr/xenomai/bin/xeno-config --skin=posix --cflags --ldflags)

    controllerClass.start("controller");

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
    controllerClass.stopThread();
    // controller_runnable->~xenoThread();
    // wrapper->~runnable();

    return 0;
}