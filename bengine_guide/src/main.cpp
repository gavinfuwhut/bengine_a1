#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include "interface/IOROS.h"
#include "interface/KeyBoard.h"

#include "control/CtrlComponents.h"
#include "control/ControlFrame.h"



bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv)
{
    /* set real-time process 调整进程优先级*/
    setProcessScheduler();
    /* set the print format 设置小数点后显示3位数字*/
    std::cout << std::fixed << std::setprecision(3);

    ros::init(argc, argv, "bengine_gazebo_servo");

    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;

    ioInter = new IOROS();
    ctrlPlat = CtrlPlatform::GAZEBO;

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 1; // run at 1毫秒间隔，如果控制超过3毫秒则提示控制循环超过1ms
    ctrlComp->running = &running;

    ctrlComp->robotModel = new BengineRobot();

    // ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot

    // ctrlComp->geneObj();
    ControlFrame ctrlFrame(ctrlComp);

    signal(SIGINT, ShutDown);

    while (running)
    {
        ctrlFrame.run();
    }
    
    return 0;
}
