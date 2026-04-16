/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <algorithm>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <thread>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/unitreeRobot.h"
#include "config/RobotConfig.h"
#include "control/CtrlComponents.h"
#include "interface/IOSDK.h"
#include "runtime/ControllerNode.h"
#include "runtime/RobotRunner.h"

namespace {

// 进程级运行标志，由 SIGINT 更新，供主循环平滑退出。
volatile sig_atomic_t g_running = 1;

void ShutDown(int) {
    std::cout << "stop the controller" << std::endl;
    g_running = 0;
}

// 尝试把主控制进程提升到实时调度，保证 500 Hz 左右循环更稳定。
void setProcessScheduler() {
    pid_t pid = getpid();
    const int fifo_max_priority = sched_get_priority_max(SCHED_FIFO);
    int requested_priority = fifo_max_priority;

    rlimit rtprio_limit{};
    if (getrlimit(RLIMIT_RTPRIO, &rtprio_limit) == 0 && rtprio_limit.rlim_cur != RLIM_INFINITY) {
        requested_priority = std::min<int>(fifo_max_priority, static_cast<int>(rtprio_limit.rlim_cur));
    }

    if (requested_priority <= 0) {
        std::cout << "[WARN] Realtime priority is unavailable for this session." << std::endl;
        return;
    }

    sched_param param{};
    param.sched_priority = requested_priority;
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
        std::cerr << "[ERROR] Function setProcessScheduler failed. priority="
                  << requested_priority << " reason=" << std::strerror(errno) << std::endl;
    }
}

// ROS executor 线程不参与硬实时控制，因此主动降回普通调度类。
void demoteCurrentThreadToNormalScheduler(const char* thread_name) {
    sched_param param{};
    const int rc = pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    if (rc != 0) {
        std::cerr << "[qr_guide][WARN] Failed to lower scheduler for "
                  << thread_name << ": " << std::strerror(rc) << std::endl;
    }
}

}  // namespace

int main(int argc, char** argv) {
    setProcessScheduler();
    std::cout << std::fixed << std::setprecision(3);

    rclcpp::init(argc, argv);
    signal(SIGINT, ShutDown);

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::thread ros_spin_thread;

    try {
        const std::string share_dir = ament_index_cpp::get_package_share_directory("qr_guide");
        const std::string config_path = share_dir + "/config/custom_quadruped.yaml";
        // 所有主参数都以安装后的 share/config 为准，避免源码目录和安装目录不一致。
        const qr_guide::RobotParameters parameters = qr_guide::LoadRobotParameters(config_path);

        // 入口只负责装配对象，不再承载 IMU 全局变量、手柄映射和控制细节。
        auto controller_node = std::make_shared<qr_guide::ControllerNode>();
        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(controller_node);
        ros_spin_thread = std::thread([executor]() {
            demoteCurrentThreadToNormalScheduler("ros_executor");
            executor->spin();
        });

        // 真机链路的 3 个关键对象：
        // 1. IOSDK：电机串口通信
        // 2. QuadrupedRobot：运动学/整机模型
        // 3. ControllerContext：把控制链共享对象集中到一个上下文
        auto io_interface = std::make_unique<IOSDK>(parameters.drive);
        auto robot_model = std::make_unique<QuadrupedRobot>(parameters);
        auto context = std::make_unique<ControllerContext>(
            std::move(io_interface), std::move(robot_model), parameters);

        context->ctrlPlatform = CtrlPlatform::REALROBOT;
        context->dt = 0.002;
        context->initialize();

        qr_guide::RobotRunner runner(controller_node, std::move(context));
        const int ret = runner.run(&g_running);
        if (executor) {
            executor->cancel();
        }
        rclcpp::shutdown();
        if (ros_spin_thread.joinable()) {
            ros_spin_thread.join();
        }
        return ret;
    } catch (const std::exception& e) {
        if (executor) {
            executor->cancel();
        }
        std::cerr << "[qr_guide][ERROR] " << e.what() << std::endl;
        rclcpp::shutdown();
        if (ros_spin_thread.joinable()) {
            ros_spin_thread.join();
        }
        return -1;
    }
}
