/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sched.h>
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

volatile sig_atomic_t g_running = 1;

void ShutDown(int) {
    std::cout << "stop the controller" << std::endl;
    g_running = 0;
}

void setProcessScheduler() {
    pid_t pid = getpid();
    sched_param param{};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

}  // namespace

int main(int argc, char** argv) {
    setProcessScheduler();
    std::cout << std::fixed << std::setprecision(3);

    rclcpp::init(argc, argv);
    signal(SIGINT, ShutDown);

    try {
        const std::string share_dir = ament_index_cpp::get_package_share_directory("qr_guide");
        const std::string config_path = share_dir + "/config/custom_quadruped.yaml";
        // 所有主参数都以安装后的 share/config 为准，避免源码目录和安装目录不一致。
        const qr_guide::RobotParameters parameters = qr_guide::LoadRobotParameters(config_path);

        // 入口只负责装配对象，不再承载 IMU 全局变量、手柄映射和控制细节。
        auto controller_node = std::make_shared<qr_guide::ControllerNode>();
        auto io_interface = std::make_unique<IOSDK>(parameters.drive);
        auto robot_model = std::make_unique<QuadrupedRobot>(parameters);
        auto context = std::make_unique<ControllerContext>(
            std::move(io_interface), std::move(robot_model), parameters);

        context->ctrlPlatform = CtrlPlatform::REALROBOT;
        context->dt = 0.002;
        context->initialize();

        qr_guide::RobotRunner runner(controller_node, std::move(context));
        const int ret = runner.run(&g_running);
        rclcpp::shutdown();
        return ret;
    } catch (const std::exception& e) {
        std::cerr << "[qr_guide][ERROR] " << e.what() << std::endl;
        rclcpp::shutdown();
        return -1;
    }
}
