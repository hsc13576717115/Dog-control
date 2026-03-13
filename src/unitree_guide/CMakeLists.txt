cmake_minimum_required(VERSION 3.14)
project(unitree_guide)

# 基础配置（适配香橙派5plus + Go1电机逻辑）
set(ROBOT_TYPE Go1)         # 保留Go1高层控制逻辑
set(PLATFORM arm64)         # 香橙派架构（aarch64）
set(CATKIN_MAKE ON)         # 启用ROS支持
set(SIMULATION OFF)         # 关闭仿真（实物机器人模式）
set(REAL_ROBOT ON)          # 启用实物机器人模式
set(DEBUG OFF)              # 关闭调试模式
set(MOVE_BASE OFF)          # 按需启用move_base（当前关闭）

# 必选参数校验
if(NOT DEFINED ROBOT_TYPE)  
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()
if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

# 机器人类型宏定义（复用Go1高层逻辑）
if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

# 仿真/实物模式互斥校验
if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
endif()

# ROS相关配置（仿真或move_base启用时自动开启ROS）
if(SIMULATION OR MOVE_BASE)
    add_definitions(-DRUN_ROS)
    set(CATKIN_MAKE ON)
endif()

# 编译标准与依赖基础配置
set(CMAKE_CXX_STANDARD 11)
find_package(Boost REQUIRED)

# ROS编译开关（含move_base支持）
if(CATKIN_MAKE)
    add_definitions(-DCOMPILE_WITH_ROS)
    if(MOVE_BASE)
        add_definitions(-DCOMPILE_WITH_MOVE_BASE)
    endif()
endif()

# 调试模式配置（按需启用）
if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python2 COMPONENTS Interpreter Development NumPy REQUIRED)
endif()

# ROS包依赖配置（实物模式下保留基础ROS支持）
if(CATKIN_MAKE)
    # 仿真模式额外依赖（当前关闭，保留配置）
    if(SIMULATION)
        add_definitions(-DCOMPILE_WITH_SIMULATION)
        find_package(catkin REQUIRED COMPONENTS
            controller_manager
            joint_state_controller
            gazebo_ros
        )
    endif()

    # 基础ROS依赖（实物模式需joint_state/TF等）
    find_package(catkin REQUIRED COMPONENTS
        genmsg
        robot_state_publisher
        roscpp
        std_msgs
        tf
        tf2               
        tf2_ros
        geometry_msgs
        unitree_legged_msgs
    )
    # ROS包导出配置
    catkin_package(
        CATKIN_DEPENDS 
        unitree_legged_msgs 
    )
endif()

# 头文件路径配置（核心：添加电机SDK头文件）
include_directories(
    include                                  # 项目自身头文件
    ${Boost_INCLUDE_DIRS}                    # Boost依赖
    ${catkin_INCLUDE_DIRS}                   # ROS头文件
    ${GAZEBO_INCLUDE_DIRS}                   # Gazebo头文件（仿真用，当前关闭）
    include/thirdParty/unitree_actuator_sdk-main/include  # 电机SDK核心头文件
)

# 编译选项（优化等级O3，保留Gazebo选项兼容仿真）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")

# -------------------------- 实物机器人核心配置（关键修改）--------------------------
if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)  # 启用实物模式宏

    # 1. 电机SDK库路径配置（指向预编译库所在目录）
    # 注意：你的库文件直接在lib目录下，无需加arm64子目录
    link_directories(
        include/thirdParty/unitree_actuator_sdk-main/lib  # 库文件实际路径
    )

    # 2. 移除旧的Go1 UDP SDK配置（已注释，避免冲突）
    # if(${ROBOT_TYPE} STREQUAL "Go1")
    #     include_directories(library/unitree_legged_sdk-3.8.0/include)
    #     link_directories(library/unitree_legged_sdk-3.8.0/lib/cpp/arm64)
    # endif()
endif()

# 其他库路径配置（Gazebo等，兼容仿真）
link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

# 源码文件列表（递归匹配src下所有cpp/cc文件）
file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

# 生成可执行文件（核心节点）
add_executable(junior_ctrl src/main.cpp ${SRC_LIST})

# -------------------------- 链接依赖（关键修改：链接正确的电机库）--------------------------
# 1. ROS依赖链接（按需）
if(CATKIN_MAKE)
    target_link_libraries(junior_ctrl ${catkin_LIBRARIES})
    add_dependencies(junior_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

# 2. 调试模式依赖链接（按需）
if(DEBUG)
    target_include_directories(junior_ctrl PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
    target_link_libraries(junior_ctrl Python2::Python Python2::NumPy)
endif()

# 3. 实物模式：链接电机SDK预编译库（关键！）
# 库名对应：libUnitreeMotorSDK_Arm64.so → 链接时用"UnitreeMotorSDK_Arm64"
if(REAL_ROBOT)
    # 移除旧的UDP SDK链接（已注释）
    # if(${ROBOT_TYPE} STREQUAL "Go1")
    #     target_link_libraries(junior_ctrl libunitree_legged_sdk.a)
    # endif()

    # 链接香橙派对应的arm64架构电机库
    target_link_libraries(junior_ctrl UnitreeMotorSDK_Arm64)
endif()

# 4. 基础系统依赖（线程库lcm）
target_link_libraries(junior_ctrl -pthread lcm)

# 可执行文件输出路径（非ROS模式下生效）
if(NOT CATKIN_MAKE)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
endif()

