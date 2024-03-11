cmake_minimum_required(VERSION 3.21)
project(CONTROL)

set(CMAKE_CXX_STANDARD 14)

# 在系统中查找安装的 Eigen3 库
find_package(Eigen3 REQUIRED)
# 添加头文件搜索路径
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OsqpEigen REQUIRED)

#PID
add_executable(pid_demo PID/pid_controller_test.cpp PID/pid_controller.cpp utils/kinematic_model.cpp)
target_link_libraries(pid_demo PRIVATE PnC)

#pure pursuit
add_executable(pure_pursuit_demo Pure_Pursuit/pure_pursuit_test.cpp  Pure_Pursuit/pure_pursuit.cpp utils/kinematic_model.cpp)
target_link_libraries(pure_pursuit_demo PRIVATE PnC)

#stanley
add_executable(stanley_demo Stanley/stanley_controller_test.cpp  Stanley/stanley_controller.cpp utils/kinematic_model.cpp)
target_link_libraries(stanley_demo PRIVATE PnC)


# 后轮位置反馈控制
add_executable(rear_wheel_feedback_demo Rear_Wheel_Feedback/main.cpp  Rear_Wheel_Feedback/RearWheelFeedback.cpp utils/MyReferencePath.cpp utils/kinematic_model.cpp)
# add_executable(rear_wheel_feedback_demo Rear_Wheel_Feedback/main.cpp   utils/reference_trajectory.cpp utils/kinematic_model.cpp)
target_link_libraries(rear_wheel_feedback_demo PRIVATE PnC)

# lqr控制
add_executable(lqr_demo LQR/lqr_controller_test.cpp  LQR/lqr_controller.cpp utils/MyReferencePath.cpp utils/kinematic_model.cpp)
target_link_libraries(lqr_demo PRIVATE PnC)

## mpc控制 osqp待实现
add_executable(mpc_demo MPC/main.cpp  MPC/MPCControl.cpp utils/kinematic_model.cpp utils/MyReferencePath.cpp )
target_link_libraries(mpc_demo PRIVATE PnC OsqpEigen::OsqpEigen)

# MPC实现轨迹跟踪，cppad/ipopt 实现
add_executable(mpc_speed_steel_demo Model_Predictive_Speed_Steel_Control/main.cpp Model_Predictive_Speed_Steel_Control/ModelPredictiveControl.cpp)
target_link_libraries(mpc_speed_steel_demo PRIVATE ipopt PnC)


