# 指定使用的 cmake 的最低版本
cmake_minimum_required(VERSION 3.21)
project(chhRobotics_CPP)

# 指定使用的C++标准
set(CMAKE_CXX_STANDARD 14) 


#include(GNUInstallDirs)
# 创建一个名为chhRobotics_CPP
set(PACKAGE_NAME chhRobotics_CPP)


# Library target
# 创建一个名为chhRobotics_CPP的接口库，与普通的静态或动态库不同，这里创建的是一个 INTERFACE 库，它本身不会产生任何构建输出。
# 相反，INTERFACE 库只是一组编译器和链接器选项的集合，这些选项可以被其他目标（如可执行文件或其他库）引用和继承。
add_library(PnC INTERFACE)


# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
# 查找Python3的解释器和开发组件，并将其作为必需组件
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
# 将Python3的Python和Module组件链接到chhRobotics_CPP库目标
target_link_libraries(PnC INTERFACE
        Python3::Python
        Python3::Module
        )
# 查找Python3的NumPy组件
find_package(Python3 COMPONENTS NumPy)
# 如果找到了NumPy组件
if(Python3_NumPy_FOUND)
# 将Python3的NumPy组件链接到chhRobotics_CPP库目标
    target_link_libraries(PnC INTERFACE
            Python3::NumPy
            )
else()
# 在chhRobotics_CPP库目标中定义WITHOUT_NUMPY宏。
    target_compile_definitions(PnC INTERFACE WITHOUT_NUMPY)
endif()
# 安装chhRobotics_CPP库目标，并导出安装目标。
install(
        TARGETS PnC
        EXPORT install_targets
)


# 定创建一个名为main的可执行目标，使用main.cpp作为源文件。
add_executable(main main.cpp)
# 将chhRobotics_CPP库目标链接到main可执行目标。链接方式为 PRIVATE，表示链接只适用于当前目标，而不会影响其他目标。
target_link_libraries(main PRIVATE PnC)

# 将Control目录作为子目录添加到构建中，并指定输出目录为bin_control。
add_subdirectory(Control bin)
# 将Planning目录作为子目录添加到构建中，并指定输出目录为bin_planning。
add_subdirectory(Planning bin_planning)

