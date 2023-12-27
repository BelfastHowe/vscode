cmake_minimum_required(VERSION 3.28.0)

project(vscode VERSION 1.0.0 LANGUAGES C CXX)

# 设置使用 vcpkg 工具链
set(CMAKE_TOOLCHAIN_FILE D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake CACHE STRING "Vcpkg toolchain file")

add_executable(vscode ./vsc_project/bresenham.cpp)

find_package(OpenCV REQUIRED)

target_include_directories(vscode PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vscode PRIVATE ${OpenCV_LIBS})


