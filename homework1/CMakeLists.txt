cmake_minimum_required(VERSION 3.16.3)
project(show_image)
add_executable(show_image show_a_image.cpp) 


# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

# 链接 OpenCV 库
target_link_libraries(show_image ${OpenCV_LIBS})