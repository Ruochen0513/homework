cmake_minimum_required(VERSION 3.16.3)
project(communication)
add_executable(client client.c) 
add_executable(server server.c) 


# 链接 pthread 库
target_link_libraries(client pthread)
target_link_libraries(server pthread)