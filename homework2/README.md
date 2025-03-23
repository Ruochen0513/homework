## Homework 2
### Requirements
##### 使用Socket和多线程编程编写一个聊天程序，功能：
- 包含一个server程序和一个client程序
- 每个程序可以包含多个线程
- 一个线程负责接收并显示对方发送的消息
- 另一个线程负责处理本方的输入并发出消息
### 步骤
1.新建`homework2`文件夹 \
2.分别新建`./homework2/client.c`与`./homework2/server.c`两个脚本并编程，代码详情请见具体文件或本文档下方代码解析  
3.新建`CMakeLists.txt`编译脚本，内容如下：
```txt
cmake_minimum_required(VERSION 3.16.3)
project(communication)
add_executable(client client.c) 
add_executable(server server.c) 
# 链接 pthread 库
target_link_libraries(client pthread)
target_link_libraries(server pthread)
```
4.在终端输入`cmake -S ./homework2`后继续输入`make`进行编译
5.在终端中输入`./homework2/server`打开服务器，新建一个终端输入`./homework2/client 127.0.0.1`打开客户端连接服务器
### 结果演示
### 核心流程
- **客户端**：主动发起连接，通过多线程实现收发分离。
- **服务器**：被动等待连接，接受后启动线程处理通信。
$$客户端: socket -> connect -> 多线程收发 \\
服务器: socket -> bind -> listen -> accept -> 多线程收发$$
### 代码解析
#### 头文件引入解释
```c
#include <stdio.h>        // 标准输入输出函数
#include <sys/types.h>    // 系统数据类型定义（如socket相关类型）
#include <sys/socket.h>   // 套接字操作函数（socket/connect/send/recv）
#include <netinet/in.h>   // IPv4地址结构定义（struct sockaddr_in）
#include <netdb.h>        // 主机名解析函数（gethostbyname）
#include <pthread.h>      // 多线程函数（pthread_create/pthread_join）
#include <string.h>       // 字符串函数
#include <stdlib.h>       // 标准库函数（exit/malloc）
#include <unistd.h>       // 系统调用封装（close）
```
#### `client.c`代码解析
##### 全局变量与宏定义
```c
#define SERVER_PORT 5432  // 服务器端口号
#define MAX_LINE 256      // 缓冲区最大长度
int sock_fd; // 全局套接字描述符，用于多线程共享
```
##### 接收消息线程函数
```c
void* receive_message(void* arg) {
    char buf[MAX_LINE]; // 接收缓冲区
    int len; // 实际接收字节数
    while((len = recv(sock_fd, buf, sizeof(buf), 0)) > 0) { // 循环接收数据
        buf[len] = '\0'; // 添加字符串终止符
        printf("收到消息: %s", buf); // 打印接收内容
    }
    return NULL; // 线程退出
}
```
- `recv()`：从套接字读取数据，阻塞等待直到有数据到达或连接关闭。
- **循环条件**：当recv()返回0时，表示连接已关闭，退出循环。
##### 发送消息线程函数
```c
void* send_message(void* arg) {
    char buf[MAX_LINE]; // 发送缓冲区
    while(fgets(buf, sizeof(buf), stdin)) { // 循环读取用户输入
        buf[MAX_LINE-1] = '\0'; // 防止输入溢出
        int len = strlen(buf) + 1; // 计算字符串长度（含终止符）
        send(sock_fd, buf, len, 0); // 发送数据到服务器
    }
    return NULL; // 线程退出
}
```
- `fgets()`：从标准输入读取一行数据。
- `send()`：将输入内容通过套接字发送给服务器。
##### 主函数逻辑
```c
int main(int argc, char* argv[]) {
    FILE *fp;
    struct hostent *hp; // 主机信息结构体
    struct sockaddr_in sin; // 服务器地址结构
    char *host; // 服务器主机名
    pthread_t recv_thread, send_thread; // 线程ID
    // 参数校验
    if(argc == 2) {
        host = argv[1]; // 从命令行参数获取服务器地址
    } else {
        fprintf(stderr, "usage: simple-talk host\n");
        exit(1);
    }
    // DNS解析
    hp = gethostbyname(host); // 将主机名转换为IP地址
    if(!hp) {
        fprintf(stderr, "simple-talk: unknown host: %s\n", host);
        exit(1);
    }
    // 初始化服务器地址结构
    bzero((char*)&sin, sizeof(sin)); // 清空内存
    sin.sin_family = AF_INET; // IPv4地址族
    bcopy(hp->h_addr_list[0], (char*)&sin.sin_addr, hp->h_length); // 复制IP地址
    sin.sin_port = htons(SERVER_PORT); // 设置端口号（转换为网络字节序）
    // 创建套接字
    if((sock_fd = socket(PF_INET, SOCK_STREAM, 0)) < 0) { // 创建TCP套接字
        perror("simple-talk: socket");
        exit(1);
    }
    // 连接服务器
    if(connect(sock_fd, (struct sockaddr*)&sin, sizeof(sin)) < 0) {
        perror("simple-talk: connect");
        close(sock_fd);
        exit(1);
    }
    // 创建收发线程
    pthread_create(&recv_thread, NULL, receive_message, NULL); // 接收线程
    pthread_create(&send_thread, NULL, send_message, NULL);     // 发送线程
    // 等待线程结束
    pthread_join(recv_thread, NULL);
    pthread_join(send_thread, NULL);
    // 关闭套接字
    close(sock_fd);
    return 0;
}
```
- `gethostbyname()`：将主机名解析为IP地址（例如将localhost转换为127.0.0.1）。
- `sin.sin_port = htons(SERVER_PORT)`：将端口号从主机字节序转换为网络字节序。
- `socket(PF_INET, SOCK_STREAM, 0)`：创建TCP套接字。
- `connect()`：主动连接到服务器地址。
- `pthread_create()`：启动线程处理收发逻辑，实现双工通信。
#### `server.c`代码解析
##### 主函数逻辑
```c
int main() {
    struct sockaddr_in sin; // 服务器地址结构
    int len; // 地址结构长度
    int server_sock; // 监听套接字
    pthread_t recv_thread, send_thread; // 线程ID
    // 初始化服务器地址结构
    bzero((char*)&sin, sizeof(sin)); // 清空内存
    sin.sin_family = AF_INET;        // IPv4地址族
    sin.sin_addr.s_addr = INADDR_ANY; // 监听所有本地IP
    sin.sin_port = htons(SERVER_PORT); // 设置端口号
    // 创建监听套接字
    if((server_sock = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        perror("simple-talk: socket");
        exit(1);
    }
    // 绑定套接字到地址
    if((bind(server_sock, (struct sockaddr*)&sin, sizeof(sin))) < 0) {
        perror("simple-talk: bind");
        exit(1);
    }
    // 开始监听
    listen(server_sock, MAX_PENDING); // 允许最大5个等待连接
    printf("服务器启动，等待连接...\n");
    // 接受客户端连接
    len = sizeof(sin);
    if((client_sock = accept(server_sock, (struct sockaddr*)&sin, &len)) < 0) {
        perror("simple-talk: accept");
        exit(1);
    }
    printf("客户端已连接\n");
    // 创建收发线程
    pthread_create(&recv_thread, NULL, receive_message, NULL);
    pthread_create(&send_thread, NULL, send_message, NULL);
    // 等待线程结束
    pthread_join(recv_thread, NULL);
    pthread_join(send_thread, NULL);
    // 关闭套接字
    close(client_sock);
    close(server_sock);
    return 0;
}
```
- `sin.sin_addr.s_addr = INADDR_ANY`：监听所有本地网络接口。
- `bind()`：将套接字绑定到指定IP和端口。
- `listen()`：开启监听模式，等待客户端连接。
- `accept()`：阻塞等待客户端连接，返回一个新的套接字client_sock用于通信。
- `pthread_create`：与客户端代码类似，启动收发线程。

### 关键区别：客户端 vs 服务器
|步骤|客户端|服务器|
|---|---|---|
|套接字用途|主动连接 (`connect`)|监听连接 (`bind+listen+accept`)|
|地址结构|指定服务器IP (`gethostbyname`)|绑定到`INADDR_ANY`|
|多线程触发条件|连接成功后启动线程|接受连接后启动线程|
|套接字关闭|只关闭`sock_fd`|关闭`client_sock`和`server_sock`|