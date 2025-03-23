#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define SERVER_PORT 5432
#define MAX_LINE 256
#define MAX_PENDING 5

// 全局变量存储客户端socket
int client_sock;

// 接收消息的线程函数
void* receive_message(void* arg) {
    char buf[MAX_LINE];
    int len;
    
    while((len = recv(client_sock, buf, sizeof(buf), 0)) > 0) {
        buf[len] = '\0';
        printf("收到消息: %s", buf);
    }
    return NULL;
}

// 发送消息的线程函数
void* send_message(void* arg) {
    char buf[MAX_LINE];
    
    while(fgets(buf, sizeof(buf), stdin)) {
        buf[MAX_LINE-1] = '\0';
        int len = strlen(buf) + 1;
        send(client_sock, buf, len, 0);
    }
    return NULL;
}

int main() {
    struct sockaddr_in sin;
    int len;
    int server_sock;
    pthread_t recv_thread, send_thread;

    bzero((char*)&sin, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = INADDR_ANY;
    sin.sin_port = htons(SERVER_PORT);

    if((server_sock = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        perror("simple-talk: socket");
        exit(1);
    }

    if((bind(server_sock, (struct sockaddr*)&sin, sizeof(sin))) < 0) {
        perror("simple-talk: bind");
        exit(1);
    }

    listen(server_sock, MAX_PENDING);
    
    printf("服务器启动，等待连接...\n");
    
    // 接受一个连接
    len = sizeof(sin);
    if((client_sock = accept(server_sock, (struct sockaddr*)&sin, &len)) < 0) {
        perror("simple-talk: accept");
        exit(1);
    }
    
    printf("客户端已连接\n");

    // 创建接收和发送线程
    pthread_create(&recv_thread, NULL, receive_message, NULL);
    pthread_create(&send_thread, NULL, send_message, NULL);

    // 等待线程结束
    pthread_join(recv_thread, NULL);
    pthread_join(send_thread, NULL);

    close(client_sock);
    close(server_sock);
    return 0;
}