#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define SERVER_PORT 5432
#define MAX_LINE 256

// 全局socket变量
int sock_fd;

// 接收消息的线程函数
void* receive_message(void* arg) {
    char buf[MAX_LINE];
    int len;
    
    while((len = recv(sock_fd, buf, sizeof(buf), 0)) > 0) {
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
        send(sock_fd, buf, len, 0);
    }
    return NULL;
}

int main(int argc, char* argv[]) {
    FILE *fp;
    struct hostent *hp;
    struct sockaddr_in sin;
    char *host;
    pthread_t recv_thread, send_thread;

    if(argc == 2) {
        host = argv[1];
    } else {
        fprintf(stderr, "usage: simple-talk host\n");
        exit(1);
    }

    hp = gethostbyname(host);
    if(!hp) {
        fprintf(stderr, "simple-talk: unknown host: %s\n", host);
        exit(1);
    }

    bzero((char*)&sin, sizeof(sin));
    sin.sin_family = AF_INET;
    bcopy(hp->h_addr_list[0], (char*)&sin.sin_addr, hp->h_length);
    sin.sin_port = htons(SERVER_PORT);

    if((sock_fd = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        perror("simple-talk: socket");
        exit(1);
    }

    if(connect(sock_fd, (struct sockaddr*)&sin, sizeof(sin)) < 0) {
        perror("simple-talk: connect");
        close(sock_fd);
        exit(1);
    }

    // 创建接收和发送线程
    pthread_create(&recv_thread, NULL, receive_message, NULL);
    pthread_create(&send_thread, NULL, send_message, NULL);

    // 等待线程结束
    pthread_join(recv_thread, NULL);
    pthread_join(send_thread, NULL);

    close(sock_fd);
    return 0;
}