#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#define SERVER_PORT 5432
#define MAX_LINE 256
#define MAX_PENDING 5

int main(){
    struct sockaddr_in sin;
    char buf[MAX_LINE];
    int len;
    int s, new_s;
    bzero((char*)&sin,sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = INADDR_ANY;
    sin.sin_port = htons(SERVER_PORT);
    if((s = socket(PF_INET,SOCK_STREAM,0))<0){
        perror("simple-talk: socket");
        exit(1);
    }
    if((bind(s,(struct sockaddr*)&sin,sizeof(sin)))<0){
        perror("simple-talk: bind");
        exit(1);
    }
    listen(s,MAX_PENDING);
    while(1){
        if((new_s = accept(s,(struct sockaddr*)&sin,&len))<0){
            perror("simple-talk: accept");
            exit(1);
        }
        
        while((len = recv(new_s,buf,sizeof(buf),0))){
            fputs(buf,stdout);
        }
        close(new_s);
    }

}