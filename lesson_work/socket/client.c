#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#define SERVER_PORT 5432
#define MAX_LINE 256

int main(int argc,char* argv[]){
    FILE *fp;
    struct hostent *hp;
    struct sockaddr_in sin;
    char *host;
    char buf[MAX_LINE];
    int s;
    int len;

    if(argc==2){
        host = argv[1];
    }
    else{
        fprintf(stderr,"usage: simple-talk host\n");
        exit(1);
    }

    hp = gethostbyname(host);
    if(!hp){
        fprintf(stderr,"simple-talk: unknown host: %s\n",host);
        exit(1);
    }

    bzero((char*)&sin,sizeof(sin));
    sin.sin_family = AF_INET;
    bcopy(hp->h_addr_list[0],(char*)&sin.sin_addr,hp->h_length);
    sin.sin_port = htons(SERVER_PORT);

    if((s = socket(PF_INET,SOCK_STREAM,0))<0){
        perror("simple-talk: socket");
        exit(1);
    }

    if(connect(s,(struct sockaddr*)&sin,sizeof(sin))<0){
        perror("simple-talk: connect");
        close(s);
        exit(1);
    }

    while(fgets(buf,sizeof(buf),stdin)){
        buf[MAX_LINE-1] = '\0';
        len = strlen(buf)+1;
        send(s,buf,len,0);
    }
}