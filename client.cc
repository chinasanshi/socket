
// #include<stdio.h>
// #include<stdlib.h>
// #include<string.h>
// #include<errno.h>
// #include<sys/types.h>
// #include<sys/socket.h>
// #include<netinet/in.h>

// #define MAXLINE 4096

// int main(int argc, char** argv)
// {
//     int    sockfd, n;
//     char    recvline[4096], sendline[4096];
//     struct sockaddr_in    servaddr;

//     if ( argc != 2) {
//         printf("usage: ./client <ipaddress>\n");
//         exit(0);
//     }

//     if ( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
//         printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
//         exit(0);
//     }

//     memset(&servaddr, 0, sizeof(servaddr));
//     servaddr.sin_family = AF_INET;
//     servaddr.sin_port = htons(6666);
//     if ( inet_pton(AF_INET, argv[1], &servaddr.sin_addr) <= 0) {
//         printf("inet_pton error for %s\n", argv[1]);
//         exit(0);
//     }

//     if ( connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
//         printf("connect error: %s(errno: %d)\n", strerror(errno), errno);
//         exit(0);
//     }

//     printf("send msg to server: \n");
//     fgets(sendline, 4096, stdin);
//     if ( send(sockfd, sendline, strlen(sendline), 0) < 0)
//     {
//         printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
//         exit(0);
//     }

//     close(sockfd);
//     exit(0);
// }


// #include <stdio.h>
// #include <string.h>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <syslog.h>
// #include <errno.h>
// #include <stdlib.h>
// #define MAX_LISTEN_NUM 5
// #define SEND_BUF_SIZE 100
// #define RECV_BUF_SIZE 100
// #define SERVER_PORT 1010
// int main()
// {
//     int sock_fd = 0;
//     char recvbuf[RECV_BUF_SIZE] = {0};
//     char sendbuf[SEND_BUF_SIZE] = {0};
//     int recvlen = 0;
//     int retlen = 0;
//     int sendlen = 0;
//     int leftlen = 0;
//     char *ptr = NULL;
//     struct sockaddr_in ser_addr;

//     memset(&ser_addr, 0, sizeof(ser_addr));
//     ser_addr.sin_family = AF_INET;
//     inet_aton("127.0.0.1", (struct in_addr *)&ser_addr.sin_addr);
//     ser_addr.sin_port = htons(SERVER_PORT);
//     sock_fd = socket(AF_INET, SOCK_STREAM, 0);
//     if (sock_fd < 0)
//     {
//         syslog(LOG_ERR, "%s:%d, create socket failed", __FILE__, __LINE__);
//         exit(1);
//     }
//     if (connect(sock_fd, (struct sockaddr *)&ser_addr, sizeof(ser_addr)) < 0)
//     {
//         syslog(LOG_ERR, "%s:%d, connect socket failed", __FILE__, __LINE__);
//         exit(1);
//     }
//     //receive data
//     recvlen = 0;
//     retlen = 0;
//     ptr = recvbuf;
//     leftlen = RECV_BUF_SIZE - 1;
//     //do
//     {
//         retlen = recv(sock_fd, ptr, leftlen, 0) ;
//         if (retlen < 0)
//         {
//             if (errno == EINTR)
//                 retlen = 0;
//             else
//                 exit(1);
//         }
//         recvlen += retlen;
//         leftlen -= retlen;
//         ptr += retlen;
//     }
//     //while(recvlen && leftlen);
//     printf("receive data is : %s", recvbuf);
//     sprintf(sendbuf, "hello server/n");
//     //send data
//     sendlen = strlen(sendbuf) + 1;
//     retlen = 0;
//     leftlen = sendlen;
//     ptr = sendbuf;
//     // while(leftlen)
//     {
//         retlen = send(sock_fd, ptr, sendlen, 0);
//         if (retlen < 0)
//         {
//             if (errno == EINTR)
//                 retlen = 0;
//             else
//                 exit(1);
//         }
//         leftlen -= retlen;
//         ptr += retlen;
//     }
//     close(sock_fd);

// }



// #include <stdlib.h>
// #include <sys/types.h>
// #include <stdio.h>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <string.h>

// int main()
// {
//     int cfd;
//     int recbytes;
//     int sin_size;
//     char buffer[1024] = {0};
//     struct sockaddr_in s_add, c_add;
//     unsigned short portnum = 0x8888;

//     printf("Hello,welcome to client !\r\n");

//     cfd = socket(AF_INET, SOCK_STREAM, 0);
//     if (-1 == cfd)
//     {
//         printf("socket fail ! \r\n");
//         return -1;
//     }
//     printf("socket ok !\r\n");

//     bzero(&s_add, sizeof(struct sockaddr_in));
//     s_add.sin_family = AF_INET;
//     s_add.sin_addr.s_addr = inet_addr("192.168.1.2");
//     s_add.sin_port = htons(portnum);
//     printf("s_addr = %#x ,port : %#x\r\n", s_add.sin_addr.s_addr, s_add.sin_port);


//     if (-1 == connect(cfd, (struct sockaddr *)(&s_add), sizeof(struct sockaddr)))
//     {
//         printf("connect fail !\r\n");
//         return -1;
//     }
//     printf("connect ok !\r\n");

//     if (-1 == (recbytes = read(cfd, buffer, 1024)))
//     {
//         printf("read data fail !\r\n");
//         return -1;
//     }
//     printf("read ok\r\nREC:\r\n");

//     buffer[recbytes] = '\0';
//     printf("%s\r\n", buffer);

//     getchar();
//     close(cfd);
//     return 0;
// }


#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>

#define MYPORT  8887
#define BUFFER_SIZE 1024

int main() {
    ///定义sockfd
    // int socket(int domain, int type, int protocol);
    // 功能说明：
    //    调用成功，返回socket文件描述符；失败，返回－1，并设置errno
    // 参数说明：
    //   domain：即协议域，又称为协议族（family）。常用的协议族有，AF_INET、AF_INET6、AF_LOCAL（或称AF_UNIX，Unix域socket）、AF_ROUTE等等。
    //   协议族决定了socket的地址类型，在通信中必须采用对应的地址，如AF_INET决定了要用ipv4地址（32位的）与端口号（16位的）的组合、
    //   AF_UNIX决定了要用一个绝对路径名作为地址。通常为AF_INET

    // 　　type参数指定socket的类型，基本上有三种：数据流套接字、数据报套接字、原始套接字;
    //   (常用的socket类型有，SOCK_STREAM、SOCK_DGRAM、SOCK_RAW、SOCK_PACKET、SOCK_SEQPACKET等等)

    // 　　protocol故名思意，就是指定协议。常用的协议有，IPPROTO_TCP、IPPTOTO_UDP、IPPROTO_SCTP、IPPROTO_TIPC等，它们分别对应TCP传输协议、UDP传输协议、STCP传输协议、TIPC传输协议
    //   protocol通常赋值"0"，会自动选择type类型对应的默认协议。

    // 　　两个网络程序之间的一个网络连接包括五种信息：通信协议、本地协议地址、本地主机端口、远端主机地址和远端协议端口。socket数据结构中包含这五种信息。
    int sock_cli = socket(AF_INET, SOCK_STREAM, 0);

    ///定义sockaddr_in
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(MYPORT);  ///服务器端口
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");  ///服务器ip

    ///连接服务器，成功返回0，错误返回-1
    // int connect(int sock_fd, struct sockaddr *serv_addr,int addrlen);
    // 功能说明：
    //    客户端发送服务请求。成功返回0，否则返回－1，并置errno。
    // 参数说明：
    //    sock_fd 是socket函数返回的socket描述符；serv_addr是包含远端主机IP地址和端口号的指针；addrlen是结构sockaddr_in的长度。
    if (connect(sock_cli, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("connect");
        exit(1);
    }

    char sendbuf[BUFFER_SIZE];
    char recvbuf[BUFFER_SIZE];
    while (fgets(sendbuf, sizeof(sendbuf), stdin) != NULL) {
        send(sock_cli, sendbuf, strlen(sendbuf), 0); ///发送
        if (strcmp(sendbuf, "exit\n") == 0)
            break;
        recv(sock_cli, recvbuf, sizeof(recvbuf), 0); ///接收
        fputs(recvbuf, stdout);

        memset(sendbuf, 0, sizeof(sendbuf));
        memset(recvbuf, 0, sizeof(recvbuf));
    }

    close(sock_cli);
    return 0;
}