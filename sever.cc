
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
// 	int    listenfd, connfd;
// 	struct sockaddr_in     servaddr;
// 	char    buff[4096];
// 	int     n;

// 	if ( (listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1 ) {
// 		printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
// 		exit(0);
// 	}

// 	memset(&servaddr, 0, sizeof(servaddr));
// 	servaddr.sin_family = AF_INET;
// 	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
// 	servaddr.sin_port = htons(6666);

// 	if ( bind(listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1) {
// 		printf("bind socket error: %s(errno: %d)\n", strerror(errno), errno);
// 		exit(0);
// 	}

// 	if ( listen(listenfd, 10) == -1) {
// 		printf("listen socket error: %s(errno: %d)\n", strerror(errno), errno);
// 		exit(0);
// 	}

// 	printf("======waiting for client's request======\n");
// 	while (1) {
// 		if ( (connfd = accept(listenfd, (struct sockaddr*)NULL, NULL)) == -1) {
// 			printf("accept socket error: %s(errno: %d)", strerror(errno), errno);
// 			continue;
// 		}
// 		n = recv(connfd, buff, MAXLINE, 0);
// 		buff[n] = '\0';
// 		printf("recv msg from client: %s\n", buff);
// 		close(connfd);
// 	}

// 	close(listenfd);
// }



// #include <stdio.h>
// #include <string.h>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <stdlib.h>
// #include <syslog.h>
// #include <errno.h>
// #define MAX_LISTEN_NUM 5
// #define SEND_BUF_SIZE 100
// #define RECV_BUF_SIZE 100
// #define LISTEN_PORT 1010
// int main()
// {
// 	int listen_sock = 0;
// 	int app_sock = 0;
// 	struct sockaddr_in hostaddr;
// 	struct sockaddr_in clientaddr;
// 	int socklen = sizeof(clientaddr);
// 	char sendbuf[SEND_BUF_SIZE] = {0};
// 	char recvbuf[RECV_BUF_SIZE] = {0};
// 	int sendlen = 0;
// 	int recvlen = 0;
// 	int retlen = 0;
// 	int leftlen = 0;
// 	char *ptr = NULL;
// 	memset((void *)&hostaddr, 0, sizeof(hostaddr));
// 	memset((void *)&clientaddr, 0, sizeof(clientaddr));
// 	hostaddr.sin_family = AF_INET;
// 	hostaddr.sin_port = htons(LISTEN_PORT);
// 	hostaddr.sin_addr.s_addr = htonl(INADDR_ANY);
// 	listen_sock = socket(AF_INET, SOCK_STREAM, 0);
// 	if (listen_sock < 0)
// 	{
// 		syslog(LOG_ERR, "%s:%d, create socket failed", __FILE__, __LINE__);
// 		exit(1);
// 	}
// 	if (bind(listen_sock, (struct sockaddr *)&hostaddr, sizeof(hostaddr)) < 0)
// 	{
// 		syslog(LOG_ERR, "%s:%d, bind socket failed", __FILE__, __LINE__);
// 		exit(1);
// 	}
// 	if (listen(listen_sock, MAX_LISTEN_NUM) < 0)
// 	{
// 		syslog(LOG_ERR, "%s:%d, listen failed", __FILE__, __LINE__);
// 		exit(1);
// 	}
// 	while (1)
// 	{
// 		app_sock = accept(listen_sock, (struct sockaddr *)&clientaddr, &socklen);
// 		if (app_sock < 0)
// 		{
// 			syslog(LOG_ERR, "%s:%d, accept failed", __FILE__, __LINE__);
// 			exit(1);
// 		}
// 		sprintf(sendbuf, "welcome %s:%d here!/n", inet_ntoa(clientaddr.sin_addr.s_addr), clientaddr.sin_port);
// 		//send data
// 		sendlen = strlen(sendbuf) + 1;
// 		retlen = 0;
// 		leftlen = sendlen;
// 		ptr = sendbuf;
// 		//while(leftlen)
// 		{
// 			retlen = send(app_sock, ptr, sendlen, 0);
// 			if (retlen < 0)
// 			{
// 				if (errno == EINTR)
// 					retlen = 0;
// 				else
// 					exit(1);
// 			}
// 			leftlen -= retlen;
// 			ptr += retlen;
// 		}
// 		//receive data
// 		recvlen = 0;
// 		retlen = 0;
// 		ptr = recvbuf;
// 		leftlen = RECV_BUF_SIZE - 1;
// 		//do
// 		{
// 			retlen = recv(app_sock, ptr, leftlen, 0) ;
// 			if (retlen < 0)
// 			{
// 				if (errno == EINTR)
// 					retlen = 0;
// 				else
// 					exit(1);
// 			}
// 			recvlen += retlen;
// 			leftlen -= retlen;
// 			ptr += retlen;
// 		}
// 		//while(recvlen && leftlen);
// 		printf("receive data is : %s", recvbuf);
// 		close(app_sock);
// 	}
// 	close(listen_sock);

// 	return 0;


// }


// #include <stdlib.h>
// #include <sys/types.h>
// #include <stdio.h>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <string.h>

// int main()
// {
// 	int sfp, nfp;
// 	struct sockaddr_in s_add, c_add;
// 	int sin_size;
// 	unsigned short portnum = 0x8888;

// 	printf("Hello,welcome to my server !\r\n");
// 	sfp = socket(AF_INET, SOCK_STREAM, 0);
// 	if (-1 == sfp)
// 	{
// 		printf("socket fail ! \r\n");
// 		return -1;
// 	}
// 	printf("socket ok !\r\n");


// 	bzero(&s_add, sizeof(struct sockaddr_in));
// 	s_add.sin_family = AF_INET;
// 	s_add.sin_addr.s_addr = htonl(INADDR_ANY);
// 	s_add.sin_port = htons(portnum);

// 	if (-1 == bind(sfp, (struct sockaddr *)(&s_add), sizeof(struct sockaddr)))
// 	{
// 		printf("bind fail !\r\n");
// 		return -1;
// 	}
// 	printf("bind ok !\r\n");

// 	if (-1 == listen(sfp, 5))
// 	{
// 		printf("listen fail !\r\n");
// 		return -1;
// 	}
// 	printf("listen ok\r\n");

// 	while (1)
// 	{
// 		sin_size = sizeof(struct sockaddr_in);

// 		nfp = accept(sfp, (struct sockaddr *)(&c_add), &sin_size);
// 		if (-1 == nfp)
// 		{
// 			printf("accept fail !\r\n");
// 			return -1;
// 		}
// 		printf("accept ok!\r\nServer start get connect from %#x : %#x\r\n", ntohl(c_add.sin_addr.s_addr), ntohs(c_add.sin_port));


// 		if (-1 == write(nfp, "hello,welcome to my server \r\n", 32))
// 		{
// 			printf("write fail!\r\n");
// 			return -1;
// 		}
// 		printf("write ok!\r\n");
// 		close(nfp);

// 	}
// 	close(sfp);
// 	return 0;
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
#define QUEUE   20
#define BUFFER_SIZE 1024

int main() {
	///定义sockfd
	int server_sockfd = socket(AF_INET, SOCK_STREAM, 0);

	///定义sockaddr_in
	struct sockaddr_in server_sockaddr;
	server_sockaddr.sin_family = AF_INET;
	server_sockaddr.sin_port = htons(MYPORT);
	server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

	///bind，成功返回0，出错返回-1
	//   int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen);
	// 功能说明：
	//   将套接字和指定的端口相连。成功返回0，否则，返回－1，并置errno.
	// 参数说明：
	//   sock_fd是调用socket函数返回值,
	// 　　addr是一个指向包含有本机IP地址及端口号等信息的sockaddr类型的指针；
	// addr：一个const struct sockaddr *指针，指向要绑定给sockfd的协议地址。
	// 　　struct sockaddr_in结构类型是用来保存socket信息的：
	// 这个地址结构根据地址创建socket时的地址协议族的不同而不同，如ipv4对应的是：
	// struct sockaddr_in {
	//     sa_family_t    sin_family; /* address family: AF_INET */
	//     in_port_t      sin_port;   /* port in network byte order */
	//     struct in_addr sin_addr;   /* internet address */
	// };
	// /* Internet address. */
	// struct in_addr {
	//     uint32_t       s_addr;     /* address in network byte order */
	// };

	// ipv6对应的是：
	// struct sockaddr_in6 {
	//     sa_family_t     sin6_family;   /* AF_INET6 */
	//     in_port_t       sin6_port;     /* port number */
	//     uint32_t        sin6_flowinfo; /* IPv6 flow information */
	//     struct in6_addr sin6_addr;     /* IPv6 address */
	//     uint32_t        sin6_scope_id; /* Scope ID (new in 2.4) */
	// };
	// struct in6_addr {
	//     unsigned char   s6_addr[16];   /* IPv6 address */
	// };

	// Unix域对应的是：
	// #define UNIX_PATH_MAX    108
	// struct sockaddr_un {
	//     sa_family_t sun_family;               /* AF_UNIX */
	//     char        sun_path[UNIX_PATH_MAX];  /* pathname */
	// };

	//   addrlen为sockaddr的长度。
	if (bind(server_sockfd, (struct sockaddr *)&server_sockaddr, sizeof(server_sockaddr)) == -1) {
		perror("bind");
		exit(1);
	}

	///listen，成功返回0，出错返回-1
	// int listen(int sockfd, int backlog);
	// listen函数的第一个参数即为要监听的socket描述字，第二个参数为相应socket可以排队的最大连接个数。
	// socket()函数创建的socket默认是一个主动类型的，listen函数将socket变为被动类型的，等待客户的连接请求。
	if (listen(server_sockfd, QUEUE) == -1) {
		perror("listen");
		exit(1);
	}

	///客户端套接字
	char buffer[BUFFER_SIZE];
	struct sockaddr_in client_addr;
	socklen_t length = sizeof(client_addr);

	///成功返回非负描述字，出错返回-1
	// int accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen);
	// accept函数的第一个参数为服务器的socket描述字，第二个参数为指向struct sockaddr *的指针，用于返回客户端的协议地址，
	// 第三个参数为协议地址的长度。如果accpet成功，那么其返回值是由内核自动生成的一个全新的描述字，代表与返回客户的TCP连接。
	int conn = accept(server_sockfd, (struct sockaddr*)&client_addr, &length);
	if (conn < 0) {
		perror("connect");
		exit(1);
	}

	while (1) {
		memset(buffer, 0, sizeof(buffer));
		int len = recv(conn, buffer, sizeof(buffer), 0);
		if (strcmp(buffer, "exit\n") == 0)
			break;
		fputs(buffer, stdout);
		send(conn, buffer, len, 0);
	}

	// int close(int fd);
	// close一个TCP socket的缺省行为时把该socket标记为以关闭，然后立即返回到调用进程。该描述字不能再由调用进程使用，
	// 也就是说不能再作为read或write的第一个参数。
	// 注意：close操作只是使相应socket描述字的引用计数-1，只有当引用计数为0的时候，才会触发TCP客户端向服务器发送终止连接请求。
	close(conn);
	close(server_sockfd);
	return 0;
}