#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>



// void printID() {
// 	printf("the exit pid:%d\n",getpid() );
// }

// int main() {
// 	pid_t pid;
// 	atexit(printID);
// 	pid = fork();
// 	if (0 == pid) {
// 		printf("i am the child process, my process id is %d\n",getpid());
// 	}
// 	else {
// 		printf("i am the parent process, my process id is %d\n",getpid());
// 		sleep(2);
// 		wait();
// 	}
// 	return 0;
// }

// int main(){
// 	fork();
// 	fork() && fork() || fork();
// 	fork();
// 	printf("+/n");
// 	return 0;
// }

int main() {
	pid_t pid1;
	pid_t pid2;
	pid1 = fork();
	pid2 = fork();
	printf("pid1:%d,pid2:%d\n", pid1, pid2);
}