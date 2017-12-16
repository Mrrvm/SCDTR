// C program to implement one side of FIFO
// This side reads first, then reads
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
 
int main()
{
    int fd1;
 
    // FIFO file path
    char * myfifo = "myfifo";
 
    // Creating the named file(FIFO)
    // mkfifo(<pathname>,<permission>)
    mkfifo(myfifo, 0666);
    fd1 = open(myfifo,O_RDONLY);

    char str1[80];
    while (1)
    {
        // First open in read only and read
        
        if(read(fd1, str1, 80) < 0)
            printf("Error\n");
 
        // Print the read string and close
        //printf("%s\n", str1);
 
    }
    close(fd1);
    return 0;
}