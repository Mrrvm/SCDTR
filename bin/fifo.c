// C program to implement one side of FIFO
// This side writes first, then reads
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
 
int main()
{
    int fd;
 
    // FIFO file path
    char * myfifo = "myfifo";
 
    // Creating the named file(FIFO)
    // mkfifo(<pathname>, <permission>)
    mkfifo(myfifo, 0666);
    fd = open(myfifo, O_RDONLY);
    char arr1[80];

    while (1)
    {
         // Read from FIFO
        read(fd, arr1, sizeof(arr1));
 
        // Print the read message
        printf("%s\n", arr1);
        close(fd);
    }
    return 0;
}