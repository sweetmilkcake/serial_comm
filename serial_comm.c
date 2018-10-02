#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>

#include <linux/ioctl.h>

// dev control date end
// 02 01 12 7F
#define SERIAL_REC_NUM 4
// VTIME and VMIN is very important.
// VTIME: Time to wait for data (tenths of seconds)
#define SERIAL_VTIME 1
// VMIN: Minimum number of characters to read
#define SERIAL_VMIN SERIAL_REC_NUM

int serial_set(int fd, int speed, int bits, int event, int stop) {
    struct termios ttys;

    memset(&ttys, 0, sizeof(ttys));

    // Enable the receiver and set local mode
    // CLOCAL: Local line - do not change "owner" of port
    // CREAD: Enable receiver
    ttys.c_cflag |= (CLOCAL | CREAD);

    // Mask the character size bits
    // CSIZE: Bit mask for data bits
    ttys.c_cflag &= ~CSIZE;

    switch (speed) {
        case 9600:
            // B9600: 9600 baud
            cfsetispeed(&ttys, B9600);
            cfsetospeed(&ttys, B9600);
            break;
        case 115200:
            // B115200: 115,200 baud
            cfsetispeed(&ttys, B115200);
            cfsetospeed(&ttys, B115200);
            break;
        default:
            cfsetispeed(&ttys, B115200);
            cfsetospeed(&ttys, B115200);
            break;
    }

    switch (bits) {
        case 7:
            // 	7 data bits
            ttys.c_cflag |= CS7;
            break;
        case 8:
            // 	8 data bits
            ttys.c_cflag |= CS8;
            break;
        default:
            ttys.c_cflag |= CS8;
            break;
    }

    switch (event) {
        case 'o':
        case 'O':
            // PARENB: Enable parity bit
            ttys.c_cflag |= PARENB;
            // INPCK: Enable parity check
            // ISTRIP: Strip parity bits
            ttys.c_cflag |= (INPCK | ISTRIP);
            // PARODD: Use odd parity instead of even
            ttys.c_cflag |= PARODD;
            break;
        case 'e':
        case 'E':
            ttys.c_cflag |= PARENB;
            ttys.c_cflag |= (INPCK | ISTRIP);
            ttys.c_cflag &= ~PARODD;
            break;
        case 'n':
        case 'N':
            ttys.c_cflag &= ~PARENB;
            break;
        default:
            ttys.c_cflag &= ~PARENB;
            break;
    }

    switch (stop) {
        case 1:
            // CSTOPB: 	2 stop bits (1 otherwise)
            ttys.c_cflag &= ~CSTOPB;
            break;
        case 2:
            ttys.c_cflag |= CSTOPB;
            break;
        default:
            ttys.c_cflag &= ~CSTOPB;
            break;
    }

    // VTIME: Time to wait for data (tenths of seconds)
    ttys.c_cc[VTIME] = SERIAL_VTIME;
    // VMIN: Minimum number of characters to read
    ttys.c_cc[VMIN] = SERIAL_VMIN;

    // Hardware flow control using the CTS (Clear To Send) and RTS (Request To Send) signal lines
    // CNEW_RTSCTS, CRTSCTS: Enable hardware flow control (not supported on all platforms)
    // CNEW_RTSCTS: Also called CRTSCTS
    //ttys.c_cflag |= CRTSCTS; // Enable hardware flow control
    ttys.c_cflag &= ~CRTSCTS; // Disable hardware flow control

    // Choosing Canonical Input
    // Canonical input is line-oriented. Input characters are put into a buffer
    // which can be edited interactively by the user until a CR (carriage return)
    // or LF (line feed) character is received.
    // When selecting this mode you normally select the ICANON, ECHO, and ECHOE options
    //ttys.c_lflag |= (ICANON | ECHO | ECHOE);

    // Choosing Raw Input
    // Raw input is unprocessed. Input characters are passed through exactly as they are received,
    // when they are received. Generally you'll deselect the ICANON, ECHO, ECHOE, and ISIG options when using raw input
    ttys.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // OPOST: Postprocess output (not set = raw output)
    //ttys.c_oflag |= OPOST; // Choosing Processed Output
    ttys.c_oflag &= ~OPOST; // Choosing Raw Output


    // Flushes the input and/or output queue.
    // TCIFLUSH: flushes data received but not read.
    // TCOFLUSH: flushes data written but not transmitted.
    // TCIOFLUSH: flushes both data received but not read, and data written but not transmitted.
    tcflush(fd, TCIOFLUSH);

    // Sets the serial port settings immediately.
    // TCSANOW: Make changes now without waiting for data to complete
    // TCSADRAIN: Wait until everything has been transmitted
    // TCSAFLUSH: Flush input and output buffers and make the change
    if (tcsetattr(fd, TCSANOW, &ttys) != 0) {
        perror("serial set fail!\n");
        return -2;
    }

    return 0;
}


int main(int argc, char *argv[]) {
    int fd = 0;
    int ret = 0;
    int n = 0;
    char buf[SERIAL_REC_NUM];

    printf("serial test start.\n");

    fd = open("/dev/ttyS3", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "%s, open serial failed!\n", __FUNCTION__);
        return -1;
    }

    // Baud rate: 115200, Data bits: 8, Parity: None, Stop bits: 1
    ret = serial_set(fd, 115200, 8, 'N', 1);
    if (ret) {
        fprintf(stderr, "%s, serial set failed!\n", __FUNCTION__);
        return -1;
    }

    while (1) {
        memset(&buf, 0, sizeof(buf));
        // blocking here
        n = read(fd, &buf, SERIAL_REC_NUM);
        if (n < SERIAL_REC_NUM) {
            fprintf(stderr, "serial read fail!, n = %d\n", n);
        } else {
            printf("buf[0] = 0x%x, buf[1] = 0x%x, buf[2] = 0x%x, buf[3] = 0x%x, \n",
                   buf[0], buf[1], buf[2], buf[3]);

            n = write(fd, &buf, SERIAL_REC_NUM);
            if (n < SERIAL_REC_NUM) {
                fprintf(stderr, "serial write fail!, n = %d\n", n);
            }
        }
    }

    close(fd);

    printf("serial test end.\n");
    return 0;
}
