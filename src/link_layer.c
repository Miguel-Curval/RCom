// Link layer protocol implementation

#include "link_layer.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "alarm.h"

// MISC

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7E
#define ADDRESS_TRANSMISSION 0x03
#define ADDRESS_RECEPTION 0x01

#define ESCAPE 0x7D

#define STUFF(b) ((b) ^ 0x20)
#define DESTUFF(b) STUFF(b)

#define C_RR(n) ((unsigned char) (((n) << 7) | 0x5))
#define C_REJ(n) ((unsigned char) (((n) << 7) | 0x1))
#define C_R(isPositive, n) ((unsigned char) ((((n) << 7) ^ 0x80) | 0x1 | ((isPositive) << 2)))
#define C_I(n) ((unsigned char) (((n) << 6) & 0x40))

// #define ADDRESS(isReceptor) ((isReceptor) ? ADDRESS_RECEPTION : ADDRESS_TRANSMISSION)

#define BCC1(control) (FLAG ^ ADDRESS_TRANSMISSION ^ (control))

#define FRAME_SU_SIZE 5
#define FRAME_I_HEADER_SIZE 4
#define FRAME_I_TRAILER_SIZE 2

typedef  enum {
    C_SET  = 0x03,
    C_DISC = 0x0B,
    C_UA   = 0x07,
    C_RR0  = C_RR(0),
    C_RR1  = C_RR(1),
    C_REJ0 = C_REJ(0),
    C_REJ1 = C_REJ(1),
    C_I0 = C_I(0),
    C_I1 = C_I(1)
} Control;

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    DATA_RCV,
    ESC_RCV
} StateMachine;

#define IS_ACK(c) (((c) == C_RR0) || ((c) == C_RR1) || ((c) == C_REJ0) || ((c) == C_REJ1))
#define CAN_SEND_PACKET(n, c) ((((c) == C_RR0) && !((n) & 0x1)) || (((c) == C_RR1) && ((n) & 0x1)))

// GLOBALS

LinkLayer connectionParameters = {0};

int fd = -1;

struct termios oldtio = {0};
struct termios newtio = {0};

int numPackets = 0;

unsigned char FLAG_SEQUENCE[2] = {ESCAPE, STUFF(FLAG)};
unsigned char ESCAPE_SEQUENCE[2] = {ESCAPE, STUFF(ESCAPE)};

int awaitFrameSU(Control control) {
    StateMachine sm = START;
    unsigned char c = 0;
    unsigned char bcc1 = BCC1(control);
    while (!isTimeout) {
        read(fd, &c, 1);
        switch (sm) {
            case START:
                if (c == FLAG) sm = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (c == ADDRESS_TRANSMISSION) sm = A_RCV;
                else if (c != FLAG) sm = START;
                break;
            case A_RCV:
                if (c == control) sm = C_RCV;
                else if (c == FLAG) sm = FLAG_RCV;
                else sm = START;
                break;
            case C_RCV:
                if (c == bcc1) sm = BCC_OK;
                else if (c == FLAG) sm = FLAG_RCV;
                else sm = START;
                break;
            case BCC_OK:
                if (c == FLAG) return TRUE;
                else sm = START;
                break;
            default:
                printf("State Machine go brrrrr! : %d\n", sm);
                exit(1);
        }
    }
    return FALSE;
}

int sendFrameSU(Control control) {
    unsigned char frame[FRAME_SU_SIZE] = {FLAG, ADDRESS_TRANSMISSION, control, BCC1(control), FLAG};
    return write(fd, frame, FRAME_SU_SIZE);
}

int sendAwait(Control c1, Control c2) {
    while (numRetransmissions < connectionParameters.nRetransmissions) {
        sendFrameSU(c1);
        setAlarm();
        if (awaitFrameSU(c2)){
            resetAlarm();
            return TRUE;
        }
    }
    return FALSE;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer newConnectionParameters) {
    printf("Establishing connection.\n");
    connectionParameters = newConnectionParameters;
    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd < 0) {
        perror(connectionParameters.serialPort);
        return -1;
    }

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        return -1;
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 5;  // Blocking read until 5 chars received

    // VTIME and VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    printf("New termios structure set\n");

    initAlarm(connectionParameters.timeout);

    if (connectionParameters.role == LlTx) {
        sendAwait(C_SET, C_UA);
    } else if (connectionParameters.role == LlRx) {
        awaitFrameSU(C_SET);
        sendFrameSU(C_UA);
    }
    
    printf("Connection established.\n");
    
    return 1;
}

int awaitACK() {
    printf("Awaiting ACK for packet number %d.\n", numPackets);
    StateMachine sm = START;
    unsigned char c = 0;
    unsigned char control = 0;
    while (!isTimeout) {
        if (read(fd, &c, 1) != 1) continue;
        switch (sm) {
            case START:
                if (c == FLAG) sm = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (c == ADDRESS_TRANSMISSION) sm = A_RCV;
                else if (c != FLAG) sm = START;
                break;
            case A_RCV:
                if (IS_ACK(c)) {
                    control = c;
                    sm = C_RCV;
                } else if (c == FLAG) sm = FLAG_RCV;
                else sm = START;
                break;
            case C_RCV:
                if (c == BCC1(control)) sm = BCC_OK;
                else if (c == FLAG) sm = FLAG_RCV;
                else sm = START;
                break;
            case BCC_OK:
                if (c == FLAG) return CAN_SEND_PACKET(numPackets + 1, control);
                else sm = START;
                break;
            default:
                printf("State Machine go brrrrr! : %d\n", sm);
                exit(1);
        }
    }
    return FALSE;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
    unsigned char control = C_I(numPackets);
    unsigned char frameHeader[FRAME_I_HEADER_SIZE] = {FLAG, ADDRESS_TRANSMISSION, control, BCC1(control)};
    unsigned char frameTrailer[FRAME_I_TRAILER_SIZE] = {0, FLAG};
    while (numRetransmissions < connectionParameters.nRetransmissions) {
        printf("Sending packet number %d. Transmission number %d.\n", numPackets, numRetransmissions);
        int bytes = write(fd, frameHeader, FRAME_I_HEADER_SIZE);

        unsigned char bcc2 = 0;
    
        for (int i = 0; i < bufSize; ++i) {
            bcc2 ^= buf[i];
            if (buf[i] == 0x7E) {
                bytes += write(fd, FLAG_SEQUENCE, sizeof FLAG_SEQUENCE);
            } else if (buf[i] == 0x7D) {
                bytes += write(fd, ESCAPE_SEQUENCE, sizeof ESCAPE_SEQUENCE);
            } else {
                bytes += write(fd, buf + i, 1);
            }
        }
        frameTrailer[0] = bcc2;
        bytes += write(fd, frameTrailer, FRAME_I_TRAILER_SIZE);

        setAlarm();
        if (awaitACK()) {
            ++numPackets;
            resetAlarm();
            return bytes;
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {
    StateMachine sm = START;
    unsigned char control = C_I(numPackets);
    printf("Reading packet number %d.\n", numPackets);
    unsigned char bcc1 = BCC1(control);
    unsigned char bcc2 = 0;
    unsigned char c = 0;
    int idx = 0;
    unsigned char frame[FRAME_SU_SIZE] = {FLAG, ADDRESS_TRANSMISSION, 0, 0, FLAG};
    while (!isTimeout) {
        if (read(fd, &c, 1) != 1) continue;
        switch (sm) {
            case START:
                if (c == FLAG) sm = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (c == ADDRESS_TRANSMISSION) sm = A_RCV;
                else if (c != FLAG) sm = START;
                break;
            case A_RCV:
                if (c == control) sm = C_RCV;
                else if (c == FLAG) sm = FLAG_RCV;
                else sm = START;
                break;
            case C_RCV:
                if (c == bcc1) sm = BCC_OK;
                else if (c == FLAG) sm = FLAG_RCV;
                else sm = START;
                break;
            case BCC_OK:
                if (c == FLAG) {
                    sm = FLAG_RCV;
                    break;
                }
                idx = 0;
                bcc2 = 0;
                if (c == ESCAPE) sm = ESC_RCV;
                else {
                    bcc2 ^= c;
                    packet[idx++] = c;
                    sm = DATA_RCV;
                }
                break;
            case DATA_RCV:
                if (c == FLAG) {
                    int isPositive = !bcc2;
                    unsigned char controlReply = C_R(isPositive, numPackets);
                    frame[2] = controlReply;
                    frame[3] = BCC1(controlReply);
                    /*for (int i = 0; i < 1000; ++i)*/ write(fd, frame, FRAME_SU_SIZE); // TODO

                    if (isPositive) {
                        ++numPackets;
                        return idx;
                    }
                    else sm = FLAG_RCV;
                } else if (c == ESCAPE) sm = ESC_RCV;
                else {
                    bcc2 ^= c;
                    packet[idx++] = c;
                }
                break;
            case ESC_RCV:
                if (c == FLAG) {
                    sm = FLAG_RCV;
                    break;
                }
                c = DESTUFF(c);
                if (c == FLAG || c == ESCAPE) {
                    bcc2 ^= c;
                    packet[idx++] = c;
                    sm = DATA_RCV;
                } else sm = START;
                break;
            default:
                printf("State Machine go brrrrr! : %d\n", sm);
                exit(1);
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics) {
    printf("Closing connection.\n");
    if (connectionParameters.role == LlTx) {
        sendAwait(C_DISC, C_DISC);
        sendFrameSU(C_UA);
    }
    else if (connectionParameters.role == LlRx) {
        awaitFrameSU(C_DISC);
        sendAwait(C_DISC, C_UA);
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    close(fd);

    printf("Connection closed.\n");

    return 1;
}
