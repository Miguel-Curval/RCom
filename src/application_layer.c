 // Application layer protocol implementation

#include "application_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "helpers.h"
#include "link_layer.h"

#define DATA_PACKET_HEADER_SIZE 4

typedef  enum {
    C_DATA  = 1,
    C_START = 2,
    C_END   = 3,
} Control;

typedef enum {
    TLV_FILESIZE,
    TLV_FILENAME
} TlvType;

typedef struct {
    unsigned char type;
    unsigned char length;
    unsigned char *value_ptr;
} Tlv;

unsigned char *writeTlv(const Tlv tlv, unsigned char *bytearray) {
    bytearray[0] = tlv.type;
    bytearray[1] = tlv.length;
    memcpy(bytearray + 2, tlv.value_ptr, tlv.length);
    return bytearray + 2 + tlv.length;
}

Tlv createTlv(unsigned char *bytearray) {
    unsigned char len = bytearray[1];
    Tlv tlv = {.type = bytearray[0], .length = len, .value_ptr = malloc(len)};
    bytearray += 2;
    memcpy(tlv.value_ptr, bytearray, len);
    bytearray += len;
    return tlv;
}

void sendFile(const LinkLayer connectionParameters, const char *filename) {
    FILE* fp = fopen(filename, "rb");
    if(!fp) {
        printf("Could not read file.\n");
        exit(1);
    }
    fseek(fp, 0 , SEEK_END);
    long filesize = ftell(fp);
    rewind(fp);

    unsigned char *buffer = malloc(filesize + DATA_PACKET_HEADER_SIZE);
    if (!buffer) {
        printf("Could not allocate memory for buffer.\n");
        exit(1);
    }
    fread(buffer + DATA_PACKET_HEADER_SIZE, 1, filesize, fp);
    fclose(fp);
    
    Tlv tlvFilesize = {.type = TLV_FILESIZE, .length = sizeof filesize, .value_ptr = (unsigned char *) &filesize};
    Tlv tlvFilename = {.type = TLV_FILENAME, .length = strlen(filename), .value_ptr = (unsigned char *) filename};

    int controlPacketSize = 1 + (1 + 1 + tlvFilesize.length) + (1 + 1 + tlvFilename.length);
    unsigned char *controlPacket = malloc(controlPacketSize);
    if (!controlPacket) {
        printf("Could not allocate memory for control packet.\n");
        exit(1);
    }
    controlPacket[0] = C_START;

    unsigned char *tmp_ptr = writeTlv(tlvFilesize, controlPacket + 1);
    writeTlv(tlvFilename, tmp_ptr);

    llwrite(controlPacket, controlPacketSize);

    int currPacketStart = 0;
    long bytesToWrite = filesize;
    unsigned sequenceNumber = 0;
    unsigned numOctets = 0;
    while (bytesToWrite) {
        numOctets = MIN(MAX_PAYLOAD_SIZE - DATA_PACKET_HEADER_SIZE, bytesToWrite);
        buffer[currPacketStart] = C_DATA;
        buffer[currPacketStart + 1] = sequenceNumber++ & 0xFF;
        buffer[currPacketStart + 2] = numOctets >> 8;
        buffer[currPacketStart + 3] = numOctets;
        int bytesWritten = llwrite(buffer + currPacketStart, numOctets + DATA_PACKET_HEADER_SIZE);
        if (bytesWritten == -1) {
            printf("llwrite returned -1\n");
            exit(1);
        } else {
            bytesToWrite -= numOctets;
            currPacketStart += numOctets;
        }
}
    
    controlPacket[0] = C_END;
    llwrite(controlPacket, controlPacketSize);
    free(controlPacket);
    free(buffer);
}

void receiveFile(const LinkLayer connectionParameters) {
    unsigned char *packet = malloc(MAX_PAYLOAD_SIZE);
    if (!packet) {
        printf("Could not allocate memory for packet.\n");
        exit(1);
    }

    if (llread(packet) == -1) {
        printf("Could not read Start packet.\n");
        exit(1);
    }

    unsigned char *tmp_ptr = packet + 1;
    Tlv tlvFilesize = createTlv(tmp_ptr);
    Tlv tlvFilename = createTlv(tmp_ptr);
    long filesize = *((long *) tlvFilesize.value_ptr);

    printf("Filesize: %ld\n", filesize);

    FILE* fp = fopen("../new_penguin.gif", "wb");
    if(!fp) {
        printf("Could not write to file.\n");
        exit(1);
    }
    
    long bytesToRead = filesize;
    unsigned sequenceNumber = 0;
    unsigned numOctets = 0;
    while (bytesToRead) {
        llread(packet);
        if ((sequenceNumber & 0xFF) == packet[1]) {
            numOctets = 256 * packet[2] + packet[3];
            int bytesWritten = fwrite(packet + DATA_PACKET_HEADER_SIZE, 1, numOctets, fp);
            if (bytesWritten == -1) {
                printf("Could not write sequence %u to file.\n", sequenceNumber);
                exit(1);
            } else if (bytesWritten == numOctets) {
                ++sequenceNumber;
            } else {
                printf("Wrote %d bytes to file instead of %d bytes.\n", bytesWritten, numOctets);
                exit(1);
            }
        }
        bytesToRead -= numOctets;
    }

    llread(packet);
    //free(tlvFilesize.value_ptr);
    //free(tlvFilename.value_ptr);
    free(packet);
}

void applicationLayer(const char *serialPort, const char *roleStr, int baudRate,
                      int nTries, int timeout, const char *filename) {

    LinkLayerRole role;
    if (strcmp(roleStr, "tx") == 0) role = LlTx;
    else if (strcmp(roleStr, "rx") == 0) role = LlRx;
    else {
        printf("Role must be either tx or rx.\n");
        exit(1);
    }

    LinkLayer ll = {.role = role, .baudRate = baudRate, .nRetransmissions = nTries, .timeout = timeout};
    if (strlen(ll.serialPort) < strlen(serialPort)) strcpy(ll.serialPort, serialPort);
    else {
        printf("Serial port string too long.\n");
        exit(1);
    }

    if (llopen(ll) < 0) {
        printf("Could not establish connection.\n");
        exit(1);
    };
    
    if (role == LlTx) sendFile(ll, filename);
    else if (role == LlRx) receiveFile(ll);

    llclose(FALSE);
}
