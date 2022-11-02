// Alarm example
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include "alarm.h"

#include <unistd.h>
#include <signal.h>
#include <stdio.h>

#define FALSE 0
#define TRUE 1

int isTimeout = FALSE;

int numRetransmissions = 0;

int seconds = 0;

// Alarm function handler
void alarmHandler(int signal) {
    isTimeout = TRUE;
    ++numRetransmissions;
}

void initAlarm(int timeout) {
    seconds = timeout;
    (void)signal(SIGALRM, alarmHandler);
}

void setAlarmTo(unsigned seconds) {
    isTimeout = FALSE;
    alarm(seconds);
}

void setAlarm() {
    setAlarmTo(seconds);
}

void resetAlarm() {
    setAlarmTo(0);
    numRetransmissions = 0;
}
