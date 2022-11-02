#ifndef _ALARM_H_
#define _ALARM_H_

extern int isTimeout;
extern int numRetransmissions;
extern int seconds;

void alarmHandler(int signal);

void initAlarm(int timeout);

void setAlarmTo(unsigned seconds);

void setAlarm();

void resetAlarm();

#endif // _ALARM_H_