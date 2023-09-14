#ifndef network_h
#define network_h

#include <WiFiUdp.h>

extern float PID_errorSum;

extern float Kp_user;
extern float Kd_user;
extern float Kp_thr_user;
extern float Ki_thr_user;

extern float OSCfader[4];
extern uint8_t OSCpush[4];
extern uint8_t OSCtoggle[4];

// Read control PID parameters from user. This is only for advanced users that want to "play" with the controllers...
void readControlParameters();

#endif


