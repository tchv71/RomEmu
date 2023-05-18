// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#include "common.h"
//extern void DATA_BUS_OUT();
//extern void DATA_BUS_IN();


#define DATA_OUT  DATA_BUS_OUT();
#define DATA_IN   DATA_BUS_IN();

void wait();
void sendStart(BYTE c); 
void send(BYTE c);
void recvStart();
BYTE wrecv();
