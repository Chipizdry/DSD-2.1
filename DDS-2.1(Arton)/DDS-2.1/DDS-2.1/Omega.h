




#ifndef OMEGA_H_
#define OMEGA_H_

#include "main.h"

int stats;
int active;
int low;
unsigned char bit_flag;
unsigned char hi_flag;
int tct;
int input_bit;
int adres_call;
long rd;


int adress_t;
int adress;
int directive;
int device_ID;// 157-���� ���������� ,89-���� ,108-���
int temp_ID;

int frame;
int answer;


int detect[145];




void Omega_slave();
void protocol();


#endif /* OMEGA(SLAVE)_H_ */