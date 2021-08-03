/*
 * main.h
 *
 * Created: 07.04.2021 18:10:09
 *  Author: Red
 */ 



#ifndef MAIN_H_
#define MAIN_H_

#define F_CPU              3579545UL

#define DHT_PORT        PORTD
#define DHT_DDR         DDRD
#define DHT_PIN         PIND
#define DHT_BIT         7


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>


#include "ADConverter.h"
#include "EepromDATA.h"
#include "Omega.h"


int delta_U;
int delta_F;
float delta_T;
float delta_H;
int timer_M;
int timer_U;
int timer_FLV;//������ ��������� �������

int fire;
int zero_point; //������� ���� 


int pwr;  //������� �����
unsigned char preority;
int preasure;//��������� ����� PS
unsigned char technic;
unsigned char working;
unsigned char autorun;
int smoke;
int current;
int volts;
int stat[10];//������ ������� ������
int norm[10];// ������ ������� �����
int test;//������ ������� DHT
unsigned char search;
unsigned char al;
unsigned char regim_fall;//������� �� �����
unsigned char revers;
unsigned char time_loop;
unsigned char receivemode;// ��������� ������ DHT �������
unsigned char mode;
unsigned int tt; //��������������� ���������� ��� ������������� ����
unsigned char external;//������� �������
int tmp; //����������� � INT
int relay;

#endif /* MAIN_H_ */