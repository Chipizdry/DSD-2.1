/*
 * main.h
 *
 * Created: 07.04.2021 18:10:09
 *  Author: Red
 */ 



#ifndef MAIN_H_
#define MAIN_H_

#define F_CPU              16000000UL

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
#include "Voltmeter.h"
#include "Inputs.h"
#include "Led_driver.h"
#include "Omega.h"
#include "DHT.h"
#include "MenuStatus.h"


float hd;
float tm;
float dh;
float dt;
int dh1;
int dt1;
int dh2;
int dt2;
unsigned char hum;
uint8_t data[5];
char buff [16];  //����� �������
int mn_time;
int milis;
int millis;
int interval;
int delta_U;
int delta_F;
float delta_T;
float delta_H;
int timer_M;
int timer_U;
int timer_FLV;//������ ��������� �������
unsigned char flv_alarm;// ��� ������
unsigned char temp_alarm;
unsigned char hum_alarm;
unsigned char arm;       //���� ������
int fire;
int pwr;  //������� �����
unsigned char preority;
int preasure;//��������� ����� PS
unsigned char technic;
unsigned char working;
unsigned char autorun;
unsigned char flag1;
unsigned char flag2;
unsigned char flag3;
int j;
int d;
unsigned char regim;      //����� ������ ������-����
unsigned char service;
int stat[10];//������ ������� ������
int norm[10];// ������ ������� �����
int test;//������ ������� DHT
unsigned char search;
unsigned char al;
unsigned char regim_fall;//������� �� �����
unsigned char faza_fall;
unsigned char faza_fall_1;
unsigned char faza_fall_2;
unsigned char revers;
unsigned char time_loop;
unsigned char receivemode;// ��������� ������ DHT �������
unsigned char mode;
unsigned int tt; //��������������� ���������� ��� ������������� ����
unsigned char external;//������� �������
int tmp; //����������� � INT
int relay;

#endif /* MAIN_H_ */