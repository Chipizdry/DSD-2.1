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
int timer_FLV;//Таймер положения клапана

int fire;
int zero_point; //уровень фона 


int pwr;  //Питание НОРМА
unsigned char preority;
int preasure;//состояние входа PS
unsigned char technic;
unsigned char working;
unsigned char autorun;
int smoke;
int current;
int volts;
int stat[10];//Массив статуса аварий
int norm[10];// Массив статуса НОРМА
int test;//авария датчика DHT
unsigned char search;
unsigned char al;
unsigned char regim_fall;//невыход на режим
unsigned char revers;
unsigned char time_loop;
unsigned char receivemode;// состояние аварии DHT датчика
unsigned char mode;
unsigned int tt; //вспомогательная переменная для динамического меню
unsigned char external;//внешняя тревога
int tmp; //температура в INT
int relay;

#endif /* MAIN_H_ */