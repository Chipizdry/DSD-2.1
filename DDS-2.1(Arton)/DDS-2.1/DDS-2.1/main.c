/*
 * DDS-2.1.c
 *
 * Created: 07.04.2021 18:03:57
 * Author : Red
 */ 


#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "main.h"

int  volt_1, volt_2 ,volt_3;
int fire =0;                //���� ����
int preasure=0;             //�������� ������� ��������
unsigned char flag  =0;     //������������� ����������
unsigned char TypePS=0;
unsigned char TypeAL=0;
unsigned char preority=0;
unsigned char working=0;    //������ ��������� ������ �����
unsigned char autorun=0;    //������ ��������� ���� �����
unsigned char technic=0;
int temperatur=0;
int k=5;
int past;
int past2;
int timing;

char read_enable = 0;




  //----------------������ ����������� �������------------------------------------
  ISR(TIMER2_COMPA_vect){
	 //  PORTD|= (1 <<PD1); // ��������������� �����-��������� ��������� 
	  timing++;
	  Omega_slave();  
	// PORTD &=~ (1 <<PD1);  // ��������������� �����-��������� ���������  
  }
  
  ISR(TIMER1_COMPA_vect){
	  milis++;	  
  }

int main(void)
{  
	
	
	
	ADCSRA|=(1<<ADEN);// ��������� ������ ���
	ADMUX &= (0 << REFS1);
	ADMUX|=(1 << REFS0); //���������� ������� ����������
	ADMUX |= ~(1 <<ADLAR);//�������������� ������������
	ADCSRA |=(0<<ADPS0)|(1<<ADPS1)|(1<<ADPS2); //������� ������������
	ADCSRB |=0b00000000; //������������� free running mode ����������� �������������� 
	DIDR0 |= (1 << ADC4D)|(1 << ADC5D)|(1 << ADC2D)|(1 << ADC1D)|(1 << ADC0D); // ��������� �������������� �������� �����
    ACSR |= (1 << ACD);//��������� ���������� ����������
    // ��������� ������ �����/������
    DDRC = 0b00100000;
	DDRB = 0b00000010;
	PORTB= 0b00011000;
    DDRD = 0b11111110;
	PORTC= 0b00000000;
//_________________________________________________________________________________________________
/*
    TCCR0A	= 0x00;
	TCCR0B |= (0 << CS02)|(1 << CS01)|(0 << CS00)|(1 << WGM02); //������������ clk/8, ����� ������� ���
	//TCCR0B |= (1 << CS12)|(0 << CS11)|(1 << CS10)|(1 << WGM12); //������������ clk/128, ����� ������� ���
	TCNT0 = 0x00;
	OCR0A = 150; // ������������ ������ �����
	TIMSK0 = (1 << OCIE0A); // ���������� ���������� �� ����������
	*/
	TCCR1A = 0x00;
	TCCR1B |= (1 << CS12)|(0 << CS11)|(1 << CS10)|(1 << WGM12); //������������ clk/32, ����� ������� ���
	TCNT1 = 0x00;
	OCR1A = 15625; // ������������ ������ �����
	TIMSK1 = (1 << OCIE1A); // ���������� ���������� �� ����������
	
	TCCR2A|=(1<<WGM21);
	TCCR2B |= (0 << CS22)|(1 << CS21)|(0 << CS20)|(0 << WGM22); //������������ clk/8, ����� ������� ���
	TCNT2 = 0x00;
	OCR2A = 255; // ������������ ������ �����
	TIMSK2 |= (1 << OCIE2A); // ���������� ���������� �� ����������
	
	
	
	
	
	_delay_ms(200);
	
	
	PIND|=0b01000000;//��������� ������� 
	
	arm =0; 
	interval=5;
	milis=0;
	regim=3;      //����� ������ ������-����
	flag1=0;
    flag2=0;
	flag3=0;
	delta_U=0;
	delta_F=0;
	delta_T=0;
	delta_H=0;
	pwr=0;
	timer_M=0;
	timer_U=0;
	test=0;
	faza_fall=0;
	revers=0;
	regim_fall=0;
	adress=19;
	device_ID=157;// 157-���� ���������� ,89-���� ,108-���
//===================================================================================================================================================	
 regim=  EEPROM_read(0x01);    // ���������� ������� �����
 interval=EEPROM_read(0x02);   // ����� ������ �� �����
 k=EEPROM_read(0x03);          // ���������� �����������
 TypePS=EEPROM_read(0x06);     // ��� ����� PS (NC/NO)
 if(TypePS>1){TypePS=1;}       // ����������� �������� �� ��������� 
 TypeAL=EEPROM_read(0x07);     // ��� ����� AL (NC/NO)
 if(TypeAL>1){TypeAL=1;}       // ����������� �������� �� ���������
 delta_U=EEPROM_read(0x08);	 
 if(delta_U>42){delta_U=41;}       // ����������� �������� �� ���������
 delta_F=EEPROM_read(0x09);
 if(delta_F>42){delta_F=41;}
 adress=EEPROM_read(0x10);
 if(adress>128){adress=0;}
 delta_T=EEPROM_read(0x11);
 if(delta_T>62){delta_T=61;}
 delta_H=EEPROM_read(0x12);
 if(delta_H>100){delta_T=100;}
 dt1=EEPROM_read(0x13);//���������� ����������� ����� �������
 dt2=EEPROM_read(0x14);//���������� ����������� ������� ���� �������	 
 dh1=EEPROM_read(0x15);//���������� ��������� ����� ��������
 dh2=EEPROM_read(0x16);//���������� ��������� ������� ��������
 dt=dt1+(dt2*0.1); 
 dh=dh1+(dh2*0.1);
 service=0;
 fire=read_adc(0); //������ ��������� ����� �����
 preasure=read_adc(1); // ��������� ����� ����� �� �����


volts();
meter();
status();
sei();
j=0;
J0;
    while (1) 
	
  {
	current_adc();
	Alarm_line();
	
  }