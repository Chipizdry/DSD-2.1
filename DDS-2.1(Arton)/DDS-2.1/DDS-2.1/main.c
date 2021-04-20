/*
 * DDS-2.1.c
 *
 * Created: 07.04.2021 18:03:57
 * Author : Red
 */ 


#define F_CPU 3759000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "main.h"


int fire =0;                //���� ����
int preasure=0;             //�������� ������� ��������


int temperatur=0;
int k=5;
int past;
int past2;
int timing;

char read_enable = 0;




  //----------------������ ����������� �������------------------------------------

 ISR (TIMER1_COMPA_vect)
 {
	  timing++;
	  Omega_slave(); 
  }
 

int main(void)
{  
	
	
	
	ADCSRA|=(1<<ADEN);// ��������� ������ ���
	ADMUX &= (0 << REFS1);
	ADMUX|=(1 << REFS0); //���������� ������� ����������
	ADMUX |= ~(1 <<ADLAR);//�������������� ������������
	ADCSRA |=(0<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)|(1<<ADFR); //������� ������������
    ACSR |= (1 << ACD);//��������� ���������� ����������
    // ��������� ������ �����/������
    DDRC = 0b00000000;
	DDRB = 0b00111010;
	PORTB= 0b00000000;
    DDRD = 0b11111111;
	PORTC= 0b00000000;
	PORTD= 0b00000000;
//_________________________________________________________________________________________________
    
	TCCR1B |= (1<<WGM12); // ������������� ����� ��� (����� �� ����������)
	TIMSK |= (1<<OCIE1A);	//������������� ��� ���������� ���������� 1��� �������� �� ���������� � OCR1A(H � L)
	OCR1A = 100;  //���������� � ������� ����� ��� ���������
	TCCR1B |= (1<<CS11);//��������� ��������.
	
	
	
	
	
	
	adress=19;
	device_ID=157;// 157-���� ���������� ,89-���� ,108-���
//===================================================================================================================================================	
 
 interval=EEPROM_read(0x02);   // ����� ������ �� �����
 k=EEPROM_read(0x03);          // ���������� �����������
 
 delta_U=EEPROM_read(0x08);	 
 if(delta_U>42){delta_U=41;}       // ����������� �������� �� ���������
 delta_F=EEPROM_read(0x09);
 if(delta_F>42){delta_F=41;}
 adress=EEPROM_read(0x10);
 if(adress>128){adress=0;}
 
 
 
 
 //fire=read_adc(0); //������ ��������� ����� �����
// preasure=read_adc(1); // ��������� ����� ����� �� �����



  sei();

    while (1) 
	
  {
	
	//PORTD= 0b00000001;
	
  }
  
  
  
 }