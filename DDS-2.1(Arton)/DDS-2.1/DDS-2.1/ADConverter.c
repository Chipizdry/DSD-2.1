/*
 * ADConverter.c
 *
 * Created: 07.04.2021 19:08:16
 *  Author: Red
 */ 

#include "ADConverter.h"

// ������� ������ ���
int read_adc (unsigned char ch)
{
	ADMUX = ch; // �������� ����� ���
	ADCSRA |= (1 << ADSC); // ��������� ��������������
	while((ADCSRA & (1 << ADSC))); // ���� ��������� ��������������
	ADC=(ADCL|ADCH<<8);
	return(ADC); // ���������� �������� ���
}
