/*
 * ADConverter.c
 *
 * Created: 07.04.2021 19:08:16
 *  Author: Red
 */ 

#include "ADConverter.h"

// Функция чтения АЦП
int read_adc (unsigned char ch)
{
	ADMUX = ch; // Выбираем канал АЦП
	ADCSRA |= (1 << ADSC); // Запускаем преобразование
	while((ADCSRA & (1 << ADSC))); // Ждем окончания преобразования
	ADC=(ADCL|ADCH<<8);
	return(ADC); // Возвращаем значение АЦП
}
