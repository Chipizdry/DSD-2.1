/*
 * EepromDATA.h
 *
 * Created: 07.04.2021 20:24:40
 *  Author: Red
 */ 


#ifndef EEPROMDATA_H_
#define EEPROMDATA_H_
#include "main.h"

void EEPROM_write(unsigned int uiAddress, unsigned char ucData); // ������ � EEPROM

unsigned char EEPROM_read(unsigned int uiAddress); //������ �� EEPROM

#endif /* EEPROMDATA_H_ */