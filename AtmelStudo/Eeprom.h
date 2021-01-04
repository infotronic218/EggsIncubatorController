/*
 * Eeprom.h
 *
 * Created: 9/30/2020 3:12:58 PM
 *  Author: dell
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_
#include <avr/io.h>
void EEPROM_write(unsigned int addr, uint8_t data);
char EEPROM_read(unsigned int addr);

void EEPROM_read_until(char *buffer,unsigned int start, unsigned int maxsize, char limit );
void EEPROM_write_string(char *data, unsigned int addr);





#endif /* EEPROM_H_ */