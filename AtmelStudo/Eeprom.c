#include "Eeprom.h"

void EEPROM_write(unsigned int addr, uint8_t data){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set up address and Data Registers */
	EEAR = addr;
	EEDR = data;
	/* Write logical one to EEMPE */
	EECR |= (1<<EEMPE);
	/* Start eeprom write by setting EEPE */
	EECR |= (1<<EEPE);
	
}

char EEPROM_read(unsigned int addr){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE))
	;
	/* Set up address register */
	EEAR = addr;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from Data Register */
	return EEDR;
}

void EEPROM_read_until(char *buffer,unsigned int start, unsigned int maxsize, char limit ){
	unsigned int addr=start, i=0;
	char c ;
	
	while((addr+i)<maxsize  &&  (c=EEPROM_read(addr+i))!=limit)
	{
		*buffer= c ;
		buffer++;
		i++;
	}
	
}
void  EEPROM_write_string(char *data, unsigned int addr){
	unsigned int i=0 ;
	while(*data){
		EEPROM_write(addr+i, *data);
		data++;
		i++;
	}
}
