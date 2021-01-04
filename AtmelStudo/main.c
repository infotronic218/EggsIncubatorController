/*
 * ATMEGA328P_PWM_CONTROL.c
 *
 * Created: 8/29/2020 2:07:29 AM
 * Author : dell
 */ 

#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "Eeprom.h"
#define  F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#define SET_HIGH(reg,pin)((reg)|(1<<pin))
#define SET_LOW(reg,pin)((reg)&(~(1<<pin)))
// Clear and change the new value
#define SET(reg,pin, state)( ((reg)&(~(1<<pin))) | ((state<<pin)) )
// CONSTANTS
void set_pin_to(int *porte, uint8_t pin , uint8_t state){
	*porte=( ((*porte)&(~(1<<pin))) | ((state<<pin)) );
}



// NTC Temperature sensor properties
#define NTC_RT0 10000
#define NTC_SERIE_RESISTOR 10000
#define NTC_BETA 3900
#define NTC_GET_RT(va)  (NTC_SERIE_RESISTOR*((5.0/va)-1))
#define NTC_GET_TEMP(Rt) (1/(( (1/(25+273.15)) +((log(Rt/NTC_RT0))/NTC_BETA ))))-273.15

//SENSOR  WHTM-02 Humidity and temperature 
#define WHTM_TEMP_MIN -20
#define WHTM_TEMP_MAX 100
#define WHTM_HUM_MIN 00
#define WHTM_HUM_MAX 100
#define WHTM_GET_TEMPERATURE(va) (  ( (WHTM_TEMP_MAX-WHTM_TEMP_MIN)/(1023.0-0) )*va  + WHTM_TEMP_MAX - 1023*((WHTM_TEMP_MAX-WHTM_TEMP_MIN)/(1023.0-0)))
#define WHTM_GET_HUMIDITY(va) (  ( (WHTM_HUM_MAX-WHTM_HUM_MIN)/(1023.0-0) )*va  + WHTM_HUM_MAX - 1023*((WHTM_HUM_MAX-WHTM_HUM_MIN)/(1023.0-0)))

void LCD_write_base2(uint8_t cmd, uint8_t rs){
	// Write MSB
	uint8_t pinRS = 0 ;
	uint8_t pinE = 1 ;
	PORTB = (PORTB & (0b11000011)) | ((cmd & 0xF0)>>2);
	PORTB = SET(PORTB, pinRS, rs);
	PORTB =SET_HIGH(PORTB, pinE);
	
	_delay_ms(1);
	PORTB=SET_LOW(PORTB, pinE);
	
	// Write LSB
	PORTB = (PORTB & (0b11000011)) | ((cmd & 0x0F)<<2);
	PORTB=SET_HIGH(PORTB, pinE);	
	_delay_ms(1);
	PORTB=SET_LOW(PORTB, pinE);
	
}

void LCD_write_cmd(uint8_t cmd){
	LCD_write_base2(cmd,0);
}

void LCD_write_char(uint8_t car){
	LCD_write_base2(car, 1);
}

void LCD_clear(){
	LCD_write_cmd(0x01);
}


void LCD_write_text(char *c, uint8_t line){
	//Use 0 if the want to append the line
	if(line==1){
		LCD_write_cmd(0x80);
	}else if (line==2) {
		LCD_write_cmd(0xC0);
	}
	while(*c){
		LCD_write_char(*c);
		c++;
	}
}

void LCD_init(){
	DDRD =0xFF;
	LCD_write_cmd(0x01);
    LCD_write_cmd(0x02);        // Initialize Lcd in 4-bit mode
   //_delay_ms(200);
	LCD_write_cmd(0x28);        // enable 5x7 mode for chars
	LCD_write_cmd(0x0C);        // Display OFF, Cursor ON
	LCD_write_cmd(0x01);        // Clear Display
	LCD_write_cmd(0x80);        // Move the cursor to beginning of first line
	
	
}

// calculate baud rate
void USART_println(char *);
void handle_inputs(char *data);
#define BAUD 9600
#define BAUD_RATE_CALCULATED ((F_CPU/16/BAUD) - 1)

char line[20];
char received_text[20];
volatile int i = 0 ;
volatile int8_t received = 0;

void enableExternalInterrupt(){
	asm("sei");
	//PCMSK1 = (1<<PCINT12);
	PCICR  = (1<<PCIE1);
	EIMSK = 0x03; // Enable interrupt in pin 2 and 3
}

void USART_init(){
	UCSR0A = 0x00;
	UCSR0A |=(1<<RXC0);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(0<<UMSEL00)|(0<<UMSEL01)|(0<UCSZ02);
	UCSR0C = (1<<UCSZ00) |(1<<UCSZ01) ;
	UBRR0H = (BAUD_RATE_CALCULATED>>8);
	UBRR0L = BAUD_RATE_CALCULATED ;
}

void USART_sendChar(char c)
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c ;
}

unsigned char USART_Read(){
	while ((UCSR0A & (1 << RXC0)) == 0);
	return UDR0 ;
}

ISR(USART_RX_vect){
	char c =  USART_Read();
	if (c!='\n' && c!='\r')
	{
		line [i] =  c ;
		i++ ;
		}else {
		if(c=='\r'){
			line[i] = '\0' ;
			strcpy(received_text, line);
			i = 0 ;
			received = 1 ;
		}
	}
}

void USART_println(char * text){
	while (*text)
	{
		USART_sendChar(*text);
		text++;
	}
	USART_sendChar('\n');
	USART_sendChar('\r');
}

int read_voltage(int adc_pin){
	ADMUX = 0x00 ;
	ADMUX |= (0x0F & adc_pin );
	ADCSRA = ADCSRA | (1<<ADEN) ; // Enable ADC
	ADCSRA = ADCSRA | (1<<ADSC) ;
	while((ADCSRA &(1<< ADSC)));
	int low  = ADCL ;
	int high = ADCH ;
	int value = (high<<8)+low ;
	ADCSRA |= (ADCSRA & (~(1<<ADEN))) |(0<<ADEN);
	return  value;
}

float temperature=0;
float humidity =10;
volatile float set_temperature = 30 ;
volatile float set_humidity    = 80.22 ;

void runing(){
	LCD_clear();
    LCD_write_text("Running", 1);
	char text[10] ;
	LCD_write_text("T=",2);
	sprintf(text,"%.2f", temperature);
	LCD_write_text(text,0);
	LCD_write_text("; H=",0);
	memset(text,0,10);
	sprintf(text,"%.2f", humidity);
	LCD_write_text(text,0);
}

void setting(){
	LCD_clear();
	LCD_write_text("Setting", 1);
	char text[10] ;
	LCD_write_text("T=",2);
	sprintf(text,"%.2f", set_temperature);
	LCD_write_text(text,0);
	LCD_write_text("; H=",0);
	memset(text,0,10);
	sprintf(text,"%.2f", set_humidity);
	LCD_write_text(text,0);
}


volatile int page = 0; 
void init_menu(){
	DDRD |=(0<<2) |(0<<3)|(0<<4) |(0<<5);
	PCMSK2|=(1<<PCINT20); //  Enable pin change in a specific PD4
    PCICR|=(1<<PCIE2) ; // Enable pin change interrupt
	
	// Enable INT0 and INT1
	EIMSK = 0x03; // Enable INT0 and INT1
	// Only rising interrupt are allowed
	EICRA = 0x0F;
	sei();
}


ISR(PCINT2_vect){
	// Page changing interrupt 
	if(PIND &(1<<PIND4)){
		if(page==0){page=1;}else{page=0;}
	}
	
}

void float_increment(volatile float *number){
	*number= *number+0.5 ;
}
void float_decrement(volatile float *number){
	*number= *number-0.5 ;
}


ISR(INT0_vect){
	//Plus button
	if (page==0)
	{
		float_increment(&set_temperature);
	}else{
		float_increment(&set_humidity);
	}
}

ISR(INT1_vect){
	//Minus Button
	//Plus button
	if (page==0)
	{
		float_decrement(&set_temperature);
	}else{
		float_decrement(&set_humidity);
	}
	
}

void init(void){
	DDRC = 0x00 ;
	DDRB = 0xFF ;
	PORTB = 0x0F;
	_delay_ms(1000);
	
	
	LCD_init();
	init_menu();
	
	
}




void NTC_calculation(){
	float va = read_voltage(0)*5.0/1023.0;
	USART_println("\nLa tension est : ");
	char text[20];
	sprintf(text,"%f", va);
	USART_println(text);
	float Rt = NTC_GET_RT(va);
	USART_println("\nLa résistance : ");
	memset(text, 0,20);
	sprintf(text,"%.2f", Rt);
	USART_println(text);
	
	float t2 = NTC_GET_TEMP(Rt);
	USART_println("\nLa temperature : ");
	memset(text, 0,20);
	sprintf(text,"%.2f", t2);
	USART_println(text);
	//float t = log();
	temperature = t2;
	if(page==0){
		USART_println("\nPAGE 0");
	}else{
		USART_println("\nPAGE 1");
	}
	
}

void read_configurations(){
	char buffer[10] ;
	EEPROM_read_until(buffer,0, 5, '\0');
	float temp = atof(buffer);
	USART_println("The set temperature is :");
	USART_println(buffer);
	sprintf(buffer,"%.2f", temp);
	USART_println(buffer);
	
	EEPROM_read_until(buffer,10, 5, '\0');
	float humi= atof(buffer);
	USART_println("The set humidity  is :");
	USART_println(buffer);
	sprintf(buffer,"%.2f", humi);
	USART_println(buffer);
	
}

void write_configuration(){
	char buffer[10] ;
	sprintf(buffer, "%.2f", set_temperature);
	EEPROM_write_string(buffer, 0); // Write the set temperature starting from address 0
	sprintf(buffer, "%.2f", set_humidity);
	EEPROM_write_string(buffer, 10); // Write the set temperature starting from address 10
	
}

void testEEprom(){
	EEPROM_write_string("25.5\n",0);
	char buffer[10];
	EEPROM_read_until(buffer,0,5,'\n');
	USART_println("The read message is : ");
	USART_println(buffer);
}


int main(void){
	init();
	USART_init();
	USART_println("Hello");
	LCD_write_text("Sawadogo SOUS", 2);
	write_configuration();
	read_configurations();
	while(1){
		
		//_delay_ms(2000);
		//USART_println("HELLO");
		//blink_led();
		//dc_motor_speed_control();
		if(received==1){
			handle_inputs(received_text);
			received = 0 ;
		}
		
		_delay_ms(2000);
		
		NTC_calculation();
		
		
		if(PIND&(1<<5)){
			//SETTING
			setting();
		}else{
			runing();
			//RUNNING
		}
		
	}
	return 0;
}




void handle_inputs(char *data){
			
}





