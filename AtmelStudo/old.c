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
#define  SENS 'S'
#define  SENS_DIRECT 'D'
#define  SENS_INVERSE 'I'
#define  SPEED   'V'
#define  VOLT   'U'


// NTC Temperature sensor properties
#define NTC_RT0 10000
#define NTC_SERIAL_RESISTOR 10000
#define NTC_BETA 3900
#define NTC_GET_RT(va)  (NTC_SERIAL_RESISTOR*((5.0/va)-1))
#define NTC_GET_TEMP(Rt) (1/(( (1/(25+273.15)) +((log(Rt/NTC_RT0))/NTC_BETA ))))-273.15

float Te = 0.05 ;
float Kp  =3 ;
float Ti  =0.25;
float Td  =0;


typedef struct LCD_props{
	uint8_t port ;
	uint8_t pinE;
	uint8_t pinRS;
	uint8_t pin7;
	uint8_t pin6 ;
	uint8_t pin5 ;
	uint8_t pin4 ;
}LCD_props;

LCD_props lcd_pros = {0};
void LCD_write_base(uint8_t cmd, uint8_t rs){
	// Write MSB 
	PORTB=SET(PORTB, lcd_pros.pinRS, rs);
	PORTB=SET(PORTB, lcd_pros.pin7, (cmd&(0x80)));
	PORTB=SET(PORTB, lcd_pros.pin6, (cmd&(0x40)));
	PORTB=SET(PORTB, lcd_pros.pin5, (cmd&(0x20)));
	PORTB=SET(PORTB, lcd_pros.pin4, (cmd&(0x10)));
	
	PORTB=SET_HIGH(PORTB, lcd_pros.pinE);
	
	_delay_ms(50);
	PORTB=SET_LOW(PORTB, lcd_pros.pinE);
	
	// Write LSB
	PORTB=SET(PORTB, lcd_pros.pin7, (cmd&(0x08)));
	PORTB=SET(PORTB, lcd_pros.pin6, (cmd&(0x04)));
	PORTB=SET(PORTB, lcd_pros.pin5, (cmd&(0x02)));
	PORTB=SET(PORTB, lcd_pros.pin4, (cmd&(0x01)));
	
	PORTB=SET_HIGH(PORTB, lcd_pros.pinE);
	
	_delay_ms(50);
	PORTB=SET_LOW(PORTB, lcd_pros.pinE);
		
	
}

void LCD_write_base2(uint8_t cmd, uint8_t rs){
	// Write MSB
	
	PORTB = (PORTB & (0b11000011)) | ((cmd & 0xF0)>>2);
	PORTB = SET(PORTB, lcd_pros.pinRS, rs);
	PORTB =SET_HIGH(PORTB, lcd_pros.pinE);
	
	_delay_ms(1);
	PORTB=SET_LOW(PORTB, lcd_pros.pinE);
	
	// Write LSB
	
	PORTB = (PORTB & (0b11000011)) | ((cmd & 0x0F)<<2);
	
	PORTB=SET_HIGH(PORTB, lcd_pros.pinE);	
	_delay_ms(1);
	PORTB=SET_LOW(PORTB, lcd_pros.pinE);
	
	
}

void LCD_write_cmd(uint8_t cmd){
	LCD_write_base2(cmd,0);
}

void LCD_write_char(uint8_t car){
	LCD_write_base2(car, 1);
}

void LCD_write_text(char *c){
	while(*c){
		LCD_write_char(*c);
		c++;
	}
}


void LCD_init(LCD_props *props){
	DDRB = SET_HIGH(DDRB,props->pinRS);
	DDRB = SET_HIGH(DDRB,props->pinE);
	DDRB = SET_HIGH(DDRB,props->pin7);
	DDRB = SET_HIGH(DDRB,props->pin6);
	DDRB = SET_HIGH(DDRB,props->pin5);
	DDRB = SET_HIGH(DDRB,props->pin4);
	LCD_write_cmd(0x01);
    LCD_write_cmd(0x02);        // Initialize Lcd in 4-bit mode
   //_delay_ms(200);
	LCD_write_cmd(0x28);        // enable 5x7 mode for chars
	LCD_write_cmd(0x0E);        // Display OFF, Cursor ON
	LCD_write_cmd(0x01);        // Clear Display
	LCD_write_cmd(0x80);        // Move the cursor to beginning of first line

	
	LCD_write_text("Hello");
	LCD_write_char('H');
}


//CONSTANTS



// calculate baud rate
void USART_println(char *);
void handle_inputs(char *data);
#define BAUD 9600
#define BAUD_RATE_CALCULATED ((F_CPU/16/BAUD) - 1)

char line[20];
char received_text[20];
volatile int i = 0 ;
volatile int8_t received = 0;
int set_speed = 0 ;
char  ROTATION_SENS = 'D';


void enableExternalInterrupt(){
	asm("sei");
	//PCMSK1 = (1<<PCINT12);
	PCICR  = (1<<PCIE1);
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


void set_pwm_A(int width){
	if(width!=0){
		TCCR0A = 0x83 ;
		TCCR0B = 0x02;
		OCR0A  = width;
		}else{
		TCCR0A =(TCCR0A & (~(1<<COM0A1))) &(TCCR0A & (~(1<<COM0A0)));
	}
}

void set_pwm_B(int width){
	TCCR0A = 0x23 ;
	TCCR0B = 0x02;
	OCR0B  = width;
}


void init(void){
	DDRC = 0x00 ;
	DDRB = 0xFF ;
	DDRD |=(1<<PORTD6) |(1<<PORTD5);
	PORTB = 0x0F;
	_delay_ms(1000);
	
	lcd_pros.port = PORTB ;
	lcd_pros.pin7 = 5;
	lcd_pros.pin6 = 4;
	lcd_pros.pin5 = 3;
	lcd_pros.pin4 = 2;
	lcd_pros.pinE = 1;
	lcd_pros.pinRS= 0;
	
	LCD_init(&lcd_pros);
	
	
}

int current_v=0, past_v =0;


float cm[]= {0,0,0};
float en[]= {0,0,0};
int n2=0, n1=1, n =2 ;


float error = 0 ;

typedef struct PID_info {
	float Kp ;
	float Te ;
	float Ti ;
	float Td ;
	float cm_tab[3];
    float en_tab[3];
	int n_2;
	int n_1;
	int n_;
}PID_info;

float calcultePID_OUT2(PID_info info, float error){
	info.en_tab[info.n_] = error;
	float s1 = info.Kp * (info.en_tab[info.n_]- info.en_tab[info.n_1]);
	float s2 = (Te/Ti)*info.en_tab[info.n_];
	float s3 = (Td/Te)*(info.en_tab[info.n_]-2*info.en_tab[info.n_1]+info.en_tab[info.n_2]) ;
	info.cm_tab[info.n_] = info.cm_tab[n1] + s1 +s2 +s3 ;
	info.n_++;
	info.n_1++;
	info.n_2++;
	if (info.n_==3) {info.n_=0;}
	if (info.n_1==3) {info.n_1=0;}
	if (info.n_2==3) {info.n_2=0;}
	//limit the output
	if(info.cm_tab[info.n_]>5){cm[info.n_]=5;}
	if(info.cm_tab[info.n_]<=0 ){info.cm_tab[info.n_]=0;}
	
	float out = info.cm_tab[info.n_];
	return out;
}


float calcultePID_OUT(){
	float s1 = Kp * (en[n]- en[n1]);
	float s2 = (Te/Ti)*en[n];
	float s3 = (Td/Te)*(en[n]-2*en[n1]+en[n2]) ;
	cm[n] = cm[n1] + s1 +s2 +s3 ;
	n++;
	n1++;
	n2++;
	if (n==3) {n=0;}
	if (n1==3) {n1=0;}
	if (n2==3) {n2=0;}
	//limit the output
	if(cm[n]>5){cm[n]=5;}
	if(cm[n]<=0 ){cm[n]=0;}
	
	float out = cm[n];
	return out;
}


float  u = 1.5 ;
float out =0 ;
char temp[20] ;
void dc_motor_speed_control(){
	out =  read_voltage(4)*5.0/1023.0;
	dtostrf(out, 6, 4, temp) ;
	sprintf(temp,"%f", out) ;
	en[n] = u - out ;
	float commande  = calcultePID_OUT();
	int width= (commande*255/5.0);
	
	set_pwm_A(width) ;
	_delay_ms(5);
	USART_println(temp);
	
	if(received==1){
		USART_println("ok");
		handle_inputs(received_text);
		received = 0;
		dtostrf(u, 6, 4, temp) ;
		USART_println(temp);
	}
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
	sprintf(text,"%f", Rt);
	USART_println(text);
	
	float t2 = NTC_GET_TEMP(Rt);
	USART_println("\nLa temperature : ");
	memset(text, 0,20);
	sprintf(text,"%f", t2);
	USART_println(text);
	//float t = log();
	
}

int main(void){
	init();
	enableExternalInterrupt();
	USART_init();
	USART_println("Hello");
	LCD_write_text("Hello");
	
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
		
	}
	return 0;
}




void handle_inputs(char *data){
			u = atof(data);
}





