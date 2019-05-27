//********************************************************************************
// CES 520 Final Project
// Movie Clapper using STM32F401RE
// Written By: Geoff Thomas & Manthan Gajjar
// Date:4/25/18
//********************************************************************************
#include "stdint.h"
#include "stm32f4xx.h"
#include "math.h"

// Clock Registers 
#define RCC_AHB1ENR 0x40023830 // enables/disables clocks for peripherals - GPIO WRITE
#define RCC_APB2ENR 0x40023844 // enable peripheral clock register for SYSCFG -ADC WRITE
//GPIO registers
#define GPIOA_MODER 0x40020000 // Set GPIOA pin mode
#define GPIOA_IDR   0x40020010 //GPIOA input register
#define GPIOA_ODR   0x40020014 //GPIOA output register
#define GPIOA_AFRH  0x40020024 //GPIOA alternate fucntion register
#define GPIOB_MODER 0x40020400 // Set GPIOB pin mode
#define GPIOB_IDR   0x40020410 //GPIOB input register
#define GPIOB_ODR	  0x40020414 //GPIOB output register
#define GPIOC_MODER 0x40020800 // Set GPIOC pin mode
#define GPIOC_IDR		0x40020810 // GPIOC input register
#define GPIOC_ODR	  0x40020814 //GPIOC output register
#define GPIOC_PUPDR	0x4002080C //GPIOC pull-up pull down register
// Interrupt Registers: 
#define SYSCFG_EXTICR4 0x40013814// Set PC12 as interrupt source 
#define SYSCFG_EXTICR3 0x40013810// Set PC11 as interrupt source 
#define EXTI_IMR			 0x40013C00// Interrupt Mask Register
#define EXTI_RTSR			 0x40013C08// Falling Edge Trigger 
#define EXTI_FTSR	     0x40013C0C// Falling Edge Trigger 
#define NVIC_ISR			 0xE000E104// Nested Vector Interrupt Controller 
#define EXTI_PR				 0x40013C14// Pending IRQ
//USART registers
#define USART_SR 		0x40011000 //USART status register
#define USART_DR 		0x40011004 //USART data register
#define USART_BRR		0x40011008 //USART baudrate register
#define USART_CR1 	0x4001100C //USART control register
//LCD constants
#define D4_ON 0x10   // Turn D4 ON
#define D5_ON 0x20   // Turn D5 ON
#define D6_ON 0x40   // Turn D6 ON
#define D7_ON 0x80   // Turn D7 ON
#define RS_ON 0x100  // Turn D8 ON
#define E_ON 0x200   // Turn D9 ON

//Clock registers
volatile uint32_t *clk1_reg = (volatile uint32_t *)RCC_AHB1ENR; 
volatile uint32_t *clk2_reg = (volatile uint32_t *)RCC_APB2ENR;
//USART registers
volatile uint32_t *usartsr_reg = (volatile uint32_t *)USART_SR;
volatile uint32_t *usartdr_reg = (volatile uint32_t *)USART_DR;
volatile uint32_t *usartbrr_reg = (volatile uint32_t *)USART_BRR;
volatile uint32_t *usartcr1_reg = (volatile uint32_t *)USART_CR1;
// GPIOA registers
volatile uint32_t *mod_reg = (volatile uint32_t *)GPIOA_MODER;
volatile uint32_t *out_reg = (volatile uint32_t *)GPIOA_ODR;
volatile uint32_t *in_reg = (volatile uint32_t *)GPIOA_IDR;
volatile uint32_t *arfh_reg = (volatile uint32_t *)GPIOA_AFRH;
// GPIOB registers
volatile uint32_t *mod_reg_b = (volatile uint32_t *)GPIOB_MODER;
volatile uint32_t *out_reg_b = (volatile uint32_t *)GPIOB_ODR;
volatile uint32_t *in_reg_b = (volatile uint32_t *)GPIOB_IDR;
// GPIOC registers
volatile uint32_t *mod_reg_c = (volatile uint32_t *)GPIOC_MODER;
volatile uint32_t *out_reg_c = (volatile uint32_t *)GPIOC_ODR;
volatile uint32_t *in_reg_c = (volatile uint32_t *)GPIOC_IDR;
volatile uint32_t *pup_reg_c = (volatile uint32_t *)GPIOC_PUPDR;
// Declare Interrupt registers 
volatile uint32_t *exticr4 = (volatile uint32_t *)SYSCFG_EXTICR4;
volatile uint32_t *exticr3 = (volatile uint32_t *)SYSCFG_EXTICR3;
volatile uint32_t *exti_imr = (volatile uint32_t *)EXTI_IMR;
volatile uint32_t *exti_rtsr = (volatile uint32_t *)EXTI_RTSR;
volatile uint32_t *exti_ftsr = (volatile uint32_t *)EXTI_FTSR;
volatile uint32_t *exti_pr = (volatile uint32_t *)EXTI_PR;
volatile uint32_t *nvic_enable = (volatile uint32_t *)NVIC_ISR;
//Global variable
volatile uint8_t c1 = 0x00; // SET COUNTER 1 AS 0
volatile uint8_t c2 = 0x00; // SET COUNTER 2 AS 0
//Function prototype
void LCD_SendCmd( int c);
void LCD_SendData ( int c);
void delay(int num);

void delay(int num){//delay function
	while(num>0)
	num--;
}

void display1(int a){
switch(a)
{
case 0: 
*out_reg_b = 0x1E6; //display 0
break;
case 1:
*out_reg_b = 0xC0; //display 1
break;
case 2:
*out_reg_b = 0x1A5;//display 2
break;
case 3:
*out_reg_b = 0xE5;//display 3
break;
case 4:
*out_reg_b = 0xC3;//display 4
break;
case 5:
*out_reg_b = 0x67;//display 5
break;
case 6:
*out_reg_b = 0x167;//display 6
break;
case 7:
*out_reg_b = 0xC4;//display 7
break;
case 8:
*out_reg_b = 0x1E7;//display 8
break;
case 9:
*out_reg_b = 0xC7;//display 9
break;
 }
}
void display2(int a){
switch(a)
{
case 0: 
*out_reg_c = 0xFE;//display 0

break;
case 1:
*out_reg_c = 0x48;//display 1
delay(3000);
break;
case 2:
*out_reg_c = 0x3D;//display 2
delay(3000);
break;
case 3:
*out_reg_c = 0x6D;//display 3
delay(3000);
break;
case 4:
*out_reg_c = 0x4B;//display 4
delay(3000);
break;
case 5:
*out_reg_c = 0x67;//display 5
delay(3000);
break;
case 6:
*out_reg_c = 0xF7;//display 6
delay(3000);
break;
case 7:
*out_reg_c = 0x4C;//display 7
delay(3000);
break;
case 8:
*out_reg_c = 0xFF;//display 8
delay(3000);
break;
case 9:
*out_reg_c = 0x4F;//display 9
delay(3000);
break;
 }
}
//Main Function
int main(){
		// Turn on Clocks 
		*clk1_reg |= 0x7; // Turn on GPIOA,GPIOB,GPIOC
		*clk2_reg |= 0x4010 ; // Clock SYSCFG used to set PORTC to Line13
	
		// Configure GPIO
	  *mod_reg_c |= 0x5555;  // Set GPIOA PA05 as an OUTPUT 
		*mod_reg_b |= 0x55555555;  // Set all GPIOB as OUTPUT 
		*pup_reg_c |= 0x05500000; // Set PC13-PC10 to pull up
		*mod_reg |= 0x55555;  // Set GPIOA PA0 analog input PA6 output 
		*mod_reg |= (2<<20);  // Set GPIOA PA10 as aletrnate funtion
		*mod_reg |= (2<<18);  // Set GPIOA PA9 as aletrnate funtion
		*arfh_reg |= (7<<4);  //Set ARFH9
		*arfh_reg |= (7<<8);  //Set ARFH10
		*mod_reg_c |=(0x1<<20);
		//Configure USART
		*usartcr1_reg = 0x200C; //enable recieve,transmit,USART, OVER8 = 0:sampling rate= 16
		*usartbrr_reg = 0x683; //set the baudrate
		// Configure Interrupts 
		*exticr4 |= 0x22; // Sets Line13&12 to be from Port C[13]&Port C[12]
		*exticr3 |= 0x2200; // Sets Line11&10 to be from Port C[11]&Port C[10]
		*exti_ftsr |= 0x3C00; // Generate interrupt on FALLING edge for Line13-10
		*exti_imr |= 0x3C00; // Unmask interrupt for Line13-10
	  *nvic_enable |= 0x100; // Enable Interrupt 40 - EXTI15_10: From Vector Table 199
//LCD initialize
	LCD_SendCmd(0x01); //clear
	delay(240000);
	LCD_SendCmd(0x02); //Diaplay ON
	delay(5000);
	LCD_SendCmd(0xF); //Cursor on
	delay(5000);
	LCD_SendCmd(0x6); //Cursor on
	delay(5000);
	LCD_SendCmd(0x80); //first line
	delay(5000);

		while(1){
			display1(c1); //display takes
			delay(5000);
			display2(c2); //display Scenes		
			delay(5000);
			if(*usartsr_reg == 0xF8){//checks if data recieved from USART
			
		if(*usartdr_reg == 0x7C){
				LCD_SendCmd(0x01);//LCD clear
			}
		if(*usartdr_reg == 0x5C){
				LCD_SendCmd(0x80);//FIRST LINE
			}
			if(*usartdr_reg == 0x2F){
				LCD_SendCmd(0xC0);//SECOND LINE
			}
			
			if(*usartdr_reg == 0x2F || *usartdr_reg == 0x5C)
				continue;//Skip the remaining statements
      
				LCD_SendData(*usartdr_reg); //send character recieved from USART
				delay(5000);
      }
		} 
}
//LCD put-nibble
void LCD_PutNibble( int c){
*out_reg &= 0x100; //register select
if(c & 0x8)
*out_reg |= D7_ON; //DATA BIT 7 IS HIGH
if(c & 0x4)
*out_reg |= D6_ON; //DATA BIT 6 IS HIGH
if(c & 0x2)
*out_reg |= D5_ON; //DATA BIT 5 IS HIGH
if(c & 0x1)
*out_reg |= D4_ON; //DATA BIT 4 IS HIGH
}
//LCD enable fucntion
void LCD_Pulse(void){
*out_reg |= 0x1; //lcd enable
delay(500); 
*out_reg &= ~(0x1);
delay(500); 
}
//LCD send command fucntion
void LCD_SendCmd(int c) {
//upper 4 bits
*out_reg &= 0x000;
LCD_PutNibble( c >> 4);
LCD_Pulse();
//Lower four bits
LCD_PutNibble(c & 0xF);
LCD_Pulse();
}
//LCD send data fucntion
void LCD_SendData(int c) {
*out_reg |= RS_ON; //register select
//upper 4 bits
LCD_PutNibble( c >> 4);
LCD_Pulse();
//Lower four bits
LCD_PutNibble(c & 0xF);
LCD_Pulse();
*out_reg &= RS_ON;
}

// ISR for line15-10 interrupt 
void EXTI15_10_IRQHandler(void){
	if((*exti_pr & 0x2000) != 0)   // Test if Line 13 interrupt has occured 
  {
		*exti_pr |= 0x2000;        // Clear Line13 by writing to it 
		if(c2 != 0)
		++c1;//take increase
		if(c1==10) //counter ranges from 0-9
			c1=0;
	}
	if((*exti_pr & 0x1000) != 0)   // Test if Line 12 interrupt has occured 
  {
		*exti_pr |= 0x1000;        // Clear Line12 by writing to it 
		c1=0; //take reset
		c2=0; //scene reset
	}
	if((*exti_pr & 0x800) != 0)   // Test if Line 11 interrupt has occured 
  { 
		*exti_pr |= 0x800;        // Clear Line 11 by writing to it 
		++c2;//Scene increase
		c1=0;//take reset
		if(c2==10) //counter ranges from 0-9
			c2=0;
	}
	
}
