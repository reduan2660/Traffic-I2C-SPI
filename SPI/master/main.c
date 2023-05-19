#include "CLOCK.h"
#include "GPIO.h"
#include "SYS_INIT.h"
#include "USART.h"
#include "TRAFFIC_SYSTEM.h"
#include "SPI.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx.h"

static char input_buff[100];

/*FUNCTION PROTOTYPES*/
void USART2_IRQHandler(void);
void SPI1_IRQHandler(void);
void getString(void);
void init(void);

/*************************
	TRAFFIC MASTER
*************************/


void getString(void){
    uint8_t ch,idx = 0;
    ch = UART_GetChar(USART2);
    while(ch != '!'){
        input_buff[idx++] = ch;
        ch = UART_GetChar(USART2);
        if(ch == '!')break;
    }      
    input_buff[idx] = '\0';
    
}

void USART2_IRQHandler(void){
    USART2->CR1 &= ~(USART_CR1_RXNEIE);
    getString();
    USART2->CR1 |= (USART_CR1_RXNEIE);
}



void init(void){
    /*	Configuration */
    GPIO_InitTypeDef gpio_config;
	
	initClock();
	sysInit();
	UART2_Config();
	
    //config for output 
	gpio_config.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_config.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_Init(GPIOB, &gpio_config);
    gpio_config.Pin = GPIO_PIN_1|GPIO_PIN_2;
	GPIO_Init(GPIOB, &gpio_config);

	//timer start
	TIM2->CNT = 0;
}




int main(void)
{   
	uint8_t temp = 1; // master
	
	init();
	
	SPI1_Config(temp);

    
    strcpy(input_buff,"");
		sendString("-------------- SPI MASTER -------------- \n");

		while(1){
			if(strlen(input_buff) != 0){
				sendString("Sending -> "); sendString(input_buff);
				
				SPI1_Send(input_buff);
				
				strcpy(input_buff,"");
			}
		}

}






