#include "CLOCK.h"
#include "GPIO.h"
#include "SYS_INIT.h"
#include "USART.h"
#include "I2C.h"
//#include "TRAFFIC_SYSTEM.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "stm32f4xx.h"

static char input_buff[100];
void USART2_IRQHandler(void);
void getString(void);

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


int main(void)
{   
		initClock();
		sysInit();
		UART2_Config();
    I2C1_Config(1); // MASTER
    strcpy(input_buff,"");
	
		NVIC_SetPriority(USART2_IRQn, 1);
		NVIC_EnableIRQ(USART2_IRQn);
	
    sendString(" -------------- I2C MASTER --------------\n");
    
		while(1){
				if (strlen(input_buff) != 0){  
					
						sendString(" Sending  -> "); sendString(input_buff);
						
						while(!I2C1_TransmitMaster(input_buff,strlen(input_buff)));
						
						sendString("  ->  Transmitted");	
						strcpy(input_buff, "");
				}
		}
}






