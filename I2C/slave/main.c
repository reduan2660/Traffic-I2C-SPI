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

static char input_buffer[100];
static uint8_t rcv_ptr;
static char *rcv_str;

static uint32_t global_time = 0;

static uint32_t in_idx,out_idx;

static uint32_t G = 10, Y = 2, R = 5, L = 8;
static uint32_t frame_period = 100;
static GPIO_PinState high 	= GPIO_PIN_SET;
static GPIO_PinState low 	= GPIO_PIN_RESET;

#define TIM6_CLOCK ((uint32_t)0x0010)
#define TIM6_CR1_CEN ((uint32_t)0x0001)
#define TIM6_SR_UIF ((uint32_t)0x0001)
static uint32_t counter = 0;

static int logBuffer = 500;
static int logTrafficLR[500][3];
static int logTrafficUD[500][3];
static int logLoad[500][2];

/*FUNCTION PROTOTYPES*/
char* strcpy(char* destination, const char* source);
size_t strlen(const char *str);
int sscanf(const char *str, const char *format, ...);


void TIM5Config(void);
void tim5_delay(uint16_t ms);
void TIM6_Config(void);
void TIM6_DAC_IRQHandler(void);


void USART2_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void getString(void);

void parseCommand(void);
void printStatus(void);
uint32_t random(void);
void RefreshLoad(uint32_t times, uint32_t loadL, uint32_t loadR, uint32_t loadU, uint32_t loadD, GPIO_InitTypeDef *carL, GPIO_InitTypeDef *carR, GPIO_InitTypeDef *carU, GPIO_InitTypeDef *carD);


/*************************
	TRAFFIC SYSTEM
*************************/

void printStatus(void){
		char lrDetails[200];
		sprintf(lrDetails, "\ntraffic light 1 G Y R %d %d %d %d", G, Y, R, L);
		
		char udDetails[200];
		sprintf(udDetails, "\ntraffic light 2 G Y R %d %d %d %d\n", R, Y, G, L);
		
		UART_SendString(USART2, lrDetails);
		UART_SendString(USART2, udDetails);
}

void parseCommand(void){
	
//	UART_SendString(USART2, output_buff);
	
	/* READ */
	if(input_buffer[0] == 'r' && input_buffer[1] == 'e' && input_buffer[2] == 'a' && input_buffer[3] == 'd'){
		printStatus();
	}
	
	/* Monitor */
	else if(input_buffer[8] == 'm'){
		
//		UART_SendString(USART2, "\nMonitor: ");
		
		char temp1[10], temp2[10];
		int monitor;
		sscanf(input_buffer, "%s %s %d", temp1, temp2, &monitor);
		
		sprintf(temp1, "%d\n", monitor);
		UART_SendString(USART2, temp1);
		
		/* 12 Timestamps */
		for(int id=2; id>=0; id--){
			// logTrafficLR
			uint32_t indx = (counter - ( ((uint32_t)monitor)* ((uint32_t)id))) % ((uint32_t)logBuffer);
			
//			sprintf(temp1, "%d\n", indx);
//			UART_SendString(USART2, temp1);
			
			char toPrint[200];
			
			int serial = (int)counter - (int)( ((uint32_t)monitor) *  ((uint32_t)id)  );
			sprintf(toPrint, "%d  traffic light 1 G Y B %d %d %d\n", serial, logTrafficLR[indx][0],  logTrafficLR[indx][1],  logTrafficLR[indx][2]);
			UART_SendString(USART2, toPrint);
			
			
			sprintf(toPrint, "%d  traffic light 2 G Y B %d %d %d\n", serial, logTrafficUD[indx][0],  logTrafficUD[indx][1],  logTrafficUD[indx][2]);
			UART_SendString(USART2, toPrint);
			
			if(logLoad[indx][0]){
				sprintf(toPrint, "%d  traffic light left right heavy\n", serial);
				UART_SendString(USART2, toPrint);
			}
			else {
				sprintf(toPrint, "%d  traffic light left right light\n", serial);
				UART_SendString(USART2, toPrint);
			}
			
			if(logLoad[indx][1]){
				sprintf(toPrint, "%d  traffic light top down heavy\n", serial);
				UART_SendString(USART2, toPrint);
			}
			else {
				sprintf(toPrint, "%d  traffic top down right light\n", serial);
				UART_SendString(USART2, toPrint);
			}
			
			
		}
	}	
	


	/* Config */
	else if(input_buffer[8] == 'l'){
//		UART_SendString(USART2, "\nConfig\n");
		
		char temp[100];
		uint32_t direction, newG, newY, newR, newU;
		sscanf(input_buffer, "%s %s %d %s %s %s %d %d %d %d", temp, temp, &direction, temp, temp, temp, &newG, &newY, &newR, &newU);
//		sprintf(temp, "Updated Value: %d %d %d %d", newG, newY, newR, newU);
//		UART_SendString(USART2, temp);
		
		if(direction == 1){
			G = newG; Y = newY; R = newR; L = newU;
		}
		else if(direction == 2){
			G = newR; Y = newY; R = newG; L = newU;
		}
		
//		sprintf(temp, "Updated Value: %d %d %d %d", G, Y, R, L);
//		UART_SendString(USART2, temp);
		
		UART_SendString(USART2, "\nConfig Updated\n");
		
	}
}



uint32_t random(void){
	uint32_t upper = 1, lower = 0;
	return ((uint32_t)rand() % (upper - lower + 1)) + lower;
	
}

void RefreshLoad(uint32_t times, uint32_t loadL, uint32_t loadR, uint32_t loadU, uint32_t loadD, GPIO_InitTypeDef *carL, GPIO_InitTypeDef *carR, GPIO_InitTypeDef *carU, GPIO_InitTypeDef *carD) {
	while(times--){
		uint32_t cari = 0;

		/* TURNING OFF ALL LOAD LEDs*/
		cari = 0;
		while(cari < 3){
			GPIO_WritePin(carL[cari], low); /*  CAR LEFT   */
			GPIO_WritePin(carR[cari], low); /*  CAR RIGHT  */
			GPIO_WritePin(carD[cari], low); /*  CAR DOWN     */
			GPIO_WritePin(carU[cari], low); /*  CAR UP     */

			cari = cari + 1;
		}
		
		
		/* TURNING ON LOAD LEDs BASED ON LOAD */
		
		/* NO ANIMATION */
		
//		char sLoad[100];
//		sprintf(sLoad, "LOAD: D %d    U %d    L %d  R %d\n", loadD, loadU, loadL, loadR);
//		UART_SendString(USART2, sLoad);
		
		cari = 3;
		while(cari > 0){
			if(loadD >= (cari)) GPIO_WritePin(carD[cari-1], high);
			if(loadU >= (cari)) GPIO_WritePin(carU[cari-1], high);
			if(loadL >= (cari)) GPIO_WritePin(carL[cari-1], high); 
			if(loadR >= (cari)) GPIO_WritePin(carR[cari-1], high); 
			cari = cari - 1;
		}
		ms_delay(frame_period);
		
		
		
		/* ANIMATION */
		
		/* -- ANIUMTAION STARTS 
		
		if(RedLRflag){
			cari = 3;
			while(cari > 0){
				if(loadL>= (cari)) GPIO_WritePin(carL[cari-1], high); 
				if(loadR>= (cari)) GPIO_WritePin(carR[cari-1], high); 
				cari = cari - 1;
			}
			
			cari = 3;
			while(cari > 0){
				if(loadD>= (cari)) GPIO_WritePin(carD[cari-1], high);
				if(loadU>= (cari)) GPIO_WritePin(carU[cari-1], high); 
				
				ms_delay(200);
				cari = cari - 1;
			}
			
		}
		
		else if(GreenLRflag){
			cari = 3;
			while(cari > 0){
				if(loadD>= (cari)) GPIO_WritePin(carD[cari-1], high);
				if(loadU>= (cari)) GPIO_WritePin(carU[cari-1], high); 
				cari = cari - 1;
			}
			
			cari = 3;
			while(cari > 0){
				if(loadL>= (cari)) GPIO_WritePin(carL[cari-1], high); 
				if(loadR>= (cari)) GPIO_WritePin(carR[cari-1], high);
				
				ms_delay(200);
				cari = cari - 1;
			}
		}
	
		 -- ANIUMTAION ENDS */
	}
			
}



/**************************
     TIMER CONFIG
*************************/

void TIM6_Config(void) {
    RCC->APB1ENR |= TIM6_CLOCK; 
    
    TIM6->PSC = 8999; //10khz frequency ....
    TIM6->ARR = 9999; 
    
    TIM6->DIER |= TIM_DIER_UIE; 
    NVIC_EnableIRQ(TIM6_DAC_IRQn); 
    
    TIM6->CR1 |= TIM_CR1_CEN; 
}

void TIM6_DAC_IRQHandler(void) {
	
    if (TIM6->SR & TIM_SR_UIF) 
			{ // Check if timer update interrupt flag is set
        TIM6->SR &= ~TIM_SR_UIF; // Clear the interrupt flag
        
        counter++; 
//				char t[100];
//				sprintf(t, "%d\n", counter);
//				UART_SendString(USART2,t);
				
//				if(counter%monitor == 0){
//					printStatus();
//				}
    }
}


void TIM5Config(void){
	RCC->APB1ENR |= (1<<3);
	
	TIM5->PSC = 45000 - 1; /* fck = 45 mhz, CK_CNT = fck / (psc[15:0] + 1)*/
	TIM5->ARR = 0xFFFF; /*maximum clock count*/
	
	TIM5->CR1 |= (1<<0);
	
	while(!(TIM5->SR & (1<<0)));
	
}

void tim5_delay(uint16_t ms){
	ms = (uint16_t)2 * ms;
	TIM5->CNT = 0;
	while(TIM5->CNT < ms){
		I2C1->CR1 |= I2C_CR1_ACK;
	}
}



/* Commmunication */

void I2C1_EV_IRQHandler(void){ 
    
    I2C1->CR2 &= ~I2C_CR2_ITEVTEN;

    if(I2C1->SR1 & I2C_SR1_ADDR)
        rcv_str = I2C1_ReceiveSlave(&rcv_ptr);     

    //rcv_str = (char *)&rcv_ptr;

    if(strlen(rcv_str) != 0){
        strcpy(input_buffer,rcv_str);
        sendString("Received -> ");
        sendString(input_buffer);
        sendString("\n");
        parseCommand();
    }

    I2C1->CR2 |= I2C_CR2_ITEVTEN;
}




int main(void)
{   
		uint8_t setupMode = 0; // SLAVE
    I2C1_Config(setupMode);
	
		
    
    /* Init */
		initClock();
		sysInit();
		TIM6_Config();
		UART2_Config();
			
		strcpy(input_buffer,"");
		sendString("--------------- I2C SLAVE --------------\n");
		
		
		NVIC_SetPriority(USART2_IRQn, 1);
		NVIC_EnableIRQ(USART2_IRQn);
		NVIC_SetPriority(UART4_IRQn, 1);
		NVIC_EnableIRQ(UART4_IRQn);
		NVIC_SetPriority(UART5_IRQn, 1);
		NVIC_EnableIRQ(UART5_IRQn);
		
		/* CLOCK ENABLE */
		RCC->AHB1ENR |= 1; /* PORT A */
		RCC->AHB1ENR |= 1 << 1; /* PORT B */
		RCC->AHB1ENR |= 1 << 2; /* PORT C */
		RCC->AHB1ENR |= 1 << 7; /* PORT H */
		
		
		
		/* CAR RIGHT */
		GPIO_InitTypeDef PC15 = {13, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOC}; GPIO_Init(GPIOC, &PC15);
		// GPIO_InitTypeDef PA2 = {2, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOA}; GPIO_Init(GPIOA, &PA2);
		GPIO_InitTypeDef PB5 = {5, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOB}; GPIO_Init(GPIOB, &PB5);
		
		/* LOAD RIGHT SIGNAL INPUT */
		GPIO_InitTypeDef IN_LoadR  =  {0, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOC}; GPIO_Init(GPIOC, &IN_LoadR); // C0
		
		// GPIO_InitTypeDef carR[] = {PA7, PA6, PA5};
		// GPIO_InitTypeDef carR[] = {PA5, PA6, PA7};
		GPIO_InitTypeDef carR[] = {PC15, PC15, PB5};
		
		/* CAR LEFT */
		GPIO_InitTypeDef PB13 = {13, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOB}; GPIO_Init(GPIOB, &PB13);
		// GPIO_InitTypeDef PB14 = {14, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOB}; GPIO_Init(GPIOB, &PB14);
		GPIO_InitTypeDef PB15 = {15, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOB}; GPIO_Init(GPIOB, &PB15);
		
		// GPIO_InitTypeDef carL[] = {PB13, PB14, PB15};
		GPIO_InitTypeDef carL[] = {PB13, PB13, PB15};
		
		/* LOAD LEFT SIGNAL INPUT */
		GPIO_InitTypeDef IN_LoadL  =  {10, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOC}; GPIO_Init(GPIOC, &IN_LoadL); // C10
		
		/* CAR DOWN */
		// GPIO_InitTypeDef PA12 = {12, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOA}; GPIO_Init(GPIOA, &PA12);
		GPIO_InitTypeDef PA11 = {11, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOA}; GPIO_Init(GPIOA, &PA11);
		GPIO_InitTypeDef PA9  = { 9, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOA}; GPIO_Init(GPIOA, &PA9);
		
		// GPIO_InitTypeDef carD[] = {PA12, PA11, PA9};
		GPIO_InitTypeDef carD[] = {PA9, PA9, PA11};
		
		/* LOAD DOWN SIGNAL INPUT */
		GPIO_InitTypeDef IN_LoadD  =  {1, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOC}; GPIO_Init(GPIOC, &IN_LoadR); // C1
		
		/* CAR UP */
		GPIO_InitTypeDef PA10 = {10, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOA}; GPIO_Init(GPIOA, &PA10);
		// GPIO_InitTypeDef PB5 =  { 5, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOB}; GPIO_Init(GPIOB, &PB5);
		GPIO_InitTypeDef PA8 =  { 8, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOA}; GPIO_Init(GPIOA, &PA8);
		
		GPIO_InitTypeDef carU[] = {PA10, PA10, PA8};
		// GPIO_InitTypeDef carU[] = {PA13, PA14, PA15};
		
		/* LOAD UP SIGNAL INPUT */
		GPIO_InitTypeDef IN_LoadU  =  {11, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOC}; GPIO_Init(GPIOC, &IN_LoadU); // C11
		
		
		/* LEFT RIGHT SIGNAL LED */
		GPIO_InitTypeDef YellowLR 	=  {6, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOC}; GPIO_Init(GPIOC, &YellowLR); // C6
		GPIO_InitTypeDef RedLR      =  {5, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOC}; GPIO_Init(GPIOC, &RedLR);	 // C5
		GPIO_InitTypeDef GreenLR  	=  {8, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOC}; GPIO_Init(GPIOC, &GreenLR ); // C8
		
		/* LEFT RIGHT SIGNAL INPUT */
		GPIO_InitTypeDef IN_RedLR    =  {2, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOB}; GPIO_Init(GPIOB, &IN_RedLR); // B2
		GPIO_InitTypeDef IN_GreenLR  =  {1, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOB}; GPIO_Init(GPIOB, &IN_GreenLR); // B1
		
		/* UP DOWN SIGNAL LED */
		GPIO_InitTypeDef RedUD    =  { 9, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOC}; GPIO_Init(GPIOC, &RedUD); // C9
		GPIO_InitTypeDef YellowUD =  { 12, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOA}; GPIO_Init(GPIOA, &YellowUD);	// A12
		GPIO_InitTypeDef GreenUD  =  { 14, GPIO_MODE_OUTPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOB}; GPIO_Init(GPIOB, &GreenUD ); // B14
		
		/* UP DOWN SIGNAL INPUT */
		GPIO_InitTypeDef IN_RedUD    =  {3, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOC}; GPIO_Init(GPIOC, &IN_RedUD); // C3
		GPIO_InitTypeDef IN_GreenUD  =  {2, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIOC}; GPIO_Init(GPIOC, &IN_GreenUD); // C2
		
		
		GPIO_WritePin(RedLR,	  low);
		GPIO_WritePin(YellowLR, low);
		GPIO_WritePin(GreenLR,  low);
		
		GPIO_WritePin(RedUD,	  low);
		GPIO_WritePin(YellowUD, low);
		GPIO_WritePin(GreenUD,  low);
		
		GPIO_WritePin(carR[0],	low);
		GPIO_WritePin(carR[1],	low);
		GPIO_WritePin(carR[2],	low);
		
		GPIO_WritePin(carL[0],	low);
		GPIO_WritePin(carL[1],	low);
		GPIO_WritePin(carL[2],	low);
		
		GPIO_WritePin(carU[0],	low);
		GPIO_WritePin(carU[1],	low);
		GPIO_WritePin(carU[2],	low);
		
		GPIO_WritePin(carD[0],	low);
		GPIO_WritePin(carD[1],	low);
		GPIO_WritePin(carD[2],	low);
		
		
		uint32_t loadL = 0;
		uint32_t loadR = 0;
		uint32_t loadU = 0;
		uint32_t loadD = 0;
		
		int GreenLRflag 	= 1;
		int YellowLRflag = 0;
		int RedLRflag 	= 0;
		int YellowUDflag = 0;
		
		while(1)
		{
			tim5_delay(1*1000);
			
			RefreshLoad(1, loadL, loadR, loadU, loadD, carL,carR, carU, carD);
			
			
			if(strlen(input_buffer)){
				// parseCommand();
				// strcpy(input_buffer, "");
			}
		
			uint32_t totalTime = G+Y+R;
			uint32_t currentTime = counter % (totalTime);
			if(currentTime < G){
				GreenLRflag = 1U; YellowLRflag = 0U; YellowUDflag = 0U; RedLRflag = 0U; 
			}
			
			else if(currentTime < (G+Y)){
				GreenLRflag = 0; YellowLRflag = 1; YellowUDflag = 0; RedLRflag = 0; 
			}
			
			else if(currentTime < (totalTime - Y)){
				GreenLRflag = 0; YellowLRflag = 0; YellowUDflag = 0; RedLRflag = 1; 
			}
			
			else if(currentTime < totalTime){
				GreenLRflag = 0; YellowLRflag = 0; YellowUDflag = 1; RedLRflag = 1; 
			}
			
			
			
			
			if(GreenLRflag){
				
				/* LEFT - RIGHT */
				GPIO_WritePin(GreenLR, 	high);
				GPIO_WritePin(RedLR, 		low);
				GPIO_WritePin(YellowLR, low);
				
				
				/* UP - DOWN */
				GPIO_WritePin(GreenUD, 	low);
				GPIO_WritePin(RedUD, 		high);
				GPIO_WritePin(YellowUD, low);
			}
			
			if(RedLRflag){
				/* LEFT - RIGHT */
				GPIO_WritePin(GreenLR, 	low);
				GPIO_WritePin(RedLR, 		high);
				GPIO_WritePin(YellowLR, low);
				
				
				/* UP - DOWN */
				GPIO_WritePin(GreenUD, 	high);
				GPIO_WritePin(RedUD, 		low);
				GPIO_WritePin(YellowUD, low);
			}
			
			if(YellowLRflag){
				/* LEFT - RIGHT */
				GPIO_WritePin(GreenLR, 	low);
				GPIO_WritePin(RedLR, 		low);
				GPIO_WritePin(YellowLR, high);
				
				
				/* UP - DOWN */
				GPIO_WritePin(GreenUD, 	low);
				GPIO_WritePin(RedUD, 		high);
				GPIO_WritePin(YellowUD, low);
			}
			
			if(YellowUDflag){
				/* LEFT - RIGHT */
				GPIO_WritePin(GreenLR, 	low);
				GPIO_WritePin(RedLR, 		high);
				GPIO_WritePin(YellowLR, low);
				
				
				/* UP - DOWN */
				GPIO_WritePin(GreenUD, 	low);
				GPIO_WritePin(RedUD, 		low);
				GPIO_WritePin(YellowUD, high);
			}
			
			
			RefreshLoad(5, loadL, loadR, loadU, loadD, carL,carR, carU, carD);
			
			
			if(GPIO_ReadPin(IN_GreenLR)){
				if(loadL > 0) loadL = loadL-1;
				if(loadR > 0)loadR = loadR-1;
			}
			else if(GPIO_ReadPin(IN_GreenUD)){
				if(loadU > 0) loadU = loadU-1;
				if(loadD > 0)loadD = loadD-1;
			}
			
			
			if(GPIO_ReadPin(IN_RedLR)){
				if(loadL < 3) loadL = loadL+random();
				if(loadR < 3) loadR = loadR+random();
			}
			else if(GPIO_ReadPin(IN_RedUD)){
				if(loadU < 3) loadU = loadU+random();
				if(loadD < 3) loadD = loadD+random();
			}
			
			
				
			
			
			
			uint32_t indx = counter% ((uint32_t)logBuffer);
			logTrafficLR[indx][0] = GreenLRflag; logTrafficLR[indx][1] = YellowLRflag; logTrafficLR[indx][2] = RedLRflag;
			
			logTrafficUD[indx][0] = RedLRflag; logTrafficUD[indx][1] = YellowUDflag;	logTrafficUD[indx][2] = GreenLRflag;
			
			if(GPIO_ReadPin(IN_LoadL) || GPIO_ReadPin(IN_LoadR)) {
				logLoad[indx][0] = 1;
			}
			else {
				logLoad[indx][0] = 0;
			}
			
			if(GPIO_ReadPin(IN_LoadU) || GPIO_ReadPin(IN_LoadD)) {
				logLoad[indx][1] = 1;
			}
			else {
				logLoad[indx][1] = 0;
			}
			
			
			RefreshLoad(2, loadL, loadR, loadU, loadD, carL,carR, carU, carD);
		}
    
}
