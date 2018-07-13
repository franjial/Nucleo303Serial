/**
 * @file    periph_uart.c
 * @author  FJJA
 * @mail    multicore8@gmail.com
 * @brief   driver for manage serial communication with USART1 and USART2 of stm32f303k8
 */

#include <stm32f3xx.h>
#include "periph_uart.h"
#include "stdio.h"
#include "string.h"
#include <stdint.h>

/** \addtogroup USAR2 FIFO BUFFER (SERIAL A)
 *  @{
 */
volatile char tbuf[TBUF_SIZE];
volatile uint8_t t_in = 0;
volatile uint8_t t_out = 0;
volatile uint8_t t_disabled = 0;

volatile char rbuf[TBUF_SIZE];
volatile uint8_t r_in = 0;
volatile uint8_t r_out = 0;
volatile uint8_t r_disabled = 0;
/** @}*/

/** \addtogroup USAR1 FIFO BUFFER (SERIAL B)
 *  @{
 */
volatile char tbuf_b[TBUF_SIZE];
volatile uint8_t t_in_b = 0;
volatile uint8_t t_out_b = 0;
volatile uint8_t t_disabled_b = 0;

volatile char rbuf_b[TBUF_SIZE];
volatile uint8_t r_in_b = 0;
volatile uint8_t r_out_b = 0;
volatile uint8_t r_disabled_b = 0;
/** @}*/

extern uint32_t millis(void);

static SER_MUX _serial_a_config;
static SER_MUX _serial_b_config;
static FUNCTIONALSTATE _serial_a_st;
static FUNCTIONALSTATE _serial_b_st;

void SerialInit(void){
	_serial_a_st = OFF;
	_serial_b_st = OFF;
	_serial_a_config = SER_DIS;    /*< serial A configurado y en A1 (PC) */
	_serial_b_config = SER_DIS;  /*< serial B deshabilidato */
}

ERRORSTATUS SerialAConfig(SER_MUX mux_conf, BOOL gpio_only, ERRORCODE* err){
	
	/* (0) inicializar variables */
	_serial_a_config = mux_conf;

	t_in = 0;
	t_out = 0;
	t_disabled = 1;
	
	r_in = 0;
	r_out = 0;
	r_disabled = FALSE;

	if(mux_conf == SER_1){
		/* (1) configurar puerto TX y RX */
		/* se configura PA_2 Y PA_3 como alternate function PP+PU*/
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
		GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER2|GPIO_MODER_MODER15)) | ((GPIO_MODER_MODER2|GPIO_MODER_MODER15) & (uint32_t)0x80000020); /* AF */
		GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPDR2|GPIO_PUPDR_PUPDR15)) | ((GPIO_PUPDR_PUPDR2|GPIO_MODER_MODER15) & (uint32_t)0x40000010); /* PU */
		GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_2|GPIO_OTYPER_OT_15); /* PP */
		GPIOA->AFR[0] |= 0x700;      /* se selecciona la funcion USART2 TX */
		GPIOA->AFR[1] |= 0x70000000; /* se selecciona la funcion USART2 RX */
	}

	else if(mux_conf == SER_2){
		/* (1) configurar puerto TX y RX */
		/* se configura PB_3 Y PB_4 como alternate function PP+PU*/
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER3|GPIO_MODER_MODER4)) | ((GPIO_MODER_MODER3|GPIO_MODER_MODER4) & (uint32_t)0x280); /* AF */
		GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPDR3|GPIO_PUPDR_PUPDR4)) | ((GPIO_PUPDR_PUPDR3|GPIO_MODER_MODER4) & (uint32_t)0x140); /* PU */
		GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_3|GPIO_OTYPER_OT_4); /* PP */
		GPIOB->AFR[0] |= 0x77000; /* se selecciona la funcion USART2 TX RX */
	}

	if(gpio_only==FALSE){
		/* (2) se configura USART2 para transmitir */
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

		/* START|8BITS|STOP */
		USART2->CR1 &= ~USART_CR1_M1;  /* M1 */
		USART2->CR1 &= ~USART_CR1_M0;  /* M0 */
		USART2->CR2 &= ~(((uint32_t)0x3)<<12); /* STOP 1bit */
		USART2->CR1 |= (((uint32_t)0x1)<<15);  /* OVER8<-1 (oversampling 16) */
		//USART2->CR1 |= USART_CR1_PCE;        /* PARIDAD */
		USART2->BRR |= 1667;                   /* baudrate 9600 */
	}

	return SUC;

}

ERRORSTATUS SerialBConfig(SER_MUX mux_conf, BOOL gpio_only, ERRORCODE* err){
	/* (0) inicializar variables */
	_serial_b_config = mux_conf;
	_serial_b_st = OFF;

	t_in_b = 0;
	t_out_b = 0;
	t_disabled_b = 1;

	r_in_b = 0;
	r_out_b = 0;
	r_disabled_b = FALSE;

	if(mux_conf == SER_1){
		/* (1) configurar puerto TX y RX */
		/* se configura PA_9 Y PA_10 como alternate function PP+PU*/
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
		GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9|GPIO_MODER_MODER10)) | ((GPIO_MODER_MODER9|GPIO_MODER_MODER10) & (uint32_t)0x280000); /* AF */
		GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPDR9|GPIO_PUPDR_PUPDR10)) | ((GPIO_PUPDR_PUPDR9|GPIO_MODER_MODER10) & (uint32_t)0x140000); /* PU */
		GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_9|GPIO_OTYPER_OT_10); /* PP */
		GPIOA->AFR[0] |= 0x770; /* se selecciona la funcion USART2 TX RX */
	}
	else if(mux_conf == SER_2){
		/* (1) configurar puerto TX y RX */
		/* se configura PA_6 Y PA_7 como alternate function PP+PU*/
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER6|GPIO_MODER_MODER7)) | ((GPIO_MODER_MODER6|GPIO_MODER_MODER7) & (uint32_t)0xA000); /* AF */
		GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPDR6|GPIO_PUPDR_PUPDR7)) | ((GPIO_PUPDR_PUPDR6|GPIO_MODER_MODER7) & (uint32_t)0x5000); /* PU */
		GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_6|GPIO_OTYPER_OT_7); /* PP */
		GPIOB->AFR[0] |= 0x77000000; /* se selecciona la funcion USART2 TX RX */
	}

	/* (2) se configura USART2 para transmitir */
	if(gpio_only==FALSE){
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

		/* START|8BITS|STOP */
		USART1->CR1 &= ~(((uint32_t)0x1)<<28); /* M1 */
		USART1->CR1 &= ~(((uint32_t)0x1)<<12); /* M0 */
		USART1->CR2 &= ~(((uint32_t)0x3)<<12); /* STOP 1bit */
		USART1->CR1 |= (((uint32_t)0x1)<<15);  /* OVER8<-1 (oversampling 16) */
		USART1->BRR |= 1667;                   /* baudrate 9600 */
	}

	return SUC;
}


ERRORSTATUS SerialSetMux(SER_MUX ser_a_conf, SER_MUX ser_b_conf, ERRORCODE* err){

	if(ser_a_conf != SER_DIS && _serial_a_st == OFF){
		// turn on usart2
		SerialAConfig(ser_a_conf, FALSE, err);
		SerialAOn();
	}
	else if (ser_a_conf != SER_DIS && ser_a_conf!=_serial_a_config && _serial_a_st == ON ){
		// si cambias de un puerto a otro, no hay que reconfigurar periferico, solo gpio

		SerialADisablePorts(_serial_a_config, err); // se deshabilitan los pines antiguos
		SerialAConfig(ser_a_conf, TRUE, err);  // configurar solo puertos de gpio a los nuevos
	}

	if(ser_a_conf == SER_DIS && _serial_a_st == ON){
		// turn off usart2
		SerialAOff(_serial_a_config, err);
	}


	if(ser_b_conf != SER_DIS && _serial_b_st == OFF){
		// turn on usart1
		SerialBConfig(ser_b_conf, FALSE, err);
		SerialBOn();
	}
	else if (ser_b_conf != SER_DIS && ser_b_conf!=_serial_b_config && _serial_b_st == ON ){
		//TODO
	}


	if(ser_b_conf == SER_DIS && _serial_b_st == ON){
		// turn off usart1
		SerialBOff(_serial_b_config, err);
	}

	_serial_a_config = ser_a_conf;
	_serial_b_config = ser_b_conf;

	return SUC;
}

void SerialAOn(void){
	_serial_a_st = ON;

	/* habilita usart2 y recepcion */
	USART2->CR1 |= (((uint32_t)0x1)<<2);  /* RE */
	USART2->CR1 |= (uint32_t)0x1;         /* UE habilita USART2 */
	
	/* (4) se configuran las interrupciones */
	USART2->CR1 |= (uint32_t)0x21;        /* se habilita RXNEIE, EIE */
	NVIC_EnableIRQ(USART2_IRQn);

}

ERRORSTATUS SerialADisablePorts(SER_MUX mux_conf, ERRORCODE* err){

	if(mux_conf == SER_1){
		/* puertos TX y RX se ponen como entradas pull-down para evitar alimentacion no deseada de modulos */
		GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER2|GPIO_MODER_MODER15)) | ((GPIO_MODER_MODER2|GPIO_MODER_MODER15) & (uint32_t)0x0);        /* input */
		GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPDR2|GPIO_PUPDR_PUPDR15)) | ((GPIO_PUPDR_PUPDR2|GPIO_PUPDR_PUPDR15) & (uint32_t)0x40000010); /* PD */
	}
	else if(mux_conf == SER_2){
		/* puertos TX y RX se ponen como entradas pull-down para evitar alimentacion no deseada de modulos */
		GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER3|GPIO_MODER_MODER4)) | ((GPIO_MODER_MODER3|GPIO_MODER_MODER4) & (uint32_t)0x0);        /* input */
		GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPDR3|GPIO_PUPDR_PUPDR4)) | ((GPIO_PUPDR_PUPDR3|GPIO_PUPDR_PUPDR4) & (uint32_t)0x240); /* PD */
	}
	else{
		if (err != NULL)
			*err = ERR_G_BPAR;
		return ERR;
	}



	return SUC;

}

ERRORSTATUS SerialAOff(SER_MUX mux_conf, ERRORCODE* err){
	_serial_a_st = OFF;
	
	if(mux_conf == SER_1){
		/* puertos TX y RX se ponen como entradas pull-down para evitar alimentacion no deseada de modulos */
		GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER2|GPIO_MODER_MODER15)) | ((GPIO_MODER_MODER2|GPIO_MODER_MODER15) & (uint32_t)0x0);        /* input */
		GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPDR2|GPIO_PUPDR_PUPDR15)) | ((GPIO_PUPDR_PUPDR2|GPIO_PUPDR_PUPDR15) & (uint32_t)0x40000010); /* PD */
	}
	else if(mux_conf == SER_2){
		/* puertos TX y RX se ponen como entradas pull-down para evitar alimentacion no deseada de modulos */
		GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER3|GPIO_MODER_MODER4)) | ((GPIO_MODER_MODER3|GPIO_MODER_MODER4) & (uint32_t)0x0);        /* input */
		GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPDR3|GPIO_PUPDR_PUPDR4)) | ((GPIO_PUPDR_PUPDR3|GPIO_PUPDR_PUPDR4) & (uint32_t)0x240); /* PD */
	}
	else{
		if (err != NULL)
			*err = ERR_G_BPAR;
		return ERR;
	}
	
	/* deshabilita usart2 y recepcion */
	USART2->CR1 &= ~(((uint32_t)0x1)<<2);    /* RE deshabilitado */
	USART2->CR1 &= ~((uint32_t)0x1);         /* UE deshabilita USART2 */


	return SUC;
}

ERRORSTATUS SerialBOff(SER_MUX mux_conf, ERRORCODE* err){
	//TODO
	return SUC;
}

void SerialBOn(void){
	//TODO
}

ERRORSTATUS SerialATXPutchar(int ch, ERRORCODE* error){

	tbuf [t_in] = ch;
	t_in++;
	
	if(t_disabled){
		t_disabled = 0;
	}
	USART2->CR1 |= (((uint32_t)0x1)<<7); /* enable TXE */
	USART2->CR1 |= (((uint32_t)0x1)<<3); /* enable TE*/
	
	return SUC;
}

ERRORSTATUS SerialBTXPutchar(int ch, ERRORCODE* error){

	tbuf_b [t_in] = ch;
	t_in_b++;

	if(t_disabled_b){
		t_disabled_b = 0;
	}
	USART1->CR1 |= (((uint32_t)0x1)<<7); /* enable TXE */
	USART1->CR1 |= (((uint32_t)0x1)<<3); /* enable TE*/

	return SUC;
}

void SerialATXPrintFloat(float num, ERRORCODE* error){
	//TODO
}

void SerialATXHex(uint8_t n, ERRORCODE* error){
	uint8_t str[3];
	int i;
	
	str[1] = n%16;
	str[0] = ((n/16)%16);
	
	for(i=0;i<2;i++){
		switch(str[i]){
			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
				SerialATXPutchar(str[i]+0x30, error);
				break;
			case 10:
				SerialATXPutchar('A', error);
				break;
			case 11:
				SerialATXPutchar('B', error);
				break;
			case 12:
				SerialATXPutchar('C', error);
				break;
			case 13:
				SerialATXPutchar('D', error);
				break;
			case 14:
				SerialATXPutchar('E', error);
				break;
			case 15:
				SerialATXPutchar('F', error);
				break;
		}
	}
}

ERRORSTATUS SerialATXPrintC(char* str, int len, ERRORCODE* error){
	int i;

	for(i=0;i<len;i++){
		SerialATXPutchar(str[i], error);
	}

	return SUC;
}

ERRORSTATUS SerialATXPrint(char* str, ERRORCODE* error){
	int i;

	for(i=1;i<str[0];i++){
		SerialATXPutchar(str[i], error);
	}

	return SUC;
}

void USART2_IRQHandler(){

	if((USART2->ISR&USART_ISR_ORE) > 0){
		/* overrun */
		USART2->ICR |= (uint32_t)(0x1<<3); /* clear OVERUN */
		//while(1);
	}

	/* interrupcion debida a que se recibio un byte */
	else if( (USART2->ISR&USART_ISR_RXNE) > 0){
		/* hay un byte sin leer en RDR, leer */
		rbuf[r_in] = USART2->RDR;
		r_in++;
	}

	/* interrupcion debida a que finalizo la transmision del dato al registro de desplazamiento */
	else if( (USART2->ISR&USART_ISR_TXE) > 0 ){ /*TXE*/
		if(t_in != t_out){
			/* hay algo en el buffer sin transmitir */
			USART2->TDR = tbuf[t_out];
			t_out++;
		}
		else{
			/* all transmited */
			t_disabled=1;

			USART2->CR1 &= ~(((uint32_t)0x1)<<7); /* disable TXE */
			USART2->CR1 &= ~(((uint32_t)0x1)<<3); /* disable TE*/
			USART2->ICR |= (uint32_t)(0x1<<6);    /* clear TC */
		}
	}
}

void USART1_IRQHandler(){
	if((USART1->ISR&USART_ISR_ORE) > 0){
		/* overrun */
		USART1->ICR |= (uint32_t)(0x1<<3); /* clear OVERUN */
		//while(1);
	}

	/* interrupcion debida a que se recibio un byte */
	else if( (USART1->ISR&USART_ISR_RXNE) > 0){
		/* hay un byte sin leer en RDR, leer */
		rbuf_b[r_in_b] = USART1->RDR;
		r_in_b++;
	}

	/* interrupcion debida a que finalizo la transmision del dato al registro de desplazamiento */
	else if( (USART1->ISR&USART_ISR_TXE) > 0 ){ /*TXE*/
		if(t_in_b != t_out_b){
			/* hay algo en el buffer sin transmitir */
			USART1->TDR = tbuf_b[t_out_b];
			t_out_b++;
		}
		else{
			/* all transmited */
			t_disabled_b=1;

			USART1->CR1 &= ~(((uint32_t)0x1)<<7); /* disable TXE */
			USART1->CR1 &= ~(((uint32_t)0x1)<<3); /* disable TE*/
			USART1->ICR |= (uint32_t)(0x1<<6);    /* clear TC */
		}
	}
}

int SerialATXAvailable(ERRORCODE* error){
	return t_in - t_out;
}

void SerialATXFlush(ERRORCODE* error){
	t_in = 0;
	t_out = 0;
}

void SerialARXFlush(ERRORCODE* error){
	r_in = 0;
	r_out = 0;
}

ERRORSTATUS SerialARXGetchar(char* c, uint32_t timeout, ERRORCODE* error){
	uint32_t tini;
	BOOL wait_char;

	wait_char = TRUE;
	tini = millis();
	*error = 0;

	while(wait_char==TRUE){
		if(millis()-tini > timeout){
			wait_char=FALSE;
			*error = ERR_G_TOUT;
		}
		else{
			/* si hay char pendiente de leer en buffer */
			if(r_in != r_out){
				*c = rbuf[r_out];
				r_out++;
				return SUC;
			}
		}
	}

	if(*error != 0){
		return ERR;
	}
	return SUC;
}

ERRORSTATUS SerialARXGetString(char* buf, uint32_t len, uint32_t timeout, ERRORCODE* error){
	int i;
	ERRORCODE myerror;

	myerror = 0;
	i = 1;

	while(myerror==0 && i<len){
		if(SerialARXGetchar(buf+i, timeout, &myerror) == SUC){
			i++;
		}
	}
	SerialARXFlush(error);

	if(i>1){
		buf[0] = i;
		return SUC;
	}

	/* no ha leido nada (timeout) */
	*error = ERR_G_TOUT;
	return ERR;
}


ERRORSTATUS SerialBTXPrintC(char* str, int len, ERRORCODE* error){
	int i;

	for(i=0;i<len;i++){
		SerialBTXPutchar(str[i], error);
	}

	return SUC;
}
ERRORSTATUS SerialBTXPrint(char* str, ERRORCODE* error){
	int i;

	for(i=0;i<str[0];i++){
		SerialBTXPutchar(str[i], error);
	}

	return SUC;
}
void SerialBTXFlush(ERRORCODE* error){
	t_in_b = 0;
	t_out_b = 0;
}
int SerialBTXAvailable(ERRORCODE* error){
	return t_in_b - t_out_b;
}
void SerialBTXHex(uint8_t n, ERRORCODE* error){
	uint8_t str[3];
	int i;

	str[1] = n%16;
	str[0] = ((n/16)%16);

	for(i=0;i<2;i++){
		switch(str[i]){
			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
				SerialBTXPutchar(str[i]+0x30, error);
				break;
			case 10:
				SerialBTXPutchar('A', error);
				break;
			case 11:
				SerialBTXPutchar('B', error);
				break;
			case 12:
				SerialBTXPutchar('C', error);
				break;
			case 13:
				SerialBTXPutchar('D', error);
				break;
			case 14:
				SerialBTXPutchar('E', error);
				break;
			case 15:
				SerialBTXPutchar('F', error);
				break;
		}
	}

}
void SerialBTXPrintFloat(float num, ERRORCODE* error){
	//TODO
}
ERRORSTATUS SerialBRXGetchar(char* c, uint32_t timeout, ERRORCODE* error){
	uint32_t tini;
	BOOL wait_char;

	wait_char = TRUE;
	tini = millis();
	*error = 0;

	while(wait_char==TRUE){
		if(millis()-tini > timeout){
			wait_char=FALSE;
			*error = ERR_G_TOUT;
		}
		else{
			/* si hay char pendiente de leer en buffer */
			if(r_in_b != r_out_b){
				*c = rbuf_b[r_out_b];
				r_out_b++;
				return SUC;
			}
		}
	}

	if(*error != 0){
		return ERR;
	}
	return SUC;
}
void SerialBRXFlush(ERRORCODE* error){
	r_in_b = 0;
	r_out_b = 0;
}
ERRORSTATUS SerialBRXGetString(char* buf, uint32_t len, uint32_t timeout, ERRORCODE* error){
	int i;
	ERRORCODE myerror;

	myerror = 0;
	i = 1;

	while(myerror==0 && i<len){
		if(SerialBRXGetchar(buf+i, timeout, &myerror) == SUC){
			i++;
		}
	}
	SerialBRXFlush(error);

	if(i>1){
		buf[0] = i;
		return SUC;
	}

	/* no ha leido nada (timeout) */
	*error = ERR_G_TOUT;
	return ERR;

}


