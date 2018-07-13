/**
 * @file  periph_uart_tx.h
 * @brief transmision serie a traves de uart2
 */

#ifndef PERIPH_UART_H
#define PERIPH_UART_H

#include <stdint.h>
#include <stm32f303x8.h>
#include "mistipos.h"

#define TBUF_SIZE 256

#define SER_MUX uint32_t

#define SER_1 (SER_MUX)0x01 /*< HOST USART2 -> PA_2 PA_3 */
#define SER_2 (SER_MUX)0x02 /*< USART2 -> PB_3 PB_4      */
#define SER_DIS (SER_MUX)0x00 /*< deshabilitado            */

extern volatile char tbuf[TBUF_SIZE];  /*< FIFO de mensajes a transmitir */
extern volatile uint8_t t_in;          /*< posicion en t_buf para almacenar byte nuevo */
extern volatile uint8_t t_out;         /*< posicion en t_buf del siguiente byte a enviar y sacar de la FIFO. */
extern volatile uint8_t t_disabled;    /*< flag >1 si transmisión deshabilitada. 0 si habilitada. */

extern volatile char rbuf[TBUF_SIZE];  /*< FIFO de mensajes a transmitir */
extern volatile uint8_t r_in;          /*< posicion en t_buf para almacenar byte nuevo */
extern volatile uint8_t r_out;         /*< posicion en t_buf del siguiente byte a enviar y sacar de la FIFO. */
extern volatile uint8_t r_disabled;    /*< flag >1 si transmision deshabilitada. 0 si habilitada. */


void SerialInit(void);
ERRORSTATUS SerialSetMux(SER_MUX ser_a_conf, SER_MUX ser_b_conf, ERRORCODE* err);

/**
 * @brief Inicializa Serial (periferico USART2) para trabajar a 9600baud, 1 bit
 * de stop y sin paridad. Inicialmente deshabilitado. Se configura puerto 
 * PA_2 como TX, y PA_3 como RX.
 */
ERRORSTATUS SerialAConfig(SER_MUX mux_conf, BOOL gpio_only, ERRORCODE* err);
ERRORSTATUS SerialBConfig(SER_MUX mux_conf, BOOL gpio_only, ERRORCODE* err);

/**
 * @brief Deshabilita USART2 para enviar y recibir bytes.
 *
 * Se deshabilita la transmision y la recepcion. Ademas, se configuran los puertos TX y RX como
 * entradas digitales con pull-down para evitar realimentaciones no deseadas.
 */
ERRORSTATUS SerialAOff(SER_MUX mux_conf, ERRORCODE* err);
ERRORSTATUS SerialADisablePorts(SER_MUX mux_conf, ERRORCODE* err);

/**
 * @brief Habilita puerto Serial para enviar y recibir bytes.
 */
void SerialAOn(void);

void SerialBOn(void);
ERRORSTATUS SerialBOff(SER_MUX mux_conf, ERRORCODE* err);


/**
 * @brief Se pone un caracter en la FIFO de transmision.
 * @param ch byte a agregar
 * @retval ERROR si no se pudo agregar el byte a la FIFO.
 * @retval SUCCESS si se agregÃ³ el byte correctamente.
 */
ERRORSTATUS SerialATXPutchar(int ch, ERRORCODE* error);
ERRORSTATUS SerialBTXPutchar(int ch, ERRORCODE* error);

ERRORSTATUS SerialATXPrintC(char* str, int len, ERRORCODE* error);
ERRORSTATUS SerialATXPrint(char* str, ERRORCODE* error);
void SerialATXFlush(ERRORCODE* error);
int SerialATXAvailable(ERRORCODE* error);
void SerialATXHex(uint8_t n, ERRORCODE* error);
void SerialATXPrintFloat(float num, ERRORCODE* error);
ERRORSTATUS SerialARXGetchar(char* c, uint32_t timeout, ERRORCODE* error);
void SerialARXFlush(ERRORCODE* error);

/**
 * @brief  espera recibir datos del puerto serie configurado
 * @param  buf[out]        informacion recibida. longitud en la posicion 0
 * @param  len[in]         longitud maxima del buffer
 * @param  timeout[in]     tiempo maximo esperando el siguiente caracter
 * @param  error[inout]    codigo de error si se produce
 * @retval SUC             proceso termino correctamente
 * @retval ERR             el proceso termino con algun error
 */
ERRORSTATUS SerialARXGetString(char* buf, uint32_t len, uint32_t timeout, ERRORCODE* error);

ERRORSTATUS SerialBTXPrintC(char* str, int len, ERRORCODE* error);
ERRORSTATUS SerialBTXPrint(char* str, ERRORCODE* error);
void SerialBTXFlush(ERRORCODE* error);
int SerialBTXAvailable(ERRORCODE* error);
void SerialBTXHex(uint8_t n, ERRORCODE* error);
void SerialBTXPrintFloat(float num, ERRORCODE* error);
ERRORSTATUS SerialBRXGetchar(char* c, uint32_t timeout, ERRORCODE* error);
void SerialBRXFlush(ERRORCODE* error);
ERRORSTATUS SerialBRXGetString(char* buf, uint32_t len, uint32_t timeout, ERRORCODE* error);

/**
 * @brief Manejador de interrupcion para el periferico USART2
 */
void USART2_IRQHandler(void);
void USART1_IRQHandler(void);


#endif
