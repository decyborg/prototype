#include "Bluetooth.h"


void Send_Data_BL(unsigned char* g_curr_send_buf, unsigned char size){
  unsigned char* data_ptr = g_curr_send_buf;
  unsigned char data_length = size;
  
  /* Send all Data */
  unsigned char buffer;
  for(buffer = 0; buffer < size; ++buffer){
    uart_putchar(BL_PORT, *(data_ptr + buffer));
  }
}

void configure_BL(void){
    char command[30];
    char response[30];
    char tmp;
    int i = 0;
    while(1){
    send_string("\r\nType command\r\n", TERM_PORT);
    
    while((tmp = uart_getchar(TERM_PORT)) != '\r'){
        command[i++] = tmp;
        uart_putchar(TERM_PORT, tmp);
    }
    
    command[i] = 0;
    i = 0;
    send_string(command, BL_PORT);
    
    while((tmp = uart_getchar(BL_PORT)) != 0){
        response[i++] = tmp;
        uart_putchar(TERM_PORT, tmp);
    }
    }
}