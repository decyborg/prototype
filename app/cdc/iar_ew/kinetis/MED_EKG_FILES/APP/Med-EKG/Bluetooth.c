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