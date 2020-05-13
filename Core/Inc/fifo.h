#ifndef _FIFO_H_
#define _FIFO_H_

#define FIFO_SIZE 100


int fifo_write_byte(uint8_t byte);
int fifo_read_byte(void);
int fifo_data_available(void);


#endif // _FIFO_H_
