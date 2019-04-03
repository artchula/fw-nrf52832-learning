#ifndef __BASIC_FIFO_H__
#define __BASIC_FIFO_H__


#include <stdint.h>
#include <stdlib.h>


typedef enum{
	QUE_FALSE,
	QUE_TURE,
}queue_status_t;

typedef struct{
		int16_t q_n;
		int16_t q_f;
		int16_t q_r;
}queue_member_t;

uint8_t que_initial(queue_member_t *p_que, uint16_t array_num);
uint8_t que_size(const queue_member_t *p_que);
uint8_t que_isEmpty(const queue_member_t *p_que);
uint8_t que_isFull(const queue_member_t *p_que);
uint8_t que_front(queue_member_t *p_que, uint16_t *p_q_f);
uint8_t que_r(queue_member_t *p_que, uint16_t *p_q);
uint8_t que_pop(queue_member_t *p_que);
uint8_t que_append(queue_member_t *p_que,void *destination, void *source, size_t num);
uint8_t que_reset(queue_member_t *p_que);



//#define __TEST_FIFO__
#ifdef __TEST_FIFO__

void TEST_FIFO(void);

#endif

#endif


