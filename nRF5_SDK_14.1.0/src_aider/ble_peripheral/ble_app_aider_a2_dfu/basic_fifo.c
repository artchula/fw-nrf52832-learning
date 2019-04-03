#include "basic_fifo.h"
#include <string.h>

uint8_t que_initial(queue_member_t *p_que, uint16_t array_num)
{
		if(array_num == 0)
		{
				return QUE_FALSE;
		}
		p_que->q_n = array_num;
		p_que->q_f = 0;
		p_que->q_r = 0;		
		return QUE_TURE;
}

uint8_t que_size(const queue_member_t *p_que)
{
		return (p_que->q_n - p_que->q_f + p_que->q_r)%p_que->q_n;
}


uint8_t que_isEmpty(const queue_member_t *p_que)
{	
		if(p_que->q_f == p_que->q_r)
		{
				return QUE_TURE;
		}		
		return QUE_FALSE; 
}


uint8_t que_isFull(const queue_member_t *p_que)
{
		if(que_size(p_que) == (p_que->q_n-1))
		{
				return QUE_TURE;
		}		
		
		return QUE_FALSE;
}


uint8_t que_front(queue_member_t *p_que, uint16_t *p_q_f)
{		
//		if(que_isEmpty(p_que) == QUE_TURE)
//		{
//				return QUE_FALSE;
//		}
		*p_q_f = p_que->q_f;
		return QUE_TURE;
}


uint8_t que_r(queue_member_t *p_que, uint16_t *p_q_r)
{
//		if(que_isEmpty(p_que) == QUE_TURE)
//		{
//				return QUE_FALSE;
//		}
		*p_q_r = p_que->q_r;
		return QUE_TURE;
}

uint8_t que_pop(queue_member_t *p_que)
{
		if(que_isEmpty(p_que) == QUE_TURE)
		{
				return QUE_FALSE;
		}
		p_que->q_f = (p_que->q_f + 1) % p_que->q_n;
		return QUE_TURE;
}


uint8_t que_append(queue_member_t *p_que,void *destination, void *source, size_t num)
{
		if(que_size(p_que) == (p_que->q_n-1))
		{
				return QUE_FALSE;
		}		
		
		uint8_t *p_data = destination;
		memcpy(  (void*)(p_data+(p_que->q_r*num)) , source, num);		
		p_que->q_r = (p_que->q_r+1)%p_que->q_n;
		
		return QUE_TURE;
}

uint8_t que_reset(queue_member_t *p_que)
{
		p_que->q_r = 0;
		p_que->q_f = 0;

		return QUE_TURE;
}


#ifdef __TEST_FIFO__

#define QUEUE_SIZE		10

typedef enum{
	TEST_PASS,
	TEST_FALL,
}queue_test_t;

typedef struct{
		uint8_t  data8;
		uint16_t data16;
		uint32_t data32;
}ex_struct_t;

uint32_t buffer;
uint8_t result_test;
uint16_t buf_queue_size; 
uint16_t res_queue;
ex_struct_t que_buffer[QUEUE_SIZE];
queue_member_t que_member;
ex_struct_t *p_que_buffer[QUEUE_SIZE];
ex_struct_t ex_data;
uint16_t front_index;


void TEST_FIFO(void)
{
		//Initial FIFO
		que_initial(&que_member, QUEUE_SIZE);	
	
		//Test query size of queue
		buf_queue_size = que_size(&que_member);
		if(buf_queue_size == 0)
		{
				result_test = TEST_PASS;
		}else{
				result_test = TEST_FALL;
		}
		
		//Test empty of queue
		
		res_queue = que_isEmpty(&que_member);
		if(res_queue == QUE_TURE)
		{
				result_test = TEST_PASS;
		}else{
				result_test = TEST_FALL;
		}
		
		//Test write data to fifo 1 byte
	
		ex_data.data8 = 0x11;
		ex_data.data16 = 0x2233;
		ex_data.data32 = 0x44556677;
		
		for(int i= 0;i<QUEUE_SIZE;i++)
		{
				p_que_buffer[i] = &que_buffer[i];
		}
		
		//que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));
		res_queue = que_append(&que_member, (void**)&(*p_que_buffer), &ex_data, sizeof(ex_struct_t));
	
		ex_data.data8 = 0x99;
		ex_data.data16 = 0x8877;
		ex_data.data32 = 0x66554433;
		//2
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		//3
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		//4
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		//5
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		//6
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		//7
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		//8
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		//9
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		//10
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		//11
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		
		res_queue = que_pop(&que_member);
		
		//12
		ex_data.data8 = 0x5E;
		ex_data.data16 = 0xE55E;
		ex_data.data32 = 0x65544332;
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		res_queue = que_front(&que_member, &front_index);
		res_queue = que_pop(&que_member);
		
		ex_data.data8 = 0x54;
		ex_data.data16 = 0x1234;
		ex_data.data32 = 0x87654321;
		res_queue = que_append(&que_member, &que_buffer, &ex_data, sizeof(ex_struct_t));	
		
}

#endif

