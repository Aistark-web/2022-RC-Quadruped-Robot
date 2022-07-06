#ifndef __CAN_QUEUE_H_
#define __CAN_QUEUE_H_

#include "stdint.h"
#include "stdlib.h"

typedef enum{
	QUEUE_EMPTY,
	QUEUE_FILL
}CAN_QUEUE_Status;

typedef struct Can_Queue_h{
	struct Can_Queue_h *next;
	struct Can_Queue_h *end;
	uint8_t *data;
}Can_Queue_Handle;


void can_queue_push(Can_Queue_Handle *Can_Queue,uint8_t *data,uint8_t len);
CAN_QUEUE_Status can_queue_pop(Can_Queue_Handle *Can_Queue);


#endif