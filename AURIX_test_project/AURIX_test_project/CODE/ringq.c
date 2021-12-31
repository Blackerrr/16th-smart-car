#include <stdio.h>
#include "stdlib.h"
#include "ringq.h"
 
//extern uint16 ringq_length;

void ringq_init(RINGQ * p_ringq)
{
    p_ringq->head = 0;
    p_ringq->tail = 0;
    p_ringq->size = ringq_length;
}

void ringq_free(RINGQ * p_ringq)
{
    // return 0;
    p_ringq = NULL;        //防止野指针
}
 
/* 往队列加入数据 */
void ringq_push(RINGQ *p_ringq, int data)
{
   // print_ringq(p_ringq,data);
   
   if(ringq_is_full(p_ringq))
   {
       // printf("ringq is full,data %d\n",data);
       return;
   }
         
   p_ringq->space[p_ringq->tail] = data;
   
   p_ringq->tail = (p_ringq->tail + 1) % p_ringq->size ;   
    
   // return p_ringq->tail ;
}
 

// 从对首弹出数据
int ringq_poll(RINGQ * p_ringq)
{
    int p_data;
   // print_ringq(p_ringq,-1);
    if(ringq_is_empty(p_ringq))
    {
        // printf("ringq is empty\n");
        return -1;
    }
   
   p_data = p_ringq->space[p_ringq->head];
   
   p_ringq->head = (p_ringq->head + 1) % p_ringq->size ;
   
   return p_data;
}
