#ifndef __RINGQ_H__
#define __RINGQ_H__
#include "headfile.h"
#define RINGQ_MAX    50
 
typedef struct ringq{
   int head; /* 头部，出队列方向*/
   int tail; /* 尾部，入队列方向*/
   int size; /* 队列总尺寸 */
   int space[RINGQ_MAX]; /* 队列空间 */
}RINGQ;
 
/*
  取消tag .限制读与写之间至少要留一个空间
  队列空 head == tail .
  队列满是 (tail+1)%MAX == head
  初始化是head = tail = 0;
*/
 
void ringq_init(RINGQ *p_ringq);
 
void ringq_free(RINGQ *p_ringq);
 
void ringq_push(RINGQ *p_ringq, int data);
 
int ringq_poll(RINGQ *p_ringq);
 
#define ringq_is_empty(q) (q->head == q->tail)
 
#define ringq_is_full(q) (((q->tail+1)%q->size) == q->head )
 
#define print_ringq2(q,d) printf("ring head %d,tail %d,data %d\n", q->head,q->tail,d);
 

 
#endif /* __QUEUE_H__ */
