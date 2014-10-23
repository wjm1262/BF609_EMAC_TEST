
#ifndef __QUEUE_H
#define __QUEUE_H

#include <drivers/ethernet/adi_ether.h>

#include <stdio.h>
#include <string.h>


//////////////////

#define OK 1
#define ERROR 0
#define OVERFLOW -2




typedef int status;

typedef ADI_ETHER_BUFFER *QElem;
typedef struct
{
	QElem *base;
	int front;
	int rear;
} AdiEtherBufferQueue;

typedef AdiEtherBufferQueue QType;

status InitQueue ( QType *pQ );
int QueueLength ( QType Q );
status EnQueue ( QType *pQ, QElem e );
status DeQueue ( QType *pQ, QElem *pe );
status GetHead ( QType *pQ, QElem *pe );
//pe «Input and Output
status GetElem ( QType *pQ, QElem *pe );

void Visit_Q ( QType *pQ );



#endif


