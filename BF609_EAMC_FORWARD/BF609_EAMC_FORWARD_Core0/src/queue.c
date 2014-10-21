
/**************************************
***  循环队列 Circular Queue
***
**************************************/

#include "stdlib.h"
#include "stdlib_bf.h"
#include "queue.h"

status InitQueue ( QType *pQ )
{
	pQ->front = pQ->rear = 0;
	pQ->base = ( QElem * ) heap_malloc ( 2, sizeof ( QElem ) * QUEUE_BUFFER_SIZE );
	
	if ( pQ->base )
	{
		memset ( pQ->base , 0, sizeof ( QElem ) * QUEUE_BUFFER_SIZE );
		
		return OK;
	}
	
	printf ( "InitQueue: falied\n " );
	return ERROR;
}

int QueueLength ( QType Q )
{
	return ( Q.rear - Q.front + QUEUE_BUFFER_SIZE ) % QUEUE_BUFFER_SIZE;
}

status EnQueue ( QType *pQ, QElem e )
{
	//算法2－1 入队操作
	if ( ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE == pQ->front ) return ERROR;  // 入队前判断(预留一个存储单元)
	
	pQ->base[pQ->rear] = e;
	pQ->rear = ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE;  // 队列尾部指针增加1

	return OK;
}

status DeQueue ( QType *pQ, QElem *pe )
{
	//算法2－2 出队操作
	if ( pQ->front == pQ->rear ) return ERROR;  // 出队列前判断
	
	*pe = pQ->base[pQ->front];
	
	pQ->front = ( pQ->front + 1 ) % QUEUE_BUFFER_SIZE;
	
	return OK;
}


status GetHead ( QType *pQ, QElem *pe )
{
	if ( pQ->front == pQ->rear ) return ERROR;   // 首尾指针相等则出错
	
	*pe = pQ->base[pQ->front];
	
	return OK;
}

//pe是Input and Output
status GetElem ( QType *pQ, QElem *pe )
{
	int i = pQ->front;
	QElem temp;
	
	
	if ( pQ->front == pQ->rear )
	{
		return ERROR;
	}
	
	/*
	while ( i != pQ->rear )       // 遍历整个队列查找ID匹配的时标
	{
	
		if ( pQ->base[i].ID == pe->ID )
		{
			*pe = pQ->base[i];    // structure assignment
	
			if ( i != pQ->front ) // 如果取出的时间戳不是当前front所指
			{
				pQ->base[i] = pQ->base[pQ->front];	// 当前的值赋给取出的位置
			}
	
			pQ->front = ( pQ->front + 1 ) % QUEUE_BUFFER_SIZE;
	
			return OK;
		}
	
		i = ( i + 1 ) % QUEUE_BUFFER_SIZE;      // 遍历整个队列
	}
	*/
	return ERROR;
}

void Visit_Q ( QType *pQ )  // 遍历队列
{
	int i = pQ->front;
	//printf ( "\n\t当前队列状态：" );
	
	if ( pQ->front == pQ->rear )  printf ( "Queue is empty \n\n" );
	
	else
	{
		printf ( "->" );
		
		while ( i != pQ->rear )
		{
			//printf ( "%d, %0x; ", pQ->base[i].ID,  pQ->base[i].SnapTime );
			i = ( i + 1 ) % QUEUE_BUFFER_SIZE;
		}
		
		printf ( "\t<-\n\n" );
	}
}

