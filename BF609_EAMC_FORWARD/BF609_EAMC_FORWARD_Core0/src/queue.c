
/**************************************
***  ѭ������ Circular Queue
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
	//�㷨2��1 ��Ӳ���
	if ( ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE == pQ->front ) return ERROR;  // ���ǰ�ж�(Ԥ��һ���洢��Ԫ)
	
	pQ->base[pQ->rear] = e;
	pQ->rear = ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE;  // ����β��ָ������1

	return OK;
}

status DeQueue ( QType *pQ, QElem *pe )
{
	//�㷨2��2 ���Ӳ���
	if ( pQ->front == pQ->rear ) return ERROR;  // ������ǰ�ж�
	
	*pe = pQ->base[pQ->front];
	
	pQ->front = ( pQ->front + 1 ) % QUEUE_BUFFER_SIZE;
	
	return OK;
}


status GetHead ( QType *pQ, QElem *pe )
{
	if ( pQ->front == pQ->rear ) return ERROR;   // ��βָ����������
	
	*pe = pQ->base[pQ->front];
	
	return OK;
}

//pe��Input and Output
status GetElem ( QType *pQ, QElem *pe )
{
	int i = pQ->front;
	QElem temp;
	
	
	if ( pQ->front == pQ->rear )
	{
		return ERROR;
	}
	
	/*
	while ( i != pQ->rear )       // �����������в���IDƥ���ʱ��
	{
	
		if ( pQ->base[i].ID == pe->ID )
		{
			*pe = pQ->base[i];    // structure assignment
	
			if ( i != pQ->front ) // ���ȡ����ʱ������ǵ�ǰfront��ָ
			{
				pQ->base[i] = pQ->base[pQ->front];	// ��ǰ��ֵ����ȡ����λ��
			}
	
			pQ->front = ( pQ->front + 1 ) % QUEUE_BUFFER_SIZE;
	
			return OK;
		}
	
		i = ( i + 1 ) % QUEUE_BUFFER_SIZE;      // ������������
	}
	*/
	return ERROR;
}

void Visit_Q ( QType *pQ )  // ��������
{
	int i = pQ->front;
	//printf ( "\n\t��ǰ����״̬��" );
	
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

