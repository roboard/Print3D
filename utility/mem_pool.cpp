#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

#include "mem_pool.h"

#define _MEM_ERRSYS

#ifdef _MEM_ERRSYS

	static char ERR_MsgBuf[512] = {'\0'};
	static void set_errmsg(char* fmt, ...) 
	{
		va_list args;
		char buffer[512];

		va_start(args, fmt);
		vsprintf(buffer, fmt, args);
		va_end(args);

		strcpy(ERR_MsgBuf, buffer);

	}

	char *mem_geterrmsg(void) 
	{
		return &(ERR_MsgBuf[0]);
	}

#else

//#include xxxx.h
	char *mem_geterrmsg(void) 
	{
		return geterrmsg();
	}


#endif

#define OFFSET_SIZE (sizeof(MEM_POOL *) + sizeof(MEM_POOL_NODE *) + sizeof(long) + sizeof(void *) + sizeof(void *))
#define OFFSET_POOL 20				//pool
#define OFFSET_NODE 16				//pool_node
#define OFFSET_STATE 12				//state
#define OFFSET_PRE 8				//pre
#define OFFSET_NEXT 4				//next

#define GET_SIZE(datasize) ((datasize + OFFSET_SIZE + 0x07) & (~0x7))

#define POOLSIZE 1024               //default poolsize
#define _MEM_ALLOC (0xaa55aa55UL)   //default alloc secret code
#define _MEM_FREE (0x55aa55aaUL)    //default free secret code

typedef void MEM_NODE; 

typedef struct _MEM_POOL_NODE
{
	int count;					    //POOL_NODE的計數器，0的時候為用光，poolsize的時候為滿的

	void *array_index;			    //紀錄一剛開始alloc的記憶體起始位置，給free使用
	int flag;					    //是否可以被free的旗幟0代表未被完全使用過，因此不能free，反之1代表曾經被用光過，如果滿了可以free
	
	struct _MEM_POOL_NODE *next;
	struct _MEM_POOL_NODE *pre;

} MEM_POOL_NODE;

typedef struct _MEM_POOL
{
    MEM_POOL_NODE *head;		    //pool_node的linklist
	MEM_NODE *datahead;			    //node的linklist

    unsigned int datasize;
	unsigned int poolsize;
    
} MEM_POOL;

static inline MEM_POOL *GET_POOL(MEM_NODE *data);
static inline MEM_POOL_NODE *GET_POOL_NODE(MEM_NODE *data);
static inline long GET_STATE(MEM_NODE *data);
static inline MEM_NODE *GET_NEXT(MEM_NODE *data);
static inline MEM_NODE *GET_PRE(MEM_NODE *data);

static inline void SET_POOL(MEM_NODE *data, MEM_POOL *pool);
static inline void SET_POOL_NODE(MEM_NODE *data, MEM_POOL_NODE *node);
static inline void SET_STATE(MEM_NODE *data, long state);
static inline void SET_PRE(MEM_NODE *data, MEM_NODE *pre);
static inline void SET_NEXT(MEM_NODE *data, MEM_NODE *next);

static int initial(MEM_POOL *p);
static MEM_NODE *insert(MEM_NODE *datahead, MEM_NODE *data);
static void *pop(MEM_POOL *p);

void *mem_create(unsigned int datasize, unsigned int poolsize)
{
    MEM_POOL *p = (MEM_POOL *)malloc(sizeof(MEM_POOL));

	if(p == NULL) 
	{
		set_errmsg("mem_create ERROR: no enough memory to allocate %u bytes!", sizeof(MEM_POOL));
		return MEM_FAIL;
	}

	if(datasize <= 0) 
	{
		set_errmsg("mem_create ERROR: datasize should be greater than zero!");
		return MEM_FAIL;
	}

	p->datasize = datasize;

	if(poolsize > 0)
		p->poolsize = poolsize;
	else
		p->poolsize = POOLSIZE;

    p->head = NULL;
	p->datahead = NULL;

    return (MP_HANDLE)p;
}

void *mem_alloc(MP_HANDLE mpool)
{
    MEM_POOL *p;
    void *data;

    p = (MEM_POOL *)mpool;
	if(p == NULL)
	{
		set_errmsg("mem_alloc ERROR: memory pool is NULL, can't allocate memory!");
		return NULL;
	}
	if(p->datahead == NULL)
	{
		if(initial(p) == 1) return NULL;
	}
	data = pop(p);

	return data;    
}

int mem_free(void *data)
{
	MEM_POOL *pool;
	MEM_POOL_NODE *node;
	long state;

	if(data == NULL) return 1;
	
	pool = GET_POOL(data);
	node = GET_POOL_NODE(data);
	state = GET_STATE(data);

	if(state != _MEM_ALLOC && state != _MEM_FREE) 
	{
		set_errmsg("mem_free ERROR : data is not allocated from memory pool");
		return 1;
	}
	else if(state != _MEM_ALLOC)
	{
		set_errmsg("mem_free ERROR: data is already free");
		return 1;
	}
	pool->datahead = insert(pool->datahead, data);

	node->count++;
	
	if(node->flag == 1 && node->count == pool->poolsize)
	{
		unsigned int i;
		unsigned int size = GET_SIZE(pool->datasize);
		char *block = (char *)node->array_index;

		for(i = 0; i < pool->poolsize * size; i += size)
		{
			MEM_NODE *free_data= (MEM_NODE *)&block[i + OFFSET_SIZE];
			if(pool->datahead == free_data)
			{
				pool->datahead = GET_NEXT(pool->datahead);
				if(pool->datahead != NULL)
					SET_PRE(pool->datahead, NULL);
				
			}
			else
			{
				MEM_NODE *pre_data = GET_PRE(free_data);
				MEM_NODE *next_data = GET_NEXT(free_data);
				SET_NEXT(pre_data, next_data);
				if(next_data != NULL)
					SET_PRE(next_data, pre_data);
			}
		}
		free(block);

		if(pool->head == node)
		{
			pool->head = node->next;
			if(pool->head != NULL)
				pool->head->pre = NULL;
			
		}
		else
		{
			node->pre->next = node->next;
			if(node->next != NULL)
				node->next->pre = node->pre;
		}
		free(node);
	}
	return 0;
}

void mem_destroy(MP_HANDLE mpool)
{
    MEM_POOL *p;
	MEM_POOL_NODE *node, *temp;
    p = (MEM_POOL *)mpool;
	if(p == NULL) return;

	node = p->head;
	while(node != NULL)
	{
		temp = node;
		node = node->next;
		free(temp->array_index);
		free(temp);
	}
	free(p);
}

static int initial(MEM_POOL *p)
{
	unsigned int i;
	char *block;
	unsigned int size, alloc_size;
	MEM_POOL_NODE *node;

	size = GET_SIZE(p->datasize);
	node = (MEM_POOL_NODE *)malloc(sizeof(MEM_POOL_NODE));
	if(node == NULL) 
	{
		set_errmsg("mem_alloc ERROR: no enough memory to allocate %u bytes!", sizeof(MEM_POOL_NODE));
		return 1;
	}
	alloc_size = p->poolsize * size;
	block = (char *)malloc(alloc_size * sizeof(char));
	if(block == NULL) 
	{
		free(node);
		set_errmsg("mem_alloc ERROR: no enough memory to allocate %u bytes!", alloc_size * sizeof(char));
		return 1;
	}

	node->count = p->poolsize;
	node->array_index = block;
	node->flag = 0;
	node->pre = NULL;
	if(p->head == NULL)
	{
		node->next = NULL;
	}
	else
	{
		p->head->flag = 1;

		p->head->pre = node;
		node->next = p->head;
	}
	p->head = node;

	for(i = 0; i < alloc_size; i += size)
	{
		MEM_NODE *data = (MEM_NODE *)&block[i + OFFSET_SIZE];
		SET_POOL(data, p);
		SET_POOL_NODE(data, node);
		p->datahead = insert(p->datahead, data);
	}

	return 0;  
}


static MEM_NODE *insert(MEM_NODE *datahead, MEM_NODE *data)
{
	SET_STATE(data, _MEM_FREE);
	SET_PRE(data, NULL);

	if(datahead == NULL)
	{		
		SET_NEXT(data, NULL);
	}
	else
	{
		SET_PRE(datahead, data);
		SET_NEXT(data, datahead);
	}
	return data;
}

static void *pop(MEM_POOL *p)
{
	MEM_POOL_NODE *node;
	void *data = p->datahead;

	p->datahead = GET_NEXT(data);
	if(p->datahead != NULL)
		SET_PRE(p->datahead, NULL);

	node = GET_POOL_NODE(data);
	node->count--;

	SET_STATE(data, _MEM_ALLOC);
	SET_PRE(data, NULL);
	SET_NEXT(data, NULL);

	return data;
}

static inline MEM_POOL *GET_POOL(MEM_NODE *data)
{
	MEM_POOL **pool;
	pool = (MEM_POOL **)((char *)data - OFFSET_POOL);
    return *pool;
}

static inline MEM_POOL_NODE *GET_POOL_NODE(MEM_NODE *data)
{
	MEM_POOL_NODE **pool;
	pool = (MEM_POOL_NODE **)((char *)data - OFFSET_NODE);
    return *pool;
}

static inline long GET_STATE(MEM_NODE *data)
{
	long *state;
	state = (long *)((char *)data - OFFSET_STATE);
    return *state;
}

static inline MEM_NODE *GET_PRE(MEM_NODE *data)
{
	MEM_NODE **pre;
	pre = (MEM_NODE **)((char *)data - OFFSET_PRE);
    return *pre;
}

static inline MEM_NODE *GET_NEXT(MEM_NODE *data)
{
	MEM_NODE **next;
	next = (MEM_NODE **)((char *)data - OFFSET_NEXT);
    return *next;
}


static inline void SET_POOL(MEM_NODE *data, MEM_POOL *pool)
{
	MEM_POOL **p;
	p = (MEM_POOL **)((char *)data - OFFSET_POOL);
	*p = pool;
}

static inline void SET_POOL_NODE(MEM_NODE *data, MEM_POOL_NODE *node)
{
	MEM_POOL_NODE **p;
	p = (MEM_POOL_NODE **)((char *)data - OFFSET_NODE);
	*p = node;
}

static inline void SET_STATE(MEM_NODE *data, long state)
{
	long *p;
	p = (long *)((char *)data - OFFSET_STATE);
	*p = state;
}

static inline void SET_PRE(MEM_NODE *data, MEM_NODE *pre)
{
	MEM_NODE **p;
	p = (MEM_NODE **)((char *)data - OFFSET_PRE);
	*p = pre;
}

static inline void SET_NEXT(MEM_NODE *data, MEM_NODE *next)
{
	MEM_NODE **p;
	p = (MEM_NODE **)((char *)data - OFFSET_NEXT);
	*p = next;
}
