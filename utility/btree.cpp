#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

#include "mem_pool.h"
#include "btree.h"

#define _BTREE_ERRSYS

#ifdef _BTREE_ERRSYS

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

	char *btree_geterrmsg(void) 
	{
		return &(ERR_MsgBuf[0]);
	}

#else

//#include xxxx.h
	char *btree_geterrmsg(void) 
	{
		return geterrmsg();
	}

#endif

#define POOLSIZE 64
#define Max(a, b) ((a) > (b) ? (a) : (b))
#define Height(p) ((p == NULL) ? (-1) : (p->height))

typedef struct _BALANCE_TREE_NODE
{
    void *data;

	long  height;
	struct _BALANCE_TREE_NODE *right;
	struct _BALANCE_TREE_NODE *left;
} BALANCE_TREE_NODE;

typedef struct _ITER_STACK
{
    BALANCE_TREE_NODE *node;
    struct _ITER_STACK *next;
} ITER_STACK;

typedef struct _BALANCE_TREE
{
    BALANCE_TREE_NODE* root;
    int use_iter;
    ITER_STACK *iterator;
    
    MP_HANDLE node_pool;
    MP_HANDLE iter_pool;

    void *(*data_init)(void);
    int (*data_copy)(void *src, void *dest);
    void (*data_clear)(void *);
    int (*comp)(void*, void*);
} BALANCE_TREE;

static BALANCE_TREE_NODE *LL_Rotate( BALANCE_TREE_NODE *p );
static BALANCE_TREE_NODE *RR_Rotate( BALANCE_TREE_NODE *p );
static BALANCE_TREE_NODE *RL_Rotate( BALANCE_TREE_NODE *p );
static BALANCE_TREE_NODE *LR_Rotate( BALANCE_TREE_NODE *p );
static BALANCE_TREE_NODE *delete_node(BALANCE_TREE_NODE *t_node, BALANCE_TREE_NODE *p, BALANCE_TREE *btree);
static BALANCE_TREE_NODE *search(void *name, BALANCE_TREE_NODE *p, int (*comp)(void *, void *));
static BALANCE_TREE_NODE *insert(void *data, BALANCE_TREE_NODE *t_node, BALANCE_TREE *btree);
static BALANCE_TREE_NODE *bt_delete(void *name, BALANCE_TREE_NODE *t_node, BALANCE_TREE *btree, int *state);
static void bclear(BALANCE_TREE_NODE *p, BALANCE_TREE *btree);

static void push(BALANCE_TREE_NODE *data, BALANCE_TREE *btree);
static BALANCE_TREE_NODE *pop(BALANCE_TREE *btree);


BT_HANDLE btree_create(int (*comp)(void*, void*), void *(*data_init)(void), int (*data_copy)(void *src, void *dest), void (*data_clear)(void *))// 
{
	BALANCE_TREE *p;

	if(comp == NULL)
	{
		set_errmsg("btree_create ERROR: Parameter 1(comp) is NULL, Comparative function should be defined!");
		return BTREE_FAIL;
	}
	
	if(data_init == NULL)
	{
		set_errmsg("btree_create ERROR: Parameter 2(data_init) is NULL, data_init function should be defined!");
		return BTREE_FAIL;
	}
	
	if(data_copy == NULL)
	{
		set_errmsg("btree_create ERROR: Parameter 3(data_copy) is NULL, data_copy function should be defined!");
		return BTREE_FAIL;
	}
	
	p = (BALANCE_TREE *)malloc(sizeof(BALANCE_TREE));	
	if(p == NULL) 
	{
		set_errmsg("btree_create ERROR: no enough memory to allocate %u bytes!", sizeof(BALANCE_TREE));
		return BTREE_FAIL; 
	}

	p->root = NULL;
	p->iterator = NULL;
	p->use_iter = 0;
	
    p->data_init = data_init;	
	p->data_copy = data_copy;
    p->data_clear = data_clear;
	p->comp = comp;

	p->node_pool = mem_create(sizeof(BALANCE_TREE_NODE), POOLSIZE);
	if(p->node_pool == MEM_FAIL)
    {
        free(p);
        set_errmsg("btree_create ERROR: %s", mem_geterrmsg());
        return BTREE_FAIL;    
    }
	p->iter_pool = NULL;
		
	return (BT_HANDLE)p;
}

static void bclear(BALANCE_TREE_NODE *p, BALANCE_TREE *btree)
{
	if(p == NULL) return;

    bclear(p->left, btree);
    bclear(p->right, btree);
    btree->data_clear(p->data);    
}

void btree_destroy(BT_HANDLE btree)
{
	BALANCE_TREE *p = (BALANCE_TREE *)btree;
	if(p == NULL) return;
        
    if(p->data_clear != NULL)
        bclear(p->root, p);

    mem_destroy(p->node_pool);
    mem_destroy(p->iter_pool);

	free(p);
}

void *btree_search(BT_HANDLE btree, void *data)
{
	BALANCE_TREE *p;
	BALANCE_TREE_NODE *node;

	p = (BALANCE_TREE *)btree;
	if(p == NULL) 
	{
		set_errmsg("btree_search ERROR: Parameter 1(btree) is NULL");
		return NULL;
	}
	if(data ==NULL)
	{
		set_errmsg("btree_search ERROR: Parameter 2(data) is NULL");
		return NULL;
	}

	node = search(data, p->root , p->comp);

	if(node == NULL) 
    {
        set_errmsg("btree_search ERROR: data does not exists");
        return NULL;
    }

	return node->data;
}

int btree_insert(BT_HANDLE btree, void *data)
{
	BALANCE_TREE *p;
	BALANCE_TREE_NODE *root;

	p = (BALANCE_TREE *)btree;
	if(p == NULL)
	{
		set_errmsg("btree_insert ERROR: Parameter 1(btree) is NULL");
		return 1;
	}
	if(data ==NULL)
	{
		set_errmsg("btree_insert ERROR: Parameter 2(data) is NULL");
		return 1;
	}
	if(p->use_iter != 0)
	{
        set_errmsg("btree_insert ERROR: Using iterator now! plase use (btree_iter_end) to close iterator!");
        return 1;
    }
	
	root = insert(data, p->root, p);
    
	if(root == NULL) 
	{
		set_errmsg("btree_insert ERROR : %s", btree_geterrmsg());
		return 2;
	}
	p->root = root;
	return 0;
}

int btree_delete(BT_HANDLE btree, void *data)
{
	BALANCE_TREE *p;
	int state = 0;
	
	p = (BALANCE_TREE *)btree;
	if(p == NULL)
	{
		set_errmsg("btree_delete ERROR : Parameter 1(btree) is NULL");
		return 1;
	}
	if(data ==NULL)
	{
		set_errmsg("btree_delete ERROR : Parameter 2(data) is NULL");
		return 1;
	}
	if(p->use_iter != 0)
	{
        set_errmsg("btree_delete ERROR: Using iterator now! plase use (btree_iter_end) to close iterator!");
        return 1;
    }
    
	p->root = bt_delete(data, p->root, p, &state);
	if(state == 1)
	{
		set_errmsg("btree_delete ERROR : %s", btree_geterrmsg());
		return 1;
	}

	return 0;
}

static BALANCE_TREE_NODE *LL_Rotate( BALANCE_TREE_NODE *p )
{
    BALANCE_TREE_NODE *q;

	#ifdef DEBUG
		assert(p == NULL);
		assert(p->left == NULL);
	#endif
    q = p->left;
    p->left = q->right;
    q->right = p;

    p->height = Max( Height( p->left ), Height( p->right ) ) + 1;
    q->height = Max( Height( q->left ), p->height ) + 1;

    return q;  /* New root */
}

static BALANCE_TREE_NODE *RR_Rotate( BALANCE_TREE_NODE *p )
{
    BALANCE_TREE_NODE *q;

	#ifdef DEBUG
		assert(p == NULL);
		assert(p->right == NULL);
	#endif
    q = p->right;
    p->right = q->left;
    q->left = p;

    p->height = Max( Height( p->left ), Height( p->right ) ) + 1;
    q->height = Max( Height( q->right ), p->height ) + 1;

    return q;  /* New root */
}

static BALANCE_TREE_NODE *RL_Rotate( BALANCE_TREE_NODE *p )
{
    BALANCE_TREE_NODE *q;

	#ifdef DEBUG
		assert(p == NULL);
		assert(p->right == NULL);
		assert(p->right->left == NULL);
	#endif
	q = p->right->left;
	p->right->left = q->right;
	q->right = p->right;
	p->right = q->left;
	q->left = p;
	
    p->height = Max( Height( p->left ), Height( p->right ) ) + 1;   
    q->right->height = Max( Height( q->right->right ), Height( q->right->left ) ) + 1;
	q->height = Max( Height( q->right ), p->height ) + 1;

    return q;  /* New root */
}

static BALANCE_TREE_NODE *LR_Rotate( BALANCE_TREE_NODE *p )
{
    BALANCE_TREE_NODE *q;

	#ifdef DEBUG
		assert(p == NULL);
		assert(p->left == NULL);
		assert(p->left->right == NULL);
	#endif
	q = p->left->right;
	p->left->right = q->left;
	q->left = p->left;
	p->left = q->right;
	q->right = p;
	
    p->height = Max( Height( p->left ), Height( p->right ) ) + 1;
    q->left->height = Max( Height( q->left->right ), Height( q->left->left ) ) + 1;
	q->height = Max( Height( q->left ), p->height ) + 1;

    return q;  /* New root */
}

static BALANCE_TREE_NODE *search(void *data, BALANCE_TREE_NODE *p, int (*comp)(void *, void *))
{
	int cont;
	while(p != NULL)
	{
		if((cont = (*comp)(data, p->data)) < 0)
		{
			p = p->left;
		}
		else if(cont > 0)
		{
			p = p->right;
		}
		else
		{
			break;
		}
	}

	return p;
}

static BALANCE_TREE_NODE *insert(void *data, BALANCE_TREE_NODE *t_node, BALANCE_TREE *btree)
{
	BALANCE_TREE_NODE *temp;
	int cond, cond2;

	if(t_node == NULL)
	{
		t_node = (BALANCE_TREE_NODE *)mem_alloc(btree->node_pool);
		if(t_node == MEM_FAIL) 
		{
			set_errmsg("insert ERROR : %s", mem_geterrmsg());
			return NULL;
		}
		t_node->data = (*btree->data_init)();
		if(t_node->data == NULL)
		{
            mem_free(t_node);
            set_errmsg("insert ERROR: data_init fail!");
            return NULL;
        }
        if((*btree->data_copy)(data, t_node->data) != 0)
        {
            (*btree->data_clear)(t_node->data);
            mem_free(t_node);
            set_errmsg("insert ERROR: data_copy fail!");
            return NULL;
        }

        t_node->height = 0;
		t_node->left = NULL;
		t_node->right = NULL;
		return t_node;
	}
	if((cond = (*btree->comp)(data, t_node->data)) == 0) 
	{
		set_errmsg("insert ERROR : data already exists");
		return NULL;
	}
	else if(cond > 0)
	{
		temp = insert(data, t_node->right, btree);
		if(temp == NULL) return NULL;
		t_node->right = temp;

		if( Height( t_node->right ) - Height( t_node->left ) == 2 )//-2
		{
			if( (cond2 = (*btree->comp)(data, t_node->right->data)) > 0 )
				return RR_Rotate(t_node);
			else if(cond2 < 0)
				return RL_Rotate(t_node);
		}
		t_node->height = Max( Height( t_node->left ), Height( t_node->right ) ) + 1;
		return t_node;
	}
	else
	{
		temp = insert(data, t_node->left, btree);
		if(temp == NULL) return NULL;
		t_node->left = temp;

		if( Height( t_node->left ) - Height( t_node->right ) == 2 )//+2
		{
			if( (cond2 = (*btree->comp)(data, t_node->left->data)) < 0 )
				return LL_Rotate(t_node);
			else if(cond2 > 0)
				return LR_Rotate(t_node);
		}
		t_node->height = Max( Height( t_node->left ), Height( t_node->right ) ) + 1;
		return t_node;
	}			
}

static BALANCE_TREE_NODE *bt_delete(void *data, BALANCE_TREE_NODE *t_node, BALANCE_TREE *btree, int *state)
{
	int cond;
	BALANCE_TREE_NODE *temp;

	if(t_node != NULL)
	{
		if((cond = (*btree->comp)(data, t_node->data)) > 0)
		{
			t_node->right = bt_delete(data, t_node->right, btree, state);
			if( Height( t_node->left ) - Height( t_node->right ) == 2 )//+2
			{
				if( Height(t_node->left->right) < Height(t_node->left->left) )
					return LL_Rotate(t_node);
				else 
					return LR_Rotate(t_node);
			}
			t_node->height = Max( Height( t_node->left ), Height( t_node->right ) ) + 1;
			return t_node;
		}
		else if(cond < 0)
		{
			t_node->left = bt_delete(data, t_node->left, btree, state);
			if( Height( t_node->right ) - Height( t_node->left ) == 2 )//-2
			{
				if( Height(t_node->right->left) < Height(t_node->right->right) )
					return RR_Rotate(t_node);
				else 
					return RL_Rotate(t_node);
			}	
			t_node->height = Max( Height( t_node->left ), Height( t_node->right ) ) + 1;
			return t_node;
		}
		else
		{
			temp = t_node;
			if(t_node->right == NULL && t_node->left == NULL)
			{
				if(btree->data_clear != NULL)
                    (*btree->data_clear)(t_node->data);
				mem_free((void *)t_node);

				return NULL;
			}
			else if(t_node->right == NULL)
			{
				t_node = t_node->left;
				t_node->height = 0;

				if(btree->data_clear != NULL)
                    (*btree->data_clear)(temp->data);
				mem_free((void *)temp);

				return t_node;
			}
			else if(t_node->left == NULL)
			{
				t_node = t_node->right;
				t_node->height = 0;

				if(btree->data_clear != NULL)
                    (*btree->data_clear)(temp->data);
				mem_free((void *)temp);

				return t_node;
			}
			else
			{				
				t_node->left = delete_node(t_node, t_node->left, btree);
				if( Height( t_node->right ) - Height( t_node->left ) == 2 )//-2
				{
					if( Height(t_node->right->left) < Height(t_node->right->right) )
						return RR_Rotate(t_node);
					else 
						return RL_Rotate(t_node);
				}
				t_node->height = Max( Height( t_node->left ), Height( t_node->right ) ) + 1;
				return t_node;
			}
		}
	}
	*state = 1;
	set_errmsg("bt_delete ERROR : data dose not exists");
	return NULL;
}

static BALANCE_TREE_NODE *delete_node(BALANCE_TREE_NODE *t_node, BALANCE_TREE_NODE *p, BALANCE_TREE *btree)
{   
	BALANCE_TREE_NODE *temp;

	if(p->right != NULL)
	{
		p->right = delete_node(t_node, p->right, btree);
		if( Height( p->left ) - Height( p->right ) == 2 )//+2
		{
			if( Height(p->left->right) < Height(p->left->left) )
				p = LL_Rotate(p);
			else 
				p = LR_Rotate(p);
		}
	}
	else
	{
		if(btree->data_clear != NULL)
            (*btree->data_clear)(t_node->data);

		t_node->data = p->data;
        temp = p;
		p = p->left;

		mem_free((void *)temp);

	}
	if(p != NULL)
	{
		p->height = Max( Height( p->left ), Height( p->right ) ) + 1;
	}
	return p;
}

int btree_iter_create(BT_HANDLE btree)
{    
    BALANCE_TREE *p = (BALANCE_TREE *)btree;
    if(p == NULL)
    {
		set_errmsg("btree_iter_create ERROR : Parameter (btree) is NULL");
		return 1;
    }
    if(p->iter_pool == NULL)
    {
        p->iter_pool = mem_create(sizeof(ITER_STACK), 16);
        if(p->iter_pool == MEM_FAIL)
        {            
            set_errmsg("btree_iter_create ERROR: %s", mem_geterrmsg());
            return 1;
        }
    }
    btree_iter_end(btree);   
    if(p->root == NULL)
    {
        set_errmsg("btree_iter_create ERROR : btree is empty");
        return 1;
    }     
    push(p->root, p);
    p->use_iter = 1;
    return 0;
}

void *btree_iter_get(BT_HANDLE btree)
{
    BALANCE_TREE_NODE *node;
    BALANCE_TREE *p = (BALANCE_TREE *)btree;
    if(p == NULL)
    {
		set_errmsg("btree_iter_get ERROR : Parameter (btree) is NULL");
		return NULL;
    }
   	if(p->use_iter != 1)
	{
        set_errmsg("btree_iter_get ERROR: you need use (btree_iter_create) to creat iterator!");
        return NULL;
    }
    if(p->iterator == NULL) return NULL;
    node = pop(p);
    if (node->right != NULL) 
       push(node->right, p);
    if (node->left != NULL) 
       push(node->left, p);
    return node->data;
    
}

void btree_iter_end(BT_HANDLE btree)
{    
    ITER_STACK *temp;
    BALANCE_TREE *p = (BALANCE_TREE *)btree;
    if(p == NULL)
    {
		set_errmsg("btree_iter_end ERROR : Parameter (btree) is NULL");
		return;
    }
    while(p->iterator != NULL)
    {
        temp = p->iterator;
        p->iterator = p->iterator->next;
        mem_free((void *)temp);
    }
    p->use_iter = 0;
}

static void push(BALANCE_TREE_NODE *node, BALANCE_TREE *btree)
{
	ITER_STACK *temp;
	      
    temp = (ITER_STACK *)mem_alloc(btree->iter_pool);
    if(temp == MEM_FAIL)
    { 
        set_errmsg("push ERROR: %s", mem_geterrmsg());
        return ;
    }            
	temp->node = node;
	temp->next = btree->iterator;;
	btree->iterator = temp;
}

static BALANCE_TREE_NODE *pop(BALANCE_TREE *btree)
{
	BALANCE_TREE_NODE *node;
	ITER_STACK *temp;
	if (btree->iterator != NULL) 
    {
		node = btree->iterator->node;
		temp = btree->iterator;
		btree->iterator = btree->iterator->next;
		mem_free((void *)temp);
	}
	return node;
}

