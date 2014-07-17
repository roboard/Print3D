#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

#include "mem_pool.h"
#include "btree.h"
#include "ini.h"

#define _INI_ERRSYS

#ifdef _INI_ERRSYS

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

	char *ini_geterrmsg(void) 
	{
		return &(ERR_MsgBuf[0]);
	}

#else

//#include xxxx.h
	char *ini_geterrmsg(void) 
	{
		return geterrmsg();
	}

#endif

#define SECTION				0
#define KEY					1
#define NOTE				2
#define ERROR				3
#define OTHER				4
#define NEW_SECTION			5
#define NEW_KEY				6
#define MODIFY_KEY			7

#define SECTION_POOL_SIZE	256
#define KEY_POOL_SIZE		256
#define LINE_POOL_SIZE		256
#define STR_POOL_SIZE		256
#define STR_SIZE			128

#define MIN(a, b) ((a) <= (b) ? (a) : (b))

typedef struct _INI_STR
{
    char*		start;
    int			length;
} INI_STR;

typedef struct _INI_LINE
{
    INI_STR		str;
	INI_STR		add_str;
    int			type;

    struct _INI_LINE *pre;
    struct _INI_LINE *next;
} INI_LINE;

typedef struct _INI_SECTION
{
    INI_STR		section_name;
    
    BT_HANDLE	key_btree;

    INI_LINE*	line_node;
} INI_SECTION;

typedef struct _INI_KEY
{
    INI_STR		key_name;
    INI_STR		key_val;

    INI_LINE*	line_node;
} INI_KEY;

typedef struct _INI_FILE
{
    char*		filename;
    char*		filebuf;
	int			flag;

    BT_HANDLE	section_btree;

    INI_LINE*	head;
    INI_LINE*	tail;   
} INI_FILE;

static MP_HANDLE section_pool = MEM_FAIL;	
static MP_HANDLE key_pool	  = MEM_FAIL;	
static MP_HANDLE line_pool	  = MEM_FAIL; 
static MP_HANDLE str_pool	  = MEM_FAIL;
static int file_count = 0;

static int initial(INI_HANDLE ini_file, FILE *fp);
static void fcopy(char *buf, FILE *fp);
static char *getline(char *line, INI_STR *line_str, INI_STR *name_str, INI_STR *val_str, int *type);
static void destroy(INI_HANDLE ini_file);
static int write(INI_HANDLE ini_file);
static void pool_destroy(void);
/**********************************************************/
static int section_comp(void *data1, void *data2);
static void *section_init(void);
static int section_copy(void *src, void *dest);
static void section_clear(void *data);

#define SECTION_INSERT_ORIGINAL(ini, name, line_node) section_insert(ini, NULL, name, line_node)
#define SECTION_INSERT_NEW(ini, section) section_insert(ini, section, NULL, NULL)
static int section_insert(INI_HANDLE ini_file, char *section, INI_STR *name, INI_LINE *line_node);

static INI_SECTION *section_search(INI_HANDLE ini_file, char *section);
static int section_delete(INI_HANDLE ini_file, char *section);
/*********************************************************/
static int key_comp(void *data1, void *data2);
static void *key_init(void);
static int key_copy(void *src, void *dest);
static void key_clear(void *data);

#define KEY_INSERT_ORIGINAL(ini, section_name, name_str, val_str, line_node) key_insert(ini, section_name, NULL, NULL, name_str, val_str, line_node)
#define KEY_INSERT_NEW(ini, section_name, key_name, key_val) key_insert(ini, section_name, key_name, key_val, NULL, NULL, NULL)
static int key_insert(INI_HANDLE ini_file, char *section_name, char *key_name, char *key_val, INI_STR *name_str, INI_STR *val_str, INI_LINE *line_node);

static INI_KEY *key_search(INI_HANDLE ini_file, char *section, char *key_name);
static int key_delete(INI_HANDLE ini_file, char *section, char *key_name);
/*********************************************************/

static int initial(INI_HANDLE ini_file, FILE *fp)
{
	INI_FILE*	ini = (INI_FILE *)ini_file;
	long		filelen;
	
    INI_STR		line_str;
	INI_STR		name_str;

	INI_STR		val_str;
    int			type;
	char*		buf;
	int			linenumber = 0;
	INI_LINE*	new_line;

	char*		section = NULL;

	if(fp != NULL)
	{
		fseek(fp, 0, SEEK_END);
		filelen = ftell(fp);
		fseek(fp, 0, SEEK_SET);
   		ini->filebuf = (char *)malloc((size_t)(filelen + 2) * sizeof(char));
		if(ini->filebuf == NULL)
		{
			set_errmsg("initial ERROR: no enough memory to allocate %u bytes!", (filelen + 2) * sizeof(char));
			return -1;			
		}
		fcopy(ini->filebuf, fp);
	}
    
	if(file_count == 0)
	{
		section_pool = mem_create(sizeof(INI_SECTION), SECTION_POOL_SIZE);
		if(section_pool == MEM_FAIL)
		{
			free(ini->filebuf);
			set_errmsg("initial ERROR: %s", mem_geterrmsg());
			return -1;
		}
		key_pool = mem_create(sizeof(INI_KEY), KEY_POOL_SIZE);	
		if(key_pool == MEM_FAIL)
		{			
			mem_destroy(section_pool);
			free(ini->filebuf);
			set_errmsg("initial ERROR: %s", mem_geterrmsg());
			return -1;
		}
		line_pool = mem_create(sizeof(INI_LINE), LINE_POOL_SIZE);	
		if(line_pool == MEM_FAIL)
		{
			mem_destroy(key_pool);
			mem_destroy(section_pool);
			free(ini->filebuf);
			set_errmsg("initial ERROR: %s", mem_geterrmsg());
			return -1;
		}
		str_pool = mem_create(STR_SIZE * sizeof(char), STR_POOL_SIZE);
		if(str_pool == MEM_FAIL)
		{ 
			mem_destroy(line_pool);
			mem_destroy(key_pool);
			mem_destroy(section_pool);
			free(ini->filebuf);
			set_errmsg("initial ERROR: %s",mem_geterrmsg());
			return -1;
		}
	}

	ini->section_btree = btree_create(section_comp, section_init, section_copy, section_clear);
	if(ini->section_btree == BTREE_FAIL)
	{ 
        set_errmsg("initial ERROR: %s", btree_geterrmsg());
	    goto ERROR_END1;
    }
	
	buf = ini->filebuf;
	while((buf = getline(buf, &line_str, &name_str, &val_str, &type)) != NULL)
	{
		new_line = (INI_LINE *)mem_alloc(line_pool);
		if(new_line == MEM_FAIL)
		{
			set_errmsg("initial ERROR: %s", mem_geterrmsg());
			goto ERROR_END;
		} 
		new_line->str.start = line_str.start;
		new_line->str.length = line_str.length;
		new_line->type = type;

		new_line->next = NULL;
		
		if(ini->head == NULL)
		{
			new_line->pre = NULL;
			ini->head = new_line;
		}
		else
		{
			new_line->pre = ini->tail;
			ini->tail->next = new_line;
		}
		ini->tail = new_line;
		
		if(type == SECTION)
		{
			int i;
			if(name_str.length >= STR_SIZE)
			{
				set_errmsg("initial ERROR: line %d format is error(section name must < %d bytes), plase check file", linenumber, STR_SIZE);
				goto ERROR_END;
			}

			if(SECTION_INSERT_ORIGINAL(ini, &name_str, new_line) != 0)
			{
				set_errmsg("initial ERROR: %s", ini_geterrmsg());
				goto ERROR_END;
			}
			if(section == NULL)
			{
				section = (char *)mem_alloc(str_pool);				
				if(section == NULL)
				{
					set_errmsg("initial ERROR:  %s", mem_geterrmsg());
					goto ERROR_END;
				}
			}
			for(i = 0; i < name_str.length && i < STR_SIZE; i++)
				section[i] = name_str.start[i];
			section[i] = '\0';  
		}
		else if(type == KEY)
		{
			if(name_str.length >= STR_SIZE)
			{
				set_errmsg("initial ERROR: line %d format is error(key name must < %d bytes), plase check file", linenumber, STR_SIZE);
				goto ERROR_END;
			}
			if(val_str.length >= STR_SIZE)
			{
				set_errmsg("initial ERROR: line %d format is error(value must < %d bytes), plase check file", linenumber, STR_SIZE);
				goto ERROR_END;
			}

			if(section == NULL)
			{
				set_errmsg("initial ERROR: no section exists before key");
				goto ERROR_END;
			}
			else if(KEY_INSERT_ORIGINAL(ini, section, &name_str, &val_str, new_line) != 0)
			{
				set_errmsg("initial ERROR: %s", ini_geterrmsg());
				goto ERROR_END;
			}   
		}
		else if(type == ERROR)
		{
			set_errmsg("initial ERROR: line %d format is error, plase check file", linenumber);	
			goto ERROR_END;
		} 
		linenumber++;
	}
	mem_free(section);
	file_count++;
	return linenumber + 1;

ERROR_END:
	mem_free(section);
	while(ini->head != NULL)
	{
		new_line = ini->head;
        ini->head = ini->head->next;
		mem_free(new_line);
	}
	btree_destroy(ini->section_btree);

ERROR_END1:
	if(file_count == 0)
		pool_destroy();

	free(ini->filebuf);

	return -1;
}

static void fcopy(char *buf, FILE *fp)
{
	int c, i = 0;
	while((c = getc(fp)) != EOF)
	{
		buf[i++] = (char)c;
	}
	if(buf[i - 1] == '\n')
		buf[i] = '\0';
	else
    {
        buf[i] = '\n';
        buf[i + 1] = '\0';
    }	
}

static int section_line(char *line, INI_STR *line_str, INI_STR *name_str, int *type, int i)
{
    int j = 0; 
    int space = 0;

    i++;
    while(line[i] != '\n' && isspace(line[i]))
        i++;
    while(line[i] != '\0' && line[i] != '\n' &&  line[i] != ']' && line[i] != ';')
    {
		if(line[i] == ' ')
		{
			i++;
			space++;
		}
		else
		{
			if(isspace(line[i]))
				break;
			while(space != 0)
			{
				j++;
				space--;
			}
			j++;
			i++;
        }            
    }

    while(line[i] != '\n' && isspace(line[i]))
	{
        i++;
		space++;
	}

    if(line[i] == ']')
    {
		name_str->start = &line[i - j - space];
		name_str->length = j;
		if(j == 0)
			*type = ERROR;
		else
			*type = SECTION;
    } 
    else
    {
        *type = ERROR;
    }
	i++;//
	while(line[i] != '\0' && line[i] != '\n' && line[i] != ';')
	{
		if(!isspace(line[i]))
			*type = ERROR;
        i++;
	}
    while(line[i] != '\0' && line[i] != '\n')
        i++;
    line_str->start = &line[0];
	if(line[i] == '\n')
		line_str->length = i + 1;  
	else
		line_str->length = i;
	return i;
}

static int key_line(char *line, INI_STR *line_str, INI_STR *name_str, INI_STR *val_str, int *type, int i)
{
	int j = 0;
	int space = 0;
	while(line[i] != '\0' && line[i] != '\n' && line[i] != ';' && line[i] != '=')
	{
		if(line[i] == ' ')
		{
			space++;
			i++;
		}
		else
		{
			if(isspace(line[i]))
				break;
			while(space != 0)
			{
				j++;
				space--;
			}
			j++;
			i++;
        }
	}
    name_str->start = &line[i - j - space];
    name_str->length = j;
    while(line[i] != '\n' && isspace(line[i]))
        i++;

	if(line[i] == '=')
	{
		i++;
		while(line[i] != '\n' && isspace(line[i]))
			i++;
		j = 0;
		space = 0;
		while(line[i] != '\0' && line[i] != '\n' && line[i] != ';')
		{
			if(line[i] == ' ')
			{
				i++;
				space++;
			}
			else
			{
				if(isspace(line[i]))
					break;
				while(space != 0)
				{
					space--;
					j++;
				}
				i++;
				j++;
			}
		}
		val_str->start = &line[i - j - space];
		val_str->length = j;
		while(line[i] != '\n' && isspace(line[i]))
			i++;

		if(line[i] != '\0' && line[i] != '\n' && line[i] != ';')
			*type = ERROR;
		else
			*type = KEY;
	}
	else 
	{
		*type = ERROR;          
	}

    while(line[i] != '\0' && line[i] != '\n') 
        i++;                        
    line_str->start = &line[0];
	if(line[i] == '\n')
		line_str->length = i + 1;  
	else
		line_str->length = i;
   return i;
}

static char *getline(char *line, INI_STR *line_str, INI_STR *name_str, INI_STR *val_str, int *type)
{
    int i = 0;

    if(line == NULL)
        return NULL;
    while(line[i] != '\n' && isspace(line[i]))
        i++;
    if(line[i] == '[')
    {
		i = section_line(line, line_str, name_str, type, i);
    }
    else if(line[i] != '\0' && line[i] != '\n' && line[i] != ';' && line[i] != '=')
    {
		i = key_line(line, line_str, name_str, val_str, type, i);
    }
    else
    {
		if(line[i] == '=')
		{
			*type = ERROR;
		}
		else if(line[i] == ';')
		{
			*type = NOTE;
		}
		else
		{
			*type = OTHER;
		}
        while(line[i] != '\0' && line[i] != '\n')
            i++;
        line_str->start = &line[0];
		if(line[i] == '\n')
			line_str->length = i + 1;
		else
			line_str->length = i;
    }
    if(line[i] == '\0')
		return NULL;
    else
        return &line[i + 1];
}

static char* check_string(char *string)
{
	int len;

	if(string == NULL)
		return NULL;

	len = strlen(string);
	if(isspace(string[0]))
		return NULL;
	if(isspace(string[len]))
		return NULL;
	if(len == 0 || len >= STR_SIZE)
		return NULL;
	return string;
}

static void destroy(INI_HANDLE ini_file)
{
	INI_FILE *ini = (INI_FILE *)ini_file;
	INI_LINE *temp;
    while(ini->head != NULL)
    {
		temp = ini->head;
		ini->head = ini->head->next;
		if(temp->type == NEW_SECTION || temp->type == NEW_KEY)
            mem_free(temp->str.start);
		if(temp->type == NEW_KEY || temp->type == MODIFY_KEY)
			mem_free(temp->add_str.start);
		mem_free(temp);
    }
	
    btree_destroy(ini->section_btree); 
    free(ini->filebuf);
    free(ini->filename);
}

static int write(INI_HANDLE ini_file)
{
	INI_FILE *ini = (INI_FILE *)ini_file;
	char *name = tmpnam(NULL);
	FILE *fp;
	INI_LINE *p;

	rename(ini->filename, name);
	
    fp = fopen(ini->filename, "w");
	if(fp == NULL)
	{
		set_errmsg("write ERROR: file to open file!");
		rename(name, ini->filename);
		return 1;
	}
    
	p = ini->head;
	while(p != NULL)
	{
		int i;
		if(p->type == NEW_SECTION)
		{	
			if(fputc((int)'[', fp) == EOF) goto ERROR_END;

			for(i = 0; i < p->str.length; i++)
			{
				if(fputc((int)p->str.start[i], fp) == EOF) goto ERROR_END;
			}
			if(fputc((int)']', fp) == EOF) goto ERROR_END;
	
			if(fputc((int)'\n', fp) == EOF) goto ERROR_END;
		}
		else if(p->type == NEW_KEY)
		{
			for(i = 0; i < p->str.length; i++)
			{
				if(fputc((int)p->str.start[i], fp) == EOF) goto ERROR_END;
			}

			if(fputc((int)'=', fp) == EOF) goto ERROR_END;

			for(i = 0; i < p->add_str.length; i++)
			{
				if(fputc((int)p->add_str.start[i], fp) == EOF) goto ERROR_END;
			}

			if(fputc((int)'\n', fp) == EOF) goto ERROR_END;
		}
		else if(p->type == MODIFY_KEY)
		{
			int j, k;
			for(i = 0; p->str.start[i] != '='; i++)
			{
				if(fputc((int)p->str.start[i], fp) == EOF) goto ERROR_END;
			}
			if(fputc((int)'=', fp) == EOF) goto ERROR_END;

			i++;
			for(; p->str.start[i] != '\n' && isspace(p->str.start[i]); i++)
			{
				if(fputc((int)p->str.start[i], fp) == EOF) goto ERROR_END;
			}
			k = i;
			for(; p->str.start[i] != ';' && p->str.start[i] != '\n'; i++)
			{
				if(!isspace(p->str.start[i]))
					k = i + 1;
			}

			for(j = 0; j < p->add_str.length; j++)
			{
				if(fputc((int)p->add_str.start[j], fp) == EOF) goto ERROR_END;
			}

			for(; k < p->str.length; k++)
			{
				if(fputc((int)p->str.start[k], fp) == EOF) goto ERROR_END;
			}
		}
		else
		{
			for(i = 0; i < p->str.length; i++)
			{
				if(fputc((int)p->str.start[i], fp) == EOF) goto ERROR_END;
			}
		}
		p = p->next;
	}
	fclose(fp);
	remove(name);
	return 0;

ERROR_END:
	set_errmsg("write ERROR: file to write file!");
	fclose(fp);
	remove(ini->filename);
	rename(name, ini->filename);

	return 1;
}

static void pool_destroy(void)
{
    mem_destroy(section_pool);
    mem_destroy(key_pool); 
	mem_destroy(line_pool);
	mem_destroy(str_pool);
}

/***************************section************************/
static int section_comp(void *data2, void *data1)
{
    int i, min;
    INI_SECTION *p = (INI_SECTION *)data1;
    INI_SECTION *q = (INI_SECTION *)data2;
    min = MIN(p->section_name.length, q->section_name.length);
	for(i = 0; i < min && (tolower((int)p->section_name.start[i])) == (tolower((int)q->section_name.start[i])); i++)
    //for(i = 0; i < min && p->section_name.start[i] == q->section_name.start[i]; i++)
    {
        if(i == min - 1 && q->section_name.length == p->section_name.length)
        {
            return 0;
        }
    }
    if(i == min)
    {
        if(p->section_name.length > q->section_name.length)
            return 1;
        else
            return -1;
    }
    return (p->section_name.start[i] - q->section_name.start[i]);
}

static void *section_init(void)
{
    INI_SECTION *data = (INI_SECTION *)mem_alloc(section_pool);
	if(data == NULL) return NULL;

    data->section_name.start = NULL;
    data->section_name.length = 0;
    data->key_btree = btree_create(key_comp, key_init, key_copy, key_clear);
    if(data->key_btree == NULL)
    {
        mem_free(data);
        return NULL;
    }
    data->line_node = NULL;
    return (void *)data;
}

static int section_copy(void *src, void *dest)
{
    INI_SECTION *org = (INI_SECTION *)src;
    INI_SECTION *data = (INI_SECTION *)dest;  

    if(src == NULL || dest == NULL)
        return 1;
    data->section_name.start = org->section_name.start;
    data->section_name.length = org->section_name.length;
    data->line_node = org->line_node;
    return 0;
}

static void section_clear(void *data)
{
    INI_SECTION *temp = (INI_SECTION *)data;

    btree_destroy(temp->key_btree);
    mem_free(data);      
}
static int section_insert(INI_HANDLE ini_file, char *section, INI_STR *name, INI_LINE *line_node)
{
   	INI_FILE *ini = (INI_FILE *)ini_file;
	INI_SECTION s_node;
    char *line;

    if(section != NULL)
    {
		int len = strlen(section);
 		
        line_node = (INI_LINE *)mem_alloc(line_pool); 
        if(line_node == NULL)
        {
            set_errmsg("section_insert ERROR: %s", mem_geterrmsg());
            return 1;
        }     
		line = (char *)mem_alloc(str_pool);
        if(line == NULL)
        {
            mem_free((void *)line_node);
            set_errmsg("section_insert ERROR: %s", mem_geterrmsg());
            return 1;
        }
        
		strcpy(line, section);
		
		line_node->str.start = line;
		line_node->str.length = len;
        line_node->type = NEW_SECTION;
        		
        line_node->next = NULL;
        if(ini->head == NULL)
        {
            line_node->pre = NULL;
            ini->head = line_node;
        }
        else
        {
            line_node->pre = ini->tail;
            ini->tail->next = line_node;
        }	
        ini->tail = line_node;
        
        s_node.section_name.start = line;
        s_node.section_name.length = len;
        s_node.line_node = line_node;
    }
    else
    {
        s_node.section_name.start = name->start;
        s_node.section_name.length = name->length;
        s_node.line_node = line_node;        
    }
    
    if(btree_insert(ini->section_btree, (void *)&s_node) != 0)
    {
        if(section != NULL)
        {
            if(ini->head == line_node)
            {
                ini->head = NULL;
				ini->tail = NULL;
            }
            else
            {
                line_node->pre->next = NULL;
                ini->tail = line_node->pre;
            }
            mem_free(line); 
			mem_free(line_node);
        }
        set_errmsg("section_insert ERROR: %s", btree_geterrmsg());
        return 1;
    }
	if(line_node->type == NEW_SECTION)
		ini->flag = 1;
    return 0;
}

static INI_SECTION *section_search(INI_HANDLE ini_file, char *section)
{
    INI_FILE *ini = (INI_FILE *)ini_file;
	INI_SECTION p;
    INI_SECTION *q;
    p.section_name.start = section;
    p.section_name.length = strlen(section);
   
    q = (INI_SECTION *)btree_search(ini->section_btree, (void *)&p);
    if(q == NULL)
    {
        set_errmsg("section_search ERROR: %s", btree_geterrmsg()); 
    }
    return q;
}

static int section_delete(INI_HANDLE ini_file, char *section)
{
  	INI_FILE *ini = (INI_FILE *)ini_file;
	INI_SECTION *p;
	INI_SECTION q;
    INI_LINE *start;
    INI_LINE *end;
    INI_LINE *temp;
	INI_LINE *last;

    if((p = section_search(ini, section)) == NULL)
    {
        set_errmsg("section_delete ERROR: %s", ini_geterrmsg());
        return 1; 
    }

    start = p->line_node->pre;
    end = p->line_node;
	last = p->line_node->next;
	do
	{
		if(end->type == KEY || end->type == NEW_KEY || end->type == MODIFY_KEY)
			last = end->next;
		end = end->next;
	}
	while(end != NULL && end->type != SECTION && end->type != NEW_SECTION);

    end = p->line_node;
	while(end != last)
	{
        temp = end;
        end = end->next;
		if(temp->type == NEW_SECTION || temp->type == NEW_KEY)
            mem_free(temp->str.start);
		if(temp->type == NEW_KEY || temp->type == MODIFY_KEY)
			mem_free(temp->add_str.start);
        mem_free(temp);
	}

	q.section_name.start = section;
	q.section_name.length = strlen(section);
    btree_delete(ini->section_btree, (void *)&q);

    if(start == NULL)
        ini->head = end;        
    else
		start->next = end;

    if(end == NULL)
		ini->tail = start;
	else
		end->pre = start;
                
	ini->flag = 1;
    return 0;
}

/*************************end section****************************/

/****************************key*********************************/

char lower(char c){
	if(c >= 'A' && c <= 'Z')
		return c - 'A' + 'a';
	else
		return c;
}


static int key_comp(void *data2, void *data1)
{
    int i, min;
    INI_KEY *p = (INI_KEY *)data1;
    INI_KEY *q = (INI_KEY *)data2;
    min = MIN(p->key_name.length, q->key_name.length);
    for(i = 0; i < min && ((lower(p->key_name.start[i])) == (lower(q->key_name.start[i]))); i++)
	//for(i = 0; i < min && p->key_name.start[i] == q->key_name.start[i]; i++)
    {
        if(i == min - 1 && q->key_name.length == p->key_name.length)
        {
            return 0;
        }
    }
    if(i == min)
    {
        if(p->key_name.length > q->key_name.length)
            return 1;
        else
            return -1;
    }
    return (p->key_name.start[i] - q->key_name.start[i]);
}

static void *key_init(void)
{
    INI_KEY *data = (INI_KEY *)mem_alloc(key_pool);
	if(data == NULL) return NULL;

    data->key_name.start = NULL;
    data->key_name.length = 0;
    data->key_val.start = NULL;
    data->key_val.length = 0;

    data->line_node = NULL;
    return (void *)data;
}

static int key_copy(void *src, void *dest)
{
    INI_KEY *org = (INI_KEY *)src;
    INI_KEY *data = (INI_KEY *)dest;       
    if(src == NULL || dest == NULL)
        return 1;
    data->key_name.start = org->key_name.start;
    data->key_name.length = org->key_name.length;
    data->key_val.start = org->key_val.start;
    data->key_val.length = org->key_val.length;
    data->line_node = org->line_node;
    return 0;
}

static void key_clear(void *data)
{
    mem_free(data);      
}
static int key_insert(INI_HANDLE ini_file, char *section_name, char *key_name, char *key_val, INI_STR *key, INI_STR *val, INI_LINE *line_node)
{
	INI_FILE *ini = (INI_FILE *)ini_file;
    char *line;
    INI_SECTION *p;
    INI_KEY *key_node;

	INI_STR tmpkey;
	INI_STR tmpval;
	INI_KEY tmp_key_node;
    
    if((p = section_search(ini, section_name)) == NULL)
    {
        if(SECTION_INSERT_NEW(ini, section_name) != 0)
        {
            set_errmsg("key_insert ERROR: %s", ini_geterrmsg());
            return 1;
        }
        p = section_search(ini, section_name);
    }

    if(key_name != NULL && (key_node = key_search(ini, section_name, key_name)) != NULL)
    {
		int len = strlen(key_val);

		if(key_node->line_node->type == KEY)
		{
			line = (char *)mem_alloc(str_pool);
			if(line == NULL)
			{
				set_errmsg("key_insert ERROR: %s", mem_geterrmsg());
				return 1;
			}
		}
		else //MODFIY_KEY or NEW_KEY
		{
			line = key_node->key_val.start;
		}

		strcpy(line, key_val);
		key_node->key_val.start = line;
		key_node->key_val.length = len;

		key_node->line_node->add_str.start = line;
		key_node->line_node->add_str.length = len;

		if(key_node->line_node->type != NEW_KEY)
			key_node->line_node->type = MODIFY_KEY;
		
		ini->flag = 1;
        return 0;
    }
    if(key_name != NULL)
    {
		int keylen = strlen(key_name);
		int vallen = strlen(key_val);

        line_node = (INI_LINE *)mem_alloc(line_pool); 
        if(line_node == NULL)
        {
            set_errmsg("key_insert ERROR: %s", mem_geterrmsg());
            return 1;
        } 

		tmpkey.start = (char *)mem_alloc(str_pool);
        if(tmpkey.start == NULL)
        {
            mem_free((void *)line_node);
            set_errmsg("key_insert ERROR: %s", mem_geterrmsg());
            return 1;
        }
		tmpval.start = (char *)mem_alloc(str_pool);
        if(tmpval.start == NULL)
        {
			mem_free((void *)tmpkey.start);
            mem_free((void *)line_node);
            set_errmsg("key_insert ERROR: %s", mem_geterrmsg());
            return 1;
        }

		strcpy(tmpkey.start, key_name);
		tmpkey.length = keylen;

		strcpy(tmpval.start, key_val);
		tmpval.length = vallen;

		line_node->str.start = tmpkey.start;
		line_node->str.length = keylen;
		line_node->add_str.start = tmpval.start;
		line_node->add_str.length = vallen;

        line_node->pre = p->line_node;
        line_node->next = p->line_node->next;
        if(p->line_node->next != NULL)
        {
            p->line_node->next->pre = line_node;
        }
        else
        {        
            ini->tail = line_node;
        } 
        p->line_node->next = line_node;  
        line_node->type = NEW_KEY; 

		key = &tmpkey;
		val = &tmpval;
		
    } 
    
    tmp_key_node.key_name.start = key->start;
    tmp_key_node.key_name.length = key->length;
    tmp_key_node.key_val.start = val->start;
    tmp_key_node.key_val.length = val->length;
    tmp_key_node.line_node = line_node;
    
    if(btree_insert(p->key_btree, (void *)&tmp_key_node) != 0)
    {
		set_errmsg("key_insert ERROR: %s", btree_geterrmsg());
		if(key_name != NULL)
		{
			line_node->pre->next = line_node->next;
			if(line_node == ini->tail)
				ini->tail = line_node->pre;
			else
				line_node->next->pre = line_node->pre;

			mem_free((void *)line_node->add_str.start);
			mem_free((void *)line_node->str.start);
			mem_free((void *)line_node);
		}
        return 1;
    }
	if(line_node->type == NEW_KEY || line_node->type == MODIFY_KEY)
		ini->flag = 1;
    return 0;
}

static INI_KEY *key_search(INI_HANDLE ini_file, char *section, char *key_name)
{
	INI_FILE *ini = (INI_FILE *)ini_file;
    INI_SECTION *p;
    INI_KEY temp;
    INI_KEY *key;
    if((p = section_search(ini, section)) == NULL)
    {
        set_errmsg("key_search ERROR: %s", ini_geterrmsg());
        return NULL;
    }
    temp.key_name.start = key_name;
    temp.key_name.length = strlen(key_name);    

    key = (INI_KEY *)btree_search(p->key_btree, (void *)&temp);
	if(key == NULL)
	{
		set_errmsg("key_search ERROR: %s", btree_geterrmsg());
	}

    return key;
}

static int key_delete(INI_HANDLE ini_file, char *section, char *key_name)
{
   	INI_FILE *ini = (INI_FILE *)ini_file;
	INI_SECTION *s_node;
    INI_KEY *p;
	INI_KEY q;
	INI_LINE* line_node;

    if((s_node = section_search(ini, section)) == NULL)
    {
        set_errmsg("key_delete ERROR: %s", ini_geterrmsg());
        return 1;
    }
    if((p = key_search(ini, section, key_name)) == NULL)
    {
        set_errmsg("key_delete ERROR: %s", btree_geterrmsg());
        return 1;
    }
    p->line_node->pre->next = p->line_node->next;
    if(p->line_node->next == NULL)
        ini->tail = p->line_node->pre;
    else
        p->line_node->next->pre = p->line_node->pre;

    line_node = p->line_node;

	q.key_name.start = key_name;
	q.key_name.length = strlen(key_name);
    btree_delete(s_node->key_btree, (void *)&q);

    if(line_node->type == NEW_KEY)
        mem_free(line_node->str.start);
	if(line_node->type == NEW_KEY || line_node->type == MODIFY_KEY)
		mem_free(line_node->add_str.start);
    mem_free(line_node);

	ini->flag = 1;
    return 0;
}

/********************************end key*************************/

INI_HANDLE ini_open(char *filename)
{
	int n; 
	FILE *fp;
	INI_FILE *ini;
	if(filename == NULL)
	{
		set_errmsg("ini_open ERROR: Parameter(filename) is NULL!");
		return INI_FAIL;
	}
	ini = (INI_FILE *)malloc(sizeof(INI_FILE));
	if(ini == NULL)
    {
        set_errmsg("ini_open ERROR: no enough memory to allocate %u bytes!", sizeof(INI_FILE));
        return INI_FAIL;
    }
	ini->filename = (char *)malloc((strlen(filename) + 1) * sizeof(char));
	if(ini->filename == NULL)
    {
        free((void *)ini);
		set_errmsg("ini_open ERROR: no enough memory to allocate %u bytes!", (strlen(filename) + 1) * sizeof(char));
        return INI_FAIL;
    }	

    strcpy(ini->filename, filename); 
	ini->section_btree = BTREE_FAIL;
    ini->filebuf = NULL;
	ini->flag = 0;
    ini->head = NULL;
    ini->tail = NULL;

	if((fp = fopen(filename,"r")) != NULL)
	{
		n = initial(ini, fp);
		fclose(fp);
	}
	else
	{
		n = initial(ini, NULL);
	}
	if(n == -1)
	{
		free((void *)ini->filename);
		free((void *)ini);
		return INI_FAIL;
	}    

	return (INI_HANDLE)ini;
}

int ini_close(INI_HANDLE ini_file)
{
    int result = 0;
    
	if(ini_commit(ini_file) != 0)
	{
		set_errmsg("ini_close ERROR: %s", ini_geterrmsg());
		result = result + 0x02;
	}
	if(ini_close2(ini_file) != 0)
	{
		set_errmsg("ini_close ERROR: %s", ini_geterrmsg());
		result = result + 0x01;
	}

	return result;
}

int ini_close2(INI_HANDLE ini_file)
{
	INI_FILE *ini = (INI_FILE *)ini_file;
	if(ini == NULL)
    {
        set_errmsg("ini_close2 ERROR: Parameter(ini_file) is NULL!");
        return 1;
    }
    destroy(ini_file);
	file_count--;
	if(file_count == 0)
		pool_destroy();
	return 0;
}

int ini_commit(INI_HANDLE ini_file)
{
	INI_FILE *ini = (INI_FILE *)ini_file;
	if(ini == NULL)
    {
        set_errmsg("ini_commit ERROR: Parameter(ini_file) is NULL!");
        return 1;
    }
	if(ini->flag != 0)
	{	
		if(write(ini_file) != 0)
		{
			set_errmsg("ini_commit ERROR: %s", ini_geterrmsg());
			return 1;
		}
		else
			ini->flag =0;
	}
	return 0;
}

int ini_add_section(INI_HANDLE ini_file, char *section)
{
	if(ini_file == NULL)
    {
        set_errmsg("ini_add_section ERROR: Parameter 1(ini_file) is NULL!");
        return -1;
    }
	if(check_string(section) == NULL)
    {
        set_errmsg("ini_add_section ERROR: Parameter 2(section) is NULL!");
        return -1;
    }
    return SECTION_INSERT_NEW(ini_file, section);
}

int ini_add_key(INI_HANDLE ini_file, char *section, char *key, char *val)
{
	if(ini_file == NULL)
    {
        set_errmsg("ini_add_key ERROR: Parameter 1(ini_file) is NULL!");
        return -1;
    }
	if(check_string(section) == NULL)
	{
        set_errmsg("ini_add_key ERROR: Parameter 2(section) format is error");
        return -1;
	}
	if(check_string(key) == NULL)
	{
        set_errmsg("ini_add_key ERROR: Parameter 3(key) format is error");
        return -1;
	}
	if(check_string(val) == NULL)
	{
        set_errmsg("ini_add_key ERROR: Parameter 4(val) format is error!");
        return -1;
	}
	return KEY_INSERT_NEW(ini_file, section, key, val);
}

char *ini_get_key(INI_HANDLE ini_file, char *section, char *key, char* valbuf)
{
	INI_KEY *p;
	if(ini_file == NULL)
    {
        set_errmsg("ini_get_val ERROR: Parameter 1(ini_file) is NULL!");
        return NULL;
    }
	if(check_string(section) == NULL)
	{
        set_errmsg("ini_get_val ERROR: Parameter 2(section) format is error");
        return NULL;
	}
	if(check_string(key) == NULL)
	{
        set_errmsg("ini_get_val ERROR: Parameter 3(key) format is error");
        return NULL;
	}
	if((p = key_search(ini_file, section, key)) == NULL)
	{
		set_errmsg("ini_get_val ERROR: %s", ini_geterrmsg());
        return NULL;
	}
	else
	{
		int i;
        for(i = 0; i < p->key_val.length ; i++)
            valbuf[i] = p->key_val.start[i];
        valbuf[i] = '\0';
        return valbuf;
    }
}

int *ini_get_key_int(INI_HANDLE ini_file, char *section, char *key, int* valbuf)
{
	char temp[128] = {'\0'};
	if(ini_get_key(ini_file, section, key, &temp[0]) == NULL) return NULL; 
	*valbuf = atoi(temp);
	return valbuf;
}

unsigned long *ini_get_key_ulong(INI_HANDLE ini_file, char *section, char *key, unsigned long* valbuf)
{
	char temp[128] = {'\0'};
	if(ini_get_key(ini_file, section, key, &temp[0]) == NULL) return NULL; 
	*valbuf = (unsigned long)atol(temp);
	return valbuf;
}

double *ini_get_key_double(INI_HANDLE ini_file, char *section, char *key, double* valbuf)
{
	char temp[128] = {'\0'};
	if(ini_get_key(ini_file, section, key, &temp[0]) == NULL) return NULL; 
	*valbuf = atof(temp);
	return valbuf;
}

int ini_remove_key(INI_HANDLE ini_file, char *section, char *key)
{
	if(ini_file == NULL)
    {
        set_errmsg("ini_remove_key ERROR: Parameter 1(ini_file) is NULL!");
        return -1;
    }
	if(check_string(section) == NULL)
	{
        set_errmsg("ini_remove_key ERROR: Parameter 2(section) format is error");
        return -1;
	}
	if(check_string(key) == NULL)
	{
        set_errmsg("ini_remove_key ERROR: Parameter 3(key) format is error!");
        return -1;
	}
	return key_delete(ini_file, section, key);
}

int ini_remove_section(INI_HANDLE ini_file, char *section)
{
	if(ini_file == NULL)
    {
        set_errmsg("ini_remove_section ERROR: Parameter 1(ini_file) is NULL!");
        return -1;
    }
	if(check_string(section) == NULL)
	{
        set_errmsg("ini_remove_section ERROR: Parameter 2(section) format is error!");
        return -1;
	}
	return section_delete(ini_file, section);
}
