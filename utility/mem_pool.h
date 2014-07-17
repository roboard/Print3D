#ifndef _MEM_POOL_H
#define _MEM_POOL_H

	typedef void *MP_HANDLE;
	#define MEM_FAIL NULL


#ifdef __cplusplus
extern "C" {
#endif

    MP_HANDLE mem_create(unsigned int datasize, unsigned int poolsize);
	//input: datasize and poolsize
	//output:MEM_POOL型態的void指標，如果回傳NULL代表create不成功　
	void mem_destroy(MP_HANDLE mpool);
	//input: MEM_POOL型態的void指標
	int mem_free(void *data);
	//input: 要刪除的data資料
	void *mem_alloc(MP_HANDLE mpool);
	//input: MEM_POOL型態的void指標
	//output:void型態的指標，指向一塊大小為datasize的可用空間，如果回傳null代表配置不成功

	char *mem_geterrmsg(void);
	//取出錯誤訊息


#ifdef __cplusplus
}
#endif

#endif
