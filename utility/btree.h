#ifndef _BALANCE_TREE_H
#define _BALANCE_TREE_H

	typedef void *BT_HANDLE;
	#define BTREE_FAIL NULL


#ifdef __cplusplus
extern "C" {
#endif

	BT_HANDLE btree_create(int (*comp)(void *data, void *node_data), void *(*data_init)(void), int (*data_copy)(void *src, void *dest), void (*data_clear)(void *data));
	//輸入:比較的function:comp與資料配置的function，請注意比較的fuction中的資料順序，
    //資料初始化的function，
    //複製的function，請注意第一個參數為來源，第二個為目標， 
    //清除(釋放)的function。 
	//輸出:BALANCE_TREE型態的void指標，不成功則回傳NULL，用btree_geterrmsg()取得錯誤訊息
	
	void btree_destroy(BT_HANDLE btree);
    //輸入:BALANCE_TREE型態的void指標位址 
	
	void* btree_search(BT_HANDLE btree, void *data);
	//輸入:BALANCE_TREE型態的void指標與BALANCE_TREE_NODE中data的指標
	//輸出:BALANCE_TREE_NODE中data的指標，沒找到則回傳NULL，用btree_geterrmsg()取得錯誤訊息
	
	int btree_insert(BT_HANDLE btree, void *data);
	//輸入:BALANCE_TREE型態的void指標與BALANCE_TREE_NODE中data的指標
	//輸出:0--完成插入的動作，1--用btree_geterrmsg()取得錯誤訊息
	
	int btree_delete(BT_HANDLE btree, void *data);
	//輸入:BALANCE_TREE型態的void指標與BALANCE_TREE_NODE中data的指標
	//輸出:回傳被刪除的資料指標，當回傳NULL時，用btree_geterrmsg()取得錯誤訊息

	int btree_iter_create(BT_HANDLE btree);
	//輸入:BALANCE_TREE型態的void指標
	//迭代器不會隨著insert或delete更新，create當下內容即為迭代器的內容，可以重新create來更新 
	
	void *btree_iter_get(BT_HANDLE btree);
	//從迭代器中取出資料 
	
	void btree_iter_end(BT_HANDLE btree);
    //結束迭代器，會釋放所有暫存在迭代器的資料 

	char *btree_geterrmsg(void);
	//取得錯誤訊息

#ifdef __cplusplus
}
#endif

#endif
