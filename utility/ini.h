/*section, key, value的大小必須小於128 bit(包含結尾符號'\0')且前後不能是space，
 *但允許字中間有空格(空格以外的space是不被允許的)。
 *
 */
#ifndef _INI_H
#define _INI_H
    
	typedef void *INI_HANDLE;
	#define INI_FAIL NULL

#ifdef __cplusplus
extern "C" {
#endif

	INI_HANDLE ini_open(char *filename);
	//開啟一個ini的檔案，輸入檔案名稱，輸出為ini的handle。
	//如果回傳為NULL時，代表開啟不成功。

	int ini_close(INI_HANDLE ini_file);
	//寫入(更新)並關閉已開啟的ini，輸入ini的handle。
	//成功時，回傳0，若不成功則回傳1並保留原始檔案。

	int ini_close2(INI_HANDLE ini_file);
	//關閉已開啟的ini(不進行寫入的動作)，輸入ini的handle。
	//成功時，回傳0，若不成功則回傳1。

	int ini_commit(INI_HANDLE ini_file);
	//寫入(更新)已開啟的ini(不進行關閉的動作)，輸入ini的handle。
	//成功時，回傳0，若不成功則回傳1並保留原始檔案。

	int ini_add_section(INI_HANDLE ini_file, char *section);
	//加入一個section，輸入ini的handle與section名稱。
	//成功時，回傳0，若不成功則回傳1。

	int ini_add_key(INI_HANDLE ini_file, char *section, char *key, char *val);
	//加入一個key，如果沒有找到section則會先加入新的section再將新的key加入，
	//如果key的名稱相同，則會更新val的值。
	//輸入ini的handle與section，key，val的資料。
	//成功時，回傳0，若不成功則回傳1。

	char *ini_get_key(INI_HANDLE ini_file, char *section, char *key, char* valbuf);
	//取得一個key的val，輸入ini的handle、section、key與存放找到的val的buffer的資料。
	//valbuf須足夠大，若回傳NULL代表沒有找到。

	int *ini_get_key_int(INI_HANDLE ini_file, char *section, char *key, int* valbuf);
	unsigned long *ini_get_key_ulong(INI_HANDLE ini_file, char *section, char *key, unsigned long* valbuf);
	double *ini_get_key_double(INI_HANDLE ini_file, char *section, char *key, double* valbuf);
	//取得一個key的val(型態為int, long, float)，輸入ini的handle、section、key與存放找到的val的buffer的資料。
	//valbuf須足夠大，若回傳NULL代表沒有找到。	
	
	int ini_remove_section(INI_HANDLE ini_file, char *section);
	//移除整個section，輸入ini的handle與section的資料。
	//成功時，回傳0，若不成功則回傳1。

	int ini_remove_key(INI_HANDLE ini_file, char *section, char *key);
	//移除一個key，輸入ini的handle與section，key的資料。
	//成功時，回傳0，若不成功則回傳1。

	char *ini_geterrmsg(void);
	//取出錯誤訊息
	
#ifdef __cplusplus
}
#endif

#endif
