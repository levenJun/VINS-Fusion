#pragma once

// #include <leven_bf_core/MultiPlatform.h>
#include <string>
#include <sstream>
#include <memory>
#include <vector>
#include <iostream>

using namespace std;

namespace LevenBF{
namespace Utils{

#ifndef STRING_PATCH
#define STRING_PATCH
inline string operator +(const string& left, const int& right) {
	ostringstream tmp;
	tmp << left << right;
	return tmp.str();
}
inline string operator +(const int& left, const string& right) {
	ostringstream tmp;
	tmp << left << right;
	return tmp.str();
}
#endif //STRING_PATCH

inline void my_strcpy_s(char* des, char* src, size_t start, size_t length)
{
	if(des == NULL || src == NULL || start < 0 )
	{
		return;
	}
	for (size_t i = 0; i < length; ++i)
	{
		des[i] = *(src + start + i);
	}
}


inline void my_strcpy_s(char* des, const char* src, size_t start, size_t length)
{
	if (des == NULL || src == NULL || start < 0)
	{
		return;
	}
	for (size_t i = 0; i < length; ++i)
	{
		des[i] = *(src + start + i);
	}
}

class UtilsString
{
public:

    /*** 
     * @description: 从string复制得到一份非const char*串
     *               这个地方是new出来的char数组，使用后必须记得delete掉
     * @param {string} str
     * @return {*}
     */    
	static char* Str26Char(string str);

    /*** 
     * @description: 清除字符串前后的空格
     * @param {string&} s
     * @return {*}
     */    
    static bool Trim(string& s);	


    /*** 
     * @description: 将基本类型数据data转化成char*，且char*末尾加上'\0'
     *                 注意，返回的指针需要手动删除？
     * @return {*}
     */    
	template<class BASE_TYPE>
	inline static char* BaseType2Char(BASE_TYPE data)
	{
		int size = sizeof(data);
		char* result = new char[size + 1];
		char* data_p = (char*)&data;
		for(int i = 0; i < size; i++)
		{
			result[i] = *(data_p + i);
		}
		result[size] = '\0';
		return result;
	}

	
    /*** 
     * @description: 
     * @param {string} source 待分割的源字符串
     * @param {char} key   按key进行分割
     * @return {*}
     */    
	static shared_ptr<vector<std::string>> Split(std::string source, char key);	

	static void Split2(const string& srcStr, vector<string>& vec, const string& separator);

	/*** 
	 * @description: 字符串位数对齐
	 * @param {string&} srcStr		被处理的字符串
	 * @param {int} length			要求的对齐位数
	 * @param {char} placeholder	字符串位数不够的补位字符
	 * @param {bool} cutExtra		为true时，如果原始字符串长度超过对齐位数，即删除左边多余的字符
	 * @return {*}
	 */	
	static bool LengthAlign(string& srcStr, size_t length, char placeholder = '0', bool cutExtra = false);

	/*** 
	 * @description: 将other拼接到src上
	 * @param {string&} src		源字符串，输入And输出
	 * @param {string&} other	被拼接进来的字符串
	 * @return {*}
	 */	
	static inline void StringCat(string& src, string& other)
	{
		src.append(other);
	};

};

}
}