#pragma once

#include "UtilsString.h"
#include "UtilsLog.h"
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>  
//#include <io.h>
#include <stdarg.h>
#include <unistd.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>  

#include <vector>
#include <algorithm>

using namespace std;

namespace LevenBF{
namespace Utils{

class UtilsFile
{
public:
    UtilsFile(/* args */){};
    ~UtilsFile(){};

public:
	
	/*** 
	 * @description: 创建文件夹，如果待创建的文件夹父级目录不存在，会首先创建父级目录
	 * 					（特别注意，可能会因为权限问题导致创建失败，如是，请手动修改对应文件夹权限）
	 * @param {string} fullpath_str	待创建文件夹完整路径
	 * @param {char} splitKey		文件夹路径的分割字符
	 * @param {int} permit			文件夹创建后的权限，最好就设置为这个权限
	 * @return {*}	0，成功；其他，失败
	 */	
	static int CreateDir(string fullpath_str, char splitKey = '/', int permit = 0777);		

	/*** 
	 * @description: 利用system命令方式创建文件夹
	 * @param {string} fullpath_str	待创建文件夹完整路径
	 * @return {*}
	 */	
	static inline int CreateDirSimple(string fullpath_str){
        int ret = system((std::string("mkdir -p ") + fullpath_str).c_str());
		return ret;
	};

	/*** 
	 * @description: （system命令方式）删除文件夹，指的是文件夹下所有内容都删除，注意权限问题
	 * @param {string} fullpath_str	待删除文件夹完整路径
	 * @return {*}	0，成功；其他，失败	
	 */	
	static int DeleteDir(string fullpath_str);
	
	//遍历文件夹
	static void TraverDir(string dirName, char splitKey = '/');
	
	//遍历文件夹
	//遍历一轮时，先将当前目录下所有子文件排序，再执行递归遍历等操作
	static void TraverDir2(string dirName, char splitKey = '/');					
	
	//将data写入到指定文件中
	static bool Write2File(string dirName, string fileNameSimple, stringstream& data, char splitKey = '/', int permit = 0777);


	//将data写入到指定文件中
	static bool Write2File(string dirName, string fileNameSimple, string& data, char splitKey = '/', int permit = 0777);
	
	static bool _exist_file_or_dir(const std::string &path);

	static void _mkdir_recursive(const std::string &path);
};


}
}