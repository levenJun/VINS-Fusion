#include <dirent.h>
#include <sys/sysinfo.h>
#include <sys/statfs.h>
#include <sys/vfs.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <vector>
#include <algorithm>
using namespace std;
namespace DATA_READER
{

void sortFileName(std::vector<std::string> &vFileName)
{

	sort(vFileName.begin(), vFileName.end(), [](std::string a, std::string b)
		 {
			 std::string::size_type nPos1 = std::string::npos;
			 nPos1 = a.find_last_of("/");
			 std::string::size_type nPos2 = std::string::npos;
			 nPos2 = a.find_last_of(".");
			 std::string aSub = a.substr(nPos1 + 1, nPos2-nPos1-1);
			 nPos1 = b.find_last_of("/");
			 nPos2 = b.find_last_of(".");
			 std::string bSub = b.substr(nPos1 + 1, nPos2-nPos1-1);

			 long long aNum, bNum;
			 aNum = strtoll(aSub.c_str(), NULL, 10);
			 bNum = strtoll(bSub.c_str(), NULL, 10);
			 return aNum < bNum; });
}
void checkLastCharForPath(std::string &dir)
{
	if (dir.size() == 0)
	{
		return;
	}

	if (dir[dir.size() - 1] != '/')
	{
		dir.push_back('/');
	}
	return;
}
int getFiles(std::string dir, std::vector<std::string> &vFileName)
{
	if (dir.empty())
	{
		return -1;
	}
	vFileName.clear();

	checkLastCharForPath(dir);

#ifdef _WINDOWS
	struct _finddata_t fileinfo; //_finddata_t是一个结构体，要用到#include <io.h>头文件；
	intptr_t ld;

	if ((ld = _findfirst((dir + "*").c_str(), &fileinfo)) != -1l)
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))
			{ //如果是文件夹；
			  // 				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0) {  //如果文件名不是.或者,,,则递归获取子文件中的文件；
			  //
			  // 					getfileall(dir + "\\" + fileinfo.name, vFileName);  //递归子文件夹；
			  // 				}
			}
			else //如果是文件；
			{
				vFileName.push_back(fileinfo.name);
			}

		} while (_findnext(ld, &fileinfo) == 0);
		_findclose(ld);
	}
#else
	DIR *dp;
	struct dirent *dirp;
	if ((dp = opendir(dir.c_str())) == NULL)
	{
		return -1;
	}

	while ((dirp = readdir(dp)) != NULL)
	{
		std::string name = std::string(dirp->d_name);
		if (name != "." && name != "..")
		{
			vFileName.push_back(name);
		}
	}

	closedir(dp);
#endif
	return vFileName.size();
}

int getFilesAllPath(std::string dir, std::vector<std::string> &vFileName)
{
	getFiles(dir, vFileName);
	checkLastCharForPath(dir);
	for (int i = 0; i < vFileName.size(); i++)
	{
		vFileName[i] = dir + vFileName[i];
	}
	return vFileName.size();
}

int getImageFilesAllPath(std::string dir, std::vector<std::string> &vFileName)
{
	vFileName.clear();
	std::vector<std::string> fileNames;
	getFiles(dir, fileNames);
	checkLastCharForPath(dir);
	for (int i = 0; i < fileNames.size(); i++)
	{
		if(fileNames[i].find("jpg")!=string::npos || fileNames[i].find("bmp")!=string::npos || fileNames[i].find("png")!=string::npos)
			vFileName.push_back(dir + fileNames[i]);
	}
	return vFileName.size();
}
}