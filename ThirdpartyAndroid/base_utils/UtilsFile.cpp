#include "UtilsFile.h"
// #include "UtilsString.h"
namespace LevenBF{
namespace Utils{

	int UtilsFile::CreateDir(string fullpath_str, char splitKey , int permit )
	{		
		
		if (access(fullpath_str.c_str(), 0) == 0)//已经存在
		{
			MYLOG_V(fullpath_str + string("已经存在，无需重新创建:") + fullpath_str);
			return 0;
		}
		
		// shared_ptr<vector<std::string>> fullpath = UtilsString::Split(fullpath_str, splitKey);
		vector<std::string> fullpath;
		string separator(1, splitKey); 
		UtilsString::Split2(fullpath_str, fullpath, separator);
		if (fullpath.size() <= 0)
		{
			return -1;
		}
	
		int ret;
		
		stringstream pathtmp;
		
		int size = fullpath.size();
		for (int i = 0; i < size; i++)
		{
			if("" == fullpath[i]){
				if(0 == i){
					pathtmp << "/";
				}
				continue;							
			}
			pathtmp << fullpath[i] << "/";
			
			if (access(pathtmp.str().c_str(), 0) != 0)//不存在，创建
			{
				MYLOG_V(pathtmp.str() + string("不存在"));
				if ((ret = mkdir(pathtmp.str().c_str(), 0777)) != 0)
				{
					MYLOG_V(pathtmp.str() + string("创建失败"));                  
					return ret;
				}
				else
				{
					MYLOG_V(pathtmp.str() + string("创建成功"));                 
				}								
			}
			
			if (access(pathtmp.str().c_str(), 06) != 0)
			{
				MYLOG_V(pathtmp.str() + string("没有读写权限"));             
				/*return -1;*/
			}
			
		}
		
		return 0;
	}
		
	
	int UtilsFile::DeleteDir(string fullpath_str)
	{
		if (access(fullpath_str.c_str(), 0) != 0)//不存在
		{
			MYLOG_V(fullpath_str + string("不存在，无需删除"));       
			return -1;
		}		
		/*int ret = rmdir(fullpath_str.c_str());//只能删除空文件夹*/
/*		int ret = remove(fullpath_str.c_str()); //只能删除空文件夹*/
		string cmd = string("rm -rf ") + fullpath_str;
		int ret = system(cmd.c_str());
		if (0 == ret)
		{
			MYLOG_V(fullpath_str + string("删除成功"));
		}
		else
		{
			stringstream ss;
			ss << fullpath_str << "删除出错，且返回值是：" << ret;
			MYLOG_W(ss.str());
		}		
		return ret;
		
	}
	
	void UtilsFile::TraverDir(string dirName, char splitKey )
	{
		const char * dir_name = dirName.c_str();  
		
		//检测输入参数是否是文件夹
		struct stat s;  
/*		
		lstat(dir_name, &s);  
		if (!S_ISDIR(s.st_mode))  
		{  
			cout << "dir_name is not a valid directory !" << endl;  
			return;  
		} */ 
		
		DIR * dir = opendir(dir_name);  
		if (NULL == dir)  
		{  
			cout << "打开目录失败：" << dir_name << endl;  
			return;  
		}      
		struct dirent * filename;  		
		stringstream ss_filefullpath;//为了复用，所以提到循环外面，如果此处报错，就重新放进循环即可
		while ((filename = readdir(dir)) != NULL)  
		{   
			ss_filefullpath.str("");
			ss_filefullpath << dirName << splitKey << filename->d_name;							
			string filefullpath = ss_filefullpath.str();
			
			//读取文件状态信息，判断是否是目录
			lstat(filefullpath.c_str(), &s); 
			if (S_ISDIR(s.st_mode))//是目录
			{				
				if (strcmp(filename->d_name, ".") == 0 ||  strcmp(filename->d_name, "..") == 0)//跳过
				{
					continue; 
				}
				
				//处理目录
				cout << filefullpath << "	<dir>" << endl;  				
				
				if (true)//需要往里面递归
				{									
					TraverDir(filefullpath);
				}
				
			}
			else//
			{
				cout << filefullpath << "	<file>" << endl;  
			}						
		}  		
		
	}
	
	void UtilsFile::TraverDir2(string dirName, char splitKey)
	{
		const char * dir_name = dirName.c_str();  
		
		//检测输入参数是否是文件夹
		struct stat s;  
		
		DIR * dir = opendir(dir_name);  
		if (NULL == dir)  
		{  
			cout << "打开目录失败：" << dir_name << endl;  
			return;  
		}      
		
		vector<string> dirs;
		vector<string> files;
		struct dirent * filename;  		
		stringstream ss_filefullpath; //为了复用，所以提到循环外面，如果此处报错，就重新放进循环即可
		while ((filename = readdir(dir)) != NULL)  
		{   
			ss_filefullpath.str("");
			ss_filefullpath << dirName << splitKey << filename->d_name;							
			string filefullpath = ss_filefullpath.str();
			
			//读取文件状态信息，判断是否是目录
			lstat(filefullpath.c_str(), &s); 
			if (S_ISDIR(s.st_mode))//是目录
			{				
				if (strcmp(filename->d_name, ".") == 0 ||  strcmp(filename->d_name, "..") == 0)//跳过
				{
					continue; 
				}				
				//处理目录			
				dirs.push_back(filefullpath);								
			}
			else//
			{
				files.push_back(filefullpath);
			}						
		}  		
		
		//先操作文件夹
		int size = dirs.size();
		if (size > 0)
		{
			sort(dirs.begin(), dirs.end());			
			for (int i = 0; i < size; i++)
			{
				cout << dirs[i] << "	<dir>" << endl;  	
				TraverDir2(dirs[i]);
			}			
		}
		
		//再操作文件
		size = files.size();
		if (size > 0)
		{
			sort(files.begin(), files.end());
			for (int i = 0; i < size; i++)
			{
				cout << files[i] << "	<file>" << endl;  
			}			
		}		
	}		
	
	
	bool UtilsFile::Write2File(string dirName, string fileNameSimple, stringstream& data, char splitKey, int permit)
	{
		if (access(dirName.c_str(), 0) != 0)//目录还不存在
		{
			if (CreateDir(dirName, splitKey, permit) != 0)
			{
				return false;
			}
		}
		
		string logfilename = dirName + string("/") + fileNameSimple;
		
		ofstream fout;
		fout.open(logfilename, ios_base::app);
		fout << data.str();
		fout << flush; 
		fout.close(); 			
		return true;
	}
	
	bool UtilsFile::Write2File(string dirName, string fileNameSimple, string& data, char splitKey , int permit)
	{
		if (access(dirName.c_str(), 0) != 0)//目录还不存在
		{
			if (CreateDir(dirName, splitKey, permit) != 0)
			{
				return false;
			}
		}
		
		string logfilename = dirName + string("/") + fileNameSimple;
		
		ofstream fout;
		fout.open(logfilename, ios_base::app);
		fout << data;
		fout << flush; 
		fout.close(); 			
		return true;
	}	

	bool UtilsFile::_exist_file_or_dir(const std::string &path){
		struct stat st;
		if(stat(path.c_str(), &st) != 0){//不存在
			return false;
		}
		return true;   
	}

	void UtilsFile::_mkdir_recursive(const std::string &path) {
		std::cout << "try _mkdir_recursive:" << path << std::endl;
		std::string sub_path = "";
		size_t prev = 0, pos;
		while ((pos = path.find_first_of('/', prev)) != std::string::npos) {
			// std::cout << "prev=" << prev << ",pos=" << pos << std::endl;
			if(0 == prev){//the first sub_path
			if(0 == pos){
				sub_path = "/";//path begins with /
			}else{
				sub_path = path.substr(prev, pos - prev);
			}
			}else{
			sub_path += "/" + path.substr(prev, pos - prev);
			}
			// std::cout << "try mkdir:" << sub_path << std::endl;
			if(!_exist_file_or_dir(sub_path)){
				if (mkdir(sub_path.c_str(), 0755) && errno != EEXIST)
				{
					std::cerr << "Failed to create directory 1, prev=" << prev << ",pos=" << pos << ",dir=" << sub_path << std::endl;
					return;
				}else{
					std::cout << "mkdir dir success: prev=" << prev << ",pos=" << pos << ",dir=" << sub_path << std::endl;
				}
			}
			prev = pos + 1;
		}
		if(!_exist_file_or_dir(path)){
			if (mkdir(path.c_str(), 0755) && errno != EEXIST)
			{
				std::cerr << "Failed to create directory 2" << path << std::endl;
			}else{
				std::cout << "mkdir dir success:" << path << std::endl;
			}
		}
	}

}
}