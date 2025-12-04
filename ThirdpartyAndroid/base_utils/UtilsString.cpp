#include "UtilsString.h"
#include <stdio.h>

namespace LevenBF{
namespace Utils{

    char* UtilsString::Str26Char(string str)
    {
        char* result = new char[str.length() + 1];	
        my_strcpy_s(result, str.c_str(), 0, str.length() + 1);	
    //	strcpy(result, str.c_str());
        //reslut末尾需要加上'/0'否？
        return result;
    };	

    bool UtilsString::Trim(string& s)
	{
        if(s.empty())
            return true;

        s.erase(0, s.find_first_not_of(" "));
        s.erase(s.find_last_not_of(" ")+1);
        return true;
    }	

    shared_ptr<vector<std::string>> UtilsString::Split(std::string source, char key)
	{
		
		if (source.empty())
		{
			return nullptr;
		}
		shared_ptr<vector<std::string>> result = make_shared<vector<std::string>>();
		const char* source_char = source.c_str();
		int size = source.size();
		
		
		int startIndex = 0;
		int endIndex = 0;
		int length_piece = 0;
		
		for (int i = 0; i < size; i++)
		{
			if (source_char[i] == key)
			{
				endIndex = i;
				length_piece =  endIndex - startIndex;
				if (length_piece > 0)//正式裁剪
				{
					char* piece = new char[length_piece + 1];
					my_strcpy_s(piece, source_char, startIndex, length_piece);
					piece[length_piece] = '\0';
					result->push_back(std::string(piece));
					delete[] piece;
				}				
				startIndex = endIndex + 1;				
			}
		}
		
		//还有最后一片
		length_piece =  size - startIndex;
		if (length_piece > 0)//正式裁剪
		{
			char* piece = new char[length_piece + 1];
			my_strcpy_s(piece, source_char, startIndex, length_piece);
			piece[length_piece] = '\0';
			result->push_back(std::string(piece));
			delete[] piece;
		}
		
		return result;
	}	

	void UtilsString::Split2(const string& srcStr, vector<string>& vec, const string& separator)
	{

		string::size_type posSubstringStart; // 子串开始位置
		string::size_type posSeparator;        //  分隔符位置

		posSeparator = srcStr.find(separator);
		posSubstringStart = 0;
		while (string::npos != posSeparator)
		{
			vec.push_back(srcStr.substr(posSubstringStart, posSeparator - posSubstringStart));

			posSubstringStart = posSeparator + separator.size();
			posSeparator = srcStr.find(separator, posSubstringStart);
		}

		// 截取最后一段数据
		if (posSubstringStart != srcStr.length()){
			vec.push_back(srcStr.substr(posSubstringStart));
		}else{
			vec.push_back("");
		}
	}

	bool UtilsString::LengthAlign(string& srcStr, size_t length, char placeholder, bool cutExtra)
	{
		if (length <= 0)
		{
			cout << "补位length不合法，length = " << length << endl;
			return false;
		}
		if (srcStr.length() == length)
		{
			return true;
		}
		
		if (srcStr.length() > length)
		{
			if (cutExtra == false)
			{
				return false;
			}

			srcStr.erase(0, srcStr.length() - length);
			return true;
		}

		//正式执行补位操作
		int fixSize = length - srcStr.length();
		srcStr.insert(0, fixSize, placeholder);
		
		return true;
	}

}
}