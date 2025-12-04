/*** 
 * @Author: leven
 * @LastEditors: leven
 * @Description: UtilsInstance 对象销毁自动处理工具
 */
#pragma once

#include <functional>

using namespace std;

namespace LevenBF{
namespace Utils{

class UtilsInstance
{
private:
    
    std::function<void()> f;

public:
    UtilsInstance(const std::function<void()>& f_):f(f_){};
    ~UtilsInstance(){ f(); };
};
};
};