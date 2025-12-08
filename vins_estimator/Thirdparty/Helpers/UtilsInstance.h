#pragma once

#include <functional>
#include <iostream>

using namespace std;

namespace ORB_SLAM3{
class UtilsInstance
{
private:
    
    std::function<void()> f;

public:
    UtilsInstance(const std::function<void()>& f_):f(f_){};
    ~UtilsInstance(){ f(); };
};

};