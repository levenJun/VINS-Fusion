#pragma once

#include <functional>
#include <iostream>

using namespace std;

namespace MyHelpers{
class UtilsInstance
{
private:
    
    std::function<void()> f;

public:
    UtilsInstance(const std::function<void()>& f_):f(f_){};
    ~UtilsInstance(){ f(); };
};

};