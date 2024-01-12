
#pragma once
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <atomic>
#include <memory>
#include <iostream>

namespace LevenBF{
namespace Utils{
class UtilsKeybordManager : public enable_shared_from_this<UtilsKeybordManager>
{
protected:

const char mcDefaultKey = 'u';
std::atomic<char> mCurKey;

std::atomic<bool> mThreadOn;
std::shared_ptr<std::thread> mKeybordManagerThread = nullptr;

public:
    UtilsKeybordManager():mCurKey(' '), mThreadOn(false){};
    virtual ~UtilsKeybordManager();

public:

    bool start();
    bool stop();
    bool isKeySpace(bool resetKey_ = true);
    void resetKey();
    static void keybordReaderFun(std::weak_ptr<UtilsKeybordManager> keybordManager);

    static char getche(void);
    static char getch(void);

};

}
}

namespace LevenBF{
namespace Utils{

UtilsKeybordManager::~UtilsKeybordManager()
{
    stop();
    if (mKeybordManagerThread)
    {
        if (mKeybordManagerThread->joinable())
        {
            mKeybordManagerThread->join();
        }
        mKeybordManagerThread = nullptr;
    }
};

bool UtilsKeybordManager::start(){
    if(mKeybordManagerThread){  //只能启动并允许使用一次!
        std::cout << ("Warn, start too many times is not allowed") << std::endl;
        return false;
    }
    mThreadOn = true;
    mKeybordManagerThread=std::make_shared<std::thread>(UtilsKeybordManager::keybordReaderFun, shared_from_this());
    return true;
}

bool UtilsKeybordManager::stop(){
    mThreadOn = false;
}

bool UtilsKeybordManager::isKeySpace(bool resetKey_){
    bool ret = ' ' == mCurKey;
    if(resetKey_ && ret){
        resetKey();
    }
    return ret;
}

void UtilsKeybordManager::resetKey(){
    mCurKey = mcDefaultKey;
};

void UtilsKeybordManager::keybordReaderFun(std::weak_ptr<UtilsKeybordManager> keybordManager){

    while(keybordManager.lock() && keybordManager.lock()->mThreadOn){
        // keybordManager.lock()->mCurKey = getchar();
        // keybordManager.lock()->mCurKey = cin.get();
        keybordManager.lock()->mCurKey = getche();
        std::cout << ("read key from bord, key=," + std::string(1, keybordManager.lock()->mCurKey) + ".")  << std::endl;
        usleep(1000 * 5);
    }

};

char UtilsKeybordManager::getche(void)
{
    struct termios oldattr, newattr;
    char ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;    
};

char UtilsKeybordManager::getch(void){
    struct termios oldattr, newattr;
    char ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}
}
}