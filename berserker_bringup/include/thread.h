#ifndef THREAD_H
#define THREAD_H
#include <iostream>
#include <pthread.h>
#include <unistd.h> /*Unix 标准函数定义*/

using namespace std;

class Thread
{
  private:
    //当前线程的线程ID
    pthread_t tid;
    pthread_t tid_2;
    //线程的状态
    int threadStatus;
    //获取执行方法的指针
    static void *thread_proxy_func(void *args);
    static void *thread_proxy_func_2(void *args);
    //内部执行方法
    void *run1();

  public:
    //线程的状态－新建
    static const int THREAD_STATUS_NEW = 0;
    //线程的状态－正在运行
    static const int THREAD_STATUS_RUNNING = 1;
    //线程的状态－运行结束
    static const int THREAD_STATUS_EXIT = -1;
    //构造函数
    Thread();
    //线程的运行实体
    virtual void run() = 0;
    virtual void run_2() = 0;
    //开始执行线程
    bool start();
    bool start_2();
    //获取线程ID
    pthread_t getThreadID();
    //获取线程状态
    int getState();
    //等待线程直至退出
    void join();
    void join_2();
    //等待线程退出或者超时
    void join(unsigned long millisTime);
};

class MultiThread : public Thread
{
  public:
    void run()
    {
        int number = 0;
        for (int i = 0; i < 10; i++)
        {
            cout << "Current number is " << number++;
            cout << " PID is " << getpid() << " TID is " << getThreadID() << endl;
            sleep(1);
        }
    }
};

#endif