#include "thread.h"

void *Thread::run1()
{
    threadStatus = THREAD_STATUS_RUNNING;
    tid = pthread_self();
    run();
    threadStatus = THREAD_STATUS_EXIT;
    tid = 0;
    pthread_exit(NULL);
}

Thread::Thread()
{
    tid = 0;
    tid_2 = 0;
    threadStatus = THREAD_STATUS_NEW;
}

bool Thread::start()
{
    int iRet = 0;
    pthread_create(&tid, NULL, thread_proxy_func, this) == 0;
}

bool Thread::start_2()
{
    int iRet = 0;
    pthread_create(&tid_2, NULL, thread_proxy_func_2, this) == 0;
}

pthread_t Thread::getThreadID()
{
    return tid;
}

int Thread::getState()
{
    return threadStatus;
}

void Thread::join()
{
    if (tid > 0)
    {
        pthread_join(tid, NULL);
    }
}

void Thread::join_2()
{
    if (tid_2 > 0)
    {
        pthread_join(tid_2, NULL);
    }
}

void *Thread::thread_proxy_func(void *args)
{
    Thread *pThread = static_cast<Thread *>(args);

    pThread->run();

    return NULL;
}

void *Thread::thread_proxy_func_2(void *args)
{
    Thread *pThread = static_cast<Thread *>(args);

    pThread->run_2();

    return NULL;
}

void Thread::join(unsigned long millisTime)
{
    if (tid == 0)
    {
        return;
    }
    if (millisTime == 0)
    {
        join();
    }
    else
    {
        unsigned long k = 0;
        while (threadStatus != THREAD_STATUS_EXIT && k <= millisTime)
        {
            usleep(100);
            k++;
        }
    }
}
