#include "ConnectionObserver.h"

#include <iostream>
#include <thread>

ConnectionObserver::ConnectionObserver(long millTimeOut)
{
    m_timeOut = millTimeOut;
    m_isConnected = false;
    m_runningFlag = true;
    m_updateTime = 0;
}

ConnectionObserver::~ConnectionObserver()
{
    close();
}

void ConnectionObserver::start()
{
    while(m_runningFlag)
    {
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            std::chrono::system_clock::time_point currTime = std::chrono::system_clock::now();
            auto currTime_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(currTime);
            if((currTime_ms.time_since_epoch().count() - m_updateTime)< m_timeOut)
            {
                m_isConnected = true;
            }
            else
            {
                m_isConnected = false;
            }
            emit signal_connection_state(m_isConnected);
        }
        std::this_thread::sleep_for(std::chrono:: milliseconds (300));
    }
}

void ConnectionObserver::close()
{
    m_runningFlag = false;
}

void ConnectionObserver::update()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    std::chrono::system_clock::time_point currTime = std::chrono::system_clock::now();
    auto currTime_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(currTime);
    m_updateTime = currTime_ms.time_since_epoch().count();
}

const bool ConnectionObserver::get_connection_state()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    return m_isConnected;
}
