#pragma once

#include <iostream>
#include <QObject>
#include <chrono>
#include <mutex>

class ConnectionObserver :public QObject
{
    Q_OBJECT
public:
    explicit ConnectionObserver(long millTimeOut = 3000);
    ~ConnectionObserver();

    void start();
    void update();
    void close();
    const bool get_connection_state();

signals:
    void signal_connection_state(bool);

private:
    long m_timeOut;
    bool m_runningFlag;
    bool m_isConnected;
    long m_updateTime;
    std::mutex m_mutex;


};

