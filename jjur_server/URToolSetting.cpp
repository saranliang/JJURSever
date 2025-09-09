#include "URToolSetting.h"

class URToolSettingPrivate : public QObject
{
public:
    URTcp *_tcpServer;
};

URToolSetting::URToolSetting(QObject *parent, URTcp *tcpServer) :
    QObject (parent), d_ptr(new URToolSettingPrivate)
{
    Q_D(URToolSetting);
    d->_tcpServer = tcpServer;
    connect(this, SIGNAL(signal_send_command(const char*)), d->_tcpServer, SLOT(slot_send_command(const char*)), Qt::BlockingQueuedConnection);
}

URToolSetting::~URToolSetting()
{
    Q_D(URToolSetting);
}

void URToolSetting::set_payload(Payload load)
{
    Q_D(URToolSetting);
    char szCmd[0x100];
    //sprintf(szCmd, "movej([%f, %f, %f, %f, %f, %f], a=1.2, v=0.05, t=0, r=0)\n", jPos.j1, jPos.j2, jPos.j3, jPos.j4, jPos.j5, jPos.j6);
    sprintf(szCmd, "set_payload(%f, [%f, %f, %f])\n", load.m, load.x, load.y, load.z);
    emit signal_send_command(szCmd);
}

void URToolSetting::set_tcp_pos(CartPos cpos)
{
    Q_D(URToolSetting);
    char szCmd[0x100];
    sprintf(szCmd, "set_tcp(p[%f, %f, %f, %f, %f, %f])\n", cpos.x/1000, cpos.y/1000, cpos.z/1000, cpos.rx, cpos.ry, cpos.rz);
    emit signal_send_command(szCmd);
}
