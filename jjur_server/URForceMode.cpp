#include "URForceMode.h"

class URForceModePrivate : public QObject
{
public:
    URTcp *_tcpServer;
};

URForceMode::URForceMode(QObject *parent, URTcp *tcpServer) :
    QObject (parent), d_ptr(new URForceModePrivate)
{
    Q_D(URForceMode);
    d->_tcpServer = tcpServer;
}

URForceMode::~URForceMode()
{
    Q_D(URForceMode);
}

void URForceMode::slot_start_force_mode()
{
    Q_D(URForceMode);   
    char szCmd[0x100];
    //sprintf(szCmd, "force_mode(p[0.1,0,0,0,0.785],[1,0,0,0,0,0],[0,0,0,0,0,0],2,[0.2,0.1,0.1,0.785,0.785,1.57])\n");
    //sprintf(szCmd, "force_mode(p[0.1,0,0,0,0.785],[1,0,0,0,0,0],[0,0,0,0,0,0],2,[0.2,0.1,0.1,0.785,0.785,1.57])\n");
    //sprintf(szCmd, "force_mode(p[0.00083, 0.02306, 0.21195, 2.1633, 2.2591, 0], [1, 0, 0, 0, 0, 0], [20, 0, 40, 0, 0, 0], 2, [0.2, 0.1, 0.1, 0.785, 0.785, 1.57])\n");
    //sprintf(szCmd, "def enter_force_mode():\n force_mode(p[0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.1, 0.35, 0.35, 0.60])\n sleep(10000)\nend\n");
    sprintf(szCmd, "def enter_force_mode():\n force_mode(p[0.00083, 0.02306, 0.21195, 2.1633, 2.2591, 0], [1, 0, 0, 0, 0, 0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.1, 0.3, 0.3, 0.3])\n sleep(10000)\nend\n");
    d->_tcpServer->send_command(szCmd);
}

void URForceMode::end_force_mode()
{
    Q_D(URForceMode);
    char szCmd[0x100];
    sprintf(szCmd, "end_force_mode()\n");
    d->_tcpServer->send_command(szCmd);
}
