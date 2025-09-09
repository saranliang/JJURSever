#include "URBasicMotion.h"

class URBasicMotionPrivate : public QObject
{
public:
    URTcp *_tcpServer;
};

URBasicMotion::URBasicMotion(QObject *parent, URTcp *tcpServer) :
    QObject (parent), d_ptr(new URBasicMotionPrivate)
{
    Q_D(URBasicMotion);
    d->_tcpServer = tcpServer;
    connect(this, SIGNAL(signal_send_command(const char*)), d->_tcpServer, SLOT(slot_send_command(const char*)), Qt::BlockingQueuedConnection);
}

URBasicMotion::~URBasicMotion()
{
    Q_D(URBasicMotion);
}

void URBasicMotion::moveRobot_JointPosition(JointPos jPos)
{
    Q_D(URBasicMotion);
    char szCmd[0x100];
    sprintf(szCmd, "movej([%f, %f, %f, %f, %f, %f], a=1.2, v=0.05, t=0, r=0)\n", jPos.j1, jPos.j2, jPos.j3, jPos.j4, jPos.j5, jPos.j6);
    emit signal_send_command(szCmd);
}

void URBasicMotion::moveRobot_CartPosition(CartPos cPos)
{
    Q_D(URBasicMotion);
    char szCmd[0x100];
    sprintf(szCmd, "movel(p[%f, %f, %f, %f, %f, %f], a=1.2, v=0.02, t=0, r=0)\n", cPos.x / 1000, cPos.y / 1000, cPos.z / 1000, cPos.rx, cPos.ry, cPos.rz);
    emit signal_send_command(szCmd);
}

void URBasicMotion::moveRobot_left_init_pos(JointPos jPos)
{
    Q_D(URBasicMotion);
    char szCmd[0x100];
    sprintf(szCmd, "def moveRobot_left_pos():\n movej([%f, %f, %f, %f, %f, %f], a=1.2, v=0.05, t=0, r=0)\nend\n", jPos.j1, jPos.j2, jPos.j3, jPos.j4, jPos.j5, jPos.j6);
    emit signal_send_command(szCmd);
}

void URBasicMotion::moveRobot_right_init_pos(JointPos jPos)
{
    Q_D(URBasicMotion);
    char szCmd[0x100];
    sprintf(szCmd, "def moveRobot_right_pos():\n movej([0, 0, 0, 0, 0, 0], a=1.2, v=0.05, t=0, r=0)\n movej(p[%f, %f, %f, %f, %f, %f], a=1.2, v=0.05, t=0, r=0)\nend\n", jPos.j1, jPos.j2, jPos.j3, jPos.j4, jPos.j5, jPos.j6);
    emit signal_send_command(szCmd);
}
