#include "URStatus.h"
#include <QTimer>

Q_DECLARE_METATYPE(JointPos);
Q_DECLARE_METATYPE(CartPos);
Q_DECLARE_METATYPE(Wrench);
Q_DECLARE_METATYPE(Speed);

class URStatusPrivate : public QObject
{
public:
    URTcp *_tcpServer;
    JointPos jpos;
    CartPos cpos;
    Wrench wrench;
    Speed speed;
    QTimer *timer;
    double input_value;
    int digital_value = 769;
    double safety_status;
};

URStatus::URStatus(QObject *parent, URTcp *tcpServer):
    QObject (parent), d_ptr(new URStatusPrivate)
{
    Q_D(URStatus);
    d->_tcpServer = tcpServer;
    d->timer = new QTimer;
    qRegisterMetaType<JointPos>("JointPos");
    qRegisterMetaType<CartPos>("CartPos");
    qRegisterMetaType<Wrench>("Wrench");
    qRegisterMetaType<Speed>("Speed");
    //hand_over();
    //take_over();
}

URStatus::~URStatus()
{
    Q_D(URStatus);
    hand_over();
}

void URStatus::take_over()
{
    Q_D(URStatus);
    //connect(d->timer, SIGNAL(timeout()), this, SLOT(slot_timer_start()));
    //d->timer->start(10);
    connect(d->_tcpServer, SIGNAL(signal_joint_pos(JointPos)), this, SLOT(slot_get_joint_pos(JointPos)));
    connect(d->_tcpServer, SIGNAL(signal_cart_pos(CartPos)), this, SLOT(slot_get_cart_pos(CartPos)));
    connect(d->_tcpServer, SIGNAL(signal_tcp_wrench(Wrench)), this, SLOT(slot_get_tcp_wrench(Wrench)));
    connect(d->_tcpServer, SIGNAL(signal_tcp_speed(Speed)), this, SLOT(slot_get_tcp_speed(Speed)));
    connect(d->_tcpServer, SIGNAL(signal_digital_input(double)), this, SLOT(slot_get_digital_input(double)));
    connect(d->_tcpServer, SIGNAL(signal_safety_status(double)), this, SLOT(slot_get_safety_status(double)));
    connect(d->_tcpServer, SIGNAL(signal_send_connect_status(int)), this, SLOT(slot_get_connect_status(int)));
}

void URStatus::hand_over()
{
    Q_D(URStatus);
    disconnect(d->_tcpServer, SIGNAL(signal_joint_pos(JointPos)), this, SLOT(slot_get_joint_pos(JointPos)));
    disconnect(d->_tcpServer, SIGNAL(signal_cart_pos(CartPos)), this, SLOT(slot_get_cart_pos(CartPos)));
    disconnect(d->_tcpServer, SIGNAL(signal_tcp_wrench(Wrench)), this, SLOT(slot_get_tcp_wrench(Wrench)));
    disconnect(d->_tcpServer, SIGNAL(signal_tcp_speed(Speed)), this, SLOT(slot_get_tcp_speed(Speed)));
    disconnect(d->_tcpServer, SIGNAL(signal_digital_input(double)), this, SLOT(slot_get_digital_input(double)));
    disconnect(d->_tcpServer, SIGNAL(signal_safety_status(double)), this, SLOT(slot_get_safety_status(double)));
    disconnect(d->_tcpServer, SIGNAL(signal_send_connect_status(int)), this, SLOT(slot_get_connect_status(int)));
}

double URStatus::ntohd(double d)
{
    double ret;
    unsigned long i;
    unsigned char *p = (unsigned char  *)&d;
    unsigned char  *r = (unsigned char  *)&ret;
    for (i = 0; i < sizeof(double); i++)
    {
        r[i] = p[sizeof(double) - 1 - i];
    }
    return ret;
}

void URStatus::slot_timer_start()
{
    Q_D(URStatus);
    d->_tcpServer->get_current_robot_status(jpos, cpos, twrench);
//    slot_get_joint_pos(jpos);
//    slot_get_cart_pos(cpos);
//    slot_get_tcp_wrench(twrench);
}

void URStatus::slot_get_joint_pos(JointPos jpos)
{
    Q_D(URStatus);
    d->jpos.j1 = ntohd(jpos.j1);
    d->jpos.j2 = ntohd(jpos.j2);
    d->jpos.j3 = ntohd(jpos.j3);
    d->jpos.j4 = ntohd(jpos.j4);
    d->jpos.j5 = ntohd(jpos.j5);
    d->jpos.j6 = ntohd(jpos.j6);
//    if (d->jpos.j6 > 0) {
//        while (d->jpos.j6 - 2 * M_PI > 0) {
//            d->jpos.j6 = d->jpos.j6 - 2 * M_PI;
//        }
//    } else {
//        while (d->jpos.j6 + 2 * M_PI < 0) {
//            d->jpos.j6 = d->jpos.j6 + 2 * M_PI;
//        }
//    }
    d->jpos.j1 = d->jpos.j1 * RAD2DEG;
    d->jpos.j2 = d->jpos.j2 * RAD2DEG;
    d->jpos.j3 = d->jpos.j3 * RAD2DEG;
    d->jpos.j4 = d->jpos.j4 * RAD2DEG;
    d->jpos.j5 = d->jpos.j5 * RAD2DEG;
    d->jpos.j6 = d->jpos.j6 * RAD2DEG;
    emit signal_send_joint_pos(d->jpos);
}

void URStatus::slot_get_cart_pos(CartPos cpos)
{
    Q_D(URStatus);
    d->cpos.x = ntohd(cpos.x);
    d->cpos.y = ntohd(cpos.y);
    d->cpos.z = ntohd(cpos.z);
    d->cpos.rx = ntohd(cpos.rx);
    d->cpos.ry = ntohd(cpos.ry);
    d->cpos.rz = ntohd(cpos.rz);
    if (d->cpos.rx < 0)
    {
        double theta = sqrt(d->cpos.rx * d->cpos.rx + d->cpos.ry * d->cpos.ry + d->cpos.rz * d->cpos.rz);
        double new_theta = 2 * M_PI - theta;
        d->cpos.rx = -d->cpos.rx / theta * new_theta;
        d->cpos.ry = -d->cpos.ry / theta * new_theta;
        d->cpos.rz = -d->cpos.rz / theta * new_theta;
    }
    d->cpos.x = d->cpos.x * 1000;
    d->cpos.y = d->cpos.y * 1000;
    d->cpos.z = d->cpos.z * 1000;
    d->cpos.rx = d->cpos.rx * RAD2DEG;
    d->cpos.ry = d->cpos.ry * RAD2DEG;
    d->cpos.rz = d->cpos.rz * RAD2DEG;
    emit signal_send_cart_pos(d->cpos);
}

void URStatus::slot_get_tcp_wrench(Wrench wrench)
{
    Q_D(URStatus);
    d->wrench.x = ntohd(wrench.x);
    d->wrench.y = ntohd(wrench.y);
    d->wrench.z = ntohd(wrench.z);
    d->wrench.rx = ntohd(wrench.rx);
    d->wrench.ry = ntohd(wrench.ry);
    d->wrench.rz = ntohd(wrench.rz);
    emit signal_send_tcp_wrench(d->wrench);
}

void URStatus::slot_get_tcp_speed(Speed speed)
{
    Q_D(URStatus);
    d->speed.x = ntohd(speed.x);
    d->speed.y = ntohd(speed.y);
    d->speed.z = ntohd(speed.z);
    d->speed.rx = ntohd(speed.rx);
    d->speed.ry = ntohd(speed.ry);
    d->speed.rz = ntohd(speed.rz);
    emit signal_send_tcp_speed(d->speed);
}

void URStatus::slot_get_digital_input(double value)
{
    Q_D(URStatus);
    d->input_value = ntohd(value);
    int m_value;
    m_value = static_cast<int>(d->input_value);
    if (d->digital_value != m_value) {
        d->digital_value = m_value;
        if (d->digital_value>>16 == 0) {
            emit signal_send_input_status(0);
        } else if (d->digital_value>>16 == 1) {
            emit signal_send_input_status(1);
        } else if (d->digital_value>>16 == 2) {
            emit signal_send_input_status(2);
        } else {
            emit signal_send_input_status(0);
        }
    }
    //emit signal_send_digital_input(d->input_value);
    //qDebug() << "URStatus =" << d->input_value;
}

void URStatus::slot_get_safety_status(double value)
{
    Q_D(URStatus);
    d->safety_status = ntohd(value);
    emit signal_send_safety_status(d->safety_status);
}

void URStatus::get_current_cart_pos(CartPos &cpos)
{
    Q_D(URStatus);
    cpos.x = d->cpos.x;
    cpos.y = d->cpos.y;
    cpos.z = d->cpos.z;
    cpos.rx = d->cpos.rx;
    cpos.ry = d->cpos.ry;
    cpos.rz = d->cpos.rz;
}

void URStatus::slot_get_connect_status(int flag)
{
    Q_D(URStatus);
    emit signal_connect_status(flag);
}
