#include "URTcp.h"
#include "ConnectionObserver.h"
#include <thread>

using namespace std;
Q_DECLARE_METATYPE(JointPos);
Q_DECLARE_METATYPE(CartPos);
Q_DECLARE_METATYPE(Wrench);
Q_DECLARE_METATYPE(Speed);

class URTcpPrivate : public QObject {
  public:
    QTcpSocket *tcpSocket;
    QTcpServer *tcpServer;
    QTcpSocket *robotSocket;
    QTcpSocket *analogSocket;
    ConnectionObserver *m_pConnectionObserver;
    std::thread *m_pTimeOutTh;
    QString ip = "172.31.1.147";
    // QString ip = "192.168.127.200";
    quint16 tcp_port = 30003;
    JointPos jpos;
    CartPos cpos;
    Wrench wrench;
    double digital;
    QString rbmsg = "False";
};

URTcp::URTcp(QObject *parent) : QObject(parent), d_ptr(new URTcpPrivate) {
    Q_D(URTcp);
    d->tcpServer = new QTcpServer(this);
    d->tcpSocket = new QTcpSocket(this);
    d->analogSocket = new QTcpSocket(this);
    d->robotSocket = new QTcpSocket(this);
    qRegisterMetaType<JointPos>("JointPos");
    qRegisterMetaType<CartPos>("CartPos");
    qRegisterMetaType<Wrench>("Wrench");
    qRegisterMetaType<Speed>("Speed");
    d->tcpServer->listen(QHostAddress("172.31.1.100"), 5000); // 监听指定的主机地址和端口号
    disconnect(d->tcpServer, SIGNAL(newConnection()), this, SLOT(slot_server_newConnection()));
    connect(d->tcpServer, SIGNAL(newConnection()), this, SLOT(slot_server_newConnection()));
    d->tcpSocket->setReadBufferSize(64 * 1024 * 1024);
    d->tcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    d->analogSocket->setReadBufferSize(1220);
    d->analogSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    // d->tcpSocket->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, 64 * 1024 * 1024);
    // int increasedSize = d->tcpSocket->socketOption(QAbstractSocket::ReceiveBufferSizeSocketOption).toInt();
    // qDebug() << "increasedSize: " << increasedSize;
    // d->robotSocket = d->tcpServer->nextPendingConnection();
    // connect(d->robotSocket, SIGNAL(readyRead()),this, SLOT(slot_receive_robot()));

    d->m_pConnectionObserver = new ConnectionObserver(3000); // 3000ms的时间间隔检测更新时间，超时视为断开连接
    connect(d->m_pConnectionObserver, &ConnectionObserver::signal_connection_state, this, &URTcp::slot_connection_state, Qt::DirectConnection);
    d->m_pTimeOutTh = new std::thread(&ConnectionObserver::start, d->m_pConnectionObserver); // 用子线程启动某个对象的成员函数thread(&func,ptr,para)
}

URTcp::~URTcp() {
    Q_D(URTcp);
    d->tcpSocket->flush();
    d->tcpSocket->close();
    d->analogSocket->flush();
    d->analogSocket->close();
    if (d->m_pConnectionObserver) {
        d->m_pConnectionObserver->close();
        delete d->m_pConnectionObserver;
    }
}

void URTcp::slot_newConnection(quint16 port) {
    Q_D(URTcp);
    // d->tcpSocket->disconnectFromHost();
    // QThread::msleep(100);
    qDebug() << "开始连接......";
    d->tcpSocket->connectToHost(d->ip, port, QTcpSocket::ReadWrite);
    if (!d->tcpSocket->waitForConnected(3000)) {
        qDebug() << "连接机械臂失败！";
        emit signal_send_connect_status(0);
    } else {
        qDebug() << "连接机械臂成功.";
        // emit signal_send_connect_status(1);
        if (port == 30003) {
            disconnect(d->tcpSocket, SIGNAL(readyRead()), this, SLOT(slot_receive()));
            connect(d->tcpSocket, SIGNAL(readyRead()), this, SLOT(slot_receive()), Qt::QueuedConnection);
        }
        d->tcpSocket->waitForReadyRead(3000);
        emit signal_send_connect_status(1);
    }
    d->tcpSocket->flush();
}

void URTcp::slot_connect_30002_port() {
    Q_D(URTcp);
    d->analogSocket->disconnectFromHost();
    d->analogSocket->connectToHost(d->ip, 30002, QTcpSocket::ReadWrite);
    if (!d->analogSocket->waitForConnected(3000)) {
        qDebug() << "30004连接机械臂失败!";
    } else {
        qDebug() << "30004连接机械臂成功.";
        d->analogSocket->waitForReadyRead(3000);
    }
    d->analogSocket->flush();
}

void URTcp::slot_disconnect() {
    Q_D(URTcp);
    d->tcpSocket->disconnectFromHost();
}

void URTcp::IOConnect(QString ip, quint16 port) {
    Q_D(URTcp);
    qDebug() << "IO Connect!";
    d->tcpSocket->connectToHost(ip, port, QTcpSocket::ReadWrite);
    if (!d->tcpSocket->waitForConnected(3000)) {
        qDebug() << "连接机械臂失败！！！";
    } else {
        qDebug() << "连接机械臂成功...";
        // connect(d->tcpSocket, SIGNAL(readyRead()), this, SLOT(slot_receive()));
        // d->tcpSocket->waitForReadyRead(3000);
    }
}

void URTcp::slot_server_newConnection() {
    Q_D(URTcp);
    // qDebug() << "new server_Connection()!!!";
    disconnect(d->robotSocket, SIGNAL(readyRead()), this, SLOT(slot_receive_robot()));
    //    d->robotSocket->~QTcpSocket();
    //    d->robotSocket = new QTcpSocket();
    d->robotSocket->flush();
    d->robotSocket = d->tcpServer->nextPendingConnection();
    d->robotSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    connect(d->robotSocket, SIGNAL(readyRead()), this, SLOT(slot_receive_robot()));
}

void URTcp::slot_receive() {
    Q_D(URTcp);
    int length = 1220; // UR机械臂发过来整个数据的大小
    char *data;
    QByteArray array = d->tcpSocket->readAll();
    QByteArray valid_array = array.left(1220);
    data = valid_array.data();
    JointPos jpos1;
    CartPos cpos1;
    Wrench wrench1;
    Speed speed;
    double digital1;
    double safestatus;
    double programestate;
    CartPos target_pos;
    memcpy(&jpos1, data + 252, sizeof(JOINTPOS));               // 关节角度的数据在第252位 32-37 ------ realtime的文档里面查阅
    memcpy(&cpos1, data + 444, sizeof(CARTPOS));                // 笛卡尔位置的数据在第444位 56-61
    memcpy(&wrench1, data + 540, sizeof(WRENCH));               // 机械臂TCP的wrench数据在第540位 68-73
    memcpy(&speed, data + 492, sizeof(SPEED));                  // 机械臂TCP的speed数据在第492位 62-67
    memcpy(&target_pos, data + 588, sizeof(CARTPOS));           // 机械臂TCP的目标位置数据在第596位 74-79
    memcpy(&digital1, data + 684, sizeof(digital1));            // io输入状态的数据在第684位------（86-1）* 8 + 4
    memcpy(&safestatus, data + 1108, sizeof(safestatus));       // 安全状态的数据在第1108位
    memcpy(&programestate, data + 1052, sizeof(programestate)); // 机械臂模式数据在756位------(95-1) * 8 + 4
    emit signal_joint_pos(jpos1);
    emit signal_cart_pos(cpos1);
    emit signal_tcp_wrench(wrench1);
    emit signal_tcp_speed(speed);
    emit signal_digital_input(digital1);
    emit signal_safety_status(safestatus);
    emit signal_programe_state(programestate);
    emit signal_target_pos(target_pos);
    // qDebug() << "m_pConnectionObserver update " << jpos1.j1 << " " << jpos1.j2;
    d->m_pConnectionObserver->update();
}

void URTcp::slot_receive_robot() {
    Q_D(URTcp);
    QByteArray msg = d->robotSocket->readAll();
    //qDebug() << "slot_receive_robot: " << msg;
    if (msg.contains("[")) {
        // emit signal_get_inverse_jpos(msg);
        emit signal_get_forward_cpos(msg);
    } else {
        emit signal_get_tool_analog_in(msg);
    }
}

void URTcp::get_current_robot_status(JointPos &jpos, CartPos &cpos, Wrench &twrench) {
    Q_D(URTcp);
    jpos = d->jpos;
    cpos = d->cpos;
    twrench = d->wrench;
}

QString URTcp::ByteArrayToHexString(QByteArray data) {
    QString ret(data.toHex().toUpper());
    int len = ret.length() / 2;
    for (int i = 1; i < len; i++) {
        ret.insert(2 * i + i - 1, " ");
    }
    return ret;
}

void URTcp::send_command(QString cmd) {
    Q_D(URTcp);
    qDebug() << "cmd: " << (cmd.toStdString() + std::string("\n")).data();
    d->tcpSocket->write((cmd.toStdString() + std::string("\n")).data());
    d->tcpSocket->flush();
}

void URTcp::send_command(const char *cmd) {
    Q_D(URTcp);
    qDebug() << "cmd in" << cmd;
    d->tcpSocket->write(cmd);
    d->tcpSocket->flush();
}

void URTcp::slot_send_command(const char *cmd) {
    Q_D(URTcp);
    // qDebug() << "command in" << cmd;
    d->tcpSocket->write(cmd);
    d->tcpSocket->flush();
}

void URTcp::slot_send_command(QString cmd) {
    Q_D(URTcp);
    qDebug() << "cmd: " << (cmd.toStdString() + std::string("\n")).data();
    d->tcpSocket->write((cmd.toStdString() + std::string("\n")).data());
    d->tcpSocket->flush();
}

void URTcp::slot_send_30002_command(const char *cmd) {
    Q_D(URTcp);
    // qDebug() << "30002 command in" << cmd;
    d->analogSocket->write(cmd);
    d->analogSocket->flush();
}

QTcpSocket *URTcp::get_Socket() {
    Q_D(URTcp);
    return d->tcpSocket;
}

void URTcp::slot_connection_broken() {
    Q_D(URTcp);
    emit signal_is_ur_connected(false);
}

void URTcp::slot_connection_state(bool state) {
    emit signal_send_connect_status(state ? 1 : 0);
}
