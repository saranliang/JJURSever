#ifndef URTCP_H
#define URTCP_H

#include "QThread"
#include "URStruct.h"
#include "pthread.h"
#include <QDebug>
#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <arpa/inet.h>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>

class URTcpPrivate;
class URTcp : public QObject {
    Q_OBJECT
  public:
    explicit URTcp(QObject *parent = nullptr);
    ~URTcp();

    void send_command(QString cmd);
    void send_command(const char *cmd);
    QTcpSocket *get_Socket();
    void get_current_robot_status(JointPos &, CartPos &, Wrench &);
    void IOConnect(QString, quint16);

  signals:
    void signal_is_ur_connected(bool);
    void signal_joint_pos(JointPos);
    void signal_cart_pos(CartPos);
    void signal_tcp_wrench(Wrench);
    void signal_tcp_speed(Speed);
    void signal_digital_input(double);
    void signal_safety_status(double);
    void signal_programe_state(double);
    //    void signal_current_located_jpos(QByteArray);
    void signal_send_connect_status(int);
    void signal_get_inverse_jpos(QByteArray);
    void signal_get_forward_cpos(QByteArray);
    void signal_get_tool_analog_in(QByteArray);
    void signal_target_pos(CartPos);

  public slots:
    void slot_receive();
    void slot_receive_robot();
    void slot_newConnection(quint16);
    void slot_connection_broken();
    void slot_disconnect();
    void slot_server_newConnection();
    void slot_send_command(const char *);
    void slot_send_command(QString);
    void slot_connection_state(bool);
    void slot_connect_30002_port();
    void slot_send_30002_command(const char *);

  private:
    QString ByteArrayToHexString(QByteArray data);

  protected:
    QScopedPointer<URTcpPrivate> d_ptr;

  private:
    Q_DECLARE_PRIVATE(URTcp)
    Q_DISABLE_COPY(URTcp)
};

#endif // URTCP_H
