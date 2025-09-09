#ifndef URSTATUS_H
#define URSTATUS_H

#include <QObject>
#include <URTcp.h>
#include <URStruct.h>

class URStatusPrivate;

class URStatus : public QObject
{
    Q_OBJECT
public:
    explicit URStatus(QObject *parent = nullptr, URTcp *tcpServer = nullptr);
    ~URStatus();

    void take_over();
    void hand_over();
    double ntohd(double);
    JointPos jpos;
    CartPos cpos;
    Wrench twrench;
    Speed speed;

    void get_current_cart_pos(CartPos&);

signals:
    void signal_send_joint_pos(JointPos);
    void signal_send_cart_pos(CartPos);
    void signal_send_tcp_wrench(Wrench);
    void signal_send_tcp_speed(Speed);
    //void signal_send_digital_input(double);
    void signal_send_input_status(int);
    void signal_send_safety_status(double);
    void signal_connect_status(int);

public slots:
    void slot_get_joint_pos(JointPos);
    void slot_get_cart_pos(CartPos);
    void slot_get_tcp_wrench(Wrench);
    void slot_get_tcp_speed(Speed);
    void slot_timer_start();
    void slot_get_digital_input(double);
    void slot_get_safety_status(double);
    void slot_get_connect_status(int);

protected:
    QScopedPointer<URStatusPrivate> d_ptr;
private:
    Q_DECLARE_PRIVATE(URStatus)
    Q_DISABLE_COPY(URStatus)
};

#endif // URSTATUS_H
