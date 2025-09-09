#ifndef URBASICMOTION_H
#define URBASICMOTION_H

#include <QObject>
#include "URTcp.h"

class URBasicMotionPrivate;

class URBasicMotion : public QObject
{
    Q_OBJECT
public:
    explicit URBasicMotion(QObject *parent = nullptr, URTcp *tcpServer = nullptr);
    ~URBasicMotion();

    void moveRobot_JointPosition(JointPos);
    void moveRobot_CartPosition(CartPos);
    void moveRobot_left_init_pos(JointPos);
    void moveRobot_right_init_pos(JointPos);

signals:
    void signal_send_command(const char*);

protected:
    QScopedPointer<URBasicMotionPrivate> d_ptr;

private:
    Q_DECLARE_PRIVATE(URBasicMotion)
    Q_DISABLE_COPY(URBasicMotion)
};

#endif // URBASICMOTION_H
