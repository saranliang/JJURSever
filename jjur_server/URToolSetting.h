#ifndef URTOOLSETTING_H
#define URTOOLSETTING_H

#include <QObject>
#include "URTcp.h"
#include "URStruct.h"

class URToolSettingPrivate;
class URToolSetting : public QObject
{
    Q_OBJECT
public:
    explicit URToolSetting(QObject *parent = nullptr, URTcp *tcpServer = nullptr);
    ~URToolSetting();
    void set_payload(Payload);
    void set_tcp_pos(CartPos);

signals:
    void signal_send_command(const char*);

protected:
    QScopedPointer<URToolSettingPrivate> d_ptr;

private:
    Q_DECLARE_PRIVATE(URToolSetting)
    Q_DISABLE_COPY(URToolSetting)
};

#endif // URTOOLSETTING_H
