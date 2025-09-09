#ifndef URSETPAYLOAD_H
#define URSETPAYLOAD_H

#include <QObject>
#include "URTcp.h"

class URSetPayloadPrivate;
class URSetPayload : public QObject
{
    Q_OBJECT
public:
    explicit URSetPayload(QObject *parent = nullptr, URTcp *tcpServer = nullptr);
    ~URSetPayload();

protected:
    QScopedPointer<URSetPayloadPrivate> d_ptr;

private:
    Q_DECLARE_PRIVATE(URSetPayload)
    Q_DISABLE_COPY(URSetPayload)
};

#endif // URSETPAYLOAD_H
