#include "URSetPayload.h"

class URSetPayloadPrivate : public QObject
{
public:
    URTcp *_tcpServer;
};

URSetPayload::URSetPayload(QObject *parent, URTcp *tcpServer) :
    QObject (parent), d_ptr(new URSetPayloadPrivate)
{
    Q_D(URSetPayload);
    d->_tcpServer = tcpServer;
}

URSetPayload::~URSetPayload()
{
    Q_D(URSetPayload);
}
