#ifndef UR3DMOUSE_H
#define UR3DMOUSE_H

#include <QObject>
#include <QDebug>
#include <QThread>
#include "stdio.h"
#include "stdlib.h"
#include "signal.h"
#include "spnav.h"
#include "URStruct.h"

class UR3DMousePrivate;

class UR3DMouse : public QObject
{
    Q_OBJECT
public:
    explicit UR3DMouse(QObject *parent = nullptr);
    ~UR3DMouse();
    int open_spnav();
    void close_spnav();
    int axistype[6];

signals:
    void signal_send_axisstatus(int*);
    void signal_stop_movel();

public slots:
    void slot_start_mouse_move();

protected:
    QScopedPointer<UR3DMousePrivate> d_ptr;

private:
    Q_DECLARE_PRIVATE(UR3DMouse)
    Q_DISABLE_COPY(UR3DMouse)
};

#endif // UR3DMOUSE_H
