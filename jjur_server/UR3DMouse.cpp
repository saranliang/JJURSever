#include "UR3DMouse.h"

#define LIMIT 200

class UR3DMousePrivate : public QObject
{
public:
    spnav_event sev;
    int ret = -1;
};

UR3DMouse::UR3DMouse(QObject *parent) : QObject (parent), d_ptr(new UR3DMousePrivate)
{
    Q_D(UR3DMouse);
}

UR3DMouse::~UR3DMouse()
{
    Q_D(UR3DMouse);
}

int UR3DMouse::open_spnav()
{
    Q_D(UR3DMouse);
    d->ret = spnav_open();
    return d->ret;
}

void UR3DMouse::close_spnav()
{
    Q_D(UR3DMouse);
    if (d->ret == -1) {
        return;
    }
    spnav_remove_events(SPNAV_EVENT_ANY);
    spnav_close();
    d->ret = -1;
}

void UR3DMouse::slot_start_mouse_move()
{
    Q_D(UR3DMouse);
    if (d->ret == -1) return;
    //qDebug() << "子线程ID2: " << QThread::currentThreadId();
    d->sev.motion.x = 0;
    d->sev.motion.y = 0;
    d->sev.motion.z = 0;
    d->sev.motion.rx = 0;
    d->sev.motion.ry = 0;
    d->sev.motion.rz = 0;
    spnav_wait_event(&d->sev);
    if (d->sev.type == SPNAV_EVENT_MOTION) {
        printf("got motion event: t(%d, %d, %d) ", d->sev.motion.x, d->sev.motion.y, d->sev.motion.z);
        printf("r(%d, %d, %d)\n", d->sev.motion.rx, d->sev.motion.ry, d->sev.motion.rz);
        if (d->sev.motion.x > LIMIT) {
            axistype[0] = 1;
        } else if (d->sev.motion.x < -LIMIT) {
            axistype[0] = 2;
        } else {
            axistype[0] = 0;
        }
        if (d->sev.motion.y > LIMIT) {
            axistype[2] = 1;
        } else if (d->sev.motion.y < -LIMIT) {
            axistype[2] = 2;
        } else {
            axistype[2] = 0;
        }
        if (d->sev.motion.z > LIMIT) {
            axistype[1] = 1;
        } else if (d->sev.motion.z < -LIMIT) {
            axistype[1] = 2;
        } else {
            axistype[1] = 0;
        }
        if (d->sev.motion.rx > LIMIT) {
            axistype[3] = 1;
        } else if (d->sev.motion.rx < -LIMIT) {
            axistype[3] = 2;
        } else {
            axistype[3] = 0;
        }
        if (d->sev.motion.ry > LIMIT) {
            axistype[5] = 1;
        } else if (d->sev.motion.ry < -LIMIT) {
            axistype[5] = 2;
        } else {
            axistype[5] = 0;
        }
        if (d->sev.motion.rz > LIMIT) {
            axistype[4] = 1;
        } else if (d->sev.motion.rz < -LIMIT) {
            axistype[4] = 2;
        } else {
            axistype[4] = 0;
        }
        if (axistype[0] == 0 && axistype[1] == 0 && axistype[2] == 0 && axistype[3] == 0 && axistype[4] ==0 && axistype[5] == 0) {
            emit signal_stop_movel();
        } else {
            emit signal_send_axisstatus(axistype);
        }
    } else {//SPNAV_EVENT_MOTION
        printf("got button %s event b(%d)\n", d->sev.button.press ? "press" : "release", d->sev.button.bnum);
    }
}
