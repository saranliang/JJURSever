#ifndef URSTRUCT_H
#define URSTRUCT_H

#include "math.h"
#include <QObject>

#define SERVER_HOST "172.31.1.147"
#define SERVER_PORT 30003
#define RAD2DEG 180.0 / M_PI
#define DEG2RAD M_PI / 180.0

typedef struct CARTPOS {
    double x;
    double y;
    double z;
    double rx; //是旋转矢量，不是欧拉角
    double ry;
    double rz;
} CartPos;

typedef struct JOINTPOS {
    double j1;
    double j2;
    double j3;
    double j4;
    double j5;
    double j6;
} JointPos;

typedef struct WRENCH {
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;
} Wrench;

typedef struct PAYLOAD {
    double m;
    double x;
    double y;
    double z;
} Payload;

typedef struct SPEED {
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;
} Speed;

enum MARKER_TYPE {
    ARM_EFFECTOR = 0,
    IMPLANT,
    ATRACSYS
};

enum ROBOT_TYPE {
    UR_3E = 0,
    UR_5E,
    UR_10E,
    UR_16E,
    UR_20E
};

#endif // URSTRUCT_H
