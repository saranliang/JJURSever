#include "URServer.h"
#include "PID.h"
#include <QDateTime>
#include <QTimer>
#include <QVector>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "Eigen/Eigen/Core"
#include "Eigen/Eigen/Dense"
#include "Eigen/Eigen/Geometry"
#include "generated/version.hpp"
#include <QFile>
#include <QStandardPaths>
#define LSPEED 0.02
#define RSPEED 20
#define FORCELIMIT 5
#define TORGUELIMIT 0.5

Q_DECLARE_METATYPE(JointPos);
Q_DECLARE_METATYPE(CartPos);
Q_DECLARE_METATYPE(Wrench);
Q_DECLARE_METATYPE(Speed);
Q_DECLARE_METATYPE(const char *);

class URServerPrivate : public QObject {
  public:
    URTcp *_tcpServer;
    QThread *tcpThread;
    QTimer *force_timer;
    QTimer *send_timer;
    int digital_value = 769;
    double input_value;
    CartPos drill_offset;
    double drill_length = 40.0;
    CartPos current_tool_pos;
    JointPos current_jpos;
    Wrench current_wrench;
    Speed current_speed;
    Payload m_load;
    int safety_status = 1;
    int programe_state = 0;
    double movetype[6];
    int dof[6];
    CartPos speed2flange;
    int force_type = 0;
    int toothID;
    double implant_length;
    bool isEnableFixLine = false;

    CartPos *target_pos;
    CartPos *virtual_pos;
    int virtual_axis; // 限深的轴方向， X/Y/Z
    Eigen::Matrix4d matrix_virtual_wall_pos;
    Eigen::Vector3d vector_original_normal; // 定线的Z方向向量
    CartPos *fix_line_pos;
    QTimer *detect_timer = nullptr;
    QTimer *special_detect_timer = nullptr;
    QTimer *fixpoint_singularity_detect_timer = nullptr;
    int head_radius = 300;
    double robot_score = 0.0;
    QTimer *detect_robot_pos_timer = nullptr;
    /*使用force_mode+speedl的定线随动相关变量*/
    CartPos speed_pos; // speedl的参数
    double delta_time = 0.1;
    double speed_t = 20;                                                     // speedl中位移的最大速度
    double speed_r = 0.03;                                                   // speedl姿态的最大速度
    double kp_speed_t = 3, ki_speed_t = 0, kd_speed_t = 0;                   // speedl中位移的PID参数
    double kp_speed_r = 3, ki_speed_r = 0, kd_speed_r = 0;                   // speedl中姿态的PID参数
    double kp_fixline_speed = 3, ki_fixline_speed = 0, kd_fixline_speed = 0; // force_mode中下钻速度的PID参数
    double integral_speed_t = 0;
    double integral_speed_r = 0;
    double integral_fixline_speed = 0;
    double derivative_speed_t = 0;
    double derivative_speed_r = 0;
    double derivative_fixline_speed = 0;
    double previous_error_speed_t = 0;
    double previous_error_speed_r = 0;
    double previous_error_fixline_speed = 0;
    double max_fixline_speed = 30;

    Eigen::Matrix4d mat_tool2flange = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_tool2base = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_flange2base = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_tool2flangetop = Eigen::Matrix4d::Identity();
    double flangebottom2base;
    double flangetop2base;
    int flangetop2bottom = 150; // 52 150 机械臂法兰面到法兰底端的距离，主要用来计算头部奇异点
    CartPos target_cart_pos;    // 目标笛卡尔坐标

    std::vector<double> target_kp = {3, 2, 3, 4.5, 4.5, 4.5}; // 比例增益
    // std::vector<double> target_ki = {0.02, 0.02, 0.02, 0.01, 0.01, 0.01}; // 积分增益
    std::vector<double> target_ki = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00}; // 积分增益
    std::vector<double> target_kd = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00}; // 微分增益
    std::vector<double> max_arm_velocity = {10 * 3.14, 10 * 3.14, 10 * 3.14, 10 * 3.14, 10 * 3.14, 10 * 3.14};
    std::vector<PIDController> pid_joints;
    double analysisSpeed(std::vector<double> pidSpeed);
};

double URServerPrivate::analysisSpeed(std::vector<double> pidSpeed) {
    double targetAcce = 10.0;
    double maxSpeedForBase = M_PI;
    if (pidSpeed[0] > maxSpeedForBase || pidSpeed[0] > maxSpeedForBase || pidSpeed[0] > maxSpeedForBase) {
        targetAcce = 5.0;
    }
    return targetAcce;
}

URServer::URServer(QObject *parent) : QObject(parent), d_ptr(new URServerPrivate) {
    Q_D(URServer);
    qRegisterMetaType<JointPos>("JointPos");
    qRegisterMetaType<CartPos>("CartPos");
    qRegisterMetaType<Wrench>("Wrench");
    qRegisterMetaType<Speed>("Speed");
    d->_tcpServer = new URTcp(nullptr);
    d->tcpThread = new QThread();
    d->tcpThread->setObjectName("URTcp");
    d->tcpThread->start();
    d->_tcpServer->moveToThread(d->tcpThread);
    d->force_timer = new QTimer(this);
    d->send_timer = new QTimer(this);
    d->detect_timer = new QTimer(this);
    d->special_detect_timer = new QTimer(this);
    d->detect_robot_pos_timer = new QTimer(this);
    d->fixpoint_singularity_detect_timer = new QTimer(this);
    d->drill_length = 22;
    d->target_pos = new CartPos();
    d->virtual_pos = new CartPos();
    d->fix_line_pos = new CartPos();
    d->target_pos->x = 0;
    d->target_pos->y = 0;
    d->target_pos->z = 0;
    d->target_pos->rx = 0;
    d->target_pos->ry = 0;
    d->target_pos->rz = 0;
    d->virtual_pos->x = 0;
    d->virtual_pos->y = 0;
    d->virtual_pos->z = 0;
    d->virtual_pos->rx = 0;
    d->virtual_pos->ry = 0;
    d->virtual_pos->rz = 0;
    d->fix_line_pos->x = 0;
    d->fix_line_pos->y = 0;
    d->fix_line_pos->z = 0;
    d->fix_line_pos->rx = 0;
    d->fix_line_pos->ry = 0;
    d->fix_line_pos->rz = 0;

    for (int i = 0; i < 6; i++) {
        d->pid_joints.push_back(PIDController(d->target_kp[i], d->target_ki[i], d->target_kd[i]));
    }

    set_robot_type(ROBOT_TYPE::UR_5E);

    hand_over();
    take_over();
    d->send_timer->start(1000 / 30);
    set_robot_type(ROBOT_TYPE::UR_5E);
    QString doc_path = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/Sirius.gitinfo";
    QFile gitinfo(doc_path);
    if (gitinfo.exists()) {
        gitinfo.open(QIODevice::Append);
        QString info;
        info = "\n************\nGit Info\nJJURServer\nGit Branch: " + QString(GIT_BRANCH) + "\nGit Commit ID: " + QString(GIT_HASH) + "\n************\n";
        gitinfo.write(info.toStdString().data());
        gitinfo.close();
    }
}

URServer::~URServer() {
    Q_D(URServer);
    d->send_timer->stop();
    hand_over();
    if (d->send_timer) {
        delete d->send_timer;
    }
    if (d->detect_timer) {
        delete d->detect_timer;
    }
    if (d->special_detect_timer) {
        delete d->special_detect_timer;
    }
    if (d->fixpoint_singularity_detect_timer) {
        delete d->fixpoint_singularity_detect_timer;
    }
    if (d->detect_robot_pos_timer) {
        delete d->detect_robot_pos_timer;
    }
}

void URServer::take_over() {
    Q_D(URServer);
    connect(this, SIGNAL(signal_new_connection(quint16)), d->_tcpServer, SLOT(slot_newConnection(quint16)), Qt::BlockingQueuedConnection);
    connect(this, SIGNAL(signal_disconnect()), d->_tcpServer, SLOT(slot_disconnect()), Qt::BlockingQueuedConnection);
    connect(d->_tcpServer, SIGNAL(signal_joint_pos(JointPos)), this, SLOT(slot_update_joint_pos(JointPos)), Qt::QueuedConnection);
    connect(d->_tcpServer, SIGNAL(signal_cart_pos(CartPos)), this, SLOT(slot_update_cart_pos(CartPos)), Qt::QueuedConnection);
    connect(d->_tcpServer, SIGNAL(signal_tcp_wrench(Wrench)), this, SLOT(slot_update_tcp_wrench(Wrench)), Qt::QueuedConnection);
    connect(d->_tcpServer, SIGNAL(signal_tcp_speed(Speed)), this, SLOT(slot_update_tcp_speed(Speed)), Qt::QueuedConnection);
    connect(d->_tcpServer, SIGNAL(signal_send_connect_status(int)), this, SLOT(slot_get_connect_status(int)));
    connect(d->_tcpServer, SIGNAL(signal_digital_input(double)), this, SLOT(slot_update_digital_input(double)));
    connect(d->_tcpServer, SIGNAL(signal_safety_status(double)), this, SLOT(slot_update_safety_status(double)));
    connect(d->_tcpServer, &URTcp::signal_programe_state, this, &URServer::slot_update_programe_state);
    connect(d->_tcpServer, SIGNAL(signal_target_pos(CartPos)), this, SLOT(slot_update_target_pos(CartPos)));
    connect(this, SIGNAL(signal_send_command(const char *)), d->_tcpServer, SLOT(slot_send_command(const char *)), Qt::BlockingQueuedConnection);
    // connect(this, SIGNAL(signal_send_command(QString)), d->_tcpServer, SLOT(slot_send_command(QString)));
    connect(d->force_timer, SIGNAL(timeout()), this, SLOT(slot_force_timer_start()));
    connect(d->send_timer, &QTimer::timeout, this, &URServer::slot_send_timer_start);
    connect(d->detect_timer, &QTimer::timeout, this, &URServer::slot_detect_timer_start);
    connect(d->special_detect_timer, &QTimer::timeout, this, &URServer::slot_special_detect_timer_start);
    connect(d->fixpoint_singularity_detect_timer, &QTimer::timeout, this, &URServer::slot_fixpoint_singularity_detect_timer_start);
    connect(d->_tcpServer, &URTcp::signal_get_inverse_jpos, this, &URServer::slot_get_inverse_jpos);
    connect(d->_tcpServer, &URTcp::signal_get_forward_cpos, this, &URServer::slot_get_forward_cpos);
    connect(d->_tcpServer, &URTcp::signal_get_tool_analog_in, this, &URServer::slot_get_tool_analog_in);
    connect(d->detect_robot_pos_timer, &QTimer::timeout, this, &URServer::slot_detect_robot_pos_timer_start);

    connect(this, &URServer::signal_connect_30002_port, d->_tcpServer, &URTcp::slot_connect_30002_port);
    connect(this, &URServer::signal_send_30002_command, d->_tcpServer, &URTcp::slot_send_30002_command);
}

void URServer::hand_over() {
    Q_D(URServer);
    disconnect(this, SIGNAL(signal_new_connection(quint16)), d->_tcpServer, SLOT(slot_newConnection(quint16)));
    disconnect(this, SIGNAL(signal_disconnect()), d->_tcpServer, SLOT(slot_disconnect()));
    disconnect(d->_tcpServer, SIGNAL(signal_joint_pos(JointPos)), this, SLOT(slot_update_joint_pos(JointPos)));
    disconnect(d->_tcpServer, SIGNAL(signal_cart_pos(CartPos)), this, SLOT(slot_update_cart_pos(CartPos)));
    disconnect(d->_tcpServer, SIGNAL(signal_tcp_wrench(Wrench)), this, SLOT(slot_update_tcp_wrench(Wrench)));
    disconnect(d->_tcpServer, SIGNAL(signal_tcp_speed(Speed)), this, SLOT(slot_update_tcp_speed(Speed)));
    disconnect(d->_tcpServer, SIGNAL(signal_send_connect_status(int)), this, SLOT(slot_get_connect_status(int)));
    disconnect(d->_tcpServer, SIGNAL(signal_digital_input(double)), this, SLOT(slot_update_digital_input(double)));
    disconnect(d->_tcpServer, SIGNAL(signal_safety_status(double)), this, SLOT(slot_update_safety_status(double)));
    disconnect(d->_tcpServer, &URTcp::signal_programe_state, this, &URServer::slot_update_programe_state);
    disconnect(d->force_timer, SIGNAL(timeout()), this, SLOT(slot_force_timer_start()));
    disconnect(d->send_timer, &QTimer::timeout, this, &URServer::slot_send_timer_start);
    disconnect(d->detect_timer, &QTimer::timeout, this, &URServer::slot_detect_timer_start);
    disconnect(d->special_detect_timer, &QTimer::timeout, this, &URServer::slot_special_detect_timer_start);
    disconnect(d->fixpoint_singularity_detect_timer, &QTimer::timeout, this, &URServer::slot_fixpoint_singularity_detect_timer_start);
    disconnect(d->_tcpServer, &URTcp::signal_get_inverse_jpos, this, &URServer::slot_get_inverse_jpos);
    disconnect(d->_tcpServer, &URTcp::signal_get_forward_cpos, this, &URServer::slot_get_forward_cpos);
    disconnect(this, &URServer::signal_connect_30002_port, d->_tcpServer, &URTcp::slot_connect_30002_port);
    disconnect(this, &URServer::signal_send_30002_command, d->_tcpServer, &URTcp::slot_send_30002_command);
    disconnect(d->_tcpServer, SIGNAL(signal_target_pos(CartPos)), this, SLOT(slot_update_target_pos(CartPos)));
}

void URServer::send_command(QString cmd) {
    Q_D(URServer);
    d->_tcpServer->send_command(cmd);
}

void URServer::send_command(char *cmd) {
    Q_D(URServer);
    emit signal_send_command(cmd);
}

double URServer::ntohd(double d) {
    double ret;
    unsigned long i;
    unsigned char *p = (unsigned char *)&d;
    unsigned char *r = (unsigned char *)&ret;
    for (i = 0; i < sizeof(double); i++) {
        r[i] = p[sizeof(double) - 1 - i];
    }
    return ret;
}

void URServer::slot_update_joint_pos(JointPos jPos) {
    Q_D(URServer);
    d->current_jpos.j1 = ntohd(jPos.j1);
    d->current_jpos.j2 = ntohd(jPos.j2);
    d->current_jpos.j3 = ntohd(jPos.j3);
    d->current_jpos.j4 = ntohd(jPos.j4);
    d->current_jpos.j5 = ntohd(jPos.j5);
    d->current_jpos.j6 = ntohd(jPos.j6);
    d->current_jpos.j1 = d->current_jpos.j1 * RAD2DEG;
    d->current_jpos.j2 = d->current_jpos.j2 * RAD2DEG;
    d->current_jpos.j3 = d->current_jpos.j3 * RAD2DEG;
    d->current_jpos.j4 = d->current_jpos.j4 * RAD2DEG;
    d->current_jpos.j5 = d->current_jpos.j5 * RAD2DEG;
    d->current_jpos.j6 = d->current_jpos.j6 * RAD2DEG;
    // emit signal_update_joint_pos(d->current_jpos);
}

void URServer::slot_update_cart_pos(CartPos cPos) {
    Q_D(URServer);
    d->current_tool_pos.x = ntohd(cPos.x);
    d->current_tool_pos.y = ntohd(cPos.y);
    d->current_tool_pos.z = ntohd(cPos.z);
    d->current_tool_pos.rx = ntohd(cPos.rx);
    d->current_tool_pos.ry = ntohd(cPos.ry);
    d->current_tool_pos.rz = ntohd(cPos.rz);
    if (d->current_tool_pos.rx < 0) {
        double theta = sqrt(d->current_tool_pos.rx * d->current_tool_pos.rx + d->current_tool_pos.ry * d->current_tool_pos.ry + d->current_tool_pos.rz * d->current_tool_pos.rz);
        double new_theta = 2 * M_PI - theta;
        d->current_tool_pos.rx = -d->current_tool_pos.rx / theta * new_theta;
        d->current_tool_pos.ry = -d->current_tool_pos.ry / theta * new_theta;
        d->current_tool_pos.rz = -d->current_tool_pos.rz / theta * new_theta;
    }
    d->current_tool_pos.x = d->current_tool_pos.x * 1000;
    d->current_tool_pos.y = d->current_tool_pos.y * 1000;
    d->current_tool_pos.z = d->current_tool_pos.z * 1000;
    // emit signal_update_cart_pos(d->current_tool_pos);
    if (d->isEnableFixLine) {
        handle_fixline_virtual_wall();
    }
}

void URServer::slot_update_tcp_wrench(Wrench tWrench) {
    Q_D(URServer);
    Wrench current_wrench, current_Twrench;
    current_wrench.x = ntohd(tWrench.x);
    current_wrench.y = ntohd(tWrench.y);
    current_wrench.z = ntohd(tWrench.z);
    current_wrench.rx = ntohd(tWrench.rx);
    current_wrench.ry = ntohd(tWrench.ry);
    current_wrench.rz = ntohd(tWrench.rz);
    Eigen::Matrix3d R_inv;
    Eigen::Matrix4d matrix_pos;
    matrix_pos = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    R_inv = get_rotation(matrix_pos);
    Eigen::Vector3d vector_TT, vector_TR, vector_BT, vector_BR;
    vector_BT = {current_wrench.x, current_wrench.y, current_wrench.z};
    vector_BR = {current_wrench.rx, current_wrench.ry, current_wrench.rz};
    vector_TT = R_inv.inverse() * vector_BT;
    vector_TR = R_inv.inverse() * vector_BR;
    d->current_wrench.x = vector_TT[0];
    d->current_wrench.y = vector_TT[1];
    d->current_wrench.z = vector_TT[2];
    d->current_wrench.rx = vector_TR[0];
    d->current_wrench.ry = vector_TR[1];
    d->current_wrench.rz = vector_TR[2];
    // emit signal_update_tcp_wrench(d->current_wrench);
}

void URServer::slot_update_tcp_speed(Speed speed) {
    Q_D(URServer);
    d->current_speed.x = ntohd(speed.x);
    d->current_speed.y = ntohd(speed.y);
    d->current_speed.z = ntohd(speed.z);
    d->current_speed.rx = ntohd(speed.rx);
    d->current_speed.ry = ntohd(speed.ry);
    d->current_speed.rz = ntohd(speed.rz);
    // emit signal_update_tcp_speed(d->current_speed);
}

void URServer::slot_update_digital_input(double value) {
    Q_D(URServer);
    d->input_value = ntohd(value);
    int m_value;
    m_value = static_cast<int>(d->input_value);
    if (d->digital_value != m_value) {
        d->digital_value = m_value;
        if (d->digital_value >> 16 == 0) {
            emit signal_update_button_status(NO_BTN);
            emit signal_send_tool_io_status(NO_IO);
        } else if (d->digital_value >> 16 == 1) {
            emit signal_update_button_status(TEACH_BTN);
            emit signal_send_tool_io_status(FIRST_IO);
        } else if (d->digital_value >> 16 == 2) {
            emit signal_update_button_status(FIXLINE_BTN);
            emit signal_send_tool_io_status(SECOND_IO);
        } else if (d->digital_value >> 16 == 3) {
            emit signal_update_button_status(NO_BTN);
            emit signal_send_tool_io_status(BOTH_IO);
        }
    }
}

void URServer::slot_update_safety_status(double value) {
    Q_D(URServer);
    value = ntohd(value);
    int m_value;
    m_value = static_cast<int>(value);
    d->safety_status = m_value;
    if (d->safety_status == 3 || d->safety_status == 5 || d->safety_status == 9 || d->safety_status == 1) {
        emit signal_update_safety_status(d->safety_status);
    }
}

void URServer::slot_update_programe_state(double value) {
    Q_D(URServer);
    value = ntohd(value);
    int m_value;
    m_value = static_cast<int>(value);
    if (d->programe_state != m_value) {
        // qDebug() << "slot_update_robot_mode: " << m_value;
        d->programe_state = m_value;
        emit signal_send_motion_status(d->programe_state);
    }
}

void URServer::slot_update_target_pos(CartPos cPos) {
    Q_D(URServer);
    d->target_cart_pos.x = ntohd(cPos.x);
    d->target_cart_pos.y = ntohd(cPos.y);
    d->target_cart_pos.z = ntohd(cPos.z);
    d->target_cart_pos.rx = ntohd(cPos.rx);
    d->target_cart_pos.ry = ntohd(cPos.ry);
    d->target_cart_pos.rz = ntohd(cPos.rz);
    if (d->target_cart_pos.rx < 0) {
        double theta = sqrt(d->target_cart_pos.rx * d->target_cart_pos.rx + d->target_cart_pos.ry * d->target_cart_pos.ry + d->target_cart_pos.rz * d->target_cart_pos.rz);
        double new_theta = 2 * M_PI - theta;
        d->target_cart_pos.rx = -d->target_cart_pos.rx / theta * new_theta;
        d->target_cart_pos.ry = -d->target_cart_pos.ry / theta * new_theta;
        d->target_cart_pos.rz = -d->target_cart_pos.rz / theta * new_theta;
    }
    d->target_cart_pos.x = d->target_cart_pos.x * 1000;
    d->target_cart_pos.y = d->target_cart_pos.y * 1000;
    d->target_cart_pos.z = d->target_cart_pos.z * 1000;
    emit signal_update_target_pos(d->target_cart_pos);
    // emit signal_target_pos(*d->target_pos);
}
// 触发器为：30帧
// 触发器时间<数据更新时间间隔：数据需要通过定时器触发吗，重复的数据没有意义，还会挤占新数据的更新时间
// 触发器时间>数据更新时间间隔：需要，数据只在需要时被更新，不用一直更新
void URServer::slot_send_timer_start() {
    Q_D(URServer);
    emit signal_update_joint_pos(d->current_jpos);
    emit signal_update_cart_pos(d->current_tool_pos);
    emit signal_update_tcp_wrench(d->current_wrench);
    emit signal_update_tcp_speed(d->current_speed);
    // get_tool_analog_in(0);
}

void URServer::slot_detect_timer_start() {
    Q_D(URServer);
    if ((d->current_jpos.j5 < -136 && d->current_jpos.j5 > -224) ||
        (d->current_jpos.j5 > 136 && d->current_jpos.j5 < 224) ||
        (d->current_jpos.j5 > -44 && d->current_jpos.j5 < 44) ||
        (d->current_jpos.j5 > -360 && d->current_jpos.j5 < -316) ||
        (d->current_jpos.j5 > 316 && d->current_jpos.j5 < 360)) {
        emit signal_singularity_type(1);
        qDebug() << "接近腕部奇异点！请注意！！！";
    } else {
        emit signal_singularity_type(0);
    }
    if (d->current_jpos.j3 > -40 && d->current_jpos.j3 < 40) {
        emit signal_singularity_type(2);
        qDebug() << "接近肘部奇异点！请注意！！！";
    } else {
        emit signal_singularity_type(0);
    }
    slot_calculate_flange2base();
    // qDebug() << "distance: " << sqrt(d->current_tool_pos.x * d->current_tool_pos.x + d->current_tool_pos.y * d->current_tool_pos.y);
    // if (sqrt(d->current_tool_pos.x * d->current_tool_pos.x + d->current_tool_pos.y * d->current_tool_pos.y) < d->head_radius) {
    if (d->flangebottom2base < d->head_radius || d->flangetop2base < d->head_radius) {
        emit signal_singularity_type(3);
        qDebug() << "接近头部奇异点！请注意！！！";
    } else {
        emit signal_singularity_type(0);
    }
}

void URServer::slot_special_detect_timer_start() {
    Q_D(URServer);
    if ((d->current_jpos.j5 < -165 && d->current_jpos.j5 > -195) ||
        (d->current_jpos.j5 > 165 && d->current_jpos.j5 < 195) ||
        (d->current_jpos.j5 > -15 && d->current_jpos.j5 < 15) ||
        (d->current_jpos.j5 > -360 && d->current_jpos.j5 < -345) ||
        (d->current_jpos.j5 > 345 && d->current_jpos.j5 < 360)) {
        // emit signal_special_singularity_type(1);
        emit signal_singularity_type(1);
        qDebug() << "接近腕部奇异点！请注意！！！";
    } else {
        // emit signal_special_singularity_type(0);
        emit signal_singularity_type(0);
    }
    if (d->current_jpos.j3 > -15 && d->current_jpos.j3 < 15) {
        // emit signal_special_singularity_type(2);
        emit signal_singularity_type(2);
        qDebug() << "接近肘部奇异点！请注意！！！";
    } else {
        // emit signal_special_singularity_type(0);
        emit signal_singularity_type(0);
    }
    // qDebug() << "distance: " << sqrt(d->current_tool_pos.x * d->current_tool_pos.x + d->current_tool_pos.y * d->current_tool_pos.y);
    slot_calculate_flange2base();
    if (d->flangebottom2base < (d->head_radius - 100) || d->flangetop2base < (d->head_radius - 100)) {
        // emit signal_special_singularity_type(3);
        emit signal_singularity_type(3);
        qDebug() << "接近头部奇异点！请注意！！！";
    } else {
        // emit signal_special_singularity_type(0);
        emit signal_singularity_type(0);
    }
}

void URServer::slot_fixpoint_singularity_detect_timer_start() {
    Q_D(URServer);
    if ((d->current_jpos.j5 < -126 && d->current_jpos.j5 > -234) ||
        (d->current_jpos.j5 > 126 && d->current_jpos.j5 < 234) ||
        (d->current_jpos.j5 > -56 && d->current_jpos.j5 < 54) ||
        (d->current_jpos.j5 > -360 && d->current_jpos.j5 < -306) ||
        (d->current_jpos.j5 > 306 && d->current_jpos.j5 < 360)) {
        // emit signal_fixpoint_singularity_type(1);
        emit signal_singularity_type(1);
        qDebug() << "接近腕部奇异点！请注意！！！";
    } else {
        // emit signal_fixpoint_singularity_type(0);
        emit signal_singularity_type(0);
    }
    if (d->current_jpos.j3 > -55 && d->current_jpos.j3 < 55) {
        // emit signal_fixpoint_singularity_type(2);
        emit signal_singularity_type(2);
        qDebug() << "接近肘部奇异点！请注意！！！";
    } else {
        // emit signal_fixpoint_singularity_type(0);
        emit signal_singularity_type(0);
    }
    // qDebug() << "distance: " << sqrt(d->current_tool_pos.x * d->current_tool_pos.x + d->current_tool_pos.y * d->current_tool_pos.y);
    // if (sqrt(d->current_tool_pos.x * d->current_tool_pos.x + d->current_tool_pos.y * d->current_tool_pos.y) < (d->head_radius - 100)) {
    slot_calculate_flange2base();
    if (d->flangebottom2base < (d->head_radius + 50) || d->flangetop2base < (d->head_radius + 50)) {
        // emit signal_fixpoint_singularity_type(3);
        emit signal_singularity_type(3);
        qDebug() << "接近头部奇异点！请注意！！！";
    } else {
        // emit signal_fixpoint_singularity_type(0);
        emit signal_singularity_type(0);
    }
}

void URServer::end_teach_mode() {
    Q_D(URServer);
    stop_singularity_check();
    stop_special_singularity_check();
    stop_fixpoint_singularity_check();
    char szCmd[0x100];
    sprintf(szCmd, "def end_teach_mode():\n end_teach_mode()\nend\n");
    emit signal_send_command(szCmd);
}

void URServer::moveRobot_joint(JointPos jpos) {
    Q_D(URServer);
    stop_singularity_check();
    stop_special_singularity_check();
    stop_fixpoint_singularity_check();
    char szCmd[0x100];
    sprintf(szCmd, "movej([%f, %f, %f, %f, %f, %f], a=1.2, v=0.05, t=0, r=0)\n", jpos.j1, jpos.j2, jpos.j3, jpos.j4, jpos.j5, jpos.j6);
    emit signal_send_command(szCmd);
}

void URServer::moveRobot_cart(CartPos cpos) {
    Q_D(URServer);
    char szCmd[0x100];
    sprintf(szCmd, "movel(p[%f, %f, %f, %f, %f, %f], a=1.2, v=0.02, t=0, r=0)\n", cpos.x / 1000, cpos.y / 1000, cpos.z / 1000, cpos.rx, cpos.ry, cpos.rz);
    emit signal_send_command(szCmd);
}

void URServer::stop_movej() {
    Q_D(URServer);
    char szCmd[0x100];
    sprintf(szCmd, "def stop_movej():\n stopj(2)\nend\n");
    emit signal_send_command(szCmd);
}

void URServer::stop_movel() {
    Q_D(URServer);
    char szCmd[0x100];
    sprintf(szCmd, "def stop_movel():\n stopl(2)\nend\n");
    emit signal_send_command(szCmd);
}

void URServer::jog_movel(int axistype[6]) {
    Q_D(URServer);
    if (axistype[0] == 0) {
        cPosl.x = 0;
    } else if (axistype[0] == 1) {
        cPosl.x = LSPEED;
    } else if (axistype[0] == 2) {
        cPosl.x = -LSPEED;
    }
    if (axistype[1] == 0) {
        cPosl.y = 0;
    } else if (axistype[1] == 1) {
        cPosl.y = LSPEED;
    } else if (axistype[1] == 2) {
        cPosl.y = -LSPEED;
    }
    if (axistype[2] == 0) {
        cPosl.z = 0;
    } else if (axistype[2] == 1) {
        cPosl.z = LSPEED;
    } else if (axistype[2] == 2) {
        cPosl.z = -LSPEED;
    }
    if (axistype[3] == 0) {
        cPosl.rx = 0;
    } else if (axistype[3] == 1) {
        cPosl.rx = RSPEED * DEG2RAD;
    } else if (axistype[3] == 2) {
        cPosl.rx = -RSPEED * DEG2RAD;
    }
    if (axistype[4] == 0) {
        cPosl.ry = 0;
    } else if (axistype[4] == 1) {
        cPosl.ry = RSPEED * DEG2RAD;
    } else if (axistype[4] == 2) {
        cPosl.ry = -RSPEED * DEG2RAD;
    }
    if (axistype[5] == 0) {
        cPosl.rz = 0;
    } else if (axistype[5] == 1) {
        cPosl.rz = RSPEED * DEG2RAD;
    } else if (axistype[5] == 2) {
        cPosl.rz = -RSPEED * DEG2RAD;
    }
    char szCmd[0x100];
    sprintf(szCmd, "speedl([%f, %f, %f, %f, %f, %f], 0.05, 0.2, 1)\n", cPosl.x, cPosl.y, cPosl.z, cPosl.rx, cPosl.ry, cPosl.rz);
    emit signal_send_command(szCmd);
}

void URServer::slot_start_teach_mode(bool flag) {
    Q_D(URServer);
    if (flag) {
        start_freedrive_mode();
    } else {
        end_teach_mode();
    }
}

void URServer::exit_force_mode() {
    Q_D(URServer);
    d->force_type = NO_ACTION;
    if (d->force_timer->isActive()) {
        d->force_timer->stop();
    }
    char szCmd[0x100];
    sprintf(szCmd, "end_force_mode()\n");
    emit signal_send_command(szCmd);
}

void URServer::slot_force_timer_start() {
    Q_D(URServer);
    start_force_mode();
}

void URServer::slot_recieve_force_mode(int type) {
    Q_D(URServer);
    d->force_type = type;
    d->force_timer->start(10);
}

void URServer::start_force_mode() {
    Q_D(URServer);
    // if (d->force_control_type == 2) {
    if (d->current_wrench.z < -FORCELIMIT) {
        d->force_type = FIXLINE;
    } else {
        d->force_type = NO_ACTION;
    }
    //}
    if (d->current_wrench.x > 3 * FORCELIMIT) {
        d->movetype[0] = PHIGH;
    } else if ((d->current_wrench.x > FORCELIMIT) && (d->current_wrench.x < 3 * FORCELIMIT)) {
        d->movetype[0] = PLOW;
    } else if ((d->current_wrench.x > -FORCELIMIT) && (d->current_wrench.x < FORCELIMIT)) {
        d->movetype[0] = STOP;
    } else if ((d->current_wrench.x > -3 * FORCELIMIT) && (d->current_wrench.x < -FORCELIMIT)) {
        d->movetype[0] = NLOW;
    } else if (d->current_wrench.x < -3 * FORCELIMIT) {
        d->movetype[0] = NHIGH;
    }
    if (d->current_wrench.y > 3 * FORCELIMIT) {
        d->movetype[1] = PHIGH;
    } else if ((d->current_wrench.y > FORCELIMIT) && (d->current_wrench.y < 3 * FORCELIMIT)) {
        d->movetype[1] = PLOW;
    } else if ((d->current_wrench.y > -FORCELIMIT) && (d->current_wrench.y < FORCELIMIT)) {
        d->movetype[1] = STOP;
    } else if ((d->current_wrench.y > -3 * FORCELIMIT) && (d->current_wrench.y < -FORCELIMIT)) {
        d->movetype[1] = NLOW;
    } else if (d->current_wrench.y < -3 * FORCELIMIT) {
        d->movetype[1] = NHIGH;
    }
    if (d->current_wrench.z > 3 * FORCELIMIT) {
        d->movetype[2] = PHIGH;
    } else if ((d->current_wrench.z > FORCELIMIT) && (d->current_wrench.z < 3 * FORCELIMIT)) {
        d->movetype[2] = PLOW;
    } else if ((d->current_wrench.z > -FORCELIMIT) && (d->current_wrench.z < FORCELIMIT)) {
        d->movetype[2] = STOP;
    } else if ((d->current_wrench.z > -3 * FORCELIMIT) && (d->current_wrench.z < -FORCELIMIT)) {
        d->movetype[2] = NLOW;
    } else if (d->current_wrench.z < -3 * FORCELIMIT) {
        d->movetype[2] = NHIGH;
    }
    if (d->current_wrench.rx > 3 * TORGUELIMIT) {
        d->movetype[3] = PHIGH;
    } else if ((d->current_wrench.rx > TORGUELIMIT) && (d->current_wrench.rx < 3 * TORGUELIMIT)) {
        d->movetype[3] = PLOW;
    } else if ((d->current_wrench.rx > -TORGUELIMIT) && (d->current_wrench.rx < TORGUELIMIT)) {
        d->movetype[3] = STOP;
    } else if ((d->current_wrench.rx > -3 * TORGUELIMIT) && (d->current_wrench.rx < -TORGUELIMIT)) {
        d->movetype[3] = NLOW;
    } else if (d->current_wrench.rx < -3 * TORGUELIMIT) {
        d->movetype[3] = NHIGH;
    }
    if (d->current_wrench.ry > 3 * TORGUELIMIT) {
        d->movetype[4] = PHIGH;
    } else if ((d->current_wrench.ry > TORGUELIMIT) && (d->current_wrench.ry < 3 * TORGUELIMIT)) {
        d->movetype[4] = PLOW;
    } else if ((d->current_wrench.ry > -TORGUELIMIT) && (d->current_wrench.ry < TORGUELIMIT)) {
        d->movetype[4] = STOP;
    } else if ((d->current_wrench.ry > -3 * TORGUELIMIT) && (d->current_wrench.ry < -TORGUELIMIT)) {
        d->movetype[4] = NLOW;
    } else if (d->current_wrench.ry < -3 * TORGUELIMIT) {
        d->movetype[4] = NHIGH;
    }
    if (d->current_wrench.rz > 3 * TORGUELIMIT) {
        d->movetype[5] = PHIGH;
    } else if ((d->current_wrench.rz > TORGUELIMIT) && (d->current_wrench.rz < 3 * TORGUELIMIT)) {
        d->movetype[5] = PLOW;
    } else if ((d->current_wrench.rz > -TORGUELIMIT) && (d->current_wrench.rz < TORGUELIMIT)) {
        d->movetype[5] = STOP;
    } else if ((d->current_wrench.rz > -3 * TORGUELIMIT) && (d->current_wrench.rz < -TORGUELIMIT)) {
        d->movetype[5] = NLOW;
    } else if (d->current_wrench.rz < -3 * TORGUELIMIT) {
        d->movetype[5] = NHIGH;
    }
    if (d->force_type == NO_ACTION) {
        d->dof[0] = 0;
        d->dof[1] = 0;
        d->dof[2] = 0;
        d->dof[3] = 0;
        d->dof[4] = 0;
        d->dof[5] = 0;
    }
    if (d->force_type == PANNING) {
        d->dof[0] = 1;
        d->dof[1] = 1;
        d->dof[2] = 1;
        d->dof[3] = 0;
        d->dof[4] = 0;
        d->dof[5] = 0;
    } else if (d->force_type == WHIRL) {
        d->dof[0] = 0;
        d->dof[1] = 0;
        d->dof[2] = 0;
        d->dof[3] = 1;
        d->dof[4] = 1;
        d->dof[5] = 1;
    } else if (d->force_type == FIXLINE) {
        d->dof[0] = 0;
        d->dof[1] = 0;
        d->dof[2] = 1;
        d->dof[3] = 0;
        d->dof[4] = 0;
        d->dof[5] = 0;
    }
    force_jog_movel(d->movetype, d->dof);
}

void URServer::force_jog_movel(double movetype[6], int dof[6]) {
    Q_D(URServer);
    if (movetype[0] == STOP) {
        cPosl.x = 0;
    } else if (movetype[0] == PLOW) {
        cPosl.x = LSPEED;
    } else if (movetype[0] == PHIGH) {
        cPosl.x = LSPEED;
    } else if (movetype[0] == NLOW) {
        cPosl.x = -LSPEED;
    } else if (movetype[0] == NHIGH) {
        cPosl.x = -LSPEED;
    }
    if (movetype[1] == STOP) {
        cPosl.y = 0;
    } else if (movetype[1] == PLOW) {
        cPosl.y = LSPEED;
    } else if (movetype[1] == PHIGH) {
        cPosl.y = LSPEED;
    } else if (movetype[1] == NLOW) {
        cPosl.y = -LSPEED;
    } else if (movetype[1] == NHIGH) {
        cPosl.y = -LSPEED;
    }
    if (movetype[2] == STOP) {
        cPosl.z = 0;
    } else if (movetype[2] == PLOW) {
        cPosl.z = LSPEED;
    } else if (movetype[2] == PHIGH) {
        cPosl.z = LSPEED;
    } else if (movetype[2] == NLOW) {
        cPosl.z = -LSPEED;
    } else if (movetype[2] == NHIGH) {
        cPosl.z = -LSPEED;
    }
    if (movetype[3] == STOP) {
        cPosl.rx = 0;
    } else if (movetype[3] == PLOW) {
        cPosl.rx = RSPEED * DEG2RAD;
    } else if (movetype[3] == PHIGH) {
        cPosl.rx = RSPEED * DEG2RAD;
    } else if (movetype[3] == NLOW) {
        cPosl.rx = -RSPEED * DEG2RAD;
    } else if (movetype[3] == NHIGH) {
        cPosl.rx = -RSPEED * DEG2RAD;
    }
    if (movetype[4] == STOP) {
        cPosl.ry = 0;
    } else if (movetype[4] == PLOW) {
        cPosl.ry = RSPEED * DEG2RAD;
    } else if (movetype[4] == PHIGH) {
        cPosl.ry = RSPEED * DEG2RAD;
    } else if (movetype[4] == NLOW) {
        cPosl.ry = -RSPEED * DEG2RAD;
    } else if (movetype[4] == NHIGH) {
        cPosl.ry = -RSPEED * DEG2RAD;
    }
    if (movetype[5] == STOP) {
        cPosl.rz = 0;
    } else if (movetype[5] == PLOW) {
        cPosl.rz = RSPEED * DEG2RAD;
    } else if (movetype[5] == PHIGH) {
        cPosl.rz = RSPEED * DEG2RAD;
    } else if (movetype[5] == NLOW) {
        cPosl.rz = -RSPEED * DEG2RAD;
    } else if (movetype[5] == NHIGH) {
        cPosl.rz = -RSPEED * DEG2RAD;
    }
    if (dof[0] == 1) {
        cPosl.x = cPosl.x;
    } else if (dof[0] == 0) {
        cPosl.x = 0;
    }
    if (dof[1] == 1) {
        cPosl.y = cPosl.y;
    } else if (dof[1] == 0) {
        cPosl.y = 0;
    }
    if (dof[2] == 1) {
        cPosl.z = cPosl.z;
    } else if (dof[2] == 0) {
        cPosl.z = 0;
    }
    if (dof[3] == 1) {
        cPosl.rx = cPosl.rx;
    } else if (dof[3] == 0) {
        cPosl.rx = 0;
    }
    if (dof[4] == 1) {
        cPosl.ry = cPosl.ry;
    } else if (dof[4] == 0) {
        cPosl.ry = 0;
    }
    if (dof[5] == 1) {
        cPosl.rz = cPosl.rz;
    } else if (dof[5] == 0) {
        cPosl.rz = 0;
    }
    Eigen::Vector3d vector_TT, vector_TR, vector_FT, vector_FR;
    Eigen::Matrix3d R_inv, R;
    Eigen::Matrix4d matrix_tool2base = Eigen::Matrix4d::Identity();
    matrix_tool2base = rotation_vector_urpose_to_matrix(*d->fix_line_pos);
    R_inv = get_rotation(matrix_tool2base);
    vector_TT = {cPosl.x, cPosl.y, cPosl.z};
    vector_TR = {cPosl.rx, cPosl.ry, cPosl.rz};
    vector_FT = R_inv * vector_TT;
    vector_FR = R_inv * vector_TR;
    d->speed2flange.x = vector_FT[0];
    d->speed2flange.y = vector_FT[1];
    d->speed2flange.z = vector_FT[2];
    d->speed2flange.rx = vector_FR[0];
    d->speed2flange.ry = vector_FR[1];
    d->speed2flange.rz = vector_FR[2];
    char szCmd[0x100];
    sprintf(szCmd, "speedl([%f, %f, %f, %f, %f, %f], 0.3, 0.2, 2)\n", d->speed2flange.x, d->speed2flange.y, d->speed2flange.z, d->speed2flange.rx, d->speed2flange.ry, d->speed2flange.rz);
    emit signal_send_command(szCmd);
}

void URServer::start_screw_driving_mode() {
    Q_D(URServer);
    char szCmd[0x1000];
    // sprintf(szCmd, "def enter_screw_driving_mode():\n thread myThread1():\n force_mode(tool_pose(), [0, 0, 1, 0, 0, 0], [0, 0, 0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.02, 0.1, 0.1, 0.1])\n sleep(10000)\n return False\n end\n thread myThread2():\n zero_ftsensor()\n sleep(10000)\n return False\n end\n thrd = run myThread1()\n th = run myThread2()\n sleep(100)\nend\n"); // screw_driving(0.0, 0.0)\n sleep(10000)\n
    sprintf(szCmd, "def enter_screw_driving_mode():\n force_mode(tool_pose(), [0, 0, 1, 0, 0, 0], [0, 0, 0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.02, 0.1, 0.1, 0.1])\n sleep(10000)\nend\n");
    emit signal_send_command(szCmd);
}

void URServer::exit_screw_drving_mode() {
    Q_D(URServer);
    char szCmd[0x100];
    sprintf(szCmd, "end_screw_driving()\n");
    emit signal_send_command(szCmd);
}

void URServer::set_tool_digital_out(int index, bool flag) {
    Q_D(URServer);
    char szCmd[0x100];
    if (index == 0 && flag == true) {
        sprintf(szCmd, "def set_tool_IO():\n set_tool_digital_out(0,True)\nend\n");
    } else if (index == 0 && flag == false) {
        sprintf(szCmd, "def set_tool_IO():\n set_tool_digital_out(0,False)\nend\n");
    } else if (index == 1 && flag == true) {
        sprintf(szCmd, "def set_tool_IO():\n set_tool_digital_out(1,True)\nend\n");
    } else if (index == 1 && flag == false) {
        sprintf(szCmd, "def set_tool_IO():\n set_tool_digital_out(1,False)\nend\n");
    }
    emit signal_send_command(szCmd);
}

void URServer::speedj(double j1, double j2, double j3, double j4, double j5, double j6) {
    Q_D(URServer);
    char szCmd[0x100];
    sprintf(szCmd, "speedj([%f, %f, %f, %f, %f, %f], 0.5, 50000)\n", j1, j2, j3, j4, j5, j6);
    emit signal_send_command(szCmd);
}

void URServer::speedl(double x, double y, double z, double rx, double ry, double rz) {
    Q_D(URServer);
    start_singularity_check();
    // start_special_singularity_check();
    Eigen::Vector3d vector_TT, vector_TR, vector_FT, vector_FR;
    Eigen::Matrix3d R_inv, R;
    Eigen::Matrix4d matrix_tool2base;
    matrix_tool2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    R_inv = get_rotation(matrix_tool2base);
    vector_TT = {x, y, z};
    vector_TR = {rx, ry, rz};
    vector_FT = R_inv * vector_TT;
    vector_FR = R_inv * vector_TR;
    CartPos tool2flangespeed;
    tool2flangespeed.x = vector_FT[0];
    tool2flangespeed.y = vector_FT[1];
    tool2flangespeed.z = vector_FT[2];
    tool2flangespeed.rx = vector_FR[0];
    tool2flangespeed.ry = vector_FR[1];
    tool2flangespeed.rz = vector_FR[2];
    char szCmd[0x100];
    sprintf(szCmd, "speedl([%f, %f, %f, %f, %f, %f], 0.3, 50000)\n", tool2flangespeed.x, tool2flangespeed.y, tool2flangespeed.z, tool2flangespeed.rx, tool2flangespeed.ry, tool2flangespeed.rz);
    emit signal_send_command(szCmd);
}

void URServer::slot_restart_robot() {
    Q_D(URServer);
    emit signal_disconnect();
    QThread::msleep(100);
    emit signal_new_connection(29999);
    char szCmd[0x100];
    sprintf(szCmd, "restart safety\n");
    send_command(szCmd);
    QThread::msleep(5000);
    char szCmd1[0x100];
    sprintf(szCmd1, "power on\n");
    send_command(szCmd1);
    QThread::msleep(10000);
    char szCmd2[0x100];
    sprintf(szCmd2, "brake release\n");
    send_command(szCmd2);
    QThread::msleep(10000);
    emit signal_disconnect();
    QThread::msleep(100);
    emit signal_new_connection(30003);
    QThread::msleep(100);
    exit_force_mode();
}

void URServer::slot_get_connect_status(int flag) {
    Q_D(URServer);
    emit signal_send_connect_status(flag);
}

Eigen::Matrix3d URServer::get_rotation(Eigen::Matrix4d T) {
    Eigen::Matrix3d R;
    R << T(0, 0), T(0, 1), T(0, 2),
        T(1, 0), T(1, 1), T(1, 2),
        T(2, 0), T(2, 1), T(2, 2);
    return R;
}

Eigen::Vector3d URServer::get_translation(Eigen::Matrix4d T) {
    Eigen::Vector3d t(T(0, 3), T(1, 3), T(2, 3));
    return t;
}

Eigen::Matrix4d URServer::build_transformation(Eigen::Matrix3d R, Eigen::Vector3d t) {
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out << R(0, 0), R(0, 1), R(0, 2), t[0],
        R(1, 0), R(1, 1), R(1, 2), t[1],
        R(2, 0), R(2, 1), R(2, 2), t[2],
        0, 0, 0, 1;
    return out;
}

// 旋转矢量的位置转矩阵的方法
Eigen::Matrix4d URServer::rotation_vector_urpose_to_matrix(CartPos &cpos) {
    Eigen::Vector3d rotationVector(cpos.rx, cpos.ry, cpos.rz);
    Eigen::AngleAxisd rotation(rotationVector.norm(), rotationVector.normalized());
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    T(0, 3) = cpos.x;
    T(1, 3) = cpos.y;
    T(2, 3) = cpos.z;
    return T;
}

// 旋转矩阵转换成UR位姿的旋转矢量
CartPos URServer::matrix_to_urpose_rotation_vector(Eigen::Matrix4d &transform) {
    Q_D(URServer);
    Eigen::Matrix3d R = transform.block<3, 3>(0, 0);
    Eigen::AngleAxisd rotation_vector;
    rotation_vector.fromRotationMatrix(R);
    CartPos out;
    out.rx = rotation_vector.angle() * rotation_vector.axis()[0];
    out.ry = rotation_vector.angle() * rotation_vector.axis()[1];
    out.rz = rotation_vector.angle() * rotation_vector.axis()[2];
    out.x = transform(0, 3);
    out.y = transform(1, 3);
    out.z = transform(2, 3);
    return out;
}

void URServer::restart_robot_safety() {
    Q_D(URServer);
    emit signal_disconnect();
    QThread::msleep(100);
    emit signal_new_connection(29999);
    char szCmd[0x100];
    sprintf(szCmd, "restart safety\n");
    send_command(szCmd);
    QThread::msleep(100);
    start_robot();
}

void URServer::start_robot() {
    Q_D(URServer);
    emit signal_disconnect();
    QThread::msleep(100);
    emit signal_new_connection(29999);
    QThread::msleep(1000);
    char szCmd1[0x100];
    sprintf(szCmd1, "power on\n");
    send_command(szCmd1);
    QThread::msleep(12000);
    char szCmd2[0x100];
    sprintf(szCmd2, "brake release\n");
    send_command(szCmd2);
    QThread::msleep(8000);
    char szCmd[0x100];
    sprintf(szCmd, "unlock protective stop\n");
    send_command(szCmd);
    QThread::msleep(100);
    emit signal_disconnect();
    QThread::msleep(100);
    emit signal_new_connection(30003);
    QThread::msleep(100);
    // set_robot_force_damping(0.1);
    if (!d->detect_robot_pos_timer->isActive()) {
        d->detect_robot_pos_timer->start(100);
    }
    // emit signal_connect_30002_port();
}

void URServer::shutdown_robot() {
    Q_D(URServer);
    emit signal_disconnect();
    QThread::msleep(100);
    emit signal_new_connection(29999);
    QThread::msleep(1000);
    char szCmd[0x100];
    sprintf(szCmd, "power off\n");
    send_command(szCmd);
}

void URServer::connect_to_robot() {
    Q_D(URServer);
    emit signal_disconnect();
    emit signal_new_connection(30003);
    QThread::msleep(500);
    // emit signal_connect_30002_port();
}

void URServer::unlock_protective_stop() {
    Q_D(URServer);
    emit signal_disconnect();
    QThread::msleep(100);
    emit signal_new_connection(29999);
    char szCmd[0x100];
    sprintf(szCmd, "unlock protective stop\n");
    send_command(szCmd);
    QThread::msleep(100);
    emit signal_disconnect();
    emit signal_new_connection(30003);
}

void URServer::move_robot_joint_space(JointPos *js) {
    Q_D(URServer);
    stop_singularity_check();
    stop_special_singularity_check();
    stop_fixpoint_singularity_check();
    char szCmd[0x100];
    sprintf(szCmd, "movej([%f, %f, %f, %f, %f, %f], a=1.2, v=0.2, t=0, r=0)\n", js->j1 * DEG2RAD, js->j2 * DEG2RAD, js->j3 * DEG2RAD, js->j4 * DEG2RAD, js->j5 * DEG2RAD, js->j6 * DEG2RAD);
    emit signal_send_command(szCmd);
}

void URServer::move_robot_cart_space(CartPos *cs) {
    Q_D(URServer);
    char szCmd[0x100];
    sprintf(szCmd, "movel(p[%f, %f, %f, %f, %f, %f], a=0.5, v=0.01, t=0, r=0)\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
    emit signal_send_command(szCmd);
}

void URServer::stop_robot_movement() {
    Q_D(URServer);
    stop_singularity_check();
    stop_special_singularity_check();
    stop_fixpoint_singularity_check();
    d->isEnableFixLine = false;
    if (d->force_timer->isActive()) {
        d->force_type = NO_ACTION;
        d->force_timer->stop();
        char szCmd[0x100];
        sprintf(szCmd, "end_force_mode()\n");
        emit signal_send_command(szCmd);
    } else {
        char szCmd[0x100];
        sprintf(szCmd, "def stop_movel():\n stopl(5)\nend\n");
        emit signal_send_command(szCmd);
    }
}

void URServer::stop_robot_movement_and_sleep() {
    Q_D(URServer);
    stop_singularity_check();
    stop_special_singularity_check();
    stop_fixpoint_singularity_check();
    d->isEnableFixLine = false;
    if (d->force_timer->isActive()) {
        d->force_type = NO_ACTION;
        d->force_timer->stop();
        char szCmd[0x100];
        sprintf(szCmd, "end_force_mode()\n");
        emit signal_send_command(szCmd);
    } else {
        char szCmd[0x100];
        sprintf(szCmd, "def stop_movel():\n stopl(5)\n sleep(10000)\nend\n");
        emit signal_send_command(szCmd);
    }
}

void URServer::set_tool_load(double m, double x, double y, double z) {
    Q_D(URServer);
    char szCmd[0x100];
    sprintf(szCmd, "set_target_payload(%f,[%f, %f, %f], inertia=[0, 0, 0, 0, 0, 0])\n", m, x, y, z);
    emit signal_send_command(szCmd);
}

void URServer::set_tool_offset(CartPos *cs) {
    Q_D(URServer);
    if (cs->x == 0 && cs->y == 0 && cs->z == 0) {
        d->mat_tool2flange = Eigen::Matrix4d::Identity();
        d->mat_tool2flangetop = Eigen::Matrix4d::Identity();
        d->mat_tool2flangetop(2, 3) = d->flangetop2bottom;
    } else {
        d->mat_tool2flange = rotation_vector_urpose_to_matrix(*cs);
        d->mat_tool2flangetop = d->mat_tool2flange;
        d->mat_tool2flangetop(2, 3) = d->mat_tool2flangetop(2, 3) + d->flangetop2bottom;
    }
    QTimer::singleShot(50, this, [&] {
        slot_calculate_flange2base();
    });
    char szCmd[0x100];
    sprintf(szCmd, "set_tcp(p[%f, %f, %f, %f, %f, %f])\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
    emit signal_send_command(szCmd);
}

void URServer::set_robot_max_speed_t(double max_speed_t) {
    Q_D(URServer);
    d->speed_t = max_speed_t;
}

void URServer::set_robot_max_speed_r(double max_speed_r) {
    Q_D(URServer);
    d->speed_r = max_speed_r;
}

void URServer::slot_calculate_flange2base() {
    Q_D(URServer);
    d->mat_tool2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    d->mat_flange2base = d->mat_tool2base * d->mat_tool2flange.inverse();
    Eigen::Matrix4d mat_flangetop2base = Eigen::Matrix4d::Identity();
    mat_flangetop2base = d->mat_tool2base * d->mat_tool2flangetop.inverse();
    CartPos current_flange2base = matrix_to_urpose_rotation_vector(d->mat_flange2base);
    CartPos current_flangetop2base = matrix_to_urpose_rotation_vector(mat_flangetop2base);
    d->flangebottom2base = sqrt(current_flange2base.x * current_flange2base.x + current_flange2base.y * current_flange2base.y);
    d->flangetop2base = sqrt(current_flangetop2base.x * current_flangetop2base.x + current_flangetop2base.y * current_flangetop2base.y);
    //    qDebug() << "x: " << current_flange2base.x << " y: " << current_flange2base.y << " dis: " << d->flangebottom2base;
    //    qDebug() << "x: " << current_flangetop2base.x << " y: " << current_flangetop2base.y << " dis: " << d->flangetop2base;
}

void URServer::start_freedrive_mode() {
    Q_D(URServer);
    stop_singularity_check();
    stop_special_singularity_check();
    stop_fixpoint_singularity_check();
    d->isEnableFixLine = false;
    char szCmd[0x100];
    // sprintf(szCmd, "def get_target_payload_cog():\n a = get_target_payload()\n socket_open(\"172.31.1.100\", 5000, \"test\")\n socket_send_string(a, \"test\")\n socket_close(\"test\")\nend\n");
    sprintf(szCmd, "def enter_teach_mode():\n freedrive_mode()\n sleep(50000)\nend\n");
    // sprintf(szCmd, "def enter_teach_mode():\n freedrive_mode(freeAxes=[0,0,0,1,1,1], feature=\"tool\")\n sleep(50000)\nend\n");
    emit signal_send_command(szCmd);
}

void URServer::move_robot_to_target_pos(CartPos *cs) {
    Q_D(URServer);
    start_singularity_check();
    // start_special_singularity_check();
    d->target_pos = cs;
    char szCmd[0x100];
    // sprintf(szCmd, "movel(p[%f, %f, %f, %f, %f, %f], a=1.2, v=0.01, t=0, r=0)\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
    sprintf(szCmd, "movel(p[%f, %f, %f, %f, %f, %f], a=1.2, v=0.03, t=0, r=0)\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
    // sprintf(szCmd, "movel(p[%f, %f, %f, %f, %f, %f], a=1.2, v=0.005, t=0, r=0)\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
    emit signal_send_command(szCmd);
}

void URServer::servo_target_joint(JointPos *js) {
    Q_D(URServer);
    char szCmd[0x100];
    // sprintf(szCmd, "servoj([%f, %f, %f, %f, %f, %f], 0,0,0.002, 0.1, 300)\n", js->j1 * DEG2RAD, js->j2 * DEG2RAD, js->j3 * DEG2RAD, js->j4 * DEG2RAD, js->j5 * DEG2RAD, js->j6 * DEG2RAD);
    sprintf(szCmd, "servoj([%f, %f, %f, %f, %f, %f])\n", js->j1 * DEG2RAD, js->j2 * DEG2RAD, js->j3 * DEG2RAD, js->j4 * DEG2RAD, js->j5 * DEG2RAD, js->j6 * DEG2RAD);
    emit signal_send_command(szCmd);
}

void URServer::move_robot_to_target_pos_within_time(CartPos *cs) {
    Q_D(URServer);
    start_singularity_check();
    // start_special_singularity_check();
    d->target_pos = cs;
    char szCmd[0x100];
    sprintf(szCmd, "movel(p[%f, %f, %f, %f, %f, %f], a=1.2, v=0.01, t=0.09, r=0)\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
    emit signal_send_command(szCmd);
}

void URServer::move_robot_to_hip_wall(CartPos *cs) {
    Q_D(URServer);
    start_singularity_check();
    // start_special_singularity_check();
    d->target_pos = cs;
    char szCmd[0x100];
    sprintf(szCmd, "movel(p[%f, %f, %f, %f, %f, %f], a=1.2, v=0.01, t=0.1, r=0)\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
    emit signal_send_command(szCmd);
}

void URServer::set_virtual_wall_pos(CartPos *cs) {
    Q_D(URServer);
    d->virtual_pos = cs;
    d->matrix_virtual_wall_pos = rotation_vector_urpose_to_matrix(*cs);
    d->vector_original_normal[0] = d->matrix_virtual_wall_pos(0, 2);
    d->vector_original_normal[1] = d->matrix_virtual_wall_pos(1, 2);
    d->vector_original_normal[2] = d->matrix_virtual_wall_pos(2, 2);
    if (abs(d->vector_original_normal[0]) >= abs(d->vector_original_normal[1])) {
        if (abs(d->vector_original_normal[0]) >= abs(d->vector_original_normal[2])) {
            d->virtual_axis = 1;
        } else {
            d->virtual_axis = 3;
        }
    } else {
        if (abs(d->vector_original_normal[1]) >= abs(d->vector_original_normal[2])) {
            d->virtual_axis = 2;
        } else {
            d->virtual_axis = 3;
        }
    }
}

void URServer::start_fixedline_mode(CartPos *cs) {
    Q_D(URServer);
    d->fix_line_pos = cs;
    if ((d->target_pos->x == 0 && d->target_pos->y == 0 && d->target_pos->z == 0) || (d->virtual_pos->x == 0 && d->virtual_pos->y == 0 && d->virtual_pos->z == 0)) {
        char szCmd[0x100];
        sprintf(szCmd, "def enter_force_mode():\n force_mode(tool_pose(), [0, 0, 1, 0, 0, 0], [0, 0, 0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.02, 0.1, 0.1, 0.1])\n sleep(10000)\nend\n");
        emit signal_send_command(szCmd);
    } else {
        if (d->virtual_axis == 1) {
            if (d->target_pos->x > d->virtual_pos->x) {
                if (d->current_tool_pos.x - d->virtual_pos->x < 0.1) {
                    d->force_timer->start(10);
                    d->isEnableFixLine = false;
                } else {
                    d->force_timer->stop();
                    char szCmd[0x100];
                    sprintf(szCmd, "def enter_force_mode():\n force_mode(p[%f, %f, %f, %f, %f, %f], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.02, 0.1, 0.1, 0.1])\n sleep(10000)\nend\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
                    emit signal_send_command(szCmd);
                    d->isEnableFixLine = true;
                }
            } else {
                if (d->virtual_pos->x - d->current_tool_pos.x < 0.1) {
                    d->force_timer->start(10);
                    d->isEnableFixLine = false;
                } else {
                    d->force_timer->stop();
                    char szCmd[0x100];
                    sprintf(szCmd, "def enter_force_mode():\n force_mode(p[%f, %f, %f, %f, %f, %f], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.02, 0.1, 0.1, 0.1])\n sleep(10000)\nend\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
                    emit signal_send_command(szCmd);
                    d->isEnableFixLine = true;
                }
            }
        } else if (d->virtual_axis == 2) {
            if (d->target_pos->y > d->virtual_pos->y) {
                if (d->current_tool_pos.y - d->virtual_pos->y < 0.1) {
                    d->force_timer->start(10);
                    d->isEnableFixLine = false;
                } else {
                    d->force_timer->stop();
                    char szCmd[0x100];
                    sprintf(szCmd, "def enter_force_mode():\n force_mode(p[%f, %f, %f, %f, %f, %f], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.02, 0.1, 0.1, 0.1])\n sleep(10000)\nend\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
                    emit signal_send_command(szCmd);
                    d->isEnableFixLine = true;
                }
            } else {
                if (d->virtual_pos->y - d->current_tool_pos.y < 0.1) {
                    d->force_timer->start(10);
                    d->isEnableFixLine = false;
                } else {
                    d->force_timer->stop();
                    char szCmd[0x100];
                    sprintf(szCmd, "def enter_force_mode():\n force_mode(p[%f, %f, %f, %f, %f, %f], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.02, 0.1, 0.1, 0.1])\n sleep(10000)\nend\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
                    emit signal_send_command(szCmd);
                    d->isEnableFixLine = true;
                }
            }
        } else if (d->virtual_axis == 3) {
            if (d->target_pos->z > d->virtual_pos->z) {
                if (d->current_tool_pos.z - d->virtual_pos->z < 0.1) {
                    d->force_timer->start(10);
                    d->isEnableFixLine = false;
                } else {
                    d->force_timer->stop();
                    char szCmd[0x100];
                    sprintf(szCmd, "def enter_force_mode():\n force_mode(p[%f, %f, %f, %f, %f, %f], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.02, 0.1, 0.1, 0.1])\n sleep(10000)\nend\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
                    emit signal_send_command(szCmd);
                    d->isEnableFixLine = true;
                }
            } else {
                if (d->virtual_pos->z - d->current_tool_pos.z < 0.1) {
                    d->force_timer->start(10);
                    d->isEnableFixLine = false;
                } else {
                    d->force_timer->stop();
                    char szCmd[0x100];
                    sprintf(szCmd, "def enter_force_mode():\n force_mode(p[%f, %f, %f, %f, %f, %f], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.02, 0.1, 0.1, 0.1])\n sleep(10000)\nend\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz);
                    emit signal_send_command(szCmd);
                    d->isEnableFixLine = true;
                }
            }
        }
    }
}

void URServer::handle_fixline_virtual_wall() {
    Q_D(URServer);
    if (d->virtual_axis == 1) {
        if ((d->virtual_pos->x > d->target_pos->x && d->current_tool_pos.x >= d->virtual_pos->x) ||
            (d->virtual_pos->x < d->target_pos->x && d->current_tool_pos.x <= d->virtual_pos->x)) {
            exit_force_mode();
            d->isEnableFixLine = false;
            d->force_timer->start(10);
        }
    } else if (d->virtual_axis == 2) {
        if ((d->virtual_pos->y > d->target_pos->y && d->current_tool_pos.y >= d->virtual_pos->y) ||
            (d->virtual_pos->y < d->target_pos->y && d->current_tool_pos.y <= d->virtual_pos->y)) {
            exit_force_mode();
            d->isEnableFixLine = false;
            d->force_timer->start(10);
        }
    } else if (d->virtual_axis == 3) {
        if ((d->virtual_pos->z > d->target_pos->z && d->current_tool_pos.z >= d->virtual_pos->z) ||
            (d->virtual_pos->z < d->target_pos->z && d->current_tool_pos.z <= d->virtual_pos->z)) {
            exit_force_mode();
            d->isEnableFixLine = false;
            d->force_timer->start(10);
        }
    }
}

void URServer::start_fixplane_mode(CartPos *target2base, int x, int y, int z, int rx, int ry, int rz) {
    Q_D(URServer);
    // set_robot_force_damping(1.0);
    // QThread::msleep(100);
    // start_singularity_check();
    // char szCmd[0x100];
    // sprintf(szCmd, "def enter_force_mode():\n force_mode(p[%f, %f, %f, %f, %f, %f], [%i, %i, %i, %i, %i, %i], [0, 0, 0, 0.0, 0.0, 0.0], 2, [0.001, 0.001, 0.001, 0.1, 0.1, 0.1])\n sleep(10000)\nend\n",
    //         target2base->x / 1000, target2base->y / 1000, target2base->z / 1000, target2base->rx, target2base->ry, target2base->rz, x, y, z, rx, ry, rz);
    // emit signal_send_command(szCmd);
    Eigen::Matrix4d mat_target2base = rotation_vector_urpose_to_matrix(*target2base);
    Eigen::Matrix4d mat_current2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    Eigen::Matrix4d mat_target2current = mat_current2base.inverse() * mat_target2base;
    CartPos target2current = matrix_to_urpose_rotation_vector(mat_target2current);
    CartPos speedl_pos;
    double offset_pos_t = sqrt(target2current.x * target2current.x + target2current.y * target2current.y + target2current.z * target2current.z);
    double controlSig_speed_t = calculate_pid_speed_t_iterate(offset_pos_t);
    double offset_pos_r = sqrt(target2current.rx * target2current.rx + target2current.ry * target2current.ry + target2current.rz * target2current.rz);
    double controlSig_speed_r = calculate_pid_speed_r_iterate(offset_pos_r);
    speedl_pos.x = (target2current.x / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.y = (target2current.y / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.z = (target2current.z / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.rx = (target2current.rx / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    speedl_pos.ry = (target2current.ry / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    speedl_pos.rz = (target2current.rz / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    start_fixplane_follow_movement(target2base, &speedl_pos, x, y, z, rx, ry, rz);
}

void URServer::start_fixplane_follow_movement(CartPos *force_pos, CartPos *speed_pos, int x, int y, int z, int rx, int ry, int rz) {
    Q_D(URServer);
    Eigen::Vector3d vector_TT, vector_TR, vector_FT, vector_FR;
    Eigen::Matrix3d R_inv, R;
    Eigen::Matrix4d matrix_tool2base;
    matrix_tool2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    R_inv = get_rotation(matrix_tool2base);
    vector_TT = {speed_pos->x, speed_pos->y, speed_pos->z};
    vector_TR = {speed_pos->rx, speed_pos->ry, speed_pos->rz};
    vector_FT = R_inv * vector_TT;
    vector_FR = R_inv * vector_TR;
    CartPos speed;
    speed.x = vector_FT[0];
    speed.y = vector_FT[1];
    speed.z = vector_FT[2];
    speed.rx = vector_FR[0];
    speed.ry = vector_FR[1];
    speed.rz = vector_FR[2];
    start_singularity_check();
    char szCmd[0x1000];
    // sprintf(szCmd, "def start_fixplane_follow_move():\n while (True):\n force_mode(p[%f,%f,%f,%f,%f,%f], [%i,%i,%i,%i,%i,%i], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [0.1,0.1,0.1,0.1,0.1,0.1])\n speedl([%f,%f,%f,%f,%f,%f],1.0,0.2,2.0)\n sync()\n end\n end_force_mode()\nend\n",
    //         force_pos->x / 1000, force_pos->y / 1000, force_pos->z / 1000, force_pos->rx, force_pos->ry, force_pos->rz, x, y, z, rx, ry, rz, speed.x / 1000, speed.y / 1000, speed.z / 1000, speed.rx, speed.ry, speed.rz);
    sprintf(szCmd, "def start_fixplane_follow_move():\n while (True):\n force_mode(p[%f,%f,%f,%f,%f,%f], [%i,%i,%i,%i,%i,%i], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [0.03,0.03,0.03,0.03,0.03,0.03])\n speedl([%f,%f,%f,%f,%f,%f],1.0,0.2,2.0)\n sync()\n end\n end_force_mode()\nend\n",
            force_pos->x / 1000, force_pos->y / 1000, force_pos->z / 1000, force_pos->rx, force_pos->ry, force_pos->rz, x, y, z, rx, ry, rz, speed.x / 1000, speed.y / 1000, speed.z / 1000, speed.rx, speed.ry, speed.rz);
    emit signal_send_command(szCmd);
}

void URServer::start_fixplane_mode_with_speed(CartPos *target2base, int x, int y, int z, int rx, int ry, int rz, double speed_plane) {
    Q_D(URServer);
    // set_robot_force_damping(1.0);
    // QThread::msleep(100);
    // start_singularity_check();
    // char szCmd[0x100];
    // sprintf(szCmd, "def enter_force_mode():\n force_mode(p[%f, %f, %f, %f, %f, %f], [%i, %i, %i, %i, %i, %i], [0, 0, 0, 0.0, 0.0, 0.0], 2, [0.001, 0.001, 0.001, 0.1, 0.1, 0.1])\n sleep(10000)\nend\n",
    //         target2base->x / 1000, target2base->y / 1000, target2base->z / 1000, target2base->rx, target2base->ry, target2base->rz, x, y, z, rx, ry, rz);
    // emit signal_send_command(szCmd);
    Eigen::Matrix4d mat_target2base = rotation_vector_urpose_to_matrix(*target2base);
    Eigen::Matrix4d mat_current2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    Eigen::Matrix4d mat_target2current = mat_current2base.inverse() * mat_target2base;
    CartPos target2current = matrix_to_urpose_rotation_vector(mat_target2current);
    CartPos speedl_pos;
    double offset_pos_t = sqrt(target2current.x * target2current.x + target2current.y * target2current.y + target2current.z * target2current.z);
    double controlSig_speed_t = calculate_pid_speed_t_iterate(offset_pos_t);
    double offset_pos_r = sqrt(target2current.rx * target2current.rx + target2current.ry * target2current.ry + target2current.rz * target2current.rz);
    double controlSig_speed_r = calculate_pid_speed_r_iterate(offset_pos_r);
    speedl_pos.x = (target2current.x / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.y = (target2current.y / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.z = (target2current.z / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.rx = (target2current.rx / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    speedl_pos.ry = (target2current.ry / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    speedl_pos.rz = (target2current.rz / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    start_fixplane_follow_movement_with_speed(target2base, &speedl_pos, x, y, z, rx, ry, rz, speed_plane);
}

void URServer::start_fixplane_follow_movement_with_speed(CartPos *force_pos, CartPos *speed_pos, int x, int y, int z, int rx, int ry, int rz, double speed_plane) {
    Q_D(URServer);
    Eigen::Vector3d vector_TT, vector_TR, vector_FT, vector_FR;
    Eigen::Matrix3d R_inv, R;
    Eigen::Matrix4d matrix_tool2base;
    matrix_tool2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    R_inv = get_rotation(matrix_tool2base);
    vector_TT = {speed_pos->x, speed_pos->y, speed_pos->z};
    vector_TR = {speed_pos->rx, speed_pos->ry, speed_pos->rz};
    vector_FT = R_inv * vector_TT;
    vector_FR = R_inv * vector_TR;
    CartPos speed;
    speed.x = vector_FT[0];
    speed.y = vector_FT[1];
    speed.z = vector_FT[2];
    speed.rx = vector_FR[0];
    speed.ry = vector_FR[1];
    speed.rz = vector_FR[2];
    start_singularity_check();

    double speed_x = x == 0 ? 0.03 : speed_plane;
    double speed_y = y == 0 ? 0.03 : speed_plane;
    double speed_z = z == 0 ? 0.03 : speed_plane;
    char szCmd[0x1000];
    sprintf(szCmd, "def start_fixplane_follow_move():\n while (True):\n force_mode(p[%f,%f,%f,%f,%f,%f], [%i,%i,%i,%i,%i,%i], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [%f,%f,%f,0.1,0.1,0.1])\n speedl([%f,%f,%f,%f,%f,%f],1.0,0.2,2.0)\n sync()\n end\n end_force_mode()\nend\n",
            force_pos->x / 1000, force_pos->y / 1000, force_pos->z / 1000, force_pos->rx, force_pos->ry, force_pos->rz, x, y, z, rx, ry, rz, speed_x, speed_y, speed_z, speed.x / 1000, speed.y / 1000, speed.z / 1000, speed.rx, speed.ry, speed.rz);
    emit signal_send_command(szCmd);
}

void URServer::start_fixline_movement(CartPos *cs, double speed_z) {
    Q_D(URServer);
    start_singularity_check();
    char szCmd[0x100];
    sprintf(szCmd, "def start_fixline_move():\n while (True):\n global p1=p[%f,%f,%f,%f,%f,%f]\n force_mode(p1, [0,0,1,0,0,0], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [0.1,0.1,%f,0.1,0.1,0.1])\n sync()\n end\n end_force_mode()\nend\n", cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz, speed_z / 1000.0);
    emit signal_send_command(szCmd);
}

void URServer::start_fixline_follow_movement(CartPos *cs, double speed_z, int x, int y, int z) {
    Q_D(URServer);
    start_singularity_check();
    char szCmd[0x1000];
    if (x == 1) {
        sprintf(szCmd, "def start_fixline_follow_move():\n while (True):\n global p1=p[%f,%f,%f,%f,%f,%f]\n force_mode(p1, [%i,%i,%i,0,0,0], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [%f,0.1,0.1,0.1,0.1,0.1])\n movel(p1,a=0.3,v=0.05,t=0.25)\n sync()\n end\n end_force_mode()\nend\n",
                cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz, x, y, z, speed_z / 1000.0);
    } else if (y == 1) {
        sprintf(szCmd, "def start_fixline_follow_move():\n while (True):\n global p1=p[%f,%f,%f,%f,%f,%f]\n force_mode(p1, [%i,%i,%i,0,0,0], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [0.1,%f,0.1,0.1,0.1,0.1])\n movel(p1,a=0.3,v=0.05,t=0.25)\n sync()\n end\n end_force_mode()\nend\n",
                cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz, x, y, z, speed_z / 1000.0);
    } else if (z == 1) {
        sprintf(szCmd, "def start_fixline_follow_move():\n while (True):\n global p1=p[%f,%f,%f,%f,%f,%f]\n force_mode(p1, [%i,%i,%i,0,0,0], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [0.1,0.1,%f,0.1,0.1,0.1])\n movel(p1,a=0.3,v=0.05,t=0.25)\n sync()\n end\n end_force_mode()\nend\n",
                cs->x / 1000, cs->y / 1000, cs->z / 1000, cs->rx, cs->ry, cs->rz, x, y, z, speed_z / 1000.0);
    }
    emit signal_send_command(szCmd);
}

void URServer::move_to_target_pos_with_speedl(CartPos *target2base) {
    Q_D(URServer);
    Eigen::Matrix4d mat_target2base = rotation_vector_urpose_to_matrix(*target2base);
    Eigen::Matrix4d mat_current2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    Eigen::Matrix4d mat_target2current = mat_current2base.inverse() * mat_target2base;
    CartPos target2current = matrix_to_urpose_rotation_vector(mat_target2current);
    // qDebug() << "target2current.x: " << target2current.x << "y: " << target2current.y << "z: " << target2current.z;
    CartPos speedl_pos;
    double offset_pos_t = sqrt(target2current.x * target2current.x + target2current.y * target2current.y + target2current.z * target2current.z);
    double controlSig_speed_t = calculate_pid_speed_t_iterate(offset_pos_t);
    double offset_pos_r = sqrt(target2current.rx * target2current.rx + target2current.ry * target2current.ry + target2current.rz * target2current.rz);
    double controlSig_speed_r = calculate_pid_speed_r_iterate(offset_pos_r);
    speedl_pos.x = (target2current.x / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.y = (target2current.y / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.z = (target2current.z / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.rx = (target2current.rx / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    speedl_pos.ry = (target2current.ry / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    speedl_pos.rz = (target2current.rz / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    move_robot_to_target_pos_with_speedl(&speedl_pos);
}

void URServer::move_robot_to_target_pos_with_speedl(CartPos *speed_pos) {
    Q_D(URServer);
    Eigen::Vector3d vector_TT, vector_TR, vector_FT, vector_FR;
    Eigen::Matrix3d R_inv, R;
    Eigen::Matrix4d matrix_tool2base;
    matrix_tool2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    R_inv = get_rotation(matrix_tool2base);
    vector_TT = {speed_pos->x, speed_pos->y, speed_pos->z};
    vector_TR = {speed_pos->rx, speed_pos->ry, speed_pos->rz};
    vector_FT = R_inv * vector_TT;
    vector_FR = R_inv * vector_TR;
    CartPos speed;
    speed.x = vector_FT[0];
    speed.y = vector_FT[1];
    speed.z = vector_FT[2];
    speed.rx = vector_FR[0];
    speed.ry = vector_FR[1];
    speed.rz = vector_FR[2];
    start_singularity_check();
    // start_special_singularity_check();
    char szCmd[0x100];
    sprintf(szCmd, "def move_to_target_pos_with_speedl():\n while (True):\n speedl([%f,%f,%f,%f,%f,%f], 1.0, 2.0, 2.0)\n sync()\n end\nend\n",
            speed.x / 1000, speed.y / 1000, speed.z / 1000, speed.rx, speed.ry, speed.rz);
    emit signal_send_command(szCmd);
}

void URServer::start_forcemode_speedl_follow_movement(CartPos *target2base, double rest_length, int axis) {
    Q_D(URServer);
    Eigen::Matrix4d mat_target2base = rotation_vector_urpose_to_matrix(*target2base);
    Eigen::Matrix4d mat_current2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    Eigen::Matrix4d mat_target2current = mat_current2base.inverse() * mat_target2base;
    CartPos target2current = matrix_to_urpose_rotation_vector(mat_target2current);
    CartPos speedl_pos;
    double offset_pos_t = sqrt(target2current.x * target2current.x + target2current.y * target2current.y + target2current.z * target2current.z);
    double controlSig_speed_t = calculate_pid_speed_t_iterate(offset_pos_t);
    double offset_pos_r = sqrt(target2current.rx * target2current.rx + target2current.ry * target2current.ry + target2current.rz * target2current.rz);
    double controlSig_speed_r = calculate_pid_speed_r_iterate(offset_pos_r);
    if (offset_pos_t < 1e-6) {
        speedl_pos.x = 0;
        speedl_pos.y = 0;
        speedl_pos.z = 0;
    } else {
        speedl_pos.x = (target2current.x / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
        speedl_pos.y = (target2current.y / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
        speedl_pos.z = (target2current.z / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    }
    if (offset_pos_r < 1e-6) {
        speedl_pos.rx = 0;
        speedl_pos.ry = 0;
        speedl_pos.rz = 0;
    } else {
        speedl_pos.rx = (target2current.rx / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
        speedl_pos.ry = (target2current.ry / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
        speedl_pos.rz = (target2current.rz / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    }
    double distance_to_bottom = std::max(rest_length, 0.0);
    double control_fixline_speed = calculate_pid_fixline_speed_iterate(distance_to_bottom);
    double actual_fixline_speed = std::min(control_fixline_speed, d->max_fixline_speed);
    start_forcemode_speedl_movement(target2base, actual_fixline_speed, axis, &speedl_pos);
}

void URServer::start_forcemode_speedlpos_movement(CartPos *target2base, double rest_length, int axis) {
    Q_D(URServer);
    Eigen::Matrix4d mat_target2base = rotation_vector_urpose_to_matrix(*target2base);
    Eigen::Matrix4d mat_current2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    Eigen::Matrix4d mat_target2current = mat_current2base.inverse() * mat_target2base;
    CartPos target2current = matrix_to_urpose_rotation_vector(mat_target2current);
    CartPos speedl_pos;
    double offset_pos_t = sqrt(target2current.x * target2current.x + target2current.y * target2current.y + target2current.z * target2current.z);
    double controlSig_speed_t = calculate_pid_speed_t_iterate(offset_pos_t);
    double offset_pos_r = sqrt(target2current.rx * target2current.rx + target2current.ry * target2current.ry + target2current.rz * target2current.rz);
    double controlSig_speed_r = calculate_pid_speed_r_iterate(offset_pos_r);
    speedl_pos.x = (target2current.x / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.y = (target2current.y / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.z = (target2current.z / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.rx = 0; //(target2current.rx / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    speedl_pos.ry = 0; //(target2current.ry / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    speedl_pos.rz = 0; //(target2current.rz / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    double distance_to_bottom = std::max(rest_length, 0.0);
    double control_fixline_speed = calculate_pid_fixline_speed_iterate(distance_to_bottom);
    double actual_fixline_speed = std::min(control_fixline_speed, d->max_fixline_speed);
    start_forcemode_speedl_movement(target2base, actual_fixline_speed, axis, &speedl_pos);
}

void URServer::start_forcemode_speedl_movement(CartPos *force_pos, double speed_z, int axis, CartPos *speed_pos) {
    Q_D(URServer);
    Eigen::Vector3d vector_TT, vector_TR, vector_FT, vector_FR;
    Eigen::Matrix3d R_inv, R;
    Eigen::Matrix4d matrix_tool2base;
    matrix_tool2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    R_inv = get_rotation(matrix_tool2base);
    vector_TT = {speed_pos->x, speed_pos->y, speed_pos->z};
    vector_TR = {speed_pos->rx, speed_pos->ry, speed_pos->rz};
    vector_FT = R_inv * vector_TT;
    vector_FR = R_inv * vector_TR;
    CartPos speed;
    speed.x = vector_FT[0];
    speed.y = vector_FT[1];
    speed.z = vector_FT[2];
    speed.rx = vector_FR[0];
    speed.ry = vector_FR[1];
    speed.rz = vector_FR[2];
    start_singularity_check();
    char szCmd[0x1000];
    if (axis == 1) {
        sprintf(szCmd, "def start_forcemode_speedl_move():\n while (True):\n force_mode(p[%f,%f,%f,%f,%f,%f], [1,0,0,0,0,0], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [%f,0.1,0.1,0.1,0.1,0.1])\n speedl([%f,%f,%f,%f,%f,%f],1.0,0.2,2.0)\n sync()\n end\n end_force_mode()\nend\n",
                force_pos->x / 1000, force_pos->y / 1000, force_pos->z / 1000, force_pos->rx, force_pos->ry, force_pos->rz, speed_z / 1000.0, speed.x / 1000, speed.y / 1000, speed.z / 1000, speed.rx, speed.ry, speed.rz);
    } else if (axis == 2) {
        sprintf(szCmd, "def start_forcemode_speedl_move():\n while (True):\n force_mode(p[%f,%f,%f,%f,%f,%f], [0,1,0,0,0,0], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [0.1,%f,0.1,0.1,0.1,0.1])\n speedl([%f,%f,%f,%f,%f,%f],1.0,0.2,2.0)\n sync()\n end\n end_force_mode()\nend\n",
                force_pos->x / 1000, force_pos->y / 1000, force_pos->z / 1000, force_pos->rx, force_pos->ry, force_pos->rz, speed_z / 1000.0, speed.x / 1000, speed.y / 1000, speed.z / 1000, speed.rx, speed.ry, speed.rz);
    } else if (axis == 3) {
        sprintf(szCmd, "def start_forcemode_speedl_move():\n while (True):\n force_mode(p[%f,%f,%f,%f,%f,%f], [0,0,1,0,0,0], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [0.1,0.1,%f,0.1,0.1,0.1])\n speedl([%f,%f,%f,%f,%f,%f],1.0,0.2,2.0)\n sync()\n end\n end_force_mode()\nend\n",
                force_pos->x / 1000, force_pos->y / 1000, force_pos->z / 1000, force_pos->rx, force_pos->ry, force_pos->rz, speed_z / 1000.0, speed.x / 1000, speed.y / 1000, speed.z / 1000, speed.rx, speed.ry, speed.rz);
    }
    emit signal_send_command(szCmd);
}

void URServer::start_forcemode_fixpoint_follow_movement(CartPos *target2base, double rest_length, int axis) {
    Q_D(URServer);
    Eigen::Matrix4d mat_target2base = rotation_vector_urpose_to_matrix(*target2base);
    Eigen::Matrix4d mat_current2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    Eigen::Matrix4d mat_target2current = mat_current2base.inverse() * mat_target2base;
    CartPos target2current = matrix_to_urpose_rotation_vector(mat_target2current);
    CartPos speedl_pos;
    double offset_pos_t = sqrt(target2current.x * target2current.x + target2current.y * target2current.y + target2current.z * target2current.z);
    double controlSig_speed_t = calculate_pid_speed_t_iterate(offset_pos_t);
    double offset_pos_r = sqrt(target2current.rx * target2current.rx + target2current.ry * target2current.ry + target2current.rz * target2current.rz);
    double controlSig_speed_r = calculate_pid_speed_r_iterate(offset_pos_r);
    speedl_pos.x = (target2current.x / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.y = (target2current.y / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.z = (target2current.z / offset_pos_t) * std::min(controlSig_speed_t, d->speed_t);
    speedl_pos.rx = (target2current.rx / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    speedl_pos.ry = (target2current.ry / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    speedl_pos.rz = (target2current.rz / offset_pos_r) * std::min(controlSig_speed_r, d->speed_r);
    double distance_to_bottom = std::max(rest_length, 0.0);
    double control_fixline_speed = calculate_pid_fixline_speed_iterate(distance_to_bottom);
    double actual_fixline_speed = std::min(control_fixline_speed, d->max_fixline_speed);
    start_forcemode_fixpoint_movement(target2base, actual_fixline_speed, axis, &speedl_pos);
}

void URServer::start_forcemode_fixpoint_movement(CartPos *force_pos, double speed_z, int axis, CartPos *speed_pos) {
    Q_D(URServer);
    Eigen::Vector3d vector_TT, vector_TR, vector_FT, vector_FR;
    Eigen::Matrix3d R_inv, R;
    Eigen::Matrix4d matrix_tool2base;
    matrix_tool2base = rotation_vector_urpose_to_matrix(d->current_tool_pos);
    R_inv = get_rotation(matrix_tool2base);
    vector_TT = {speed_pos->x, speed_pos->y, speed_pos->z};
    vector_TR = {speed_pos->rx, speed_pos->ry, speed_pos->rz};
    vector_FT = R_inv * vector_TT;
    vector_FR = R_inv * vector_TR;
    CartPos speed;
    speed.x = vector_FT[0];
    speed.y = vector_FT[1];
    speed.z = vector_FT[2];
    speed.rx = vector_FR[0];
    speed.ry = vector_FR[1];
    speed.rz = vector_FR[2];
    start_fixpoint_singularity_check();
    char szCmd[0x1000];
    /*if (axis == 1) {
        sprintf(szCmd, "def start_forcemode_speedl_move():\n while (True):\n force_mode(p[%f,%f,%f,%f,%f,%f], [1,0,0,1,1,1], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [%f,0.1,0.1,0.01,0.01,0.01])\n speedl([%f,%f,%f,%f,%f,%f],1.0,0.2,2.0)\n sync()\n end\n end_force_mode()\nend\n",
                force_pos->x / 1000, force_pos->y / 1000, force_pos->z / 1000, force_pos->rx, force_pos->ry, force_pos->rz, speed_z / 1000.0, speed.x / 1000, speed.y / 1000, speed.z / 1000, speed.rx, speed.ry, speed.rz);
    } else if (axis == 2) {
        sprintf(szCmd, "def start_forcemode_speedl_move():\n while (True):\n force_mode(p[%f,%f,%f,%f,%f,%f], [0,1,0,1,1,1], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [0.1,%f,0.1,0.01,0.01,0.01])\n speedl([%f,%f,%f,%f,%f,%f],1.0,0.2,2.0)\n sync()\n end\n end_force_mode()\nend\n",
                force_pos->x / 1000, force_pos->y / 1000, force_pos->z / 1000, force_pos->rx, force_pos->ry, force_pos->rz, speed_z / 1000.0, speed.x / 1000, speed.y / 1000, speed.z / 1000, speed.rx, speed.ry, speed.rz);
    } else if (axis == 3) {
        sprintf(szCmd, "def start_forcemode_speedl_move():\n while (True):\n force_mode(p[%f,%f,%f,%f,%f,%f], [0,0,1,1,1,1], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [0.1,0.1,%f,0.2,0.2,0.2])\n speedl([%f,%f,%f,%f,%f,%f],1.0,0.2,2.0)\n sync()\n end\n end_force_mode()\nend\n",
                force_pos->x / 1000, force_pos->y / 1000, force_pos->z / 1000, force_pos->rx, force_pos->ry, force_pos->rz, speed_z / 1000.0, speed.x / 1000, speed.y / 1000, speed.z / 1000, speed.rx, speed.ry, speed.rz);
    }*/
    sprintf(szCmd, "def enter_force_mode():\n force_mode(tool_pose(), [0, 0, 0, 1, 1, 0], [0, 0, 0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.02, 0.2, 0.2, 0.2])\n sleep(10000)\nend\n");
    emit signal_send_command(szCmd);
}

void URServer::set_robot_force_damping(double value) {
    Q_D(URServer);
    char szCmd[0x100];
    sprintf(szCmd, "force_mode_set_damping(%f)\n", value);
    emit signal_send_command(szCmd);
}

void URServer::start_singularity_check() {
    Q_D(URServer);
    if (!d->detect_timer->isActive()) {
        d->detect_timer->start(30);
    }
}

void URServer::stop_singularity_check() {
    Q_D(URServer);
    qDebug() << "stop_singularity_check";
    if (d->detect_timer->isActive()) {
        d->detect_timer->stop();
    }
}

void URServer::start_special_singularity_check() {
    Q_D(URServer);
    if (!d->special_detect_timer->isActive()) {
        d->special_detect_timer->start(30);
    }
}

void URServer::stop_special_singularity_check() {
    Q_D(URServer);
    if (d->special_detect_timer->isActive()) {
        d->special_detect_timer->stop();
    }
}

void URServer::start_fixpoint_singularity_check() {
    Q_D(URServer);
    if (!d->fixpoint_singularity_detect_timer->isActive()) {
        d->fixpoint_singularity_detect_timer->start(30);
    }
}

void URServer::stop_fixpoint_singularity_check() {
    Q_D(URServer);
    if (d->fixpoint_singularity_detect_timer->isActive()) {
        d->fixpoint_singularity_detect_timer->stop();
    }
}

void URServer::set_robot_type(ROBOT_TYPE type) {
    Q_D(URServer);
    if (type == ROBOT_TYPE::UR_3E) {
        d->head_radius = 240;
    } else if (type == ROBOT_TYPE::UR_5E) {
        d->head_radius = 300;
    } else if (type == ROBOT_TYPE::UR_10E) {
        d->head_radius = 380;
    } else if (type == ROBOT_TYPE::UR_16E) {
        d->head_radius = 312;
    }
}

void URServer::move_robot_emc_test() {
    Q_D(URServer);
    JointPos js1, js2;
    js1.j1 = -90, js1.j2 = -50, js1.j3 = -50, js1.j4 = 0, js1.j5 = 120, js1.j6 = 0;
    js2.j1 = -90, js2.j2 = -90, js2.j3 = -90, js2.j4 = -90, js2.j5 = 60, js2.j6 = 90;
    char szCmd[0x1000];
    sprintf(szCmd, "def move_emc_test():\n while (True):\n  movej([%f, %f, %f, %f, %f, %f], a=1.2, v=1.0, t=0, r=0)\n movej([%f, %f, %f, %f, %f, %f], a=1.2, v=1.0, t=0, r=0)\n sync()\n end\nend\n",
            js1.j1 * DEG2RAD, js1.j2 * DEG2RAD, js1.j3 * DEG2RAD, js1.j4 * DEG2RAD, js1.j5 * DEG2RAD, js1.j6 * DEG2RAD, js2.j1 * DEG2RAD, js2.j2 * DEG2RAD, js2.j3 * DEG2RAD, js2.j4 * DEG2RAD, js2.j5 * DEG2RAD, js2.j6 * DEG2RAD);
    emit signal_send_command(szCmd);
}

void URServer::get_tool_analog_in(int index) {
    Q_D(URServer);
    char szCmd[0x1000];
    sprintf(szCmd, "sec get_tool_analog_in():\n a = get_tool_analog_in(%i)\n socket_open(\"172.31.1.100\", 5000, \"tool\")\n socket_send_string(a, \"tool\")\n socket_close(\"tool\")\nend\n",
            index);
    emit signal_send_30002_command(szCmd);
}

void URServer::get_forward_kin(JointPos current_jpos) {
    Q_D(URServer);
    char szCmd[0x1000];
    sprintf(szCmd, "def get_forward_kin():\n a = get_forward_kin([%f, %f, %f, %f, %f, %f])\n"
                   "socket_open(\"172.31.1.100\", 5000, \"test\")\n socket_send_string(a, \"test\")\n socket_close(\"test\")\nend\n",
            current_jpos.j1 * DEG2RAD, current_jpos.j2 * DEG2RAD, current_jpos.j3 * DEG2RAD, current_jpos.j4 * DEG2RAD, current_jpos.j5 * DEG2RAD, current_jpos.j6 * DEG2RAD);
    //    qDebug() << "szCmd: " << szCmd;
    send_command(szCmd);
}

void URServer::get_inverse_kin(CartPos current_cpos) {
    Q_D(URServer);
    JointPos near_jpos;
    near_jpos = d->current_jpos;
    char szCmd[0x1000];
    sprintf(szCmd, "def get_inverse_kin():\n a = get_inverse_kin(p[%f, %f, %f, %f, %f, %f], [%f, %f, %f, %f, %f, %f])\n"
                   "socket_open(\"172.31.1.100\", 5000, \"test\")\n socket_send_string(a, \"test\")\n socket_close(\"test\")\nend\n",
            current_cpos.x / 1000, current_cpos.y / 1000, current_cpos.z / 1000, current_cpos.rx, current_cpos.ry, current_cpos.rz,
            near_jpos.j1 * DEG2RAD, near_jpos.j2 * DEG2RAD, near_jpos.j3 * DEG2RAD, near_jpos.j4 * DEG2RAD, near_jpos.j5 * DEG2RAD, near_jpos.j6 * DEG2RAD);
    send_command(szCmd);
}

void URServer::slot_get_inverse_jpos(QByteArray msg) {
    Q_D(URServer);
    bool is_elbow_singularity = false;
    bool is_wrist_singularity = false;
    QString str = QString(msg);
    str.remove("[");
    str.remove("]");
    QStringList list = str.split(",");
    JointPos inverse_jpos;
    inverse_jpos.j1 = list[0].toDouble() * RAD2DEG;
    inverse_jpos.j2 = list[1].toDouble() * RAD2DEG;
    inverse_jpos.j3 = list[2].toDouble() * RAD2DEG;
    inverse_jpos.j4 = list[3].toDouble() * RAD2DEG;
    inverse_jpos.j5 = list[4].toDouble() * RAD2DEG;
    inverse_jpos.j6 = list[5].toDouble() * RAD2DEG;
    emit signal_send_inverse_jpos(inverse_jpos);
    if ((inverse_jpos.j5 < -136 && inverse_jpos.j5 > -224) ||
        (inverse_jpos.j5 > 136 && inverse_jpos.j5 < 224) ||
        (inverse_jpos.j5 > -46 && inverse_jpos.j5 < 44) ||
        (inverse_jpos.j5 > -360 && inverse_jpos.j5 < -316) ||
        (inverse_jpos.j5 > 316 && inverse_jpos.j5 < 360)) {
        is_wrist_singularity = true;
        qDebug() << "接近腕部奇异点！请注意！！！";
    } else {
        is_wrist_singularity = false;
        qDebug() << "正常位置！！！";
    }
    if (inverse_jpos.j3 > -40 && inverse_jpos.j3 < 40) {
        is_elbow_singularity = true;
        qDebug() << "接近肘部奇异点！请注意！！！";
    } else {
        is_elbow_singularity = false;
        qDebug() << "正常位置！！！";
    }
    if (is_elbow_singularity || is_wrist_singularity) {
        emit signal_is_near_singularity_pos(true);
    } else {
        emit signal_is_near_singularity_pos(false);
    }
}

void URServer::slot_get_forward_cpos(QByteArray msg) {
    Q_D(URServer);
    QString str = QString(msg);
    //    qDebug() << "slot_get_forward_cpos: " << str;
    str.remove("p[");
    str.remove("]");
    QStringList list = str.split(",");
    CartPos forward_cpos;
    forward_cpos.x = list[0].toDouble() * 1000; // 转换为毫米
    forward_cpos.y = list[1].toDouble() * 1000; // 转换为毫米
    forward_cpos.z = list[2].toDouble() * 1000; // 转换为毫米
    forward_cpos.rx = list[3].toDouble();
    forward_cpos.ry = list[4].toDouble();
    forward_cpos.rz = list[5].toDouble();
    emit signal_send_forward_cpos(forward_cpos);
}

void URServer::slot_get_tool_analog_in(QByteArray msg) {
    Q_D(URServer);
    QString str = QString(msg);
    double value = str.toDouble();
    emit signal_send_tool_analog_in(value);
}

void URServer::slot_detect_robot_pos_timer_start() {
    Q_D(URServer);
    double joints_limit_score = 100, wrist_score = 60, elbow_score = 60, head_score = 60;
    if (d->current_jpos.j1 < -350 || d->current_jpos.j1 > 350 || d->current_jpos.j2 < -350 ||
        d->current_jpos.j2 > 350 || d->current_jpos.j3 < -150 || d->current_jpos.j3 > 150 ||
        d->current_jpos.j4 < -350 || d->current_jpos.j4 > 350 || d->current_jpos.j5 < -350 ||
        d->current_jpos.j5 > 350 || d->current_jpos.j6 < -350 || d->current_jpos.j6 > 350) {
        joints_limit_score = 10; // 接近关节极限，应为低评分
    }
    if ((d->current_jpos.j5 < -136 && d->current_jpos.j5 > -224) ||
        (d->current_jpos.j5 > 136 && d->current_jpos.j5 < 224) ||
        (d->current_jpos.j5 > -44 && d->current_jpos.j5 < 44) ||
        (d->current_jpos.j5 > -360 && d->current_jpos.j5 < -316) ||
        (d->current_jpos.j5 > 316 && d->current_jpos.j5 < 360)) {
        wrist_score = 10; // 接近腕部奇异点，应为低评分
    } else if (d->current_jpos.j5 >= 44 && d->current_jpos.j5 < 90) {
        wrist_score = 40.0 * (d->current_jpos.j5 - 44.0) / 46.0 + 60.0;
    } else if (d->current_jpos.j5 >= 90 && d->current_jpos.j5 < 136) {
        wrist_score = 40.0 * (136.0 - d->current_jpos.j5) / 46.0 + 60.0;
    } else if (d->current_jpos.j5 >= 224 && d->current_jpos.j5 < 270) {
        wrist_score = 40.0 * (d->current_jpos.j5 - 224.0) / 46.0 + 60.0;
    } else if (d->current_jpos.j5 >= 270 && d->current_jpos.j5 < 316) {
        wrist_score = 40.0 * (316.0 - d->current_jpos.j5) / 46.0 + 60.0;
    } else if (d->current_jpos.j5 >= -316 && d->current_jpos.j5 < -270) {
        wrist_score = 40.0 * (d->current_jpos.j5 + 316.0) / 46.0 + 60.0;
    } else if (d->current_jpos.j5 >= -270 && d->current_jpos.j5 < -224) {
        wrist_score = 40.0 * (-224.0 - d->current_jpos.j5) / 46.0 + 60.0;
    } else if (d->current_jpos.j5 >= -136 && d->current_jpos.j5 < -90) {
        wrist_score = 40.0 * (d->current_jpos.j5 + 136.0) / 46.0 + 60.0;
    } else if (d->current_jpos.j5 >= -90 && d->current_jpos.j5 < -44) {
        wrist_score = 40.0 * (-44.0 - d->current_jpos.j5) / 46.0 + 60.0;
    }
    if (d->current_jpos.j3 > -40 && d->current_jpos.j3 < 40) {
        elbow_score = 10; // 接近肘部奇异点，应为低评分
    } else if ((d->current_jpos.j3 > 40 && d->current_jpos.j3 <= 90) || (d->current_jpos.j3 < -40 && d->current_jpos.j3 >= -90)) {
        elbow_score = 40 * (abs(d->current_jpos.j3) - 40) / 50 + 60;
    } else if ((d->current_jpos.j3 > 90 && d->current_jpos.j3 <= 140) ||
               (d->current_jpos.j3 < -90 && d->current_jpos.j3 >= -140)) {
        elbow_score = 40 * (-abs(d->current_jpos.j3) + 140) / 50 + 60;
    }
    slot_calculate_flange2base();
    // qDebug() << "bottom2base: " << d->flangebottom2base << " " << d->flangetop2base << " "
    //          << d->head_radius;
    if (d->flangebottom2base < d->head_radius || d->flangetop2base < d->head_radius) {
        head_score = 10; // 接近头部奇异点，应为低评分
    } else {
        head_score = d->flangebottom2base / 10 + 30;
        head_score = std::min(head_score, d->flangetop2base / 10 + 30);
    }
    d->robot_score = std::min(joints_limit_score, wrist_score);
    d->robot_score = std::min(d->robot_score, elbow_score);
    d->robot_score = std::min(d->robot_score, head_score);
    // qDebug() << "robot_score: " << d->robot_score << "elbow: " << elbow_score
    //          << "wirst: " << wrist_score << "head: " << head_score
    //          << "joints_limit: " << joints_limit_score;
    emit signal_send_robot_score(d->robot_score);
}

double URServer::calculate_pid_speed_t_iterate(double offset) {
    Q_D(URServer);
    double error = offset;
    d->derivative_speed_t = (error - d->previous_error_speed_t) / d->delta_time;
    d->integral_speed_t += error * d->delta_time;
    double output = d->kp_speed_t * error + d->ki_speed_t * d->integral_speed_t + d->kd_speed_t * d->derivative_speed_t;
    d->previous_error_speed_t = error;
    return output;
}

double URServer::calculate_pid_speed_r_iterate(double offset) {
    Q_D(URServer);
    double error = offset;
    d->derivative_speed_r = (error - d->previous_error_speed_r) / d->delta_time;
    d->integral_speed_r += error * d->delta_time;
    double output = d->kp_speed_r * error + d->ki_speed_r * d->integral_speed_r + d->kd_speed_r * d->derivative_speed_r;
    d->previous_error_speed_r = error;
    return output;
}

double URServer::calculate_pid_fixline_speed_iterate(double offset) {
    Q_D(URServer);
    double error = offset;
    d->derivative_fixline_speed = (error - d->previous_error_fixline_speed) / d->delta_time;
    d->integral_fixline_speed += error * d->delta_time;
    double output = d->kp_fixline_speed * error + d->ki_fixline_speed * d->integral_fixline_speed + d->kd_fixline_speed * d->derivative_fixline_speed;
    d->previous_error_fixline_speed = error;
    return output;
}

void URServer::testSpeedJ() {
    // 6. 设置速度控制参数
    double target_velocity[6] = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0}; // 目标关节速度 [rad/s]
    double acceleration = 0.5;                                  // 加速度 [rad/s²]
    double control_time = 2.0;                                  // 总控制时间 [s]
    double update_interval = 0.1;                               // 命令更新间隔 [s]

    // 7. 构建并发送speedj命令
    try {
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::steady_clock::now() - start_time)
                   .count() < control_time) {

            // 构建URScript命令字符串
            char command[256];
            std::snprintf(command, sizeof(command),
                          "speedj([%f, %f, %f, %f, %f, %f], %f, %f)\n",
                          target_velocity[0], target_velocity[1], target_velocity[2],
                          target_velocity[3], target_velocity[4], target_velocity[5],
                          acceleration, update_interval);

            emit signal_send_command(command);

            // 等待更新间隔
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(update_interval * 1000)));
        }

        // 8. 发送停止命令
        const char *stop_command = "stopj(2.0)\n"; // 以2.0 rad/s²的减速度停止
        emit signal_send_command(stop_command);

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

void URServer::move_to_target_joint_speedj(JointPos pos, double control_interval) {
    Q_D(URServer);
    // 使用PID计算控制量：PID输出的是当前次控制给出的误差增量
    // 计算方法:应该基于当前速度和目标关节角度，计算给出的关节速度：配合给出的加速度和控制间隔TODO!!!!

    d->pid_joints[0].setSetpoint(pos.j1 * DEG2RAD);
    d->pid_joints[1].setSetpoint(pos.j2 * DEG2RAD);
    d->pid_joints[2].setSetpoint(pos.j3 * DEG2RAD);
    d->pid_joints[3].setSetpoint(pos.j4 * DEG2RAD);
    d->pid_joints[4].setSetpoint(pos.j5 * DEG2RAD);
    d->pid_joints[5].setSetpoint(pos.j6 * DEG2RAD);
    std::vector<double> pidSpeed(6, 0.0);
    pidSpeed[0] = d->pid_joints[0].update(d->current_jpos.j1 * DEG2RAD, control_interval);
    pidSpeed[1] = d->pid_joints[1].update(d->current_jpos.j2 * DEG2RAD, control_interval);
    pidSpeed[2] = d->pid_joints[2].update(d->current_jpos.j3 * DEG2RAD, control_interval);
    pidSpeed[3] = d->pid_joints[3].update(d->current_jpos.j4 * DEG2RAD, control_interval);
    pidSpeed[4] = d->pid_joints[4].update(d->current_jpos.j5 * DEG2RAD, control_interval);
    pidSpeed[5] = d->pid_joints[5].update(d->current_jpos.j6 * DEG2RAD, control_interval);

    for (int i = 0; i < 6; i++) {
        pidSpeed[i] = std::abs(pidSpeed[i]) > d->max_arm_velocity[i] ? d->max_arm_velocity[i] : pidSpeed[i];
    }
    const double epsilon = 0.0005;
    bool is_all_zero = true;
    for (int i = 0; i < 6; i++) {
        if (pidSpeed[i] > epsilon) {
            is_all_zero = false;
            break;
        }
    }
    if (is_all_zero) {
        stop_movej();
        return;
    }
    /*
    关节电流和末端负载及关节加速度有关系：所以关节越靠基座，负载越大，关节越大，可以实现的加速度应该越小
    */
    char command[256];
    double acceleration = d->analysisSpeed(pidSpeed); // speedj 的加速度参数 [rad/s²]
    std::sprintf(command, "def move_to_target_pos_with_speedj():\n while (True):\n speedj([%f, %f, %f, %f, %f, %f], %f, %f)\n sync()\n end\nend\n",
                 pidSpeed[0], pidSpeed[1], pidSpeed[2],
                 pidSpeed[3], pidSpeed[4], pidSpeed[5],
                 acceleration, 2.0);

    std::cout << command << std::endl;

    emit signal_send_command(command);
}

void URServer::move_to_target_joint_speedj_mine(double pos, double control_interval) {
    Q_D(URServer);
    static double currentVel = 0.0;
    static double prePos = d->current_jpos.j4 * DEG2RAD;
    // 计算方法:应该基于当前速度和目标关节角度，计算给出的关节速度：配合给出的加速度和控制间隔
    // 这样做的会一直产生抖动的情况
    double current_joint0_position = d->current_jpos.j4 * DEG2RAD;
    currentVel = (current_joint0_position - prePos) / (control_interval);
    prePos = current_joint0_position;

    double requredAcce = 0.0;
    double requredVel = 0.0;
    getTargetAcce(current_joint0_position, pos, currentVel, control_interval, requredAcce, requredVel);
    std::cout << " currentVel： " << currentVel << " requredAcce: " << requredAcce << " requredVel: " << requredVel << std::endl;

    std::vector<double> pidSpeed(6, 0.0);
    pidSpeed[3] = requredVel; // 只控制第4个关节
    //
    // WFL:这是通过速度控制关节运动：需要将上下两帧的关节运动速度
    char command[256];
    double acceleration = 5.0; // speedj 的加速度参数 [rad/s²]
    std::sprintf(command, "def move_to_target_pos_with_speedj():\n while (True):\n speedj([%f, %f, %f, %f, %f, %f], %f, %f)\n sync()\n end\nend\n",
                 pidSpeed[0], pidSpeed[1], pidSpeed[2],
                 pidSpeed[3], pidSpeed[4], pidSpeed[5],
                 requredAcce, control_interval);

    emit signal_send_command(command);
}

void URServer::getTargetAcce(double currPos, double targetPos, double currVel, double time, double &requiredAcce, double &requiredVel) {
    double distance = targetPos - currPos;
    std::cout << "distance： " << distance << " ";
    // 使用简单的线性加速度模型计算所需加速度
    requiredAcce = (2 * (distance - currVel * time)) / (time * time);
    requiredVel = currVel + requiredAcce * time;
}

/*
TODO:
出现发送指令后机械臂执行卡顿

YOZX 3397@move_to_target_joint_speedj -83.6958 -90.2749 -96.3218 -178.172 -93.1051 -150.395
YOZX 3398@move_to_target_joint_speedj -83.6985 -90.2768 -96.3228 -178.173 -93.1042 -150.396
"YOZX 3397@move_to_target_joint_speedj -83.6958 -90.2749 -96.3218 -178.172 -93.1051 -150.395"
YOZX 3399@move_to_target_joint_speedj -83.6964 -90.273 -96.3209 -178.173 -93.1035 -150.393
YOZX 3400@move_to_target_joint_speedj -83.6974 -90.2747 -96.3192 -178.171 -93.1035 -150.394
def move_to_target_pos_with_speedj():
 while (True):
 speedj([-0.000355, -0.000262, -0.001129, 0.001585, -0.000651, 0.003492], 10.000000, 2.000000)
 sync()
 end
end
YOZX 3401@move_to_target_joint_speedj -83.6953 -90.274 -96.3214 -178.173 -93.1025 -150.393

YOZX 3402@move_to_target_joint_speedj -83.6976 -90.2749 -96.3206 -178.173 -93.103 -150.395
YOZX 3403@move_to_target_joint_speedj -83.6969 -90.273 -96.3216 -178.173 -93.102 -150.394
YOZX 3404@move_to_target_joint_speedj -83.6966 -90.2733 -96.3211 -178.173 -93.1019 -150.393
YOZX 3405@move_to_target_joint_speedj -83.6997 -90.2746 -96.3208 -178.171 -93.102 -150.392
YOZX 3406@move_to_target_joint_speedj -83.6992 -90.273 -96.318 -178.171 -93.1008 -150.393
YOZX 3407@move_to_target_joint_speedj -83.6976 -90.2734 -96.3234 -178.173 -93.1035 -150.395
YOZX 3408@move_to_target_joint_speedj -83.6983 -90.2754 -96.3195 -178.171 -93.1027 -150.395
"YOZX 3398@move_to_target_joint_speedj -83.6985 -90.2768 -96.3228 -178.173 -93.1042 -150.396"
YOZX 3409@move_to_target_joint_speedj -83.6971 -90.2749 -96.3202 -178.171 -93.1044 -150.394
def move_to_target_pos_with_speedj():
 while (True):
 speedj([-0.000409, -0.000265, -0.001182, 0.001578, -0.000705, 0.003514], 10.000000, 2.000000)
 sync()
 end
end
YOZX 3410@move_to_target_joint_speedj -83.6967 -90.2742 -96.323 -178.172 -93.1022 -150.394

YOZX 3411@move_to_target_joint_speedj -83.6974 -90.2714 -96.3216 -178.172 -93.1055 -150.394
YOZX 3412@move_to_target_joint_speedj -83.6967 -90.2751 -96.3223 -178.173 -93.102 -150.395
YOZX 3413@move_to_target_joint_speedj -83.6967 -90.2751 -96.3223 -178.173 -93.102 -150.395
"YOZX 3399@move_to_target_joint_speedj -83.6964 -90.273 -96.3209 -178.173 -93.1035 -150.393"
YOZX 3414@move_to_target_joint_speedj -83.6967 -90.2751 -96.3223 -178.173 -93.102 -150.395
def move_to_target_pos_with_speedj():
 while (True):
 speedj([-0.000299, -0.000132, -0.001082, 0.001578, -0.000650, 0.003749], 10.000000, 2.000000)
 sync()
 end
end
YOZX 3415@move_to_target_joint_speedj -83.6967 -90.2751 -96.3223 -178.173 -93.102 -150.395

YOZX 3416@move_to_target_joint_speedj -83.6967 -90.2751 -96.3223 -178.173 -93.102 -150.395
YOZX 3417@move_to_target_joint_speedj -83.6967 -90.2751 -96.3223 -178.173 -93.102 -150.395
"YOZX 3400@move_to_target_joint_speedj -83.6974 -90.2747 -96.3192 -178.171 -93.1035 -150.394"
YOZX 3418@move_to_target_joint_speedj -83.6967 -90.2751 -96.3223 -178.173 -93.102 -150.395
def move_to_target_pos_with_speedj():
 while (True):
 speedj([-0.000352, -0.000192, -0.000993, 0.001735, -0.000650, 0.003671], 10.000000, 2.000000)
 sync()
 end
end

YOZX 3419@move_to_target_joint_speedj -83.6967 -90.2751 -96.3223 -178.173 -93.102 -150.395
YOZX 3420@move_to_target_joint_speedj -83.6967 -90.2751 -96.3223 -178.173 -93.102 -150.395
"YOZX 3401@move_to_target_joint_speedj -83.6953 -90.274 -96.3214 -178.173 -93.1025 -150.393"
YOZX 3421@move_to_target_joint_speedj -83.6967 -90.2751 -96.3223 -178.173 -93.102 -150.395
def move_to_target_pos_with_speedj():
 while (True):
 speedj([-0.000242, -0.000167, -0.001108, 0.001578, -0.000572, 0.003749], 10.000000, 2.000000)
 sync()
 end
end


*/