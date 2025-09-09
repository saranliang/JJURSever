#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFile>
#include <QTextStream>
#include <QVector>
#include <bitset>
#include <sstream>
#include <thread>

Q_DECLARE_METATYPE(JointPos);
Q_DECLARE_METATYPE(CartPos);
Q_DECLARE_METATYPE(Wrench);
Q_DECLARE_METATYPE(Speed);
typedef unsigned long DWORD;

class MainWindowPrivate : public Ui_MainWindow {
  public:
    URServer *_server;
    Payload load;
    QThread *_serverThread;
    const QString disconnected_style = "min-width:20px;min-height:20px;max-width:20px;max-height:20px;border-radius:10px;border:1px solid black;background:red";
    const QString connected_style = "min-width:20px;min-height:20px;max-width:20px;max-height:20px;border-radius:10px;border:1px solid black;background:green";
    QVector<double> vector_value;
    QButtonGroup *m_buttonGroup;
    QButtonGroup *joint_btn_group;
    QButtonGroup *cart_btn_group;
    int safety_status = 1;
    CartPos *fixlinePos = nullptr;
    CartPos currentpos;
    QTimer *timer = nullptr;
    JointPos current_jpos;
    JointPos record_jpos;
    JointPos servo_jpos;
    QFile file;
    QTimer *timerTest = nullptr;
  private slots:
};

MainWindow::MainWindow(QWidget *parent, URServer *urserver) : QMainWindow(parent), d_ptr(new MainWindowPrivate) {
    Q_D(MainWindow);
    d->setupUi(this);
    qRegisterMetaType<JointPos>("JointPos");
    qRegisterMetaType<CartPos>("CartPos");
    qRegisterMetaType<Wrench>("Wrench");
    qRegisterMetaType<Speed>("Speed");
    d->_server = urserver;
    d->_serverThread = new QThread();
    d->_serverThread->setObjectName("URServer");
    // d->_server->moveToThread(d->_serverThread);
    // d->_serverThread->start();
    this->setStyleSheet("background-color:rgb(105, 105, 105); color:rgb(255, 255, 255);");
    d->label_status->setStyleSheet(d->disconnected_style);
    d->warning_label->setStyleSheet("border-radius:2px;border:1px solid black;background-color:red");
    d->warning_label->hide();
    // d->label_isFixline->setStyleSheet("border-radius:2px;border:1px solid black;background-color:red");
    // d->label_isFixline->hide();
    d->m_buttonGroup = new QButtonGroup();
    d->joint_btn_group = new QButtonGroup();
    d->cart_btn_group = new QButtonGroup();
    d->timer = new QTimer(this);
    d->timerTest = new QTimer(this);

    d->fixlinePos = new CartPos();
    d->fixlinePos->x = -14.23;
    d->fixlinePos->y = -498;
    d->fixlinePos->z = -135.38;
    d->fixlinePos->rx = 0.31;
    d->fixlinePos->ry = 2.26;
    d->fixlinePos->rz = -2.20;
    d->joint_btn_group->addButton(d->btn_nj1, 0);
    d->joint_btn_group->addButton(d->btn_nj2, 1);
    d->joint_btn_group->addButton(d->btn_nj3, 2);
    d->joint_btn_group->addButton(d->btn_nj4, 3);
    d->joint_btn_group->addButton(d->btn_nj5, 4);
    d->joint_btn_group->addButton(d->btn_nj6, 5);
    d->joint_btn_group->addButton(d->btn_pj1, 6);
    d->joint_btn_group->addButton(d->btn_pj2, 7);
    d->joint_btn_group->addButton(d->btn_pj3, 8);
    d->joint_btn_group->addButton(d->btn_pj4, 9);
    d->joint_btn_group->addButton(d->btn_pj5, 10);
    d->joint_btn_group->addButton(d->btn_pj6, 11);
    d->cart_btn_group->addButton(d->btn_nx, 0);
    d->cart_btn_group->addButton(d->btn_px, 1);
    d->cart_btn_group->addButton(d->btn_ny, 2);
    d->cart_btn_group->addButton(d->btn_py, 3);
    d->cart_btn_group->addButton(d->btn_nz, 4);
    d->cart_btn_group->addButton(d->btn_pz, 5);
    d->cart_btn_group->addButton(d->btn_nrx, 6);
    d->cart_btn_group->addButton(d->btn_prx, 7);
    d->cart_btn_group->addButton(d->btn_nry, 8);
    d->cart_btn_group->addButton(d->btn_pry, 9);
    d->cart_btn_group->addButton(d->btn_nrz, 10);
    d->cart_btn_group->addButton(d->btn_prz, 11);
    hand_over();
    take_over();
    d->file.setFileName("jpos.txt");
    if (!d->file.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
        qDebug() << "Open failed.";
    }
    d->file.close();
}

MainWindow::~MainWindow() {
    Q_D(MainWindow);
    hand_over();
}

void MainWindow::take_over() {
    Q_D(MainWindow);
    connect(d->_server, SIGNAL(signal_update_joint_pos(JointPos)), this, SLOT(slot_get_joint_pos(JointPos)));
    connect(d->_server, SIGNAL(signal_update_cart_pos(CartPos)), this, SLOT(slot_get_cart_pos(CartPos)));
    connect(d->_server, SIGNAL(signal_update_tcp_wrench(Wrench)), this, SLOT(slot_get_tcp_wrench(Wrench)));
    connect(d->_server, SIGNAL(signal_update_tcp_speed(Speed)), this, SLOT(slot_get_tcp_speed(Speed)));
    connect(d->_server, SIGNAL(signal_update_safety_status(int)), this, SLOT(slot_get_safety_status(int)));
    connect(d->btn_connectUR, SIGNAL(clicked()), this, SLOT(slot_btn_connectUR_clicked()));
    connect(d->btn_teach, SIGNAL(clicked()), this, SLOT(slot_btn_teach_clicked()));
    connect(d->btn_fixline, SIGNAL(clicked()), this, SLOT(slot_btn_fixline_clicked()));
    connect(d->btn_fasten, SIGNAL(clicked()), this, SLOT(slot_btn_fasten_clicked()));
    connect(d->btn_movej, SIGNAL(clicked()), this, SLOT(slot_btn_movej_clicked()));
    connect(d->btn_movel, SIGNAL(clicked()), this, SLOT(slot_btn_movel_clicked()));
    connect(d->btn_open_mouse, SIGNAL(clicked()), this, SLOT(slot_btn_open_mouse_clicked()));
    connect(d->btn_close_mouse, SIGNAL(clicked()), this, SLOT(slot_btn_close_mouse_clicked()));
    connect(d->btn_stopj, SIGNAL(clicked()), this, SLOT(slot_btn_stopj_clicked()));
    connect(d->btn_stopl, SIGNAL(clicked()), this, SLOT(slot_btn_stopl_clicked()));
    connect(d->btn_restart_safety, SIGNAL(clicked()), this, SLOT(slot_btn_restart_safety_clicked()));
    connect(d->btn_set_payload, SIGNAL(clicked()), this, SLOT(slot_btn_set_payload_clicked()));
    connect(d->btn_set_tcp, SIGNAL(clicked()), this, SLOT(slot_btn_set_tcp_clicked()));
    connect(d->btn_screw_mode, SIGNAL(clicked()), this, SLOT(slot_btn_screw_mode_clicked()));
    connect(d->btn_end_screw_mode, SIGNAL(clicked()), this, SLOT(slot_btn_end_screw_mode_clicked()));
    connect(d->m_buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT(slot_button_group(int)));
    connect(d->btn_start_robot, SIGNAL(clicked()), this, SLOT(slot_btn_start_robot_clicked()));
    connect(d->btn_shutdown, &QPushButton::clicked, this, &MainWindow::slot_btn_shutdown_clicked);
    connect(d->btn_unlock, &QPushButton::clicked, this, &MainWindow::slot_btn_unlock_clicked);
    connect(d->joint_btn_group, SIGNAL(buttonPressed(int)), this, SLOT(slot_joint_button_group_pressed(int)));
    connect(d->joint_btn_group, SIGNAL(buttonReleased(int)), this, SLOT(slot_joint_button_group_released(int)));
    connect(d->cart_btn_group, SIGNAL(buttonPressed(int)), this, SLOT(slot_cart_button_group_pressed(int)));
    connect(d->cart_btn_group, SIGNAL(buttonReleased(int)), this, SLOT(slot_cart_button_group_released(int)));
    connect(d->timer, &QTimer::timeout, this, &MainWindow::slot_timer_start);
    connect(d->_server, &URServer::signal_send_inverse_jpos, this, &MainWindow::slot_get_inverse_jpos);
    connect(d->_server, &URServer::signal_send_forward_cpos, this, &MainWindow::slot_get_forward_cpos);
    connect(d->timerTest, &QTimer::timeout, this, &MainWindow::onTestServoBySpeedJTimeOut);
}

void MainWindow::hand_over() {
    Q_D(MainWindow);
    disconnect(d->btn_connectUR, SIGNAL(clicked()), this, SLOT(slot_btn_connectUR_clicked()));
    disconnect(d->_server, SIGNAL(signal_update_joint_pos(JointPos)), this, SLOT(slot_get_joint_pos(JointPos)));
    disconnect(d->_server, SIGNAL(signal_update_cart_pos(CartPos)), this, SLOT(slot_get_cart_pos(CartPos)));
    disconnect(d->_server, SIGNAL(signal_update_tcp_wrench(Wrench)), this, SLOT(slot_get_tcp_wrench(Wrench)));
    disconnect(d->_server, SIGNAL(signal_update_tcp_speed(Speed)), this, SLOT(slot_get_tcp_speed(Speed)));
    disconnect(d->_server, SIGNAL(signal_update_safety_status(int)), this, SLOT(slot_get_safety_status(int)));
    disconnect(d->btn_movej, SIGNAL(clicked()), this, SLOT(slot_btn_movej_clicked()));
    disconnect(d->btn_movel, SIGNAL(clicked()), this, SLOT(slot_btn_movel_clicked()));
    disconnect(d->btn_open_mouse, SIGNAL(clicked()), this, SLOT(slot_btn_open_mouse_clicked()));
    disconnect(d->btn_close_mouse, SIGNAL(clicked()), this, SLOT(slot_btn_close_mouse_clicked()));
    disconnect(d->btn_stopj, SIGNAL(clicked()), this, SLOT(slot_btn_stopj_clicked()));
    disconnect(d->btn_stopl, SIGNAL(clicked()), this, SLOT(slot_btn_stopl_clicked()));
    disconnect(d->btn_restart_safety, SIGNAL(clicked()), this, SLOT(slot_btn_restart_safety_clicked()));
    disconnect(d->btn_fixline, SIGNAL(clicked()), this, SLOT(slot_btn_fixline_clicked()));
    disconnect(d->btn_set_payload, SIGNAL(clicked()), this, SLOT(slot_btn_set_payload_clicked()));
    disconnect(d->timer, &QTimer::timeout, this, &MainWindow::slot_timer_start);
    disconnect(d->timerTest, &QTimer::timeout, this, &MainWindow::onTestServoBySpeedJTimeOut);
}

void MainWindow::slot_btn_connectUR_clicked() {
    Q_D(MainWindow);
    // d->_server->start_robot();
    d->_server->connect_to_robot();
}

void MainWindow::slot_btn_start_robot_clicked() {
    Q_D(MainWindow);
    d->_server->start_robot();
}

void MainWindow::slot_btn_shutdown_clicked() {
    Q_D(MainWindow);
    d->_server->shutdown_robot();
}

void MainWindow::slot_btn_unlock_clicked() {
    Q_D(MainWindow);
    d->_server->unlock_protective_stop();
}

void MainWindow::slot_get_joint_pos(JointPos jPos) {
    Q_D(MainWindow);
    d->current_jpos = jPos;
    d->lineEdit_j1->setText(QString::number(jPos.j1, 'd', 2));
    d->lineEdit_j2->setText(QString::number(jPos.j2, 'd', 2));
    d->lineEdit_j3->setText(QString::number(jPos.j3, 'd', 2));
    d->lineEdit_j4->setText(QString::number(jPos.j4, 'd', 2));
    d->lineEdit_j5->setText(QString::number(jPos.j5, 'd', 2));
    d->lineEdit_j6->setText(QString::number(jPos.j6, 'd', 2));
}

void MainWindow::slot_get_cart_pos(CartPos cPos) {
    Q_D(MainWindow);
    d->currentpos = cPos;
    d->lineEdit_flange_x->setText(QString::number(cPos.x, 'd', 2));
    d->lineEdit_flange_y->setText(QString::number(cPos.y, 'd', 2));
    d->lineEdit_flange_z->setText(QString::number(cPos.z, 'd', 2));
    d->lineEdit_flange_rx->setText(QString::number(cPos.rx, 'd', 2));
    d->lineEdit_flange_ry->setText(QString::number(cPos.ry, 'd', 2));
    d->lineEdit_flange_rz->setText(QString::number(cPos.rz, 'd', 2));
}

void MainWindow::slot_get_tcp_wrench(Wrench tWrench) {
    Q_D(MainWindow);
    d->lineEdit_wrench_x->setText(QString::number(tWrench.x, 'd', 2));
    d->lineEdit_wrench_y->setText(QString::number(tWrench.y, 'd', 2));
    d->lineEdit_wrench_z->setText(QString::number(tWrench.z, 'd', 2));
    d->lineEdit_wrench_rx->setText(QString::number(tWrench.rx, 'd', 2));
    d->lineEdit_wrench_ry->setText(QString::number(tWrench.ry, 'd', 2));
    d->lineEdit_wrench_rz->setText(QString::number(tWrench.rz, 'd', 2));
}

void MainWindow::slot_get_tcp_speed(Speed speed) {
    Q_D(MainWindow);
    d->lineEdit_speed_x->setText(QString::number(speed.x * 1000, 'd', 2));
    d->lineEdit_speed_y->setText(QString::number(speed.y * 1000, 'd', 2));
    d->lineEdit_speed_z->setText(QString::number(speed.z * 1000, 'd', 2));
    d->lineEdit_speed_rx->setText(QString::number(speed.rx, 'd', 2));
    d->lineEdit_speed_ry->setText(QString::number(speed.ry, 'd', 2));
    d->lineEdit_speed_rz->setText(QString::number(speed.rz, 'd', 2));
}

void MainWindow::slot_get_safety_status(int status) {
    Q_D(MainWindow);
    d->safety_status = status;
    if (d->safety_status == 3) {
        d->warning_label->setText("机械臂保护性停止！请确认安全后恢复！");
        d->warning_label->show();
    } else if (d->safety_status == 5) {
        d->warning_label->setText("机械臂紧急停止！请确认安全后恢复！");
        d->warning_label->show();
    } else if (d->safety_status == 1) {
        d->warning_label->hide();
    }
}

void MainWindow::slot_btn_teach_clicked() {
    Q_D(MainWindow);
    d->_server->start_freedrive_mode();
}

void MainWindow::slot_btn_fixline_clicked() {
    Q_D(MainWindow);
    // d->_server->start_fixedline_mode();
    // d->fixlinePos = &d->currentpos;
    // d->timer->start(30);
    d->_server->start_screw_driving_mode();
}

void MainWindow::slot_timer_start() {
    Q_D(MainWindow);
    // qDebug() << "fixlinePos.x " << d->fixlinePos->x;
    d->_server->start_fixline_follow_movement(d->fixlinePos, 20, 0, 0, 1);
}

void MainWindow::slot_btn_fasten_clicked() {
    Q_D(MainWindow);
    d->timer->stop();
    d->_server->end_teach_mode();
}

void MainWindow::slot_btn_movej_clicked() {
    Q_D(MainWindow);
    JointPos jpos;
    jpos.j1 = d->lineEdit_input_j1->text().toDouble() * DEG2RAD;
    jpos.j2 = d->lineEdit_input_j2->text().toDouble() * DEG2RAD;
    jpos.j3 = d->lineEdit_input_j3->text().toDouble() * DEG2RAD;
    jpos.j4 = d->lineEdit_input_j4->text().toDouble() * DEG2RAD;
    jpos.j5 = d->lineEdit_input_j5->text().toDouble() * DEG2RAD;
    jpos.j6 = d->lineEdit_input_j6->text().toDouble() * DEG2RAD;
    d->_server->moveRobot_joint(jpos);
}

void MainWindow::slot_btn_movel_clicked() {
    Q_D(MainWindow);
    CartPos cpos;
    cpos.x = d->lineEdit_input_x->text().toDouble() / 1000;
    cpos.y = d->lineEdit_input_y->text().toDouble() / 1000;
    cpos.z = d->lineEdit_input_z->text().toDouble() / 1000;
    cpos.rx = d->lineEdit_input_rx->text().toDouble() * DEG2RAD;
    cpos.ry = d->lineEdit_input_ry->text().toDouble() * DEG2RAD;
    cpos.rz = d->lineEdit_input_rz->text().toDouble() * DEG2RAD;
    d->_server->moveRobot_cart(cpos);
}

void MainWindow::slot_btn_open_mouse_clicked() {
    Q_D(MainWindow);
    // d->_server->open_3DMouse();
}

void MainWindow::slot_btn_close_mouse_clicked() {
    Q_D(MainWindow);
    // d->_server->close_3DMouse();
}

void MainWindow::slot_btn_stopj_clicked() {
    Q_D(MainWindow);
    d->_server->stop_movej();
}

void MainWindow::slot_btn_stopl_clicked() {
    Q_D(MainWindow);
    d->_server->stop_movel();
}

void MainWindow::slot_btn_restart_safety_clicked() {
    Q_D(MainWindow);
    d->_server->restart_robot_safety();
}

void MainWindow::slot_btn_set_payload_clicked() {
    Q_D(MainWindow);
    d->load.m = d->lineEdit_payload_m->text().toDouble();        // 1.61;
    d->load.x = d->lineEdit_payload_x->text().toDouble() / 1000; //-0.004;
    d->load.y = d->lineEdit_payload_y->text().toDouble() / 1000; //-0.002;
    d->load.z = d->lineEdit_payload_z->text().toDouble() / 1000; // 0.061;
    d->_server->set_tool_load(d->load.m, d->load.x, d->load.y, d->load.z);
}

void MainWindow::slot_btn_set_tcp_clicked() {
    Q_D(MainWindow);
    CartPos tcp_pos;
    tcp_pos.x = d->lineEdit_tcp_x->text().toDouble();   //-1.06048;
    tcp_pos.y = d->lineEdit_tcp_y->text().toDouble();   //-18.0221;
    tcp_pos.z = d->lineEdit_tcp_z->text().toDouble();   // 238.052;
    tcp_pos.rx = d->lineEdit_tcp_rx->text().toDouble(); // 0.076072
    tcp_pos.ry = d->lineEdit_tcp_ry->text().toDouble(); // 1.84272;
    tcp_pos.rz = d->lineEdit_tcp_rz->text().toDouble(); // 2.54327;
    d->_server->set_tool_offset(&tcp_pos);
}

void MainWindow::slot_btn_screw_mode_clicked() {
    Q_D(MainWindow);
    d->_server->start_screw_driving_mode();
}

void MainWindow::slot_btn_end_screw_mode_clicked() {
    Q_D(MainWindow);
    d->_server->exit_screw_drving_mode();
}

void MainWindow::slot_button_group(int buttonid) {
    Q_D(MainWindow);
}

void MainWindow::slot_joint_button_group_pressed(int buttonid) {
    Q_D(MainWindow);
    switch (buttonid) {
    case 0:
        d->_server->speedj(-0.2, 0, 0, 0, 0, 0);
        break;
    case 1:
        d->_server->speedj(0, -0.2, 0, 0, 0, 0);
        break;
    case 2:
        d->_server->speedj(0, 0, -0.2, 0, 0, 0);
        break;
    case 3:
        d->_server->speedj(0, 0, 0, -0.2, 0, 0);
        break;
    case 4:
        d->_server->speedj(0, 0, 0, 0, -0.2, 0);
        break;
    case 5:
        d->_server->speedj(0, 0, 0, 0, 0, -0.2);
        break;
    case 6:
        d->_server->speedj(0.2, 0, 0, 0, 0, 0);
        break;
    case 7:
        d->_server->speedj(0, 0.2, 0, 0, 0, 0);
        break;
    case 8:
        d->_server->speedj(0, 0, 0.2, 0, 0, 0);
        break;
    case 9:
        d->_server->speedj(0, 0, 0, 0.2, 0, 0);
        break;
    case 10:
        d->_server->speedj(0, 0, 0, 0, 0.2, 0);
        break;
    case 11:
        d->_server->speedj(0, 0, 0, 0, 0, 0.2);
        break;
    }
}

void MainWindow::slot_joint_button_group_released(int buttonid) {
    Q_D(MainWindow);
    d->_server->stop_movej();
}

void MainWindow::slot_cart_button_group_pressed(int buttonid) {
    Q_D(MainWindow);
    switch (buttonid) {
    case 0:
        d->_server->speedl(-0.02, 0, 0, 0, 0, 0);
        break;
    case 1:
        d->_server->speedl(0.02, 0, 0, 0, 0, 0);
        break;
    case 2:
        d->_server->speedl(0, -0.02, 0, 0, 0, 0);
        break;
    case 3:
        d->_server->speedl(0, 0.02, 0, 0, 0, 0);
        break;
    case 4:
        d->_server->speedl(0, 0, -0.02, 0, 0, 0);
        break;
    case 5:
        d->_server->speedl(0, 0, 0.02, 0, 0, 0);
        break;
    case 6:
        d->_server->speedl(0, 0, 0, -0.5, 0, 0);
        break;
    case 7:
        d->_server->speedl(0, 0, 0, 0.5, 0, 0);
        break;
    case 8:
        d->_server->speedl(0, 0, 0, 0, -0.5, 0);
        break;
    case 9:
        d->_server->speedl(0, 0, 0, 0, 0.5, 0);
        break;
    case 10:
        d->_server->speedl(0, 0, 0, 0, 0, -0.5);
        break;
    case 11:
        d->_server->speedl(0, 0, 0, 0, 0, 0.5);
        break;
    }
}

void MainWindow::slot_cart_button_group_released(int buttonid) {
    Q_D(MainWindow);
    d->_server->stop_movel();
}

void MainWindow::on_btn_tset_clicked() {
    Q_D(MainWindow);
    //    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    //    mat << -1, 0, 0, 72.5,
    //        0, 0.371, -0.928, 97.5,
    //        0, -0.928, -0.371, 185.0,
    //        0, 0, 0, 1;
    //    Eigen::Matrix4d mat2 = Eigen::Matrix4d::Identity();
    //    mat2 = mat.inverse();
    //    std::cout << "mat2: \n"
    //              << mat2 << std::endl;
    // d->_server->move_robot_emc_test();
    // 0.129900, -0.493600, 0.480200, 0.000000, -3.130000, -0.040000  point_start j1: 89.6553  j2:  -83.0749  j3:  113.191  j4:  -118.556  j5:  -90.4036  j6:  -0.444564
    // 0.127980, -0.488720, 0.284790, 0.000000, -3.130000, -0.040000  point_end j1:  89.9326  j2:  -89.5315  j3:  90.2242  j4:  -89.131  j5:  -90.4574  j6:  -0.308186
    CartPos start_pos = {129.900, -493.600, 480.200, 0.000000, -3.130000, -0.040000};
    CartPos end_pos = {127.980, -488.720, 284.790, 0.000000, -3.130000, -0.040000};
    CartPos way_pos;
    way_pos = end_pos;
    Eigen::Vector3d direction, vector_way, vector_start;
    double step = 0.01;
    vector_start[0] = start_pos.x;
    vector_start[1] = start_pos.y;
    vector_start[2] = start_pos.z;
    direction[0] = end_pos.x - start_pos.x;
    direction[1] = end_pos.y - start_pos.y;
    direction[2] = end_pos.z - start_pos.z;
    direction.normalized();
    for (int i = 0; i < 100; i++) {
        vector_way = vector_start + step * direction;
        way_pos.x = vector_way[0];
        way_pos.y = vector_way[1];
        way_pos.z = vector_way[2];
        d->_server->get_inverse_kin(way_pos);
        QThread::msleep(100);
        step += 0.01;
    }
    /*CartPos calpos;
    calpos.x = d->lineEdit_input_x->text().toDouble();
    calpos.y = d->lineEdit_input_y->text().toDouble();
    calpos.z = d->lineEdit_input_z->text().toDouble();
    calpos.rx = d->lineEdit_input_rx->text().toDouble();
    calpos.ry = d->lineEdit_input_ry->text().toDouble();
    calpos.rz = d->lineEdit_input_rz->text().toDouble();
    d->_server->get_inverse_kin(calpos);*/
}

void MainWindow::on_btn_get_current_pos_clicked() {
    Q_D(MainWindow);
    d->record_jpos.j1 = d->current_jpos.j1 * DEG2RAD;
    d->record_jpos.j2 = d->current_jpos.j2 * DEG2RAD;
    d->record_jpos.j3 = d->current_jpos.j3 * DEG2RAD;
    d->record_jpos.j4 = d->current_jpos.j4 * DEG2RAD;
    d->record_jpos.j5 = d->current_jpos.j5 * DEG2RAD;
    d->record_jpos.j6 = d->current_jpos.j6 * DEG2RAD;
}

void MainWindow::on_btn_move_robot_clicked() {
    Q_D(MainWindow);
    d->_server->moveRobot_joint(d->record_jpos);
}

void MainWindow::slot_get_inverse_jpos(JointPos jpos) {
    Q_D(MainWindow);
    if (!d->file.open(QIODevice::ReadWrite | QIODevice::Append)) {
        qDebug() << "Open failed.";
    }
    QTextStream txtOutput(&d->file);
    txtOutput << " " << jpos.j1 << " " << jpos.j2 << " " << jpos.j3 << " " << jpos.j4 << " "
              << jpos.j5 << " " << jpos.j6 << endl;
    d->file.close();
}

void MainWindow::slot_get_forward_cpos(CartPos cpos) {
    Q_D(MainWindow);
    qDebug() << "Forward CartPos: " << cpos.x << " " << cpos.y << " " << cpos.z << " " << cpos.rx << " " << cpos.ry << " " << cpos.rz;
}

void MainWindow::on_btn_test_2_clicked() {
    Q_D(MainWindow);

    static bool enable_test_servoj = true;
    if (enable_test_servoj) {
        d->servo_jpos = d->current_jpos;
        d->timerTest->start(10);
        enable_test_servoj = false;
    } else {
        d->timerTest->stop();
        d->_server->stop_movej();
        enable_test_servoj = true;
    }

    //    CartPos start_pos = {129.900, -493.600, 480.200, 0.000000, -3.130000, -0.040000};
    //    d->_server->get_inverse_kin(start_pos);
    // JointPos calpos;
    // calpos.j1 = d->lineEdit_input_j1->text().toDouble();
    // calpos.j2 = d->lineEdit_input_j2->text().toDouble();
    // calpos.j3 = d->lineEdit_input_j3->text().toDouble();
    // calpos.j4 = d->lineEdit_input_j4->text().toDouble();
    // calpos.j5 = d->lineEdit_input_j5->text().toDouble();
    // calpos.j6 = d->lineEdit_input_j6->text().toDouble();
    // d->_server->get_forward_kin(calpos);
}

void MainWindow::on_btn_test_3_clicked() {
    Q_D(MainWindow);
    // d->_server->move_to_target_joint_speedj(-2, 0.01);
    // d->_server->get_tool_analog_in(1);

    // testServoJ();

    static bool enable_test_servoj = true;
    if (enable_test_servoj) {
        target_joint0_position = 0.2;
        d->servo_jpos.j3 = 90.0;
        d->servo_jpos.j4 = 90.0;
        enable_test_servoj = false;
    } else {
        target_joint0_position = -0.2;
        d->servo_jpos.j3 = -90.0;
        d->servo_jpos.j4 = -90.0;
        enable_test_servoj = true;
    }
}

void MainWindow::testServoJ() {
    Q_D(MainWindow);
    std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};
    JointPos target_space = {90, -90, 90, 0.0, 90, 135};
    // Move to initial joint position with a regular moveJ
    // d->_server->move_robot_joint_space(&target_space);
    d->_server->move_to_target_joint_speedj(target_space, 30 / 1000.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
    for (unsigned int i = 0; i < 30; i++) {
        // target_space.j5 += 0.001 * 180.0 / M_PI;
        // target_space.j6 += 0.001 * 180.0 / M_PI;
        d->_server->move_to_target_joint_speedj(target_space, 30 / 1000.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

void MainWindow::onTestServoBySpeedJTimeOut() {
    Q_D(MainWindow);
    // 4. 控制参数
    double control_frequency = 100.0;                  // 控制频率 [Hz]
    double control_interval = 1.0 / control_frequency; // 控制周期 [s]

    // 5. 目标轨迹参数 (示例：正弦波)
    // double sine_amplitude = 0.2; // 弧度
    // double sine_frequency = 0.5; // Hz
    // static auto program_start_time = std::chrono::steady_clock::now();
    // static auto cycle_start_time = program_start_time;
    // cycle_start_time = std::chrono::steady_clock::now();
    // double current_time = std::chrono::duration_cast<std::chrono::duration<double>>(
    //                           std::chrono::steady_clock::now() - program_start_time)
    //                           .count();
    //
    // double target_joint0_position = sine_amplitude * std::sin(2 * M_PI * sine_frequency * current_time);

    // d->_server->move_to_target_joint_speedj_mine(target_joint0_position, control_interval);

    d->_server->move_to_target_joint_speedj(d->servo_jpos, control_interval);
}