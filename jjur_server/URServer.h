#ifndef URSERVER_H
#define URSERVER_H
#include "PID.h"
#include "URTcp.h"
#include "qthread.h"
#include <QObject>
// #include "URStatus.h"
#include "URBasicMotion.h"
// #include "UR3DMouse.h"
#include "Eigen/Eigen/Core"
#include "Eigen/Eigen/Dense"
#include "Eigen/Eigen/Geometry"
#include "URForceMode.h"
#include "URToolSetting.h"
#include <CommandStruct.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>

enum DIGITAL {
    NO_BTN = 0,
    TEACH_BTN,
    FIXLINE_BTN
};

enum TOOLIO {
    NO_IO = 0,
    FIRST_IO,
    SECOND_IO,
    BOTH_IO
};

enum FORCELEVEL {
    STOP = 0,
    NLOW,
    NHIGH,
    PLOW,
    PHIGH
};

enum FORCEDOF {
    NO_ACTION = 0,
    WHIRL,
    PANNING,
    FIXLINE
};

class URServerPrivate;

class URServer : public QObject {
    Q_OBJECT
  public:
    explicit URServer(QObject *parent = nullptr);
    ~URServer();

    CartPos cpos;
    CartPos cPosl;

    void take_over();
    void hand_over();
    double ntohd(double);
    void end_teach_mode();          // 示教结束
    void moveRobot_joint(JointPos); // 关节坐标移动
    void moveRobot_cart(CartPos);   // 笛卡尔坐标移动
    void stop_movej();
    void stop_movel();
    void jog_movel(int aixstype[6]);
    void force_jog_movel(double movetype[], int dof[6]); // 力控模式下的运动类型
    void exit_force_mode();                              // 力控结束
    void start_screw_driving_mode();                     // 开启稳定模式
    void exit_screw_drving_mode();                       // 退出稳定模式
    void set_tool_digital_out(int index, bool flag);     // 设置工具IO的输出
    void handle_fixline_virtual_wall();
    void start_force_mode(); // 接收前端界面上的力控按钮发出的信号开启力控模式
    void speedj(double, double, double, double, double, double);
    void speedl(double, double, double, double, double, double);
    void start_singularity_check();
    void stop_singularity_check();
    void set_robot_type(ROBOT_TYPE);
    void start_special_singularity_check();
    void stop_special_singularity_check();
    void start_fixpoint_singularity_check();
    void stop_fixpoint_singularity_check();
    void set_robot_force_damping(double); // 设置力控的阻尼

    /**********************通信**********************/
    // 通过tcp向控制柜发指令
    void send_command(QString cmd);
    void send_command(char *cmd);

    /**********矩阵转换相关接口*****************/

    // 将UR数据转换为末端到法兰的转换矩阵---------旋转矢量转矩阵
    Eigen::Matrix4d rotation_vector_urpose_to_matrix(CartPos &cpos);
    // 将末端到法兰的转换矩阵转换为UR的数据格式
    CartPos matrix_to_urpose_rotation_vector(Eigen::Matrix4d &transform); // 旋转矩阵转换成旋转矢量
    // 获取转换矩阵T中的旋转矩阵R
    Eigen::Matrix3d get_rotation(Eigen::Matrix4d T);
    // 获取转换矩阵T中位移向量t
    Eigen::Vector3d get_translation(Eigen::Matrix4d T);
    // 通过旋转矩阵R和位移向量t获取转换矩阵T
    Eigen::Matrix4d build_transformation(Eigen::Matrix3d R, Eigen::Vector3d t);

    // Sirius调用的接口
    void start_robot();                      // 启动机械臂（即示教板power on以及打开抱闸）
    void connect_to_robot();                 // 连接机械臂
    void shutdown_robot();                   // 机械臂关机
    void unlock_protective_stop();           // 解锁保护性停止
    void move_robot_joint_space(JointPos *); // 机械臂关节运动
    void move_robot_cart_space(CartPos *);   // 机械臂直线运动
    void stop_robot_movement();              // 机械臂停止运动
    void stop_robot_movement_and_sleep();
    void set_tool_load(double, double, double, double); // 设置工具重量及重心
    void set_tool_offset(CartPos *);                    // 设置工具机械参数
    void set_robot_max_speed_t(double);
    void set_robot_max_speed_r(double);
    void start_freedrive_mode();                                                        // 开启机械臂自由驱动模式
    void move_robot_to_target_pos(CartPos *);                                           // 机械臂主动运动的位置
    void move_robot_to_target_pos_within_time(CartPos *);                               // 机械臂在规定时间内主动运动
    void move_robot_to_hip_wall(CartPos *);                                             // 机械臂运动到虚拟墙的位置
    void servo_target_joint(JointPos *js);                                              // 机械臂关节位置伺服
    void set_virtual_wall_pos(CartPos *);                                               // 设置限深点的位置坐标
    void start_fixedline_mode(CartPos *);                                               // 开启定线模式
    void restart_robot_safety();                                                        // 重启安全检测
    void start_fixplane_mode(CartPos *cs, int x, int y, int z, int rx, int ry, int rz); // 开启定面模式--CartPos表示工作位置，x,y,z表示选定哪个平面
    void start_fixplane_follow_movement(CartPos *target_pos, CartPos *speed_pos, int x, int y, int z, int rx, int ry, int rz);
    void start_fixplane_mode_with_speed(CartPos *cs, int x, int y, int z, int rx, int ry, int rz, double speed_plan); // 开启定面模式--CartPos表示工作位置，x,y,z表示选定哪个平面
    void start_fixplane_follow_movement_with_speed(CartPos *target_pos, CartPos *speed_pos, int x, int y, int z, int rx, int ry, int rz, double speed_plan);
    void start_fixline_follow_movement(CartPos *, double, int x, int y, int z); // 定线随动的接口，第二个参数为设置Z轴速度
    void start_fixline_movement(CartPos *cs, double speed);
    // 正位使用speedl的方式，运动过程中解决机械臂异响
    void move_robot_to_target_pos_with_speedl(CartPos *cs);
    // 实时控制定线随动force_mode+speedl的功能，上位机实时刷新指令
    void start_forcemode_speedl_movement(CartPos *, double, int, CartPos *);
    // 从导航软件发送目标位置
    void move_to_target_pos_with_speedl(CartPos *cs);
    // 从导航软件发送定线的frame、剩余深度、定线轴以及目标位置
    void start_forcemode_speedl_follow_movement(CartPos *target2base, double, int);
    // 从导航软件发送定点的frame、剩余深度、定线轴以及目标位置
    void start_forcemode_fixpoint_follow_movement(CartPos *target2base, double, int);
    // 实时控制定线+定点随动的功能，上位机实时刷新指令
    void start_forcemode_fixpoint_movement(CartPos *, double, int, CartPos *);
    // 从导航软件发送定线的frame、剩余深度、定线轴以及目标位置，只随动位置不随动角度
    void start_forcemode_speedlpos_movement(CartPos *target2base, double, int);

    void move_robot_emc_test();

    void get_inverse_kin(CartPos);                             // 传入目标位置的笛卡尔空间坐标
    void get_forward_kin(JointPos);                            // 传入关节角度，获取笛卡尔空间坐标
    double calculate_pid_speed_t_iterate(double offset);       // 计算位移的PID控制器
    double calculate_pid_speed_r_iterate(double offset);       // 计算姿态的PID控制器
    double calculate_pid_fixline_speed_iterate(double offset); // 计算定线速度的PID控制器
    void get_tool_analog_in(int);                              // 传入获取末端模拟量输入的Index

  signals:
    void signal_new_connection(quint16);
    void signal_disconnect();
    void signal_update_joint_pos(JointPos); // 更新机械臂关节位置
    void signal_update_cart_pos(CartPos);   // 更新机械臂TCP坐标
    void signal_update_tcp_wrench(Wrench);  // 更新机械臂Wrench
    void signal_update_tcp_speed(Speed);    // 更新机械臂速度
    void signal_start_force_mode();
    void signal_update_button_status(int);
    void signal_send_command(const char *);
    void signal_update_safety_status(int);  // 将机械臂当前的安全状态信息发到前端显示
    void signal_send_motion_status(int);    // 发送运动状态，true为开始运动/false为运动结束
    void signal_send_connect_status(int);   // 将机械臂的连接状态发送到前端显示
    void signal_send_tool_io_status(int);   // 更新末端工具IO的状态
    void signal_update_target_pos(CartPos); // 更新目标笛卡尔坐标

    void signal_answer(QString);                  // 更新Ur的返回值，在上层程序中进行处理
    void signal_singularity_type(int);            // 发送奇异点状态
    void signal_is_near_singularity_pos(bool);    // 发送计算的目标位置是否会到达奇异点
    void signal_send_inverse_jpos(JointPos);      // 发送运动学反解得到的关节角度
    void signal_send_forward_cpos(CartPos);       // 发送运动学正解得到的笛卡尔坐标
    void signal_special_singularity_type(int);    // 发送特殊奇异点状态
    void signal_send_robot_score(double);         // 发送机械臂姿态分数
    void signal_fixpoint_singularity_type(int);   // 发送定点模式的机械臂奇异点类型
    void signal_send_tool_analog_in(double);      // 发送机械臂工具IO模拟量输入值
    void signal_send_30002_command(const char *); // 向30002端口发送非运动指令
    void signal_connect_30002_port();             // 连接机械臂30002端口

  public slots:
    void slot_update_joint_pos(JointPos);    // 从URTcp中获取关节值
    void slot_update_cart_pos(CartPos);      // 从URTcp中获取笛卡尔坐标值
    void slot_update_tcp_wrench(Wrench);     // 从URTcp中获取wrench值
    void slot_get_connect_status(int);       // 从URTcp中获取连接状态
    void slot_update_tcp_speed(Speed);       // 从URTcp中获取工具点的速度
    void slot_update_digital_input(double);  // 从URTcp中获取工具IO的状态
    void slot_update_safety_status(double);  // 从URTcp中获取当前的安全状态，需要将这个状态发给前端显示，若出现保护性停止，前端页面出现相应的提示
    void slot_update_programe_state(double); // 从URTcp中获取当前机械臂模式
    void slot_restart_robot();               // 从前端获取重启机械臂的信号
    void slot_start_teach_mode(bool);        // 接收前端界面上的示教按钮发出的信号开启示教或者停止示教
    void slot_update_target_pos(CartPos);    // 从URTcp中获取目标笛卡尔坐标值

    void slot_recieve_force_mode(int);
    void slot_force_timer_start();
    void slot_send_timer_start();
    void slot_detect_timer_start();
    void slot_special_detect_timer_start();
    void slot_get_inverse_jpos(QByteArray);
    void slot_get_forward_cpos(QByteArray);
    void slot_detect_robot_pos_timer_start();
    void slot_calculate_flange2base();
    void slot_fixpoint_singularity_detect_timer_start();
    void slot_get_tool_analog_in(QByteArray);

    void testSpeedJ();
    void move_to_target_joint_speedj(JointPos target_joint, double control_interval = 0.01);
    void move_to_target_joint_speedj_mine(double target_joint, double control_interval);
    void getTargetAcce(double currPos, double targetPos, double currVel, double time, double &requiredAcce, double &requiredVel);

  protected:
    QScopedPointer<URServerPrivate> d_ptr;

  private:
    Q_DECLARE_PRIVATE(URServer)
    Q_DISABLE_COPY(URServer)
};

#endif // URSERVER_H
