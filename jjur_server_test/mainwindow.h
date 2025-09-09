#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "URServer.h"
#include <QButtonGroup>
#include <QMainWindow>
#include <QThread>
#include <QTimer>

class MainWindowPrivate;

class MainWindow : public QMainWindow {
    Q_OBJECT
  public:
    explicit MainWindow(QWidget *parent = nullptr, URServer *urserver = nullptr);
    ~MainWindow();

    void take_over();
    void hand_over();

  public slots:
    void slot_btn_connectUR_clicked();
    void slot_btn_teach_clicked();
    void slot_btn_fixline_clicked();
    void slot_btn_fasten_clicked();
    void slot_btn_movej_clicked();
    void slot_btn_movel_clicked();
    void slot_btn_open_mouse_clicked();
    void slot_btn_close_mouse_clicked();
    void slot_btn_stopj_clicked();
    void slot_btn_stopl_clicked();
    void slot_btn_restart_safety_clicked();
    void slot_btn_set_payload_clicked();
    void slot_btn_set_tcp_clicked();
    void slot_btn_screw_mode_clicked();
    void slot_btn_end_screw_mode_clicked();
    void slot_get_joint_pos(JointPos);
    void slot_get_cart_pos(CartPos);
    void slot_get_tcp_wrench(Wrench);
    void slot_get_tcp_speed(Speed);
    void slot_get_safety_status(int);
    void slot_button_group(int);
    void slot_btn_start_robot_clicked();
    void slot_btn_shutdown_clicked();
    void slot_btn_unlock_clicked();
    void slot_joint_button_group_pressed(int);
    void slot_joint_button_group_released(int);
    void slot_cart_button_group_pressed(int);
    void slot_cart_button_group_released(int);
    void on_btn_tset_clicked();
    void slot_timer_start();
    void slot_get_inverse_jpos(JointPos);
    void slot_get_forward_cpos(CartPos);

  private slots:
    void on_btn_get_current_pos_clicked();
    void on_btn_move_robot_clicked();
    void on_btn_test_2_clicked();
    void on_btn_test_3_clicked();
    void testServoJ();
    void onTestServoBySpeedJTimeOut();

  protected:
    QScopedPointer<MainWindowPrivate> d_ptr;

  private:
    Q_DECLARE_PRIVATE(MainWindow)
    Q_DISABLE_COPY(MainWindow)

    double target_joint0_position = 0.0;
};

#endif // MAINWINDOW_H
