#ifndef URFORCEMODE_H
#define URFORCEMODE_H

#include <QObject>
#include "qthread.h"
#include "URTcp.h"

class URForceModePrivate;
class URForceMode : public QObject
{
    Q_OBJECT
public:
    explicit URForceMode(QObject *parent = nullptr, URTcp *tcpServer = nullptr);
    ~URForceMode();
    void end_force_mode();

public slots:
    /******force_mode(task_frame, selection_vector, wrench, type, limits)------参数解释******
    *** task_frame: A pose vector that defines the force frame relative to the base frame.---
    *** 定义相对于基础框架的力框架的姿势向量*****
    *** selection_vector: A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame.---
    *** 一个由 0 和 1 组成的 6d 向量。 1 表示机器人将在任务框架的相应轴上顺从，即在该方向可以运动******
    *** wrench: The forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in
    *** order to achieve the specified force/torque. Values have no effect for non-compliant axes.---
    *** 机器人将应用于其环境的力/扭矩。 机器人沿/围绕柔顺轴调整其位置，以达到指定的力/扭矩。 值对不符合要求的轴没有影响。x,y,z是力，rx,ry,rz是力矩
    *** Actual wrench applied may be lower than requested due to joint safety limits. Actual forces and torques can be read
    *** using get_tcp_force function in a separate thread.---
    *** 由于接头安全限制，实际使用的Wrench可能低于要求。 可以在单独的线程中使用 get_tcp_force 函数读取实际力和扭矩。
    *** type: An integer [1;3] specifying how the robot interprets the force frame.---
    *** 一个整数 [1；3] 指定机器人如何解释力框架。
    *** 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing from the
    *** robot tcp towards the origin of the force frame.---
    *** 力系以某种方式变换，使其 y 轴与从机器人 tcp 指向力系原点的向量对齐。
    *** 2: The force frame is not transformed.---
    *** 力系不变形
    *** 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp velocity
    *** vector onto the x-y plane of the force frame.---
    *** 力坐标系以某种方式变换，使得其 x 轴是机器人 tcp 速度矢量在力坐标系 x-y 平面上的投影。
    *** limits: (Float) 6d vector. For compliant axes, these values are the maximum allowed tcp speed along/about
    *** the axis. For non-compliant axes, these values are the maximum allowed deviation along/about an axis between
    *** the actual tcp position and the one set by the program.---
    *** （浮点型6d向量。对于兼容轴，这些值是沿/围绕轴的最大允许tcp速度。对于非兼容轴，这些值是实际tcp位置与程序设置的位置之间沿/围绕轴的最大允许偏差。
    *** Note: Avoid movements parallel to compliant axes and high deceleration (consider inserting a short sleep
    *** command of at least 0.02s) just before entering force mode. Avoid high acceleration in force mode as
    *** this decreases the force control accuracy.---
    *** 注意：在进入强制模式之前，避免平行于顺应轴的运动和高减速（考虑插入至少 0.02 秒的短睡眠命令）。避免力模式下的高加速度，因为这会降低力控制精度。
    *****************************************************************/
    void slot_start_force_mode();

protected:
    QScopedPointer<URForceModePrivate> d_ptr;

private:
    Q_DECLARE_PRIVATE(URForceMode)
    Q_DISABLE_COPY(URForceMode)

};

#endif // URFORCEMODE_H
