#include <iostream>
// PID控制器结构体定义
struct PID {
    double Kp, Ki, Kd;    // PID参数
    double target;        // 目标值
    double measure;       // 测量值
    double error[3];      // 当前、上次、上上次误差
    double integral;      // 积分项
    double output;        // 输出值
    double maxOutput;     // 输出限幅
    double integralLimit; // 积分限幅

    // 初始化PID参数
    PID(double p, double i, double d, double maxOut, double iLimit)
        : Kp(p), Ki(i), Kd(d), target(0), measure(0), integral(0), output(0),
          maxOutput(maxOut), integralLimit(iLimit) {
        error[0] = error[1] = error[2] = 0;
    }

    // 计算PID输出
    double calculate(double setpoint, double feedback, double dt) {
        target = setpoint;
        measure = feedback;
        error[0] = target - measure; // 计算当前误差

        // 积分项计算与限幅
        integral += error[0] * dt;
        if (integral > integralLimit)
            integral = integralLimit;
        if (integral < -integralLimit)
            integral = -integralLimit;

        // 微分项（使用一阶后向差分）
        double derivative = (error[0] - error[1]) / dt;

        // PID输出计算
        output = Kp * error[0] + Ki * integral + Kd * derivative;

        // 输出限幅
        if (output > maxOutput)
            output = maxOutput;
        if (output < -maxOutput)
            output = -maxOutput;

        // 更新误差历史
        error[2] = error[1];
        error[1] = error[0];

        return output;
    }

    // 重置PID状态
    void reset() {
        integral = 0;
        error[0] = error[1] = error[2] = 0;
    }
};

// 串级PID控制器
class CascadePID {
  private:
    PID outer; // 外环（位置环）
    PID inner; // 内环（速度环）
    double dt; // 控制周期（秒）

  public:
    CascadePID(double outerP, double outerI, double outerD,
               double innerP, double innerI, double innerD,
               double sampleTime, double maxOut, double iLimit)
        : outer(outerP, outerI, outerD, maxOut, iLimit),
          inner(innerP, innerI, innerD, maxOut, iLimit),
          dt(sampleTime) {}

    // 计算串级PID输出
    double compute(double targetPosition, double currentPosition, double currentVelocity) {
        // 外环（位置环）计算速度设定值
        double speedSetpoint = outer.calculate(targetPosition, currentPosition, dt);
        // std::cout << " | speedSetpoint: " << speedSetpoint << std::endl;
        // 内环（速度环）计算最终控制输出
        double controlOutput = inner.calculate(speedSetpoint, currentVelocity, dt);

        return controlOutput;
    }

    // 重置所有PID状态
    void reset() {
        outer.reset();
        inner.reset();
    }
};

class PIDController {
  public:
    PIDController(double kp, double ki, double kd, double setpoint = 0)
        : kp_(kp), ki_(ki), kd_(kd), setpoint_(setpoint), prev_error_(0), integral_(0) {}

    // 更新 PID 输出
    double update(double measured_value, double dt) {
        double error = setpoint_ - measured_value;
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = error;
        return output;
    }

    // 设置目标值
    void setSetpoint(double setpoint) {
        setpoint_ = setpoint;
    }

  private:
    double kp_;
    double ki_;
    double kd_;
    double setpoint_;
    double prev_error_;
    double integral_;
};

#pragma once