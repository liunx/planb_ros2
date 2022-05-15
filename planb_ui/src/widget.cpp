#include "widget.h"
#include "./ui_widget.h"

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , robot_({})
{
    ui->setupUi(this);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_servoDial_valueChanged(int value)
{
    ui->servoLabel->setNum(value);
    robot_.servo.angle = (int32_t)value;
    emit signal_robot(robot_);
}

void Widget::on_motorDial_valueChanged(int value)
{
    ui->motorLabel->setNum(value);
    robot_.motor.accel = (int32_t)value;
    emit signal_robot(robot_);
}

void Widget::on_servoLF_stateChanged(int arg1)
{
    robot_.servo.left_front = (uint8_t)arg1;
}

void Widget::on_servoLT_stateChanged(int arg1)
{
    robot_.servo.left_tail = (uint8_t)arg1;
}

void Widget::on_servoRF_stateChanged(int arg1)
{
    robot_.servo.right_front = (uint8_t)arg1;
}

void Widget::on_servoRT_stateChanged(int arg1)
{
    robot_.servo.right_tail = (uint8_t)arg1;
}

void Widget::on_motorLF_stateChanged(int arg1)
{
    robot_.motor.left_front = (uint8_t)arg1;
}

void Widget::on_motorLM_stateChanged(int arg1)
{
    robot_.motor.left_middle = (uint8_t)arg1;
}

void Widget::on_motorLT_stateChanged(int arg1)
{
    robot_.motor.left_tail = (uint8_t)arg1;
}

void Widget::on_motorRF_stateChanged(int arg1)
{
    robot_.motor.right_front = (uint8_t)arg1;
}

void Widget::on_motorRM_stateChanged(int arg1)
{
    robot_.motor.right_middle = (uint8_t)arg1;
}

void Widget::on_motorRT_stateChanged(int arg1)
{
    robot_.motor.right_tail = (uint8_t)arg1;
}

void Widget::on_normalMode_toggled(bool checked)
{
    if (checked)
    {
        robot_.mode = NORMAL_MODE;
        recover_servo();
        recover_motor();
        emit signal_robot(robot_);
    }
    else
    {
        recover_ = robot_;
    }
}

void Widget::on_circleMode_toggled(bool checked)
{
    if (checked)
    {
        robot_.mode = CIRCLE_MODE;
        ui->servoDial->setDisabled(true);
        disable_servo();
        disable_motor();
        emit signal_robot(robot_);
    }
    else
    {
        ui->servoDial->setEnabled(true);
    }
}

void Widget::on_arckermanMode_toggled(bool checked)
{
    if (checked)
    {
        robot_.mode = ARCKERMAN_MODE;
        disable_servo();
        disable_motor();
        emit signal_robot(robot_);
    }
}

void Widget::disable_servo()
{
    ui->servoLF->setChecked(true);
    ui->servoLF->setDisabled(true);
    ui->servoLT->setChecked(true);
    ui->servoLT->setDisabled(true);
    ui->servoRF->setChecked(true);
    ui->servoRF->setDisabled(true);
    ui->servoRT->setChecked(true);
    ui->servoRT->setDisabled(true);
}

void Widget::recover_servo()
{
    ui->servoLF->setEnabled(true);
    ui->servoLT->setEnabled(true);
    ui->servoRF->setEnabled(true);
    ui->servoRT->setEnabled(true);
    ui->servoDial->setEnabled(true);

    if (recover_.servo.left_front == 0)
        ui->servoLF->setChecked(false);
    if (recover_.servo.left_tail == 0)
        ui->servoLT->setChecked(false);
    if (recover_.servo.right_front == 0)
        ui->servoRF->setChecked(false);
    if (recover_.servo.right_tail == 0)
        ui->servoRT->setChecked(false);
}

void Widget::disable_motor()
{
    ui->motorLF->setChecked(true);
    ui->motorLF->setDisabled(true);
    ui->motorLM->setChecked(true);
    ui->motorLM->setDisabled(true);
    ui->motorLT->setChecked(true);
    ui->motorLT->setDisabled(true);
    ui->motorRF->setChecked(true);
    ui->motorRF->setDisabled(true);
    ui->motorRM->setChecked(true);
    ui->motorRM->setDisabled(true);
    ui->motorRT->setChecked(true);
    ui->motorRT->setDisabled(true);
}

void Widget::recover_motor()
{
    ui->motorLF->setEnabled(true);
    ui->motorLM->setEnabled(true);
    ui->motorLT->setEnabled(true);
    ui->motorRF->setEnabled(true);
    ui->motorRM->setEnabled(true);
    ui->motorRT->setEnabled(true);

    if (recover_.motor.left_front == 0)
        ui->motorLF->setChecked(false);
    if (recover_.motor.left_middle == 0)
        ui->motorLM->setChecked(false);
    if (recover_.motor.left_tail == 0)
        ui->motorLT->setChecked(false);
    if (recover_.motor.right_front == 0)
        ui->motorRF->setChecked(false);
    if (recover_.motor.right_middle == 0)
        ui->motorRM->setChecked(false);
    if (recover_.motor.right_tail == 0)
        ui->motorRT->setChecked(false);
}

void Widget::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Escape)
    {
        servo_dial_ = 0;
        motor_dial_ = 0;
        ui->motorDial->setValue(servo_dial_);
        ui->servoDial->setValue(motor_dial_);
    }
    else if (event->key() == Qt::Key_Q)
    {
        motor_dial_ = 0;
        ui->motorDial->setValue(servo_dial_);
    }
    else if (event->key() == Qt::Key_E)
    {
        servo_dial_ = 0;
        ui->servoDial->setValue(motor_dial_);
    }
    else if (event->key() == Qt::Key_A)
    {
        servo_dial_ -= 5;
        if (servo_dial_ < -90)
            servo_dial_ = -90;
        ui->servoDial->setValue(servo_dial_);
    }
    else if (event->key() == Qt::Key_D)
    {
        servo_dial_ += 5;
        if (servo_dial_ > 90)
            servo_dial_ = 90;
        ui->servoDial->setValue(servo_dial_);
    }
    else if (event->key() == Qt::Key_W)
    {
        motor_dial_ += 5;
        if (motor_dial_ > 0 && motor_dial_ < MIN_ACCEL)
            motor_dial_ = MIN_ACCEL;
        else if (motor_dial_ < 0 && motor_dial_ > -MIN_ACCEL)
            motor_dial_ = 0;
        else if (motor_dial_ > 100)
            motor_dial_ = 100;
        ui->motorDial->setValue(motor_dial_);
    }
    else if (event->key() == Qt::Key_S)
    {
        motor_dial_ -= 5;
        if (motor_dial_ > -MIN_ACCEL && motor_dial_ < 0)
            motor_dial_ = -MIN_ACCEL;
        else if (motor_dial_ < MIN_ACCEL && motor_dial_ > 0)
            motor_dial_ = 0;
        else if (motor_dial_ < -100)
            motor_dial_ = -100;
        ui->motorDial->setValue(motor_dial_);
    }
}

void Widget::on_close()
{
    this->close();
}