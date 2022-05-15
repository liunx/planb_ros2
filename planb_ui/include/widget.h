#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QKeyEvent>
#include "common.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();
    void keyPressEvent(QKeyEvent *event);

private slots:

    void on_servoDial_valueChanged(int value);

    void on_motorDial_valueChanged(int value);

    void on_servoLF_stateChanged(int arg1);

    void on_servoLT_stateChanged(int arg1);

    void on_servoRF_stateChanged(int arg1);

    void on_servoRT_stateChanged(int arg1);

    void on_motorLF_stateChanged(int arg1);

    void on_motorLM_stateChanged(int arg1);

    void on_motorLT_stateChanged(int arg1);

    void on_motorRF_stateChanged(int arg1);

    void on_motorRM_stateChanged(int arg1);

    void on_motorRT_stateChanged(int arg1);

    void on_normalMode_toggled(bool checked);

    void on_circleMode_toggled(bool checked);

    void on_arckermanMode_toggled(bool checked);

    void on_close();

signals:
    void signal_robot(Robot robot);

private:
    void disable_servo();
    void recover_servo();
    void disable_motor();
    void recover_motor();

private:
    Ui::Widget *ui;
    Robot robot_;
    Robot recover_;
    int servo_dial_ = 0;
    int motor_dial_ = 0;
};
#endif // WIDGET_H
