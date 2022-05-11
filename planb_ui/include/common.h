#ifndef COMMON_H
#define COMMON_H

#define MIN_ACCEL 60

struct Servo {
    int32_t angle;
    uint8_t left_front;
    uint8_t left_tail;
    uint8_t right_front;
    uint8_t right_tail;
};

struct Motor {
    int32_t accel;
    uint8_t left_front;
    uint8_t left_middle;
    uint8_t left_tail;
    uint8_t right_front;
    uint8_t right_middle;
    uint8_t right_tail;
};

struct Robot {
    uint8_t mode;
    Servo servo;
    Motor motor;
};

#endif // COMMON_H
