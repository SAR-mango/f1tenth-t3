#pragma once

namespace car_config
{
    constexpr double PI = 3.14159265358979323846;
    constexpr double DEG_TO_RAD = PI / 180;

    constexpr double WHEELBASE = 0.325;
    constexpr double WHEEL_DIAMETER = 0.098;
    constexpr double WHEEL_WIDTH = 0.042;
    constexpr double FRONT_WHEEL_DISTANCE = 0.23;
    constexpr double REAR_WHEEL_DISTANCE = 0.233;
    constexpr double WHEEL_PERIMETER = WHEEL_DIAMETER * PI;
    constexpr double TURNING_RADIUS = 0.605;

    constexpr double MAX_RPM_MECHANICAL = 60000;
    constexpr double MOTOR_POLES = 3;
    constexpr double MAX_RPM_ELECTRICAL = MAX_RPM_MECHANICAL / MOTOR_POLES;

    constexpr double ERPM_TO_SPEED = WHEEL_PERIMETER * MOTOR_POLES / 60;
    constexpr double SPEED_TO_ERPM = 1 / ERPM_TO_SPEED;
    constexpr double RPM_TO_SPEED = WHEEL_PERIMETER / 60;

    constexpr double STEERING_TO_SERVO_OFFSET = 0.5;
    constexpr double STEERING_TO_SERVO_GAIN = -3 / PI;
    constexpr double MAX_STEERING_ANGLE = 30 * DEG_TO_RAD;
    constexpr double MIN_STEERING_ANGLE = -30 * DEG_TO_RAD;

    constexpr double TRANSMISSION = 20;
    constexpr double MAX_SERVO_POSITION = 1;
    constexpr double ERPM_TO_RAD_PER_SEC = MOTOR_POLES * 2 * PI / 60;
} // namespace car_config
