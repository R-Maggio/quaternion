#include "Quaternion.hpp"

// print

std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
    os  << "[w:" << q.qw
        << ", x:" << q.qv[0]
        << ", y:" << q.qv[1]
        << ", z:" << q.qv[2]
        << "]";
    return os;
}

// constructors

Quaternion::Quaternion(const VectorND::Vector<double, 3>& axis, double angle) {
    double half_angle = angle / 2.0;
    qw = std::cos(half_angle);
    qv = axis * std::sin(half_angle);
}

VectorND::Vector<double, 3> Quaternion::toEulerAngles() const {
    Vector3D eulerAngles;
    // roll
    double sin_roll = 2 * (qw * qv[0] + qv[1] * qv[2]);
    double cos_roll = 1 - 2 * (qv[0] * qv[0] + qv[1] * qv[1]);
    eulerAngles[0] = std::atan2(sin_roll, cos_roll);

    // pitch
    double pitch_help = 2 * (qw * qv[1] - qv[2] * qv[0]);
    if (std::abs(pitch_help) >= 1) {
        eulerAngles[1] = std::copysign(M_PI/2, pitch_help); // +- pi/2
    } else {
        eulerAngles[1] = std::asin(pitch_help);
    }

    // yaw
    double sin_yaw = 2 * (qw * qv[2] + qv[0] * qv[1]);
    double cos_yaw = 1 - 2 * (qv[1] * qv[1] + qv[2] * qv[2]);
    eulerAngles[2] = std::atan2(sin_yaw, cos_yaw);

    return eulerAngles;
}