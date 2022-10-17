#pragma once

#include "Vector.hpp"
// TODO: add more operators

/**
 * @brief compute the cross product of two vectors
 * 
 * @param v1 the first vector
 * @param v2 the second vector
 */
inline VectorND::Vector<double, 3> cross(const VectorND::Vector<double, 3>& v1, const VectorND::Vector<double, 3>& v2);

/**
 * @brief a quaternion class used to represent the orientation of a 3D body
 */
class Quaternion {
    using Vector3D = VectorND::Vector<double, 3>;
    // q = [w, x, y, z]
    double qw; // real part
    Vector3D qv; // vector part

public:
    // constructors
    Quaternion(): qw{1.0}, qv{} {}
    Quaternion(double w, const Vector3D& v): qw{w}, qv{v} {}
    Quaternion(double w, double x, double y, double z): qw{w}, qv{{x, y, z}} {}
    /**
     * @brief construct a quaternion from an axis and an angle
     * 
     * @param axis the axis of rotation. It must be a unit vector
     * @param angle the angle of rotation in radians
     * 
     * @return Quaternion the quaternion representing the rotation
     */
    Quaternion(const Vector3D& axis, double angle);

    // getters

    /**
     * @brief get the real part of the quaternion
     * 
     * @return double the real part
     */
    double getRealPart() const { return qw; }

    /**
     * @brief get the vector part of the quaternion
     * 
     * @return Vector3D the vector part
     */
    Vector3D getVectorPart() const { return qv; }

    // members

    /**
     * @brief element access operator  (write)
     * 
     * @param i the index of the element
     */
    inline double& operator[](size_t i);

    /**
     * @brief element access operator (read)
     * 
     * @param i the index of the element
     * @return const double& the element
     */
    inline double operator[](size_t i) const;

    /**
     * @brief element access (read) with bouds checking
     * 
     * @param i the index of the element
     * @return double the element
     */
    inline double at(size_t i) const;

    /**
     * @brief normalize the quaternion
     * 
     */
    inline void normalize();

    /**
     * @brief compute the conjugate of the quaternion
     * 
     * @return Quaternion the conjugate of the quaternion
     */
    inline Quaternion conjugate() const;

    /**
     * @brief compute the dot product of two quaternions
     * 
     * @param q the other quaternion
     * @return double the dot product of the two quaternions
     */
    inline double dot(const Quaternion& q);

    /**
     * @brief compute the addition of two quaternions
     *  
     * @param q the quaternion to add
     * 
     * @return Quaternion the sum of the two quaternions
     */
    inline Quaternion operator+(const Quaternion& q) const;

    /**
     * @brief add a quaternion to the current quaternion
     * 
     * @param q the quaternion to add
     * 
     * @return Quaternion& the current quaternion
     */
    inline Quaternion& operator+=(const Quaternion& q);

    /**
     * @brief compute the subtraction of two quaternions
     * 
     * @param q the quaternion to subtract
     * 
     * @return Quaternion the difference of the two quaternions
     */
    inline Quaternion operator-(const Quaternion& q) const;

    /**
     * @brief subtract a quaternion from the current quaternion
     * 
     * @param q the quaternion to subtract
     * 
     * @return Quaternion& the current quaternion
     */
    inline Quaternion& operator-=(const Quaternion& q);

    /**
     * @brief compute the multiplication of two quaternions
     * 
     * @param q the quaternion to multiply
     * 
     * @return Quaternion the product of the two quaternions
     */
    inline Quaternion operator*(const Quaternion& q) const;

    /**
     * @brief multiply the current quaternion by another quaternion
     * 
     * @param q the other quaternion
     * 
     */
    inline Quaternion& operator*=(const Quaternion& q);

    /**
     * @brief compute the multiplication of a quaternion by a scalar
     * 
     * @param s the scalar
     * 
     * @return Quaternion the product of the quaternion and the scalar
     */
    inline Quaternion operator*(double s) const;

    /**
     * @brief multiply the current quaternion by a scalar
     * 
     * @param s the scalar
     * 
     * @return Quaternion& the current quaternion
     */
    inline Quaternion& operator*=(double s);

    /**
     * @brief compute the multiplication of a scalar by a quaternion
     * 
     * @param s the scalar
     * @param q the quaternion
     * 
     * @return Quaternion the product of the scalar and the quaternion
     */
    inline friend Quaternion operator*(double s, const Quaternion& q);

    /**
     * @brief compute the squared norm of the quaternion
     * 
     */
    inline double squaredNorm() const;

    /**
     * @brief compute the norm of the quaternion
     * 
     */
    inline double norm() const;

    /**
     * @brief equality operator
     * 
     * @param q the other quaternion
     * 
     */
    inline bool operator==(const Quaternion& q) const;

    /**
     * @brief inequality operator
     * 
     * @param q the other quaternion
     * 
     */
    inline bool operator!=(const Quaternion& q) const;

    // /**
    //  * @brief rotate a quaternion by another quaternion
    //  * 
    //  */
    // inline Quaternion rotate(const Quaternion& q) const;
    
    /**
     * @brief rotate a vector by a quaternion
     * 
     * @param v the vector
     * 
     * @return Vector3D the rotated vector
     */
    inline Vector3D rotate(const Vector3D& v) const;

    /**
     * @brief transform a quaternion to euler angles
     * 
     * @return Vector3D the euler angles
     */
    Vector3D toEulerAngles() const;

    /**
     * @brief print the quaternion
     * 
     * @param os the output stream
     * @param q the quaternion
     * 
     * @return std::ostream& the output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q);
};


//************ Implementation ************//

// cross product of two vectors

inline VectorND::Vector<double, 3> cross(const VectorND::Vector<double, 3>& v1, const VectorND::Vector<double, 3>& v2) {
    VectorND::Vector<double, 3> result;
    result[0] = v1[1] * v2[2] - v1[2] * v2[1];
    result[1] = v1[2] * v2[0] - v1[0] * v2[2];
    result[2] = v1[0] * v2[1] - v1[1] * v2[0];
    return result;
}

// access to the components

inline double& Quaternion::operator[](size_t i) {
    if (i == 0) {
        return qw;
    } else {
        return qv[i - 1];
    }
}

inline double Quaternion::operator[](size_t i) const {
    if (i == 0) {
        return qw;
    } else {
        return qv[i - 1];
    }
}

inline double Quaternion::at(size_t i) const {
    if (i == 0) {
        return qw;
    } else {
        return qv.at(i - 1);
    }
}

// conjugate

inline Quaternion Quaternion::conjugate() const {
    return Quaternion(qw, -qv);
}

inline double Quaternion::dot(const Quaternion& q) {
    return qw * q.qw + qv.dot(q.qv);
}

// addition and subtraction

inline Quaternion Quaternion::operator+(const Quaternion& q) const {
    return Quaternion(qw + q.qw, qv + q.qv);
}

inline Quaternion& Quaternion::operator+=(const Quaternion& q) {
    qw += q.qw;
    qv += q.qv;
    return *this;
}

inline Quaternion Quaternion::operator-(const Quaternion& q) const {
    return Quaternion(qw - q.qw, qv - q.qv);
}

inline Quaternion& Quaternion::operator-=(const Quaternion& q) {
    qw -= q.qw;
    qv -= q.qv;
    return *this;
}

// norm and squared norm

inline double Quaternion::squaredNorm() const {
    return qw * qw + qv.squaredNorm();
}

inline double Quaternion::norm() const {
    return std::sqrt(squaredNorm());
}

// normalization

inline void Quaternion::normalize() {
    double n = norm();
    qw /= n;
    qv /= n;
}

// multiplication by a scalar

inline Quaternion Quaternion::operator*(double s) const {
    return Quaternion(qw * s, qv * s);
}

inline Quaternion& Quaternion::operator*=(double s) {
    qw *= s;
    qv *= s;
    return *this;
}

// friend function for multiplication by a scalar

inline Quaternion operator*(double s, const Quaternion& q) {
    return q * s;
}

// multiplication by a quaternion

inline Quaternion Quaternion::operator*(const Quaternion& q) const {
    return Quaternion(qw * q.qw - qv.dot(q.qv), cross(qv, q.qv) + qw * q.qv + q.qw * qv);
}

inline Quaternion& Quaternion::operator*=(const Quaternion& q) {
    *this = *this * q;
    return *this;
}

inline bool Quaternion::operator==(const Quaternion& q) const {
    return qw == q.qw && qv == q.qv;
}

inline bool Quaternion::operator!=(const Quaternion& q) const {
    return !(*this == q);
}

// rotation of a vector by a quaternion
// https://blog.molecular-matters.com/2013/05/24/a-faster-quaternion-vector-multiplication/

inline VectorND::Vector<double, 3> Quaternion::rotate(const VectorND::Vector<double, 3>& v) const {
    Vector3D temp = 2*cross(qv, v);
    return v + qw*temp + cross(qv, temp);
}
