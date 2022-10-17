#include "Vector.hpp"
#include "Quaternion.hpp"
#include <string>
#include <iostream>
#include <gtest/gtest.h>
#include <cmath>

TEST(quaternionTests, constructorTest) {    
    Quaternion q{};
    EXPECT_EQ(1.0, q.getRealPart());
    EXPECT_EQ((VectorND::Vector<double, 3>{{0., 0., 0.}}), q.getVectorPart());

    Quaternion q2{11.0, VectorND::Vector<double, 3>{{1., 2., 3.}}};
    EXPECT_EQ(11.0, q2.getRealPart());
    EXPECT_EQ((VectorND::Vector<double, 3>{{1., 2., 3.}}), q2.getVectorPart());

    Quaternion q3{11.0, 1., 2., 3.};
    EXPECT_EQ(11.0, q3.getRealPart());
    EXPECT_EQ((VectorND::Vector<double, 3>{{1., 2., 3.}}), q3.getVectorPart());

    // from axis and angle
    const double angle = M_PI / 2.0; // pi / 2
    VectorND::Vector<double, 3> axis{{1., 2., 3.}};
    axis /= axis.norm(); // normalize the axis
    Quaternion q4{axis, angle};
    Quaternion expectedQ4 = Quaternion{0.7071067811882787, 0.18898223650415094, 0.3779644730083019, 0.5669467095124527};
    EXPECT_NEAR(q4[0], expectedQ4[0], 1e-6);
    EXPECT_NEAR(q4[1], expectedQ4[1], 1e-6);
    EXPECT_NEAR(q4[2], expectedQ4[2], 1e-6);
    EXPECT_NEAR(q4[3], expectedQ4[3], 1e-6);

    // test getters
    EXPECT_EQ(q3.getRealPart(), 11.0);
    EXPECT_EQ(q3.getVectorPart(), (VectorND::Vector<double, 3>{{1., 2., 3.}}));
}

TEST(quaternionTests, crossTest) {
    VectorND::Vector<double, 3> v1{{1., 2., 3.}};
    VectorND::Vector<double, 3> v2{{4., 5., 6.}};
    VectorND::Vector<double, 3> expectedCross{{-3., 6., -3.}};
    EXPECT_EQ(expectedCross, cross(v1, v2));
}

TEST(quaternionTests, accessTest) {
    Quaternion q{11.0, VectorND::Vector<double, 3>{{1., 2., 3.}}};
    EXPECT_EQ(11.0, q[0]);
    EXPECT_EQ(1., q[1]);
    EXPECT_EQ(2., q[2]);
    EXPECT_EQ(3., q[3]);
    EXPECT_EQ(11.0, q.at(0));
    EXPECT_EQ(1., q.at(1));
    EXPECT_EQ(2., q.at(2));
    EXPECT_EQ(3., q.at(3));
    // test const version
    const auto& real = q[0];
    const auto& v1 = q[1];
    EXPECT_EQ(1., v1);
    EXPECT_EQ(11.0, real);
    // modify
    q[0] = 12.0;
    EXPECT_EQ(12.0, q[0]);
}

TEST(quaternionTests, normlalizeTest) {
    Quaternion q{11.0, VectorND::Vector<double, 3>{{1., 2., 3.}}};
    auto q2 = q;
    q.normalize();
    EXPECT_DOUBLE_EQ(1., q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    EXPECT_DOUBLE_EQ(q[0], q2[0]/q2.norm());
    EXPECT_DOUBLE_EQ(q[1], q2[1]/q2.norm());
    EXPECT_DOUBLE_EQ(q[2], q2[2]/q2.norm());
    EXPECT_DOUBLE_EQ(q[3], q2[3]/q2.norm());
}

TEST(quaternionTests, conjugateTest) {
    Quaternion q{11.0, VectorND::Vector<double, 3>{{1., 2., 3.}}};
    Quaternion q2 = q.conjugate();
    EXPECT_DOUBLE_EQ(q[0], q2[0]);
    EXPECT_DOUBLE_EQ(-q[1], q2[1]);
    EXPECT_DOUBLE_EQ(-q[2], q2[2]);
    EXPECT_DOUBLE_EQ(-q[3], q2[3]);
}

TEST(quaternionTests, dotTest) {
    Quaternion q{11.0, VectorND::Vector<double, 3>{{1., 2., 3.}}};
    Quaternion q2{11.0, VectorND::Vector<double, 3>{{4., 5., 6.}}};
    EXPECT_DOUBLE_EQ(q.dot(q2), 11.0*11.0 + 1.*4. + 2.*5. + 3.*6.);
}

TEST(quaternionTests, addSubTest) {
    Quaternion q{11.0, VectorND::Vector<double, 3>{{1., 2., 3.}}};
    Quaternion q2{10.5, VectorND::Vector<double, 3>{{4., 5., 6.}}};
    Quaternion q3 = q + q2;
    EXPECT_DOUBLE_EQ(q3[0], 21.5);
    EXPECT_DOUBLE_EQ(q3[1], 5.);
    EXPECT_DOUBLE_EQ(q3[2], 7.);
    EXPECT_DOUBLE_EQ(q3[3], 9.);
    Quaternion q4 = q - q2;
    EXPECT_DOUBLE_EQ(q4[0], 0.5);
    EXPECT_DOUBLE_EQ(q4[1], -3.);
    EXPECT_DOUBLE_EQ(q4[2], -3.);
    EXPECT_DOUBLE_EQ(q4[3], -3.);
    // test +=
    q3 += q;
    EXPECT_DOUBLE_EQ(q3[0], 32.5);
    EXPECT_DOUBLE_EQ(q3[1], 6.);
    EXPECT_DOUBLE_EQ(q3[2], 9.);
    EXPECT_DOUBLE_EQ(q3[3], 12.);
    // test -=
    q3 -= q;
    EXPECT_DOUBLE_EQ(q3[0], 21.5);
    EXPECT_DOUBLE_EQ(q3[1], 5.);
    EXPECT_DOUBLE_EQ(q3[2], 7.);
    EXPECT_DOUBLE_EQ(q3[3], 9.);
}

TEST(quaternionTests, multiplicationTest) {
    Quaternion q{11.0, VectorND::Vector<double, 3>{{1., -2.5, 3.98}}};
    Quaternion q2{-7.1, VectorND::Vector<double, 3>{{4.2, 5., -6.1}}};
    Quaternion q3 = q * q2;
    Quaternion expectedQ3{-45.522, VectorND::Vector<double, 3>{{34.45, 95.566, -79.858}}};
    EXPECT_NEAR(q3[0], expectedQ3[0], 1e-6);
    EXPECT_NEAR(q3[1], expectedQ3[1], 1e-6);
    EXPECT_NEAR(q3[2], expectedQ3[2], 1e-6);
    EXPECT_NEAR(q3[3], expectedQ3[3], 1e-6);

    Quaternion q4{2., VectorND::Vector<double, 3>{{3., 2., 3.}}};
    Quaternion q5{3., VectorND::Vector<double, 3>{{2., 3., 2.}}};
    Quaternion q6 = q4 * q5;
    Quaternion expectedQ6{-12., VectorND::Vector<double, 3>{{8., 12., 18.}}};
    EXPECT_NEAR(q6[0], expectedQ6[0], 1e-6);
    EXPECT_NEAR(q6[1], expectedQ6[1], 1e-6);
    EXPECT_NEAR(q6[2], expectedQ6[2], 1e-6);
    EXPECT_NEAR(q6[3], expectedQ6[3], 1e-6);
    // *= operator
    q4 *= q5;
    EXPECT_NEAR(q4[0], expectedQ6[0], 1e-6);
    EXPECT_NEAR(q4[1], expectedQ6[1], 1e-6);
    EXPECT_NEAR(q4[2], expectedQ6[2], 1e-6);
    EXPECT_NEAR(q4[3], expectedQ6[3], 1e-6);
    
    // scalar multiplication
    Quaternion q7{2., VectorND::Vector<double, 3>{{3., 2., 3.}}};
    Quaternion q8 = q7 * 2.;
    Quaternion q9 = 2. * q7;
    Quaternion expectedQ8{4., VectorND::Vector<double, 3>{{6., 4., 6.}}};
    EXPECT_DOUBLE_EQ(q8[0], expectedQ8[0]);
    EXPECT_DOUBLE_EQ(q8[1], expectedQ8[1]);
    EXPECT_DOUBLE_EQ(q8[2], expectedQ8[2]);
    EXPECT_DOUBLE_EQ(q8[3], expectedQ8[3]);
    EXPECT_DOUBLE_EQ(q9[0], expectedQ8[0]);
    EXPECT_DOUBLE_EQ(q9[1], expectedQ8[1]);
    EXPECT_DOUBLE_EQ(q9[2], expectedQ8[2]);
    EXPECT_DOUBLE_EQ(q9[3], expectedQ8[3]);
    // *= scalar
    q7 *= 2.;
    EXPECT_DOUBLE_EQ(q7[0], expectedQ8[0]);
    EXPECT_DOUBLE_EQ(q7[1], expectedQ8[1]);
    EXPECT_DOUBLE_EQ(q7[2], expectedQ8[2]);
    EXPECT_DOUBLE_EQ(q7[3], expectedQ8[3]);
}

TEST(quaternionTests, normTest) {
    Quaternion q{11.0, VectorND::Vector<double, 3>{{1., 2., 3.}}};
    EXPECT_DOUBLE_EQ(q.norm(), std::sqrt(11.0*11.0 + 1.*1. + 2.*2. + 3.*3.));
    // test squared norm
    EXPECT_DOUBLE_EQ(q.squaredNorm(), 11.0*11.0 + 1.*1. + 2.*2. + 3.*3.);
}

TEST(quaternionTests, equalityTest) {
    Quaternion q{11.0, VectorND::Vector<double, 3>{{1., 2., 3.}}};
    Quaternion q2{11.0, VectorND::Vector<double, 3>{{1., 2., 3.}}};
    Quaternion q3{10.5, VectorND::Vector<double, 3>{{4., 5., 6.}}};
    EXPECT_TRUE(q == q2);
    EXPECT_FALSE(q == q3);
    // test !=
    EXPECT_FALSE(q != q2);
    EXPECT_TRUE(q != q3);
}

TEST(quaternionTests, rotateTest) {
    Quaternion q{0.9938, 0.0336, 0.1041, -0.0139};
    VectorND::Vector<double, 3> v{{-13., 14., 147.}};
    VectorND::Vector<double, 3> v2 = q.rotate(v);
    VectorND::Vector<double, 3> expectedV2{{18.04984, 3.9886, 147.0785}};
    EXPECT_NEAR(v2[0], expectedV2[0], 1e-4);
    EXPECT_NEAR(v2[1], expectedV2[1], 1e-4);
    EXPECT_NEAR(v2[2], expectedV2[2], 1e-4);
}