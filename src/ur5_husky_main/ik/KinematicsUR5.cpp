#include "KinematicsUR5.hpp"

#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <complex>

using namespace Eigen;

#define pi 3.14159265358979323846
#define errorMarker 1000000

/**
 * IK solver http://rasmusan.dk/wp-content/uploads/ur5_kinematics.pdf
 * */

/**
 * Denavit Hartenberg Parameters - DH Parameters
 * See more https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics
 * **/
std::vector<double> a = {0, -0.425, -0.39225, 0, 0, 0};
std::vector<double> alpha = {pi/2, 0, 0, pi/2, -pi/2, 0};
// std::vector<double> d = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996}; // для UR5e
std::vector<double> d = {0.089159, 0, 0, 0.10915, 0.09465, 0.0823}; // Для UR5


KinematicsUR5::KinematicsUR5(bool debug) {
    debug_ = debug;
}

KinematicsUR5::KinematicsUR5(double x, double y, double z) {
    posX_ = x;
    posY_ = y;
    posZ_ = z;
}

KinematicsUR5::KinematicsUR5(double x, double y, double z, bool debug) {
    posX_ = x;
    posY_ = y;
    posZ_ = z;
    debug_ = debug;
}

KinematicsUR5::KinematicsUR5(double x, double y, double z, double roll, double pitch, double yaw) {
    posX_ = x;
    posY_ = y;
    posZ_ = z;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
}

KinematicsUR5::KinematicsUR5(double x, double y, double z, double roll, double pitch, double yaw, bool debug) {
    posX_ = x;
    posY_ = y;
    posZ_ = z;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
    debug_ = debug;
}


Matrix3d euler2Quaternion(double roll, double pitch, double yaw) {
    AngleAxisd rollAngle(roll, Vector3d::UnitX());
    AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
    AngleAxisd yawAngle(yaw, Vector3d::UnitZ());

    Quaterniond q = yawAngle * pitchAngle * rollAngle;
    Matrix3d rotationMatrix = q.matrix();
    return rotationMatrix;
}


Vector3d quaternion2Euler(Matrix3d r) {

    Vector3d orientation(3);

    double beta = atan2(-r(2,0), sqrt(pow(r(0,0), 2) + pow(r(1,0), 2)));
    double alpha = atan2(r(1,0)/cos(beta), r(0,0)/cos(beta));
    double gamma = atan2(r(2,1)/cos(beta), r(2,2)/cos(beta));

    orientation(0) = alpha;
    orientation(1) = beta;
    orientation(2) = gamma;

    return orientation;
}


Matrix4d createTransformMatrix(double alpha, double a, double d, double theta) {
    Matrix4d matrix;
    matrix << cos(theta),               -sin(theta),                0,              a,
              sin(theta)*cos(alpha),    cos(theta)*cos(alpha),      -sin(alpha),    -sin(alpha)*d,
              sin(theta)*sin(alpha),    cos(theta)*sin(alpha),      cos(alpha),     cos(alpha)*d,
              0,                        0,                          0,              1;

    return matrix;
}


Matrix4d createTransformMatrix2FK(double alpha, double a, double d, double theta) {
    Matrix4d matrix;
    matrix << cos(theta),  -sin(theta)*cos(alpha),   sin(theta)*sin(alpha),   a*cos(theta),
              sin(theta),   cos(theta)*cos(alpha),  -cos(theta)*sin(alpha),   a*sin(theta),
              0,            sin(alpha),              cos(alpha),              d,
              0,            0,                       0,                       1;

    return matrix;
}


double createTheta6(Vector2d X06, Vector2d Y06, double theta1, double theta5) {
    return atan2((-X06(1) * sin(theta1) + Y06(1) * cos(theta1))/sin(theta5), (X06(0)*sin(theta1) - Y06(0)*cos(theta1))/sin(theta5));
}


double createTheta2(Vector2d P14_, double a3, double theta3, double P14_xx) {
    return atan2(-P14_(1), -P14_(0)) - asin(-a3*sin(theta3)/abs(P14_xx));
}


double createTheta4(Matrix4d T14, std::vector<double> &alpha, std::vector<double> &a, std::vector<double> &d, double theta2, double theta3) {
    Matrix4d T12 = createTransformMatrix(alpha[0], a[0], d[1], theta2);
    Matrix4d T23 = createTransformMatrix(alpha[1], a[1], d[2], theta3);
    Matrix4d T34 = T23.inverse() * T12.inverse() * T14;

    Vector2d X34(T34(0,0), T34(1, 0)); // Берем 1 столбец

    return atan2(X34(1), X34(0));
}


void printTheta(int index, double *theta, int count, bool debug) {
    if (debug) {
        std::cout << "theta" << index << ": ";
        for (int k = 0; k < count; k++) {
            std::cout << theta[k] << " ";
        }
        std::cout << std::endl;
    }
}


void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove) {
    unsigned int numRows = matrix.rows()-1;

    if(rowToRemove < numRows) {
        matrix.block(rowToRemove, 0, numRows - rowToRemove, matrix.cols()) = matrix.block(rowToRemove+1, 0, numRows - rowToRemove, matrix.cols());
    }

    matrix.conservativeResize(numRows, matrix.cols());
}


MatrixXd KinematicsUR5::calculateIKAllSolutions() {

    if (debug_) {
        std::cout << "Input data for calculate: X = " << posX_ << ", Y = " << posY_ << ", Z = " << posZ_;
        std::cout << ", roll = " << roll_ << ", pitch = " << pitch_ << ", yaw = " << yaw_ << std::endl;
    }

    double theta1[2] = {0, 0},
           theta2[8] = {0, 0, 0, 0, 0, 0, 0, 0},
           theta3[8] = {0, 0, 0, 0, 0, 0, 0, 0},
           theta4[8] = {0, 0, 0, 0, 0, 0, 0, 0},
           theta5[4] = {0, 0, 0, 0},
           theta6[4] = {0, 0, 0, 0};

    // Матрица вращения по углам Эйлера
    Matrix3d rotationMatrix = euler2Quaternion(roll_, pitch_, yaw_);
    if (debug_) {
        std::cout << "Rotation Matrix: \n" << rotationMatrix << std::endl;
    }

    Matrix4d T06;
    T06.setIdentity();
    T06.block<3,3>(0,0) = rotationMatrix;
    T06(0,3) = posX_;
    T06(1,3) = posY_;
    T06(2,3) = posZ_;

    if (debug_) {
        std::cout << "Matrix T06: " << std::endl << T06 << std::endl;
    }
    

    ////////////////////////////////// theta1 ///////////////////////////////

    Vector4d vec5(0, 0, -d[5], 1);
    Vector4d P05 = T06 * vec5;

    double theta1_ = atan2(P05(1), P05(0)) + pi/2;

    std::complex<double> denominator = sqrt(pow(P05(0), 2) + pow(P05(1), 2));
    double acosTmp_ = (P05(0) == 0 && P05(1) == 0 || denominator.real() == 0) ? 0 : acos(d[3] / denominator.real());

    theta1[0] = theta1_ + acosTmp_; // положительное θ1
    theta1[1] = theta1_ - acosTmp_; // отрицательное θ1

    printTheta(1, theta1, (sizeof(theta1)/sizeof(*theta1)), debug_);


    ////////////////////////////////// theta5 ///////////////////////////////

    Vector3d P06(posX_, posY_, posZ_);

    for (int i = 0, j = 0; i < (sizeof(theta1)/sizeof(*theta1)); i++) {
        double acosValTmp_ = (P06(0)*sin(theta1[i]) - P06(1)*cos(theta1[i]) - d[3])/d[5];
        if (acosValTmp_ > 1) {
            std::cout << "Error theta5: | 1 P 6y − d 4 | ≤ |d 6 |" << std::endl;
            theta5[j] = errorMarker;
            theta5[j+1] = errorMarker;
        } else {
            theta5[j] = acos(acosValTmp_);     // для положительного θ1
            theta5[j+1] = -acos(acosValTmp_);  // для отрицательного θ1
        }
        
        j += 2;
    }

    printTheta(5, theta5, (sizeof(theta5)/sizeof(*theta5)), debug_);


    ////////////////////////////////// theta6 ///////////////////////////////

    Vector2d X06(T06(0, 0), T06(0, 1));
    Vector2d Y06(T06(1, 0), T06(1, 1));

    for (int i = 0, j = 0; i < (sizeof(theta1)/sizeof(*theta1)); i++) {
        theta6[j] = createTheta6(X06, Y06, theta1[i], theta5[j]);
        theta6[j+1] = createTheta6(X06, Y06, theta1[i], theta5[j+1]);
        j += 2;
    }

    printTheta(6, theta6, (sizeof(theta6)/sizeof(*theta6)), debug_);


    ////////////////////////////////// theta3 ///////////////////////////////

    for (int i = 0, j = 0; i < (sizeof(theta5)/sizeof(*theta5)); i++) {
        Matrix4d T01 = createTransformMatrix(0, 0, d[0], theta1[j]);
        Matrix4d T45 = createTransformMatrix(alpha[3], a[3], d[4], theta5[i]);
        Matrix4d T56 = createTransformMatrix(alpha[4], a[4], d[5], theta6[i]);

        Matrix4d T14 = T01.inverse() * T06 * T56.inverse() * T45.inverse();

        Vector3d P14(T14(0,3), T14(1,3), T14(2,3));

        Vector2d P14_(P14(0), P14(2));
        double P14_xx = P14_.norm();

        // по условию задачи:  | 1 P 4xz | ∈ [|a 2 − a 3 |; |a 2 + a 3 |].
        if (P14_xx > abs(a[1] - a[2]) || P14_xx < abs(a[1] + a[2])) {
            acosTmp_ = acos((pow(P14_xx, 2) - pow(a[1], 2) - pow(a[2], 2)) / (2 * a[1] * a[2]));
            theta3[i*2] = acosTmp_;
            theta3[i*2+1] = -acosTmp_;

        } else {
            std::cout << "Error finding theta3: | 1 P 4xz | ∈ [|a 2 − a 3 |; |a 2 + a 3 |]" << std::endl;
            theta3[i*2] = errorMarker;
            theta3[i*2+1] = errorMarker;
        }

        ////////////////////////////////// theta2 ///////////////////////////////

        theta2[i*2] = createTheta2(P14_, a[2], theta3[i*2], P14_xx);
        theta2[i*2+1] = createTheta2(P14_, a[2], theta3[i*2+1], P14_xx);

        ////////////////////////////////// theta4 ///////////////////////////////

        theta4[i*2] = createTheta4(T14, alpha, a, d, theta2[i*2], theta3[i*2]);
        theta4[i*2+1] = createTheta4(T14, alpha, a, d, theta2[i*2+1], theta3[i*2+1]);

        if (i % 2 == 1) {
            j++; // смена знака для θ1
        }
    }

    printTheta(3, theta3, (sizeof(theta3)/sizeof(*theta3)), debug_);
    printTheta(2, theta2, (sizeof(theta2)/sizeof(*theta2)), debug_);
    printTheta(4, theta4, (sizeof(theta4)/sizeof(*theta4)), debug_);

    MatrixXd solutions(8, 6);
    solutions << theta1[0], theta2[0], theta3[0], theta4[0], theta5[0], theta6[0],
                 theta1[0], theta2[1], theta3[1], theta4[1], theta5[0], theta6[0],
                 theta1[0], theta2[2], theta3[2], theta4[2], theta5[1], theta6[1],
                 theta1[0], theta2[3], theta3[3], theta4[3], theta5[1], theta6[1],
                 theta1[1], theta2[4], theta3[4], theta4[4], theta5[2], theta6[2],
                 theta1[1], theta2[5], theta3[5], theta4[5], theta5[2], theta6[2],
                 theta1[1], theta2[6], theta3[6], theta4[6], theta5[3], theta6[3],
                 theta1[1], theta2[7], theta3[7], theta4[7], theta5[3], theta6[3];

    if (debug_) {
        std::cout << "All solutions (" << solutions.rows() << "):" << std::endl;
        std::cout << solutions << std::endl;
    }

    for (int i = 0; i < solutions.rows(); i++) {
        // Удалить все строки, которые помечены ошибочными, ошибки могут быть в theta3 и theta5
        if (solutions(i, 2) == errorMarker || solutions(i, 4) == errorMarker) {
            removeRow(solutions, i);
            i--;
        }
    }

    if (debug_) {
        std::cout << "All solutions without errors (" << solutions.rows() << "):" << std::endl;
        std::cout << solutions << std::endl;
    }
                 
    return solutions;
}


VectorXd KinematicsUR5::getBestSolutionIK(MatrixXd solutions, VectorXd currentPose) {
    std::vector<int> weights = {6, 5, 4, 3, 2, 1};
    long long minDistance = std::numeric_limits<long long>::max();
    VectorXd solution(6);

    
    for (int i = 0; i < solutions.rows(); i++) {
        double distance = 0;
        for (int j = 0; j < solutions.cols(); j++) {
            distance += pow(solutions(i, j) - currentPose(j) * weights[j], 2);
        }

        if (distance < minDistance) {
            minDistance = distance;
            solution = solutions.row(i);
            std::cout << "row = " << i << std::endl;
        }
    }

    return solution;
}


VectorXd KinematicsUR5::getBestSolutionIK(VectorXd currentPose) {
    MatrixXd solutions = calculateIKAllSolutions();
    return getBestSolutionIK(solutions, currentPose);
}


VectorXd KinematicsUR5::getForwardkinematics(VectorXd theta, bool debug = true) {
    VectorXd fk(6); // 1=X 2=Y 3=Z 4=Roll 5=Pitch 6=Yaw

    Matrix4d T01 = createTransformMatrix2FK(alpha[0], a[0], d[0], theta(0));
    Matrix4d T12 = createTransformMatrix2FK(alpha[1], a[1], d[1], theta(1));
    Matrix4d T23 = createTransformMatrix2FK(alpha[2], a[2], d[2], theta(2));
    Matrix4d T34 = createTransformMatrix2FK(alpha[3], a[3], d[3], theta(3));
    Matrix4d T45 = createTransformMatrix2FK(alpha[4], a[4], d[4], theta(4));
    Matrix4d T56 = createTransformMatrix2FK(alpha[5], a[5], d[5], theta(5));


    Matrix4d T06 = T01 * T12 * T23 * T34 * T45 * T56;

    Matrix3d rotationMatrix = T06.block(0, 0, 3, 3);
    Vector3d orientation = quaternion2Euler(rotationMatrix);

    if (debug) {
        std::cout << "theta = " << theta << std::endl;
        std::cout << "----- rotationMatrix -----" << std::endl;
        std::cout << rotationMatrix << std::endl;
        std::cout << "----- orientation -----" << std::endl;
        std::cout << orientation << std::endl;
        std::cout << "====================" << std::endl;
        std::cout << "----- T01 -----" << std::endl;
        std::cout << T01 << std::endl;
        std::cout << "----- T12 -----" << std::endl;
        std::cout << T12 << std::endl;
        std::cout << "----- T23 -----" << std::endl;
        std::cout << T23 << std::endl;
        std::cout << "----- T34 -----" << std::endl;
        std::cout << T34 << std::endl;
        std::cout << "----- T45 -----" << std::endl;
        std::cout << T45 << std::endl;
        std::cout << "----- T56 -----" << std::endl;
        std::cout << T56 << std::endl;
        std::cout << "----- T06 -----" << std::endl;
        std::cout << T06 << std::endl;
    }

    // положение
    fk(0) = T06(0,3);
    fk(1) = T06(1,3);
    fk(2) = T06(2,3);

    // ориентация
    fk(3) = orientation(2); // X (R)
    fk(4) = orientation(1); // Y (P)
    fk(5) = orientation(0); // Z (Y)

    if (debug) {
        std::cout << "Forward Kinematics = " << fk(0) << " " << fk(1) << " " << fk(2) << " " << fk(3) << " " << fk(4) << " " << fk(5) << std::endl;
    }

    return fk;
}


MatrixXd KinematicsUR5::getCheckIK(VectorXd testedPose) {

    VectorXd fk(6);
    fk = getForwardkinematics(testedPose, false);

    MatrixXd result(2, 3);
    VectorXd positionError(3);
    VectorXd orientationError(3);

    positionError(0) = fk(0) - posX_;
    positionError(1) = fk(1) - posY_;
    positionError(2) = fk(2) - posZ_;

    result.row(0) = positionError;

    orientationError(0) = fk(3) - roll_;
    orientationError(1) = fk(4) - pitch_;
    orientationError(2) = fk(5) - yaw_;

    result.row(1) = orientationError;

    return result;
}