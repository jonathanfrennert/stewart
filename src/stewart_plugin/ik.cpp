#include <cmath>
#include "eigen3/Eigen/Core"
#include "std_msgs/Float32MultiArray.h"


Eigen::Matrix<float, 4, 4> transformation_matrix(float x, float y, float z, float r, float p, float yaw)
{
    Eigen::Matrix<float, 4, 4> T;
    T << cos(yaw)*cos(p), -sin(yaw)*cos(r) + cos(yaw)*sin(p)*sin(r),  sin(yaw)*sin(r)+cos(yaw)*sin(p)*cos(r), x,
         sin(yaw)*cos(p),  cos(yaw)*cos(r) + sin(yaw)*sin(p)*sin(r), -cos(yaw)*sin(r)+sin(yaw)*sin(p)*cos(r), y,
                 -sin(p),                             cos(p)*sin(r),                         cos(p)*cos(yaw), z,
                       0,                                         0,                                       0, 1;
    return T;
}


Eigen::Matrix<float, 6, 1> ik(const std_msgs::Float32MultiArray::ConstPtr& goalPos)
{
        Eigen::Matrix<float, 1, 6> goalJointPos = {};
        goalJointPos << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        float height = 2.0;
        Eigen::Matrix<float, 6, 4> b, p;


        b << -0.101,    0.8, 0.25, 1,
              0.101,    0.8, 0.25, 1,
              0.743, -0.313, 0.25, 1,
              0.642, -0.487, 0.25, 1,
             -0.643, -0.486, 0.25, 1,
             -0.744, -0.311, 0.25, 1;

        p << -0.642,  0.487, -0.05, 1,
              0.642,  0.487, -0.05, 1,
              0.743,  0.313, -0.05, 1,
              0.101,   -0.8, -0.05, 1,
             -0.101,   -0.8, -0.05, 1,
             -0.743,  0.313, -0.05, 1;


        float x = goalPos->data[0];
        float y = goalPos->data[1];
        float z = goalPos->data[2];
        float roll = goalPos->data[3];
        float pitch = goalPos->data[4];
        float yaw = goalPos->data[5];
        Eigen::Matrix<float, 4, 4> T = transformation_matrix(x, y, z, roll, pitch, yaw);
        for (size_t i = 0; i < 6; i++)
        {
            Eigen::Matrix<float, 4, 1> length = T*p.row(i).transpose() - b.row(i).transpose();
            goalJointPos[i] = sqrt(pow(length(0), 2) + pow(length(1), 2) + pow(length(2), 2)) - height;
        }

        return goalJointPos;
}
