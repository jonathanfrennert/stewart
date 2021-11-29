#include <cmath>
#include "eigen3/Eigen/Core"
#include "std_msgs/Float32MultiArray.h"
#include "qdot_calc.h"

Eigen::Matrix<float, 4, 4> transformation_matrix_q_dot(float x, float y, float z, float r, float p, float yaw)
{
    Eigen::Matrix<float, 4, 4> T;
    T << cos(yaw)*cos(p), -sin(yaw)*cos(r) + cos(yaw)*sin(p)*sin(r),  sin(yaw)*sin(r)+cos(yaw)*sin(p)*cos(r), x,
         sin(yaw)*cos(p),  cos(yaw)*cos(r) + sin(yaw)*sin(p)*sin(r), -cos(yaw)*sin(r)+sin(yaw)*sin(p)*cos(r), y,
                 -sin(p),                             cos(p)*sin(r),                         cos(p)*cos(yaw), z,
                       0,                                         0,                                       0, 1;
    return T;
}


Eigen::Matrix<float, 6, 1> q_dot(const std_msgs::Float32MultiArray::ConstPtr& currentPos)
{
        Eigen::Matrix<float, 1, 6> currentJointPos = {};
        currentJointPos << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	Eigen::Matrix<float, 1, 6> currentJointVel = {};
	currentJointVel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        float baseLength = 1.929;
        Eigen::Matrix<float, 6, 4> b, p;


//        b << -0.101,    0.8, 0.25, 1,
//              0.101,    0.8, 0.25, 1,
//              0.743, -0.313, 0.25, 1,
//              0.642, -0.487, 0.25, 1,
//             -0.643, -0.486, 0.25, 1,
//             -0.744, -0.311, 0.25, 1;
//
//        p << -0.642,  0.487, -0.05, 1,
//              0.642,  0.487, -0.05, 1,
//              0.743,  0.313, -0.05, 1,
//              0.101,   -0.8, -0.05, 1,
//             -0.101,   -0.8, -0.05, 1,
//             -0.743,  0.313, -0.05, 1;

	// Retrieving information from message
        float x = currentPos->data[0];
        float y = currentPos->data[1];
        float z = currentPos->data[2];
	Eigen::Matrix<float, 4, 1> c;
	c << x, y, z, 1.0;

        float roll = currentPos->data[3];
        float pitch = currentPos->data[4];
        float yaw = currentPos->data[5];

	// Retrieving velocity
	float x_dot = currentPos->data[6];
	float y_dot = currentPos->data[7];
	float z_dot = currentPos->data[8];
	float theta_x_dot = currentPos->data[9];
	float theta_y_dot = currentPos->data[10];
	float theta_z_dot = currentPos->data[11];

	// Velocities
	Eigen::Matrix<float, 3, 1> v;
	Eigen::Matrix<float, 3, 1> w;
	v << x_dot, y_dot, z_dot;
	w << theta_x_dot, theta_y_dot, theta_z_dot;

        Eigen::Matrix<float, 4, 4> T = transformation_matrix_q_dot(x, y, z, roll, pitch, yaw);
        for (size_t i = 0; i < 6; i++)
        {
            Eigen::Matrix<float, 4, 1> length = T*p.row(i).transpose() - b.row(i).transpose();
	    Eigen::Matrix<float, 4, 1> R_p = T*p.row(i).transpose() - c;
	    // Taking first three components only
	    Eigen::Matrix<float, 3, 1> rp;
	    rp << R_p[0], R_p[1], R_p[2];

	    // Part of equation in parenthesis
	    Eigen::Matrix<float, 3, 1> l_dot;
	    l_dot = v + w.cross(rp);

            currentJointPos[i] = sqrt(pow(length(0), 2) + pow(length(1), 2) + pow(length(2), 2)) - baseLength;
	    // First three components of the length vector
	    Eigen::Matrix<float, 3, 1> length_three;
	    currentJointVel[i] = (1/currentJointPos[i]) * length_three.transpose()*l_dot;
        }
        return currentJointVel;
}
