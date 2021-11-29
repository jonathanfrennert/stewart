
#include <cmath>
#include "eigen3/Eigen/Core"
#include "std_msgs/Float32MultiArray.h"


float height;
Eigen::Matrix<float, 6, 4> b, p;
std_msgs::Float32MultiArray currentJointPos;

Eigen::Matrix<float, 6, 1> q_dot(const std_msgs::Float32MultiArray::ConstPtr& currentPos);
