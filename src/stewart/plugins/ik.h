#include <cmath>
#include "eigen3/Eigen/Core"
#include "std_msgs/Float32MultiArray.h"

Eigen::Matrix<float, 6, 1> ik(const std_msgs::Float32MultiArray::ConstPtr& goalPos);
