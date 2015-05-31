//hao added it
#ifndef READPARAMFILES_H
#define READPARAMFILES_H

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <iostream> 
#include <fstream>

struct RobotPose{
  float pos_left_arm[7];
  float pos_right_arm[7];
  Eigen::Vector3f head_focus;
  float torso_up;
  float down_value;
  float up_value;
};

void getRobotPose(RobotPose &robotpose);
void getSegRange(char *name, float &range_x0, float &range_x1, float &range_y0, float &range_y1, float &range_z);
void getObjectIdFromFile(unsigned short &objectId);
void getTestCount(int &testCount);//just for test

#endif //READPARAMFILES_H
