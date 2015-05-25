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
void getGlobalSegRange(Eigen::Vector3f  &global_seg_range);
void getRefinedSegRange(Eigen::Vector3f  &refined_seg_range);
void getInteractedSegRange(Eigen::Vector3f  &interacted_seg_range);
void getObjectIdFromFile(unsigned short &objectId);

#endif //READPARAMFILES_H
