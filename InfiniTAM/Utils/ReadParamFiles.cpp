//hao added it
#include "ReadParamFiles.h"

//float pos_left_arm[7];
//float pos_right_arm[7];
//Eigen::Vector3f head_focus;
//float torso_up;
//float down_value;
//float up_value;

void getRobotPose(RobotPose &robotpose){
  std::ifstream input("Files/parameter/robotpose.txt");
  if(input.fail()) {
    std::cout<<"could not open file!" << std::endl;
    return;
  }

  input >> robotpose.pos_left_arm[0] >> robotpose.pos_left_arm[1] >> robotpose.pos_left_arm[2]>> robotpose.pos_left_arm[3] >> robotpose.pos_left_arm[4] >> robotpose.pos_left_arm[5] >> robotpose.pos_left_arm[6];
  input >> robotpose.pos_right_arm[0] >> robotpose.pos_right_arm[1] >> robotpose.pos_right_arm[2]>> robotpose.pos_right_arm[3] >> robotpose.pos_right_arm[4] >> robotpose.pos_right_arm[5] >> robotpose.pos_right_arm[6];
  input >> robotpose.head_focus[0] >> robotpose.head_focus[1] >> robotpose.head_focus[2];
  input >> robotpose.torso_up >> robotpose.down_value >> robotpose.up_value;

  input.close();
}

void getGlobalSegRange(Eigen::Vector3f &global_seg_range){
  std::ifstream input("Files/parameter/global_seg_range.txt");
  if(input.fail()) {
    std::cout<<"could not open file!" << std::endl;
    return;
  }

  input >> global_seg_range[0] >> global_seg_range[1] >> global_seg_range[2];

  input.close();
}

void getRefinedSegRange(Eigen::Vector3f  &refined_seg_range){
  std::ifstream input("Files/parameter/refined_seg_range.txt");
  if(input.fail()) {
    std::cout<<"could not open file!" << std::endl;
    return;
  }

  input >> refined_seg_range[0] >> refined_seg_range[1] >> refined_seg_range[2];

  input.close();
}

void getInteractedSegRange(Eigen::Vector3f  &interacted_seg_range){
  std::ifstream input("Files/parameter/interacted_seg_range.txt");
  if(input.fail()) {
    std::cout<<"could not open file!" << std::endl;
    return;
  }

  input >> interacted_seg_range[0] >> interacted_seg_range[1] >> interacted_seg_range[2];

  input.close();
}