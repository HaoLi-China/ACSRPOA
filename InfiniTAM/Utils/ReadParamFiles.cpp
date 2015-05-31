//hao added it
#include "ReadParamFiles.h"

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

void getSegRange(char* name, float &range_x0, float &range_x1, float &range_y0, float &range_y1, float &range_z){
  std::ifstream input(name);
  if(input.fail()) {
    std::cout<<"could not open file!" << std::endl;
    return;
  }

  input >> range_x0 >> range_x1 >> range_y0 >> range_y1 >> range_z;

  input.close();
}

void getObjectIdFromFile(unsigned short &objectId){
  std::ifstream input("Files/parameter/objectId.txt");
  if(input.fail()) {
    std::cout<<"could not open file!" << std::endl;
    return;
  }

  input >> objectId;

  input.close();
}

//just for test
void getTestCount(int &testCount){
  std::ifstream input("Files/parameter/testCount.txt");
  if(input.fail()) {
    std::cout<<"could not open file!" << std::endl;
    return;
  }

  input >> testCount;

  input.close();
}