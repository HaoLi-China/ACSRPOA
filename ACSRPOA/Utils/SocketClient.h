//hao added it
#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include <stdio.h>
#include <Winsock2.h>
#include <sstream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

using namespace std;

const int ARM_JOINT_NUM = 7;

//open the socket
bool open_socket(SOCKET &sockClient);

//init robot pose
bool init_robot_pose(SOCKET &sockClient, float pos_left_arm[], float pos_right_arm[],  Eigen::Vector3f&head_focus, float torso_up);

//init left arm pose
bool init_left_arm_pose(SOCKET &sockClient, float pos_left_arm[]);

//init right arm pose
bool init_right_arm_pose(SOCKET &sockClient, float pos_right_arm[]);

//make left scanner up and down
bool up_down_left_scanner(SOCKET &sockClient, float down_value, float up_value);

//make right scanner up and down
bool up_down_right_scanner(SOCKET &sockClient, float down_value, float up_value);

//r_arm pick up kinect
bool r_pick_up_kinect(SOCKET &sockClient);

//r_arm put down kinect
bool r_put_down_kinect(SOCKET &sockClient);

//l_arm pick up kinect
bool l_pick_up_kinect(SOCKET &sockClient);

//l_arm put down kinect
bool l_put_down_kinect(SOCKET &sockClient);

//left arm push object
bool l_push_object(SOCKET &sockClient, Eigen::Vector3f& position, Eigen::Vector3f& direction);

//right arm push object
bool r_push_object(SOCKET &sockClient, Eigen::Vector3f& position, Eigen::Vector3f& direction);

//left arm take back
bool l_take_back(SOCKET &sockClient, Eigen::Vector3f& position, Eigen::Vector3f& direction);

//right arm take back
bool r_take_back(SOCKET &sockClient, Eigen::Vector3f& position, Eigen::Vector3f& direction);

//set head pose
bool set_head_pose(SOCKET &sockClient, Eigen::Vector3f&head_focus);

//close the socket
bool close_socket(SOCKET &sockClient);

//get left gripper touch point and direction
void get_l_touch_point_and_dir(SOCKET &sockClient, const Eigen::Vector3f& input_position, const Eigen::Vector3f& input_dir, Eigen::Vector3f& output_position, Eigen::Vector3f& output_dir);

//get right gripper touch point and direction
void get_r_touch_point_and_dir(SOCKET &sockClient, const Eigen::Vector3f& input_position, const Eigen::Vector3f& input_dir, Eigen::Vector3f& output_position, Eigen::Vector3f& output_dir);

//just for test 
bool test_calibration_result(SOCKET &sockClient, const Eigen::Vector3f& input_position, const Eigen::Vector3f& input_dir);

//base drive forward
bool base_drive_forward(SOCKET &sockClient, float distance);

//base drive back
bool base_drive_back(SOCKET &sockClient, float distance);

//base drive left
bool base_drive_left(SOCKET &sockClient, float distance);

//base drive right
bool base_drive_right(SOCKET &sockClient, float distance);

//base turn left
bool base_turn_left(SOCKET &sockClient, float radians);

//base turn right
bool base_turn_right(SOCKET &sockClient, float radians);


////push the object
//void excute_set_arm_pose();
//
////push the object
//void excute_move_gripper_to();
//
////push the object
//void excute_push();
//
////push the object
//void excute_move_gripper_back();
//
////scan the object
//void excute_scan_object();

#endif //SOCKET_CLIENT_H
