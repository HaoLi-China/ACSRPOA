#include "SocketClient.h"

//float to string
void float2str(float n, char* ch)
{
  stringstream ss;
  string s;
  ss << n;
  ss >> s;
  strcpy(ch,const_cast<char*>(s.c_str()));
}

//string to float
float str2float(char* ch)
{
  return atof(ch);
}

//int to string
void int2str(int n, char* ch)
{
  stringstream ss;
  string s;
  ss << n;
  ss >> s;
  strcpy(ch,const_cast<char*>(s.c_str()));
}

//string to int
int str2int(char* ch){
  return atoi(ch);
}

//open the socket
bool open_socket(SOCKET &sockClient){
  WORD wVersionRequested;
  WSADATA wsaData;
  int err;

  wVersionRequested = MAKEWORD( 1, 1 );

  err = WSAStartup( wVersionRequested, &wsaData );
  if ( err != 0 ) {
    return false;
  }

  if ( LOBYTE( wsaData.wVersion ) != 1 ||
    HIBYTE( wsaData.wVersion ) != 1 ) {
      WSACleanup( );
      return false;
  }
  sockClient=socket(AF_INET,SOCK_STREAM,0);

  SOCKADDR_IN addrSrv;
  //addrSrv.sin_addr.S_un.S_addr=inet_addr("172.21.8.160");
  addrSrv.sin_addr.S_un.S_addr=inet_addr("10.68.0.147");
  addrSrv.sin_family=AF_INET;
  addrSrv.sin_port=htons(3333);
  connect(sockClient,(SOCKADDR*)&addrSrv,sizeof(SOCKADDR));

  return true;
}

//init robot pose
bool init_robot_pose(SOCKET &sockClient, float pos_left_arm[], float pos_right_arm[],  Eigen::Vector3f&head_focus, float torso_up){
  char tem[1024];
  memset(tem, 0, sizeof(tem));

  stringstream ss;
  string s;
  ss << "a:"; 
  for(int i=0; i<ARM_JOINT_NUM; i++){
    ss << pos_left_arm[i] << ",";
  }

  for(int i=0; i<ARM_JOINT_NUM; i++){
    ss << pos_right_arm[i] << ",";
  }

  ss << head_focus[0] << "," << head_focus[1] << "," << head_focus[2] << "," << torso_up << ";";

  ss >> s;
  strcpy(tem, const_cast<char*>(s.c_str()));

  send(sockClient,tem,strlen(tem)+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//init left arm pose
bool init_left_arm_pose(SOCKET &sockClient, float pos_left_arm[]){
  char tem[1024];
  memset(tem, 0, sizeof(tem));

  stringstream ss;
  string s;
  ss << "b:"; 
  for(int i=0; i<ARM_JOINT_NUM-1; i++){
    ss << pos_left_arm[i] << ",";
  }
  ss << pos_left_arm[ARM_JOINT_NUM-1] << ";";

  ss >> s;
  strcpy(tem, const_cast<char*>(s.c_str()));

  send(sockClient,tem,strlen(tem)+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//init right arm pose
bool init_right_arm_pose(SOCKET &sockClient, float pos_right_arm[]){
  char tem[1024];
  memset(tem, 0, sizeof(tem));

  stringstream ss;
  string s;
  ss << "c:"; 
  for(int i=0; i<ARM_JOINT_NUM-1; i++){
    ss << pos_right_arm[i] << ",";
  }
  ss << pos_right_arm[ARM_JOINT_NUM-1] << ";";

  ss >> s;
  strcpy(tem, const_cast<char*>(s.c_str()));

  send(sockClient,tem,strlen(tem)+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//make left scanner up and down
bool up_down_left_scanner(SOCKET &sockClient, float down_value, float up_value){
  char tem[1024];
  memset(tem, 0, sizeof(tem));

  stringstream ss;
  string s;
  ss << "d:" << down_value << "," << up_value << ";";
  ss >> s;
  strcpy(tem, const_cast<char*>(s.c_str()));

  send(sockClient,tem,strlen(tem)+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//make right scanner up and down
bool up_down_right_scanner(SOCKET &sockClient, float down_value, float up_value){
  char tem[1024];
  memset(tem, 0, sizeof(tem));

  stringstream ss;
  string s;
  ss << "e:" << down_value << "," << up_value << ";";
  ss >> s;
  strcpy(tem, const_cast<char*>(s.c_str()));

  send(sockClient,tem,strlen(tem)+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//l_arm pick up kinect
bool l_pick_up_kinect(SOCKET &sockClient){
  send(sockClient,"f",strlen("f")+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//l_arm put down kinect
bool l_put_down_kinect(SOCKET &sockClient){
  send(sockClient,"g",strlen("g")+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//r_arm pick up kinect
bool r_pick_up_kinect(SOCKET &sockClient){
  send(sockClient,"h",strlen("h")+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//r_arm put down kinect
bool r_put_down_kinect(SOCKET &sockClient){
  send(sockClient,"i",strlen("i")+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//left arm push object
bool l_push_object(SOCKET &sockClient, Eigen::Vector3f& position, Eigen::Vector3f& direction){

  direction.normalize();

  char tem[1024];
  memset(tem, 0, sizeof(tem));

  stringstream ss;
  string s;
  ss << "j:"; 

  ss << position[0] << "," << position[1] << "," << position[2] << "," 
    << direction[0] << "," << direction[1] << "," << direction[2] << ";";

  ss >> s;
  strcpy(tem, const_cast<char*>(s.c_str()));

  send(sockClient,tem,strlen(tem)+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//right arm push object
bool r_push_object(SOCKET &sockClient, Eigen::Vector3f& position, Eigen::Vector3f& direction){
  char tem[1024];
  memset(tem, 0, sizeof(tem));

  stringstream ss;
  string s;
  ss << "k:"; 

  ss << position[0] << "," << position[1] << "," << position[2] << "," 
    << direction[0] << "," << direction[1] << "," << direction[2] << ";";

  ss >> s;
  strcpy(tem, const_cast<char*>(s.c_str()));

  send(sockClient,tem,strlen(tem)+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//set head pose
bool set_head_pose(SOCKET &sockClient, Eigen::Vector3f&head_focus){
  char tem[1024];
  memset(tem, 0, sizeof(tem));

  stringstream ss;
  string s;
  ss << "l:"; 

  ss << head_focus[0] << "," << head_focus[1] << "," << head_focus[2] << ";";

  ss >> s;
  strcpy(tem, const_cast<char*>(s.c_str()));

  send(sockClient,tem,strlen(tem)+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}


//left arm take back
bool l_take_back(SOCKET &sockClient, Eigen::Vector3f& position, Eigen::Vector3f& direction){
  char tem[1024];
  memset(tem, 0, sizeof(tem));

  stringstream ss;
  string s;
  ss << "m:"; 

  ss << position[0] << "," << position[1] << "," << position[2] << "," 
    << direction[0] << "," << direction[1] << "," << direction[2] << ";";

  ss >> s;
  strcpy(tem, const_cast<char*>(s.c_str()));

  send(sockClient,tem,strlen(tem)+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//right arm take back
bool r_take_back(SOCKET &sockClient, Eigen::Vector3f& position, Eigen::Vector3f& direction){
  char tem[1024];
  memset(tem, 0, sizeof(tem));

  stringstream ss;
  string s;
  ss << "n:"; 

  ss << position[0] << "," << position[1] << "," << position[2] << "," 
    << direction[0] << "," << direction[1] << "," << direction[2] << ";";

  ss >> s;
  strcpy(tem, const_cast<char*>(s.c_str()));

  send(sockClient,tem,strlen(tem)+1,0);

  char recvBuf[256];
  recv(sockClient,recvBuf,256,0);

  if(strcmp(recvBuf,"finished")==0){
    return true;
  }
  return false;
}

//get left gripper touch point
void get_l_touch_point_and_dir(SOCKET &sockClient, const Eigen::Vector3f& input_position, const Eigen::Vector3f& input_dir, Eigen::Vector3f& output_position, Eigen::Vector3f& output_dir){
  char tem[1024];
  memset(tem, 0, sizeof(tem));

  stringstream ss;
  string s;
  ss << "o:"; 

  ss << input_position[0] << "," << input_position[1] << "," << input_position[2] << "," 
     << input_dir[0] << "," << input_dir[1] << "," << input_dir[2] << ";";

  ss >> s;
  strcpy(tem, const_cast<char*>(s.c_str()));

  send(sockClient,tem,strlen(tem)+1,0);

  char recvBuf[1024];
  recv(sockClient,recvBuf,1024,0);

  char *ch = recvBuf;
  int index=0;
  int count=0;
  char temp[20];

  //printf("recvBuf:%s\n", recvBuf);

  while(*ch!='\0'){
    if(*ch==','||*ch==';'){
      strncpy(temp,ch-index,index);
      temp[index]='\0';

      if(count < 3){
        output_position[count]=str2float(temp);
      }
      else{
        output_dir[count-3]=str2float(temp);
      }
      index=0;
      count++;
      ch++;
      continue;
    }
    index++;
    ch++;
  }
}

//get right gripper touch point
void get_r_touch_point_and_dir(SOCKET &sockClient, const Eigen::Vector3f& input_position, const Eigen::Vector3f& input_dir, Eigen::Vector3f& output_position, Eigen::Vector3f& output_dir){
  char tem[1024];
  memset(tem, 0, sizeof(tem));

  stringstream ss;
  string s;
  ss << "p:"; 

  ss << input_position[0] << "," << input_position[1] << "," << input_position[2] << "," << input_dir[0] << "," << input_dir[1] << "," << input_dir[2] << ";";

  ss >> s;
  strcpy(tem, const_cast<char*>(s.c_str()));

  send(sockClient,tem,strlen(tem)+1,0);

  char recvBuf[1024];
  recv(sockClient,recvBuf,1024,0);

  char *ch = recvBuf;
  int index=0;
  int count=0;
  char temp[20];

  while(*ch!='\0'){
    if(*ch==','||*ch==';'){
      strncpy(temp,ch-index,index);
      temp[index]='\0';

      if(count < 3){
        output_position[count]=str2float(temp);
      }
      else{
        output_dir[count-3]=str2float(temp);
      }
      index=0;
      count++;
      ch++;
      continue;
    }
    index++;
    ch++;
  }
}

//close the socket
bool close_socket(SOCKET &sockClient){
  send(sockClient,"z",strlen("z")+1,0);
  closesocket(sockClient);
  WSACleanup();

  return true;
}
