
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "std_msgs/Bool.h"


#include <time.h>
#include <iomanip>
#include <string.h>


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>


#include <unistd.h>
using namespace std;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "play_bag_from_ipad");
  ros::NodeHandle n("~");

  ros::Publisher save_pointcloud = n.advertise<std_msgs::Bool>("/save_pointcloud",1000);


  char bStop;

  std::cout << "Enter 'q' to exit!" << std::endl;

  while (bStop != 'q'){
           bStop = std::getchar();
           std_msgs::Bool bag_close_flag;
           bag_close_flag.data = true;
           save_pointcloud.publish(bag_close_flag);
       }


  ros::shutdown();
  return 0;

}
