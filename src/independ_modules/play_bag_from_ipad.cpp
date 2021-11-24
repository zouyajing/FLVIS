
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include "std_msgs/Bool.h"


#include <time.h>
#include <iomanip>
#include <string.h>
#include<regex>
#include<dirent.h>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include<opencv2/core/core.hpp>

#include <unistd.h>
using namespace std;

/*
 *convertBinaryToMat(string depthFile, int height, int width, double depthScale, Mat &depth);
 */
void convertBinaryToMat(string depthFile, int height, int width, int depthScale, cv::Mat &depth)
{

    float d; // depth of one pixel, i.e 3244.89 mm
    ifstream depthBinary;
    depthBinary.open(depthFile, ios::in | ios::binary);

    for (int row = 0; row < height; row++)
    {
        for (int col = 0; col < width; col++)
        {
          depthBinary.read((char *)(&d), sizeof(d));
          depth.at<ushort>(row, col) =  d * depthScale;
        }
    }


}

/*
 * genFileNamesFromFolder(string folder, vector<string> &depthFiles, vector<string> &rgbFiles)
 *
 * output depth.txt rgb.txt and associations.txt in the function
 */
void readFileNamesFromFolder(string folder, vector<string> &depthFiles, vector<string> &rgbFiles)
{
  depthFiles.clear();
  rgbFiles.clear();

  // use DIR and dirent in Linux to read folder
  DIR *dp;
  struct dirent *dirp;

  cout <<  "The directory is: " <<  folder << endl;

  if((dp = opendir(folder.c_str())) == NULL)
    cout << "Can't open " << folder << endl;

  regex reg_depth("DEF.*\\.R00", regex::icase);
  regex reg_rgb(".*\\.png", regex::icase);

  while((dirp = readdir(dp)) != NULL)
  {
        if(dirp->d_type == 8)
        {
          if(regex_match(dirp->d_name, reg_depth))  // regex_match()
            depthFiles.push_back(dirp->d_name);
          if(regex_match(dirp->d_name, reg_rgb))
            rgbFiles.push_back(dirp->d_name);

        }
  }

  sort(depthFiles.begin(), depthFiles.end());
  sort(rgbFiles.begin(), rgbFiles.end());

  //std::cout<<"rgb number:"<<rgbFiles.size()<<" depth number:"<<depthFiles.size()<<std::endl;

  ofstream depFileNames, rgbFileNames, assFile;
  depFileNames.open(folder + "/depth.txt");
  rgbFileNames.open(folder + "/rgb.txt");
  assFile.open(folder + "/associations.txt");


  string tmpDepth, outDepth, tmpRGB, outRGB;
  double timeStampRGB, timeStampDepth;

  struct tm* tmp_time= (struct tm*)malloc(sizeof(struct tm));
  strptime("2021-05-2800:00:00","%Y-%m-%d%H:%M:%S",tmp_time);
  time_t t = mktime(tmp_time);double t0 = (double)(t);
  cout<<t0<<endl;
  //struct tm* tmp_time = (struct tm*)malloc(sizeof(struct tm));
  //strptime("2018-10-0500:00:00","%Y-%m-%d%H:%M:%S",tmp_time);
  //time_t t = mktime(tmp_time);double t0 = (double)(t);
  cout.setf(ios::fixed,ios::floatfield);
  rgbFileNames.setf(ios::fixed, ios::floatfield);
  depFileNames.setf(ios::fixed, ios::floatfield);
  assFile.setf(ios::fixed, ios::floatfield);
  for(int i = 0; i < depthFiles.size(); i++)
  {
    tmpDepth = depthFiles[i];
    outDepth = tmpDepth.substr(0, tmpDepth.find_last_of("."));
    double t1 = stod(outDepth.substr(4,5) + "." + outDepth.substr(9,3));
    //cout<<setprecision(4)<<t1<<endl;
    timeStampDepth = t0 + t1;
    depFileNames << setprecision(4)<<timeStampDepth << " " <<   outDepth + ".R00" << endl;
    tmpRGB = rgbFiles[i];
    outRGB = tmpRGB.substr(0, tmpDepth.find_last_of("."));
    t1 = stod(outDepth.substr(4,5) + "." + outDepth.substr(9,3));
    //cout<<t1<<endl;
    timeStampRGB = t0 + t1;
    rgbFileNames << setprecision(4)<<timeStampRGB << " " <<  outRGB + ".png" << endl;
    assFile << timeStampRGB << " " << outRGB + ".png " << timeStampDepth << " " <<  outDepth + ".R00";
    if(i < (depthFiles.size() -1))
        assFile << endl;
  }

  depFileNames.close();
  rgbFileNames.close();
  assFile.close();


  closedir(dp);

}


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "play_bag_from_ipad");
  ros::NodeHandle n("~");
  std::string dataset_folder = argv[1];
  //n.getParam("dataset_folder", dataset_folder);
  // Retrieve paths to images
  vector<string> vstrImageFilenamesRGB;
  vector<string> vstrImageFilenamesD;
  vector<double> vTimestamps;
  std::cout<<"Folder: "<<dataset_folder<<std::endl;


  std::string strSequence = dataset_folder;

  vector<string> depthFiles, rgbFiles;
  readFileNamesFromFolder(strSequence, depthFiles, rgbFiles);


  string strAssociationFilename = strSequence + "/associations.txt";
  LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

  // Check consistency in the number of images and depthmaps
  int nImages = vstrImageFilenamesRGB.size();
  if(vstrImageFilenamesRGB.empty())
  {
      cerr << endl << "No images found in provided path." << endl;
      return 1;
  }
  else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
  {
      cerr << endl << "Different number of images for rgb and depth." << endl;
      return 1;
  }

  cv::Mat imRGB, imD;

  image_transport::ImageTransport it(n);
  image_transport::Publisher pub_image_rgb = it.advertise("/rgb", 2);
  image_transport::Publisher pub_image_depth = it.advertise("/depth", 2);
  bool is_ipad_data = 1;
  ros::Publisher save_pointcloud = n.advertise<std_msgs::Bool>("/save_pointcloud",1000);

  ros::Rate rate(5);

  for(int ni=0; ni<nImages && ros::ok(); ni++)
  {
      // Read image and depthmap from file
      imRGB = cv::imread(string(argv[1])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
      if(imRGB.channels()==4)
        cv::cvtColor(imRGB,imRGB,CV_BGRA2BGR);
      cv::Mat tmp = cv::Mat::zeros(480, 640, CV_16UC1);
      convertBinaryToMat(string(argv[1]) + "/" +vstrImageFilenamesD[ni], 480, 640, 1, tmp );
      imD = tmp;
      double t = vTimestamps[ni];
      ros::Time ros_t = ros::Time(t);
      if(is_ipad_data)
        ros_t = ros::Time::now();

      cv_bridge::CvImage cvImage;
      cvImage.image = imRGB;
      cvImage.encoding = "bgr8";
      cvImage.header.stamp = ros_t;

      cv_bridge::CvImage cvDepthImage;
      cvDepthImage.image = imD;
      cvDepthImage.encoding = "16UC1";;
      cvDepthImage.header.stamp = ros_t;



      sensor_msgs::ImagePtr image_rgb_msg = cvImage.toImageMsg();
      sensor_msgs::ImagePtr image_depth_msg = cvDepthImage.toImageMsg();
      pub_image_rgb.publish(image_rgb_msg);
      pub_image_depth.publish(image_depth_msg);

      rate.sleep();

  }


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
