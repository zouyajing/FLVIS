#ifndef OCTOMAP_FEEDER_H
#define OCTOMAP_FEEDER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <include/common.h>
#include <include/depth_camera.h>

#include <thread>
#include <mutex>
#include <condition_variable>




class OctomapFeeder
{
    tf::TransformBroadcaster br;
    tf::Transform transform;
    string tf_frame_name;

    ros::Publisher octp_pc_pub;
    ros::Publisher octp_pc_pub_global;

    PointCloudRGB pc_local;
    PointCloudRGB pc_global;

    vector<cv::Mat> imgs;
    vector<cv::Mat> d_imgs;
    vector<SE3> Tcws;
    vector<ros::Time> ts;

    uint16_t  lastKeyframeSize = 0;

    std::mutex mutex_pc_queue_;
    std::queue<cv::Mat> rgb_queue_;
    std::queue<cv::Mat> depth_queue_;
    std::queue<SE3> Tcw_queue_;
    std::thread* th_octomap_viewer;

    std::mutex shutDownMutex;

    condition_variable keyFrameUpdated;
    mutex keyFrameUpdateMutex;

    std::mutex keyframeMutex;



    //sensor model depth camera


public:
    OctomapFeeder();
    OctomapFeeder(ros::NodeHandle& nh, string pc_topic_name, string tf_frame_name_in, int buffersize=2);
    void insert_frame(const SE3 &T_c_w, const cv::Mat &img, const cv::Mat &d_img, const ros::Time stamp);
    void pub(const SE3 &T_c_w,const  cv::Mat &d_img, const ros::Time stamp=ros::Time::now());
    void pub_color_depth(const SE3 &T_c_w, const cv::Mat &img, const cv::Mat &d_img, const ros::Time stamp);
    void viewer();
    DepthCamera  d_camera;
    int step_to_save = 5;
    int start = 0;
};

#endif // OCTOMAP_FEEDER_H
