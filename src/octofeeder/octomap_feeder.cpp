#include "include/octomap_feeder.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include "include/tic_toc_ros.h"


OctomapFeeder::OctomapFeeder()
{

}


OctomapFeeder::OctomapFeeder(ros::NodeHandle& nh, string pc_topic_name, string tf_frame_name_in, int buffersize)
{
    this->tf_frame_name = tf_frame_name_in;
    this->octp_pc_pub = nh.advertise<sensor_msgs::PointCloud2>(pc_topic_name,buffersize);
    this->octp_pc_pub_global = nh.advertise<sensor_msgs::PointCloud2>("/octo_global_pts",buffersize);
    th_octomap_viewer = new std::thread ( &OctomapFeeder::viewer, this );
}
void OctomapFeeder::insert_frame(const SE3 &T_c_w, const cv::Mat &img, const cv::Mat &d_img, const ros::Time stamp)
{
  unique_lock<mutex> lck(keyframeMutex);
  Tcws.push_back(T_c_w);
  d_imgs.push_back(d_img);
  imgs.push_back(img);
  ts.push_back(stamp);
  keyFrameUpdated.notify_one();

}

void OctomapFeeder::viewer()
{
  while(1)
      {
          {
              unique_lock<mutex> lck_shutdown( shutDownMutex );
          }
          {
              unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
              keyFrameUpdated.wait( lck_keyframeUpdated );
          }

          // keyframe is updated
          size_t N=0;
          {
              unique_lock<mutex> lck( keyframeMutex );
              N = imgs.size();
          }

          for ( size_t i=lastKeyframeSize; i<N ; i++ )
          {
            SE3 T_w_c=Tcws[i].inverse();
            SE3 T_c_w=Tcws[i];
            cv::Mat img = imgs[i];
            bool image_type = (img.channels()==1);
            cv::Mat d_img = d_imgs[i];
            Quaterniond q = T_w_c.so3().unit_quaternion();
            Vec3        t = T_w_c.translation();

            PointCloudRGB::Ptr pc_c(new PointCloudRGB);
            PointCloudRGB::Ptr pc_w(new PointCloudRGB);
            int width_count=0;
            int step = 5;
            for(int v = 0; v<d_img.rows; v+=step)
            {
                for(int u = 0; u<d_img.cols; u+=step)
                {
                    cv::Point2f pt=cv::Point2f(u,v);
                    Vec3 pt3d;
                    //CV_16UC1 = Z16 16-Bit unsigned int
                    if(isnan(d_img.at<ushort>(pt)))
                    {
                        continue;
                    }
                    else
                    {
                        float z = (d_img.at<ushort>(pt))/d_camera.cam_scale_factor;
                        //cout<<"Depth of point: "<<z<<endl;
                        if(z>0.2&&z<=4.0)
                        {
                            Vec3 pt_w = this->d_camera.pixel2worldT_c_w(Vec2(u,v),T_c_w,z);

                            Vec3 pt_c = this->d_camera.pixel2camera(Vec2(u,v),z);
                            PointRGB p, p_g;
                            p.x = static_cast<float>(pt_c[0]);
                            p.y = static_cast<float>(pt_c[1]);
                            p.z = static_cast<float>(pt_c[2]);
                            p_g.x = static_cast<float>(pt_w[0]);
                            p_g.y = static_cast<float>(pt_w[1]);
                            p_g.z = static_cast<float>(pt_w[2]);
                            if (image_type)
                            {
                              p.b = img.data[ v*img.step+u*img.channels()];
                              p.g = 0;
                              p.r = 0;
                              p_g.b = img.data[ v*img.step+u*img.channels()];
                              p_g.g = 0;
                              p_g.r = 0;
                            }
                            else
                            {
                              p.b = img.ptr<uchar>(v)[u*3];
                              p.g = img.ptr<uchar>(v)[u*3+1];
                              p.r = img.ptr<uchar>(v)[u*3+2];
                              p_g.b = p.b;// img.data[ v*img.step+u*img.channels()];
                              p_g.g = p.g;//img.data[ v*img.step+u*img.channels() + 1];
                              p_g.r = p.r;//img.data[ v*img.step+u*img.channels() + 2];
                            }

                            pc_c->points.push_back(p);
                            pc_w->points.push_back(p_g);
                            width_count++;
                        }else
                        {
                            continue;
                        }
                    }
                }
            }
            pc_c->width=width_count;
            pc_c->height=1;
            pc_c->is_dense = false;
            pc_w->width=width_count;
            pc_w->height=1;
            pc_w->is_dense = false;
            pc_local = *pc_w;
            pc_global += *pc_w;
          }



          if(lastKeyframeSize< N && N%step_to_save==0)
          {
            sensor_msgs::PointCloud2 output_global;
            pcl::toROSMsg(pc_global,output_global);
            output_global.header.frame_id = "map";
            octp_pc_pub_global.publish(output_global);

            //pcl::io::savePLYFileBinary("/home/lsgi/FLVIS/src/FLVIS/results/model.ply", pc_global);
          }

          lastKeyframeSize = N;
      }

}
void OctomapFeeder::pub_color_depth(const SE3 &T_c_w, const cv::Mat &img, const cv::Mat &d_img, const ros::Time stamp)
{

    bool image_type = (img.channels()==1);
    SE3 T_w_c=T_c_w.inverse();
    Quaterniond q = T_w_c.so3().unit_quaternion();
    Vec3        t = T_w_c.translation();

    this->transform.setOrigin(tf::Vector3(t[0],t[1],t[2]));
    this->transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));

    br.sendTransform(tf::StampedTransform(transform, stamp, "map", this->tf_frame_name));

    PointCloudRGB::Ptr pc_c(new PointCloudRGB);
    PointCloudRGB::Ptr pc_w(new PointCloudRGB);
    int width_count=0;
    Vec3 pos_w_c= T_c_w.inverse().translation();
    double height=pos_w_c[2];
    int step = 10;
    for(int v = 0; v<d_img.rows; v+=step)
    {
        for(int u = 0; u<d_img.cols; u+=step)
        {
            cv::Point2f pt=cv::Point2f(u,v);
            Vec3 pt3d;
            //CV_16UC1 = Z16 16-Bit unsigned int
            if(isnan(d_img.at<ushort>(pt)))
            {
                continue;
            }
            else
            {
                float z = (d_img.at<ushort>(pt))/d_camera.cam_scale_factor;
                //cout<<"Depth of point: "<<z<<endl;
                if(z>0.2&&z<=4.0)
                {
                    Vec3 pt_w = this->d_camera.pixel2worldT_c_w(Vec2(u,v),T_c_w,z);

                    Vec3 pt_c = this->d_camera.pixel2camera(Vec2(u,v),z);
                    PointRGB p, p_g;
                    p.x = static_cast<float>(pt_c[0]);
                    p.y = static_cast<float>(pt_c[1]);
                    p.z = static_cast<float>(pt_c[2]);
                    p_g.x = static_cast<float>(pt_w[0]);
                    p_g.y = static_cast<float>(pt_w[1]);
                    p_g.z = static_cast<float>(pt_w[2]);
                    if (image_type)
                    {
                      p.b = img.data[ v*img.step+u*img.channels()];
                      p.g = 0;
                      p.r = 0;
                      p_g.b = img.data[ v*img.step+u*img.channels()];
                      p_g.g = 0;
                      p_g.r = 0;
                    }
                    else
                    {
                      p.b = img.ptr<uchar>(v)[u*3];
                      p.g = img.ptr<uchar>(v)[u*3+1];
                      p.r = img.ptr<uchar>(v)[u*3+2];
                      p_g.b = p.b;// img.data[ v*img.step+u*img.channels()];
                      p_g.g = p.g;//img.data[ v*img.step+u*img.channels() + 1];
                      p_g.r = p.r;//img.data[ v*img.step+u*img.channels() + 2];
                    }

                    pc_c->points.push_back(p);
                    pc_w->points.push_back(p_g);
                    width_count++;
                }else
                {
                    continue;
                }
            }
        }
    }
    pc_c->width=width_count;
    pc_c->height=1;
    pc_c->is_dense = false;
    pc_w->width=width_count;
    pc_w->height=1;
    pc_w->is_dense = false;

    pc_local = *pc_w;
    pc_global += *pc_w;


    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pc_c,output);
    output.header.frame_id = tf_frame_name;
    octp_pc_pub.publish(output);
    //cout<<"point cloud filter cost: ";



    if(start++%step_to_save==0)
    {
      sensor_msgs::PointCloud2 output_global;
      pcl::toROSMsg(pc_global,output_global);
      output_global.header.frame_id = "map";
      octp_pc_pub_global.publish(output_global);

      pcl::io::savePLYFileBinary("/home/lsgi/FLVIS/src/FLVIS/results/model.ply", pc_global);
    }


}


void OctomapFeeder::pub(const SE3 &T_c_w,const  cv::Mat &d_img, const ros::Time stamp)
{
    SE3 T_w_c=T_c_w.inverse();
    Quaterniond q = T_w_c.so3().unit_quaternion();
    Vec3        t = T_w_c.translation();

    this->transform.setOrigin(tf::Vector3(t[0],t[1],t[2]));
    this->transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));

    br.sendTransform(tf::StampedTransform(transform, stamp, "map", this->tf_frame_name));

    PointCloudP::Ptr pc_c(new PointCloudP);
    int width_count=0;
    Vec3 pos_w_c= T_c_w.inverse().translation();
    double height=pos_w_c[2];
    int step = 7;
    int lines = 3;
    for(int v = d_img.rows/2-step*lines-1;v <d_img.rows/2+step*lines;v+=step )
    {
        for(int u = 0; u<d_img.cols; u+=step)
        {
            cv::Point2f pt=cv::Point2f(u,v);
            Vec3 pt3d;
            //CV_16UC1 = Z16 16-Bit unsigned int
            if(isnan(d_img.at<ushort>(pt)))
            {
                continue;
            }
            else
            {
                float z = (d_img.at<ushort>(pt))/d_camera.cam_scale_factor;
                if(z>=0.5&&z<=6.5)
                {
                    Vec3 pt_w = this->d_camera.pixel2worldT_c_w(Vec2(u,v),T_c_w,z);
                    if(pt_w[2]>height+0.3 || pt_w[2]<0)
                    {
                        //continue;
                    }
                    Vec3 pt_c = this->d_camera.pixel2camera(Vec2(u,v),z);
                    PointP p(static_cast<float>(pt_c[0]),
                            static_cast<float>(pt_c[1]),
                            static_cast<float>(pt_c[2]));
                    pc_c->points.push_back(p);
                    width_count++;
                }else
                {
                    continue;
                }
            }
        }
    }
    pc_c->width=width_count;
    pc_c->height=1;
    pc_c->is_dense = false;


    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pc_c,output);
    output.header.frame_id = tf_frame_name;
    octp_pc_pub.publish(output);
    //cout<<"point cloud filter cost: ";

}

