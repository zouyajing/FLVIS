## FLVIS-mapping

This is a reconstruction demo using FLVIS + pcl/open3d.

The camera pose is outputted by FLVIS, and the 3D model is from PCL/open3d.

Beyond the original dependecies of FLVIS, [open3d](http://www.open3d.org/) should be installed too.

### 1. Installation on Ubuntu 18.04 
a. Install ros melodic based on [ros wiki](http://wiki.ros.org/melodic/Installation/Ubuntu).`ros-melodic-desktop-full` is preferred.
b. Install opencv3 by `sudo apt install libopencv-dev`
c. Download the package to your working space 
```
mkdir ~/cat_ws/src
cd ~/cat_ws/src
git clone git@github.com:zouyajing/FLVIS.git
```
d. Install the dependencies of orignal FLVIS
```
cd FLVIS/3rdPartLib
sudo sh install3rdPartLib.sh
```
e. Install `open3d`
f. Build the package
```
cd ~/cat_ws
catkin_make -j
```


### 2. Run the examples

Run the demo:

```
roslaunch flvis ss_ipad.launch                  (run flvis + 3D reconstruction)
rosrun flvis play_bag_from_ipad ~/corridor_ss/  (publish RGB-D image messages using ipad+ss dataset)

```
In the second terminal, enter 'q', and the loop closing thread will run save_callback function to perform point cloud reconstrcution.

Open another terminal, and enter 'rosservice call /save_map'. The thread will run saveMapCallback function to perform TSDF reconstrcution.

You can download an example dataset [here](https://drive.google.com/drive/folders/1gPuoolWCTm3IXKiE5yxaPDEBad07vjx3?usp=sharing).

The reconstruction demo is 

![here](https://github.com/zouyajing/PhD_document_for_navlab/blob/main/imgs/FLVIS_mapping.png)


### 3. The modifications 

Some functions are added to support 3D reconstrcution:

* independ_modules
  * play_bag_from_ipad. It publishes the RGB-D image messages from the ipad+ss dataset.
  * trigger_save_pts. It publishes a bool message after the dataset is finished.
* octofeeder. It publish a real-time point cloud map.
* vo_loopclosing.cpp. Two functions are added to save the offline map:
  * save_callback(const std_msgs::Bool::ConstPtr& save). 
    
    It is to save offline point cloud map.
    
  * bool saveMapCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res).
   
    It is to perform TSDF reconstruction. Please control the size of your map, as TSDF consumes lots of memory.
  



