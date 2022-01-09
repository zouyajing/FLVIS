## FLVIS-mapping

This is a reconstruction demo using FLVIS + pcl/open3d.

The camera pose is outputted by FLVIS, and the 3D model is from PCL/open3d.

Beyond the original dependecies of FLVIS, [open3d](http://www.open3d.org/) should be installed too.

### 1. Installation on Ubuntu 18.04 
1.a Install `ros melodic` based on [ros wiki](http://wiki.ros.org/melodic/Installation/Ubuntu). `ros-melodic-desktop-full` is preferred.

1.b Install `opencv3` by `sudo apt install libopencv-dev`

1.c Download `FLVIS` to your working space 
```
mkdir -p ~/cat_ws/src
cd ~/cat_ws/src
git clone https://github.com/zouyajing/FLVIS.git
```

1.d Install the dependencies of orignal `FLVIS`
```
cd FLVIS/3rdPartLib
sudo sh install3rdPartLib.sh
```

1.e Install `open3d`

  `e1` Download `open3d` to `~/cat_ws/src/FLVIS/3rdPartLib`
  ```
  cd ~/cat_ws/src/FLVIS/3rdPartLib
  git clone https://github.com/isl-org/Open3D
  ```
  `e2` Update CMake version to 3.19 following [official APT repository](https://apt.kitware.com/)

  `e3` Install the dependencies of `open3d` by `source Open3D/util/install_deps_ubuntu.sh`

  `e4` Modify Line 77 of `CMakeLists.txt` to use the `eigen3` in system
  ```
  option(USE_SYSTEM_EIGEN3          "Use system pre-installed eigen3"          ON)
  ```
  `e5` Modify Line 56 and 350 in `Open3D/cpp/open3d/geometry/Line3D.cpp`
  ```
  this->Transform(t);
  ```
  `e6` Build the `open3d`
  ```
  cd Open3D
  mkdir build && cd build
  cmake ..
  make -j2
  sudo make install
  ```

1.f Modify the file names of the saved models in `FLVIS/src/backend/vo_loopclosing.cpp`. In Lines 198, 264,285, and 956, replace `rick` with `your user name`, i.e. `lee` 

1.g Build the package
```
cd ~/cat_ws
catkin_make -j
```


### 2. Run the examples
2.a Download an example dataset [here](https://drive.google.com/drive/folders/1gPuoolWCTm3IXKiE5yxaPDEBad07vjx3?usp=sharing).

2.b Uncompress it and copy it to `~/corridor_ss`

2.c Run the demo in two terminals

```
roslaunch flvis ss_ipad.launch                  (run flvis + 3D reconstruction)
rosrun flvis play_bag_from_ipad ~/corridor_ss//corridor_ss/  (publish RGB-D image messages using ipad+ss dataset)

```
2.d Save the reconstructed model using `pcl`. Open the third terminal, and enter `rosservice call /save_map`. The loop closing thread will run saveMapCallback function to perform pcl reconstrcution.

2.e Save the reconstructed model using `open3d`. In the second terminal, enter 'q', and the loop closing thread will run save_callback function to perform TSDF reconstrcution.

2.f The reconstruction demo is 

![here](https://github.com/zouyajing/PhD_document_for_navlab/blob/main/imgs/FLVIS_mapping.png)

However, close the dense point cloud in rviz will help to improve the robustness of the system.


### 3. The modifications 

Some functions are added to support 3D reconstrcution:

* independ_modules
  * play_bag_from_ipad. It publishes the RGB-D image messages from the ipad+ss dataset.
  * trigger_save_pts. It publishes a bool message after the dataset is finished.
* octofeeder. It publish a real-time point cloud map.
* vo_loopclosing.cpp. Two functions are added to save the offline map:
  * bool saveMapCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res).
    
    It is to save offline point cloud map.
    
  * save_callback(const std_msgs::Bool::ConstPtr& save). 
   
    It is to perform TSDF reconstruction. Please control the size of your map, as TSDF consumes lots of memory.
  



