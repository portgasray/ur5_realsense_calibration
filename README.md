# UR5 REALSENSE CALIBRATION


### Requires

Assume you have the following environment:

* ROS kinetic
* MoveIt 
* librealsense & realsense-ros

### Prepare
1. Build catkin workspace

```
mkdir -p ~/catkin_ws/src
cd  ~/catkin_ws
catkin_make
```

2. Install dependence packages:

```
$ sudo apt-get install ros-kinetic-visp
```


3. Download the calibration lib
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/portgasray/ur5_realsense_calibarion.git --recursive
```
or
```
$ git clone https://github.com/portgasray/ur5_realsense_calibarion.git
$ cd  ur5_realsense_calibarion
$ git submodule foreach --recursive git submodule init
$ git submodule foreach --recursive git submodule update
```
4. Perform catkin make
```
$ catkin_make
```

:star: Before excuting roslaunch command, you should to preform belowing cmd, **Always remember it !**

```
cd ~/catkin_ws
source ./deve/setup.bash
```

### Excution

* launch communicaiton with real ur5 robot using IP
```
$ cd ~/catkin_ws
$ roslaunch ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=169.254.6.80
```

* launch moveit 
```
$ cd ~/catkin_ws
$ roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true
```
* launch point cloud
```
$ cd ~/catkin_ws
$ roslaunch realsense2_camera rs_rgbd.launch
```
* launch calibration program
```
$ cd ~/catkin_ws
$ roslaunch easy_handeye ur5_realsense_handeyecalibration.launch
```

After that, you will see **three** GUI following:

1. ![Image text](./images/automatic_movement.png)

2. ![Image text](./images/easy_handeye.png)

3. ![Image text](./images/rviz.png)

In the GUI window 'rqt_easy_handeye.perspective -rqt': 

Plugins ->  Visualization -> Image View
(you may maximize this window to see the options on the left-top)

![Image text](./images/rqt_image_viewer.jpg)

select "/aruco_tracker/result" in listbox

Let the  broad in the center of your camera screen

![Image text](./images/center_of_screen.jpg)

See the Calibrator automatic movement 

Begin start, Pleas slow down the speed of your robot

![Image text](./images/automatic_movement.png)

Click Check starting Pose Button

Take Next Pose Plan Excute Take Sample (another pannel )as a loop

** Just remember that, robot move a new pose than Take  Sample **

Save ! Finish!

![Image text](./images/loop.png)

You can have a look by publishing your calibration reslut.

1. without close other program window.

```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch  easy_handeye  publish.launch eye_on_hand:=false namespace_prefix:=ur5_realsense_handeyecalibration

```

2. already all the terminal you opened just now.  ##TODO

```
$ roslaunch ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=169.254.6.80
$ roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true
$ roslaunch ur5_moveit_config moveit_rviz.launch config:=true # rviz
$ 
```

### Trouble Shooting

1. ERROR: 'module 'catkin_pkg' not found', try command below to install the corresponding package.
```
sudo pip install -U catkin_pkg
```
In case you experience error with command above, you might want to run it without -U option. see [ROSWiki](http://wiki.ros.org/catkin_pkg)
 
2. Revise the content of ur5_realsense_handeyecalibration.launch
[ur5_realsense_handeyecalibration.launch](https://github.com/portgasray/easy_handeye/blob/5ee30dd50f250452cdc56bfe8f4a7597f9d0b6d6/easy_handeye/launch/ur5_realsense_handeyecalibration.launch) in easy_handeye/launch changed for calibration, you may need to revise to yourself. 


