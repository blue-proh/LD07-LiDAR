## Connect LD07 of our company and start ROS.

```sh
sudo chmod 777 /dev/ttyUSB*
roscore
```


## COMPILE

Download and compile with catkin, as follows.

```sh
catkin_make

```


## RUN

```sh
source devel/setup.bash
roslaunch ldlidar ld07.launch
```

then run rviz 

```sh
rosrun rviz rviz
```

## TEST

Code in ubuntun16.04 It is tested under the version of kinetic and visualized with rviz.
