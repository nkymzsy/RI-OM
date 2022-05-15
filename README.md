# RI-OM

This repository contains code for RI-OM: Using Robust Normal Distributions Transform And Local Map Iterative Update For LiDAR Odometry and Mapping.

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) 
- [gtsam](https://bitbucket.org/gtborg/gtsam) 
- [Ceres Solver](http://ceres-solver.org/installation.html)

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/nkymzsy/RI-OM.git
cd ..
catkin_make -j1
```
When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.

## Run the package

1. Run the launch file:
```
roslaunch ri_om run.launch
```

2. Play existing bag files:
```
rosbag play *.bag --clock
```

## Demo  bag
[demo.bag](https://drive.google.com/file/d/1El3t0DGrSXxGXByLzpi03-YFfFjQAfJS/view?usp=sharing)

![demo.gif](doc/demo.gif)

# Acknowledgements
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM), [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM),[Scan Context](https://github.com/irapkaist/scancontext).


