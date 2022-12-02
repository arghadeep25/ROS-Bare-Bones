# ROS Bare Bones

The purpose of this repository is to build a framework for any ROS based project.

## Dependencies
- ROS
- C++ 14

##  Quickstart Guide
To build your own project, you need to modify few of the files.
```html
 - apps/ros_backbones_nodelet.cpp
 - nodelet_plugins.xml
 - package.xml
 - launch/ros_back_bones.launch
 - CMakeLists.txt
```


## Build

The idea is to build ROS packages separately. This helps to build any ROS based projects faster. 
```
mkdir ros_package && cd ros_package
```

## Further Reading

[ROS Nodelet Tutorial](http://wiki.ros.org/nodelet/Tutorials/Porting%20nodes%20to%20nodelets)
