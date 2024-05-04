# Planner Package

##### To spawn bot and load its ros2 controller // GAZEBO from armbot must be running :
```
ros2 launch armbot_moveit_config demo.launch.py
```

##### To execute pick and placing of object :
```
ros2 launch armbot_moveit_config pickplace.launch.py
```

##### To start Block Detector and inverse perspective mapping(IPM) :
```
ros2 run armbot_moveit_config cuboiddetector.py
```