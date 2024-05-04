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

For detecting the blocks, I have added an rgb camera to the robot model which looks at the objects from the top. Then I run opencv to get the objects.
To remove noise I use DBScan for clustering the points. Then the clusters' midpoints are found and IPM is done to get world coordinate. This coordinate can be given
to the code to pick and place from there.
