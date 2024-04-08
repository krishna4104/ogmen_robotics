# Ogmen Robotics Task
- [x] First Created URDF model using Fusion 360
- [x] Then converted ogmenbot to URDF using https://github.com/nilutpolkashyap/fusion2urdf plugin
## Rviz Launch
- [x] Added rviz.launch along with all sensor data
```
ros2 launch ogmen_description rviz.launch.py
```
![image](https://github.com/krishna4104/ogmen_ws/assets/140909916/684f8f70-6d8f-4a8a-abed-30cace72f28d)
- [x] Added spawn.launch.py
  To launch rviz and gazebo
```
ros2 launch ogmen_description rviz.launch.py
```
![image](https://github.com/krishna4104/ogmen_ws/assets/140909916/1b335c6c-7493-445a-886e-c47a53b4e9bd)
- [x] Added script modify the /scan data and filter it to have only a range of 0 to 120 degrees as its field of view and publish this filtered data to a new topic /filtered_scan. To run
```
ros2 run ogmen_description filter
```
![image](https://github.com/krishna4104/ogmen_ws/assets/140909916/dcf3e31d-86ad-4489-8f4c-fa03ae945161)
- [x] (Working) on Move.py - this file will read a list waypoints coordinates [x (meters), y (meters), yaw_angle(degrees)] : [0,0,0] [5,0,45] [5,5,90] [0,5,0] [0,0,135]
