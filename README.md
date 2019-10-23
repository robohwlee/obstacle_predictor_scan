# obstacle_predictor
ROS package which predicts the velocity of dynamic obstalces using local cost-map optical flow.


## Requirements
https://github.com/rst-tu-dortmund/costmap_converter.git

## Launch
```bash
roslaunch obstacle_predictor obstacle_predictor.launch
```

## parameters
- ```global_frame_id```: The name of the coordinate frame published by the localization system. (default: ```/map```)
- ```base_frame_id```: Which frame to use for the robot base. (default: ```/base_footprint```)
- ```global_costmap_topic```: Topic name of the local costmap message, provided by the ```/costmap_2d_node``` node. (default: ```/costmap_node/costmap/costmap```)
- ```local_costmap_topic```: Topic name of the local costmap message, provided by the ```/move_base``` node. (default: ```/move_base/local_costmap/costmap```)
- ```obstacle_topic```: Topic name of the obstacle message to publish. (default: ```/move_base/TebLocalPlanner/obstacles```)
- ```prediction_horizon```: Time horizon for generating predicted obstacles. (default: ```1.0```)
- ```movement_tol_min```: If predicted speed of some dynamic obstacles is slower than this parameter, those obstacles will be ignored. (default: ```0.1```)
- ```movement_tol_max```: If predicted speed of some dynamic obstacles is faster than this parameter, those obstacles will be ignored. (default: ```0.5```)
- ```timediff_tolerence```: If time difference between previous local costmap and current local costmap is longer than this parameter, prediction will be skipped. (default: ```2.0```)
- ```timediff_tolerence```: If difference between forward and backward optical flow is larger than this parameter, the flow vectors will be ignored. (default: ```0.01```)
- ```window_size```: Window size for Lucas-Kanade optical flow. (default: ```3```)
