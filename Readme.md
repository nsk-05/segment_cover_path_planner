# segment_cover_path_planner 

Developing Planner for covering given segment in the map  

## INSTALLATION 
    mkdir -p ~/Floor_cleaner_ws/src
    cd ~/Floor_cleaner_ws/src
    git clone {this branch}
    pip3 install shapely

### Build
    cd ~/Floor_cleaner_ws
    catkin_make

### Starting robot
    roslaunch robot_gazebo gazebo.launch

### starting navigation
    roslaunch segment_cover_path_planner test_scpp.launch

### rviz
    roslaunch segment_cover_path_planner rviz.launch

### start scpp
    rosrun segment_cover_path_planner scpp.py
### planner testing
To test the planner first set segment to be cleaned using publish point function in rviz to select segment polygon
and run planner_tester.py in robot_global_planner/scripts

### segment for planning
![plot](./data/Figure_0.png)

### created plan
![plot](./data/Figure_1.png)