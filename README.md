# Patrolling with my bot

Based on [the project](https://github.com/PlanSys2/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/params/nav2_params.yaml)

## Instalation

1. Follow the installation at [my_bot repo](https://github.com/AdrianCobo/my_bot)
2. 

```console
    sudo apt-get install ros-humble-plansys2-*
    cd ~/your_ws/src
    git clone (this repo)
    cd ~/your_ws
    colcon build --symlink-install
```

## Run Gazebo Example

```console
    ros2 launch my_bot launch_sim.launch.py
    cd ~/your_ws
    ros2 launch nav2_bringup localization_launch.py map:=./src/my_bot/maps/home.yaml use_sim_time:=true
    cd ~/your_ws
    rviz2 -d src/my_bot/robot_view.rviz
    # publish a 2D point using rviz at your robot map position
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
    ros2 launch patrolling patrol_example_launch.py
    ros2 run patrolling patrolling_controller_node
```

You can see the video demonstration here: [(Youtube)](https://youtu.be/MbuiRqzs0qQ)

[![Alt text](https://img.youtube.com/vi/MbuiRqzs0qQ/0.jpg)](https://www.youtube.com/watch?v=Q_-EYw8jdps)
