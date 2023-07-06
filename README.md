Based on the project:
https://github.com/PlanSys2/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/params/nav2_params.yaml

Execute example:
ros2 launch my_bot launch_sim.launch.py
rviz2
ros2 launch nav2_bringup localization_launch.py map:=./src/my_bot/maps/home.yaml use_sim_time:=true
publicar punto con posicion y orientacion aprox usando rviz
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
ros2 launch patrolling patrol_example_launch.py
ros2 run patrolling patrolling_controller_node
