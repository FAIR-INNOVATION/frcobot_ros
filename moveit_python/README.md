# moveit python

1. Use executor example
```bash
rosrun moveit_python task_executer.py
```
2. Use generator example
```bash
rosrun moveit_python task_generator.py robot get_robot_param
rosrun moveit_python task_generator.py fr10 joints_position
rosrun moveit_python task_generator.py fr10 joints_position 0 0 0 0 0 0
rosrun moveit_python task_generator.py fr10 end_coordinate tf_end
rosrun moveit_python task_generator.py fr10 end_coordinate hello_box
rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.5 0.2
rosrun moveit_python task_generator.py fr10 attach_object hello_box tf_end
rosrun moveit_python task_generator.py fr10 detach_object hello_box tf_end
rosrun moveit_python task_generator.py fr10 remove_object hello_box
rosrun moveit_python task_generator.py fr10 clear_scene
rosrun moveit_python task_generator.py fr10 gripper_open
rosrun moveit_python task_generator.py fr10 gripper_close
rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect
rosrun moveit_python task_generator.py fr10 choose_pipeline PILZ LIN
rosrun moveit_python task_generator.py fr10 choose_follow_mode
rosrun moveit_python task_generator.py fr10 check_json_files
rosrun moveit_python task_generator.py fr10 detele_json_sim_content test.json
rosrun moveit_python task_generator.py fr10 detele_json_temp
```
Pick and place example for fr10:
0 `rosrun moveit_python task_generator.py fr10 clear_scene`
1 `rosrun moveit_python task_generator.py fr10 remove_object hello_box`
2 `rosrun moveit_python task_generator.py fr10 joints_position 0 -1.57 1.57 0 0 0`
3 `rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.3 0.2 0.0 0.383 0.0 0.924`
4 `rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect`
5 `rosrun moveit_python task_generator.py fr10 end_coordinate hello_box`
6 `rosrun moveit_python task_generator.py fr10 attach_object hello_box tf_end`
7 `rosrun moveit_python task_generator.py fr10 gripper_close`
8 `rosrun moveit_python task_generator.py fr10 choose_pipeline PILZ LIN`
9 `rosrun moveit_python task_generator.py fr10 end_coordinate tf_end -0.33 0.63 0.11 -0.01 0.709 -0.01 0.705`
10 `rosrun moveit_python task_generator.py fr10 gripper_open`
11 `rosrun moveit_python task_generator.py fr10 detach_object hello_box tf_end`
12 `rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect`
13 `rosrun moveit_python task_generator.py fr10 joints_position 0 -1.57 1.57 0 0 0`
14 `rosrun moveit_python task_generator.py fr10 detele_json_sim_content test.json`

Pick and place example for fr3:
0 `rosrun moveit_python task_generator.py fr3 clear_scene`
1 `rosrun moveit_python task_generator.py fr3 remove_object hello_box`
2 `rosrun moveit_python task_generator.py fr3 joints_position 0 -1.57 1.57 0 0 0`
3 `rosrun moveit_python task_generator.py fr3 spawn_object hello_box 0 0.3 0.2 0.0 0.383 0.0 0.924`
4 `rosrun moveit_python task_generator.py fr3 choose_pipeline OMPL RRTConnect`
5 `rosrun moveit_python task_generator.py fr3 end_coordinate hello_box`
6 `rosrun moveit_python task_generator.py fr3 attach_object hello_box tf_end`
7 `rosrun moveit_python task_generator.py fr3 gripper_close`
8 `rosrun moveit_python task_generator.py fr3 choose_pipeline PILZ LIN`
9 `rosrun moveit_python task_generator.py fr3 end_coordinate tf_end`
10 `rosrun moveit_python task_generator.py fr3 gripper_open`
11 `rosrun moveit_python task_generator.py fr3 detach_object hello_box tf_end`
12 `rosrun moveit_python task_generator.py fr3 choose_pipeline OMPL RRTConnect`
13 `rosrun moveit_python task_generator.py fr3 joints_position 0 -1.57 1.57 0 0 0`
14 `rosrun moveit_python task_generator.py fr3 detele_json_sim_content test.json`

3. Use executor json example
```bash
rosrun moveit_python task_executer_json.py fr10 task_grip_sim.json
```

1) Remove time delay
3) Move algorithm