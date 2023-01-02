#!/bin/bash
source install/local_setup.bash
ros2 launch innobot_bringup innobot_default.launch.py use_fake_hardware:=true use_gripper:=true use_pnp:=true use_calibration:=false
